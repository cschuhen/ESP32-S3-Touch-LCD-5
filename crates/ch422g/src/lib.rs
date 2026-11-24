//! Minimal `no_std` driver for the WCH **CH422G** I²C I/O expander.
//!
//! Unlike register-addressed expanders (PCA9554/TCA9554), the CH422G uses
//! distinct 7-bit I²C addresses for each single-byte command:
//! - `0x24` (`WR_SET`): System parameter / mode control (`0x01` = push-pull outputs enabled).
//! - `0x38` (`WR_IO`): Write `EXIO0..EXIO7` output latch (write-only).
//! - `0x26` (`RD_IO`): Read `EXIO0..EXIO7` input levels.
//!
//! Because `WR_IO` is write-only, this driver maintains a software shadow byte
//! (`io_state`) so individual pin toggles preserve all other pin states.

#![no_std]
#![deny(missing_docs)]

/// 7-bit I²C address for system parameter / mode control (`WR_SET`).
pub const ADDR_WR_SET: u8 = 0x24;
/// 7-bit I²C address for writing `EXIO0..EXIO7` output state (`WR_IO`).
pub const ADDR_WR_IO: u8 = 0x38;
/// 7-bit I²C address for reading `EXIO0..EXIO7` input state (`RD_IO`).
pub const ADDR_RD_IO: u8 = 0x26;

/// System mode byte: `IO_OE = 1` (IO0..IO7 push-pull outputs enabled), `OD_EN = 0`, `SLEEP = 0`.
pub const MODE_OUTPUT_ENABLE: u8 = 0x01;

/// Synchronous CH422G driver for boot initialization.
///
/// Does not own the I²C bus so callers can share `&mut I2C` with other
/// peripherals during synchronous bring-up without wrapper types.
pub struct Ch422g {
    io_state: u8,
}

impl Ch422g {
    /// Create a driver with an initial shadow byte (does not perform I²C writes yet).
    pub const fn new(initial_io: u8) -> Self {
        Self { io_state: initial_io }
    }

    /// Pre-load the output latch (`0x38`) *before* enabling output drivers (`0x24`)
    /// so pins transition from high-Z to their intended level without glitching.
    pub fn init<I2C>(&mut self, i2c: &mut I2C) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        i2c.write(ADDR_WR_IO, &[self.io_state])?;
        i2c.write(ADDR_WR_SET, &[MODE_OUTPUT_ENABLE])?;
        Ok(())
    }

    /// Write all 8 output bits (`EXIO0..EXIO7`) in a single I²C transaction.
    pub fn write_all<I2C>(&mut self, i2c: &mut I2C, value: u8) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        i2c.write(ADDR_WR_IO, &[value])?;
        self.io_state = value;
        Ok(())
    }

    /// Set or clear a single output bit (`0..=7`) while preserving all other bits.
    pub fn set_pin<I2C>(&mut self, i2c: &mut I2C, bit: u8, high: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        debug_assert!(bit < 8);
        let next = if high {
            self.io_state | (1 << bit)
        } else {
            self.io_state & !(1 << bit)
        };
        self.write_all(i2c, next)
    }

    /// Return the current software shadow byte.
    pub const fn shadow(&self) -> u8 {
        self.io_state
    }
}

/// Asynchronous CH422G driver owning an `embedded_hal_async::i2c::I2c` device.
#[cfg(feature = "async")]
pub struct Ch422gAsync<I2C> {
    i2c: I2C,
    io_state: u8,
}

#[cfg(feature = "async")]
impl<I2C> Ch422gAsync<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    /// Construct from an already-initialized blocking [`Ch422g`] shadow byte.
    pub const fn from_shadow(i2c: I2C, io_state: u8) -> Self {
        Self { i2c, io_state }
    }

    /// Write all 8 output bits (`EXIO0..EXIO7`) asynchronously.
    pub async fn write_all(&mut self, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(ADDR_WR_IO, &[value]).await?;
        self.io_state = value;
        Ok(())
    }

    /// Set or clear a single output bit (`0..=7`) asynchronously while preserving all other bits.
    pub async fn set_pin(&mut self, bit: u8, high: bool) -> Result<(), I2C::Error> {
        debug_assert!(bit < 8);
        let next = if high {
            self.io_state | (1 << bit)
        } else {
            self.io_state & !(1 << bit)
        };
        self.write_all(next).await
    }

    /// Return the current software shadow byte.
    pub const fn shadow(&self) -> u8 {
        self.io_state
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct MockI2c {
        writes: Vec<(u8, Vec<u8>)>,
        fail_next: bool,
    }

    impl MockI2c {
        fn new() -> Self {
            Self {
                writes: Vec::new(),
                fail_next: false,
            }
        }
    }

    impl ErrorType for MockI2c {
        type Error = MockError;
    }

    impl I2c for MockI2c {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if self.fail_next {
                self.fail_next = false;
                return Err(MockError);
            }
            for op in operations {
                if let Operation::Write(bytes) = op {
                    self.writes.push((address, bytes.to_vec()));
                }
            }
            Ok(())
        }
    }

    #[test]
    fn init_preloads_io_before_enabling_outputs() {
        let mut bus = MockI2c::new();
        let mut ch = Ch422g::new(0x10);
        ch.init(&mut bus).unwrap();

        assert_eq!(
            bus.writes,
            std::vec![
                (ADDR_WR_IO, std::vec![0x10]),
                (ADDR_WR_SET, std::vec![MODE_OUTPUT_ENABLE]),
            ]
        );
        assert_eq!(ch.shadow(), 0x10);
    }

    #[test]
    fn set_pin_preserves_other_bits() {
        let mut bus = MockI2c::new();
        let mut ch = Ch422g::new(0x10); // bit 4 set (SD_CS)
        ch.init(&mut bus).unwrap();

        ch.set_pin(&mut bus, 1, true).unwrap(); // TP_RST high -> 0x12
        ch.set_pin(&mut bus, 3, true).unwrap(); // LCD_RST high -> 0x1A
        ch.set_pin(&mut bus, 2, true).unwrap(); // DISP high -> 0x1E
        ch.set_pin(&mut bus, 2, false).unwrap(); // DISP low -> 0x1A

        assert_eq!(ch.shadow(), 0x1A);
        assert_eq!(
            &bus.writes[2..],
            &[
                (ADDR_WR_IO, std::vec![0x12]),
                (ADDR_WR_IO, std::vec![0x1A]),
                (ADDR_WR_IO, std::vec![0x1E]),
                (ADDR_WR_IO, std::vec![0x1A]),
            ]
        );
    }

    #[test]
    fn error_does_not_mutate_shadow() {
        let mut bus = MockI2c::new();
        let mut ch = Ch422g::new(0x1A);
        bus.fail_next = true;

        assert_eq!(ch.set_pin(&mut bus, 2, true), Err(MockError));
        assert_eq!(ch.shadow(), 0x1A);
    }
}
