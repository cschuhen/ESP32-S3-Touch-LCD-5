//! Shared async CH422G I/O expander controller.
//!
//! Because CH422G's `0x38` (`WR_IO`) address is write-only, all runtime pin
//! updates (`DISP`, `SD_CS`, `USB_SEL`) must go through a single
//! `Mutex`-protected [`Ch422gAsync`](ch422g::Ch422gAsync) instance so their
//! software shadow byte stays consistent.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::consts::ch422g_pins;
use crate::shared_bus::BoardI2cDevice;

/// Mutex-wrapped async CH422G driver shared across BSP peripherals.
pub type SharedCh422g = Mutex<CriticalSectionRawMutex, ch422g::Ch422gAsync<BoardI2cDevice>>;

/// Error returned by expander operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpanderError {
    /// The underlying I²C bus transaction failed.
    Bus,
}

/// Handle to the non-display CH422G expansion pins (`SD_CS`, `USB_SEL`).
#[derive(Clone, Copy)]
pub struct Expander {
    ch: &'static SharedCh422g,
}

impl Expander {
    pub(crate) fn new(ch: &'static SharedCh422g) -> Self {
        Self { ch }
    }

    /// Assert (`low = true`) or deassert (`low = false`) `SD_CS` (`EXIO4`, active low).
    pub async fn set_sd_cs_active(&self, active: bool) -> Result<(), ExpanderError> {
        let mut guard = self.ch.lock().await;
        guard
            .set_pin(ch422g_pins::SD_CS_BIT, !active)
            .await
            .map_err(|_| ExpanderError::Bus)
    }

    /// Set `USB_SEL` (`EXIO5`): `false` = USB (default), `true` = CAN.
    pub async fn set_usb_sel(&self, can_mode: bool) -> Result<(), ExpanderError> {
        let mut guard = self.ch.lock().await;
        guard
            .set_pin(ch422g_pins::USB_SEL_BIT, can_mode)
            .await
            .map_err(|_| ExpanderError::Bus)
    }

    /// Return the current software shadow byte.
    pub async fn shadow(&self) -> u8 {
        self.ch.lock().await.shadow()
    }
}
