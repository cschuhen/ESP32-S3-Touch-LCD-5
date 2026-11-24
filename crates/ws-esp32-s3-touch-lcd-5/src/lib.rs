//! Board-support package for the **Waveshare ESP32-S3-Touch-LCD-5** (Rev 1.2)
//! — a 5.0" 800×480 RGB565 IPS panel (or 1024×600 with `panel-1024x600`) with
//! a Goodix GT911 capacitive touch controller and WCH CH422G I/O expander on
//! an ESP32-S3-WROOM-1-N16R8.

#![no_std]
#![deny(missing_docs)]

mod bounce_buffer;
mod framebuffer;
mod shared_bus;

mod backlight;
mod expander;
mod touch;

pub mod consts;

pub use backlight::Backlight;
pub use expander::{Expander, ExpanderError, SharedCh422g};
pub use shared_bus::{BoardI2cDevice, SharedI2c};
pub use touch::{Point, TouchPoller};

use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    lcd_cam::{
        lcd::{
            dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
            ClockMode, Phase, Polarity,
        },
        LcdCam,
    },
    peripherals,
    time::Rate,
};

pub use ch422g;
pub use gt911;

pub use bounce_buffer::BounceRing;
pub use framebuffer::{
    Framebuffer, BYTES as FRAMEBUFFER_BYTES, HEIGHT, PSRAM_BYTES_REQUIRED, WIDTH,
};

/// All board peripherals and pins consumed by [`init`].
#[allow(missing_docs)]
pub struct Resources<'d> {
    pub i2c0:      peripherals::I2C0<'d>,
    pub lcd_cam:   peripherals::LCD_CAM<'d>,
    pub dma_ch0:   peripherals::DMA_CH0<'d>,
    pub psram:     peripherals::PSRAM<'d>,

    pub sda:       peripherals::GPIO8<'d>,
    pub scl:       peripherals::GPIO9<'d>,
    pub touch_int: peripherals::GPIO4<'d>,

    pub vsync:     peripherals::GPIO3<'d>,
    pub hsync:     peripherals::GPIO46<'d>,
    pub de:        peripherals::GPIO5<'d>,
    pub pclk:      peripherals::GPIO7<'d>,

    pub data: DpiDataPins<'d>,
}

/// The 16 DPI RGB565 data pins (`B3..B7`, `G2..G7`, `R3..R7`).
#[allow(missing_docs)]
pub struct DpiDataPins<'d> {
    pub b3: peripherals::GPIO14<'d>,
    pub b4: peripherals::GPIO38<'d>,
    pub b5: peripherals::GPIO18<'d>,
    pub b6: peripherals::GPIO17<'d>,
    pub b7: peripherals::GPIO10<'d>,
    pub g2: peripherals::GPIO39<'d>,
    pub g3: peripherals::GPIO0<'d>,
    pub g4: peripherals::GPIO45<'d>,
    pub g5: peripherals::GPIO48<'d>,
    pub g6: peripherals::GPIO47<'d>,
    pub g7: peripherals::GPIO21<'d>,
    pub r3: peripherals::GPIO1<'d>,
    pub r4: peripherals::GPIO2<'d>,
    pub r5: peripherals::GPIO42<'d>,
    pub r6: peripherals::GPIO41<'d>,
    pub r7: peripherals::GPIO40<'d>,
}

/// Destructure the Waveshare ESP32-S3-Touch-LCD-5 standard pinout from `Peripherals`.
#[macro_export]
macro_rules! take_resources {
    ($p:expr) => {
        $crate::Resources {
            i2c0:      $p.I2C0,
            lcd_cam:   $p.LCD_CAM,
            dma_ch0:   $p.DMA_CH0,
            psram:     $p.PSRAM,
            sda:       $p.GPIO8,
            scl:       $p.GPIO9,
            touch_int: $p.GPIO4,
            vsync:     $p.GPIO3,
            hsync:     $p.GPIO46,
            de:        $p.GPIO5,
            pclk:      $p.GPIO7,
            data: $crate::DpiDataPins {
                b3: $p.GPIO14, b4: $p.GPIO38, b5: $p.GPIO18, b6: $p.GPIO17, b7: $p.GPIO10,
                g2: $p.GPIO39, g3: $p.GPIO0,  g4: $p.GPIO45, g5: $p.GPIO48, g6: $p.GPIO47, g7: $p.GPIO21,
                r3: $p.GPIO1,  r4: $p.GPIO2,  r5: $p.GPIO42, r6: $p.GPIO41, r7: $p.GPIO40,
            },
        }
    };
}

/// Peripheral handles returned by [`init`].
#[non_exhaustive]
pub struct Board {
    /// PSRAM double-buffered framebuffer.
    pub framebuffer: Framebuffer,
    /// Shared async I²C0 bus (`StaticCell`-backed).
    pub i2c: &'static SharedI2c,
    /// GT911 capacitive touch poller.
    pub touch: TouchPoller,
    /// LCD backlight (`DISP` on CH422G `EXIO2`).
    pub backlight: Backlight,
    /// CH422G expansion pin controller (`SD_CS`, `USB_SEL`).
    pub expander: Expander,
}

/// Errors returned by [`init`].
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// `I2c::new` rejected its configuration.
    I2cConfig,
    /// CH422G didn't ACK an I²C write (`0x24` or `0x38`).
    Ch422g,
    /// GT911 didn't ACK or returned an unexpected product ID.
    Gt911,
    /// `Dpi::new` rejected its configuration.
    Dpi,
    /// Mapped PSRAM region is smaller than [`PSRAM_BYTES_REQUIRED`].
    PsramTooSmall,
    /// DPI DMA transfer failed to start.
    DpiStart,
}

/// Bring up the Waveshare ESP32-S3-Touch-LCD-5 board.
///
/// Must be called at most once per boot.
pub fn init(r: Resources<'static>) -> Result<Board, Error> {
    let delay = Delay::new();

    // ----- I²C0 bus (400 kHz, blocking during boot) -------------------------
    let mut i2c = I2c::new(
        r.i2c0,
        I2cConfig::default().with_frequency(Rate::from_hz(consts::i2c::FREQUENCY_HZ)),
    )
    .map_err(|_| Error::I2cConfig)?
    .with_sda(r.sda)
    .with_scl(r.scl);

    // ----- CH422G + LCD/Touch Reset + GT911 0x5D strapping -----------------
    let touch_int = Output::new(r.touch_int, Level::Low, OutputConfig::default());

    let mut ch = ch422g::Ch422g::new(consts::ch422g_pins::MASK_RESET_LOW);
    ch.init(&mut i2c).map_err(|_| Error::Ch422g)?;
    delay.delay_millis(20);

    // Release TP_RST & LCD_RST while GPIO4 (TP_IRQ) is still LOW -> latches 0x5D.
    ch.write_all(&mut i2c, consts::ch422g_pins::MASK_IDLE_DARK)
        .map_err(|_| Error::Ch422g)?;
    delay.delay_millis(10);

    // Release GPIO4 to high-Z so GT911 owns the interrupt line.
    drop(touch_int);
    delay.delay_millis(100);

    // ----- GT911 probe & init ----------------------------------------------
    let touch_init = gt911::Gt911Blocking::default();
    touch_init.init(&mut i2c).map_err(|_| Error::Gt911)?;

    // ----- DPI peripheral --------------------------------------------------
    let lcd_cam = LcdCam::new(r.lcd_cam);
    let dpi_config = DpiConfig::default()
        .with_frequency(Rate::from_mhz(consts::dpi::PCLK_MHZ))
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        })
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: consts::dpi::H_ACTIVE,
            horizontal_total_width: consts::dpi::H_TOTAL,
            horizontal_blank_front_porch: consts::dpi::H_BLANK_FRONT_PORCH,
            vertical_active_height: consts::dpi::V_ACTIVE,
            vertical_total_height: consts::dpi::V_TOTAL,
            vertical_blank_front_porch: consts::dpi::V_BLANK_FRONT_PORCH,
            hsync_width: consts::dpi::HSYNC_WIDTH,
            vsync_width: consts::dpi::VSYNC_WIDTH,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low);

    let dpi = Dpi::new(lcd_cam.lcd, r.dma_ch0, dpi_config)
        .map_err(|_| Error::Dpi)?
        .with_vsync(r.vsync)
        .with_hsync(r.hsync)
        .with_de(r.de)
        .with_pclk(r.pclk)
        // Blue (B3..B7)
        .with_data0(r.data.b3)
        .with_data1(r.data.b4)
        .with_data2(r.data.b5)
        .with_data3(r.data.b6)
        .with_data4(r.data.b7)
        // Green (G2..G7)
        .with_data5(r.data.g2)
        .with_data6(r.data.g3)
        .with_data7(r.data.g4)
        .with_data8(r.data.g5)
        .with_data9(r.data.g6)
        .with_data10(r.data.g7)
        // Red (R3..R7)
        .with_data11(r.data.r3)
        .with_data12(r.data.r4)
        .with_data13(r.data.r5)
        .with_data14(r.data.r6)
        .with_data15(r.data.r7);

    // ----- PSRAM framebuffer + DRAM bounce-buffer ring ---------------------
    let (psram_ptr, psram_size) = esp_hal::psram::psram_raw_parts(&r.psram);
    let (initial_front_ptr, framebuffer) =
        unsafe { framebuffer::Framebuffer::split_from_psram(psram_ptr, psram_size) }
            .ok_or(Error::PsramTooSmall)?;

    let bounce = unsafe { bounce_buffer::BounceRing::new(initial_front_ptr as *const u8) };

    // ----- Start continuous DPI DMA transfer + EOF ISR ---------------------
    let transfer = dpi.send(true, bounce).map_err(|_| Error::DpiStart)?;
    core::mem::forget(transfer);
    unsafe { bounce_buffer::enable_eof_interrupt() };

    // ----- Backlight ON (EXIO2 = 1) ----------------------------------------
    ch.write_all(&mut i2c, consts::ch422g_pins::MASK_ACTIVE)
        .map_err(|_| Error::Ch422g)?;

    // ----- Switch I²C to async + shared CH422G -----------------------------
    static I2C_BUS: StaticCell<SharedI2c> = StaticCell::new();
    let bus: &'static SharedI2c = I2C_BUS.init(Mutex::new(i2c.into_async()));

    static CH422G_CTRL: StaticCell<SharedCh422g> = StaticCell::new();
    let ch_ctrl: &'static SharedCh422g = CH422G_CTRL.init(Mutex::new(
        ch422g::Ch422gAsync::from_shadow(BoardI2cDevice::new(bus), ch.shadow()),
    ));

    let touch_driver = gt911::Gt911::default();
    let touch_bus = BoardI2cDevice::new(bus);
    let touch = TouchPoller::new(touch_driver, touch_bus);

    let backlight = Backlight::new(ch_ctrl);
    let expander = Expander::new(ch_ctrl);

    Ok(Board {
        framebuffer,
        i2c: bus,
        touch,
        backlight,
        expander,
    })
}
