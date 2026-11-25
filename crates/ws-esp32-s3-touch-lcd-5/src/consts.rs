//! Hardware constants for the Waveshare **ESP32-S3-Touch-LCD-5** (Rev 1.2)
//! and optional **5B (1024×600)** variant.
//!
//! Pin assignments match the Waveshare ESP32-S3-Touch-LCD-5 schematic.

/// I²C bus 0. Shared by the CH422G I/O expander and the GT911 capacitive touch controller.
pub mod i2c {
    /// SDA pin on the ESP32-S3.
    pub const SDA_GPIO: u8 = 8;
    /// SCL pin on the ESP32-S3.
    pub const SCL_GPIO: u8 = 9;
    /// Default bus frequency (400 kHz).
    pub const FREQUENCY_HZ: u32 = 400_000;

    /// CH422G system parameter / mode control (`WR_SET`, write-only).
    pub const CH422G_WR_SET_ADDR: u8 = ch422g::ADDR_WR_SET;
    /// CH422G output latch (`WR_IO`, write-only).
    pub const CH422G_WR_IO_ADDR: u8 = ch422g::ADDR_WR_IO;
    /// GT911 capacitive touch controller (latched via `TP_IRQ` low on `TP_RST` rising edge).
    pub const GT911_ADDR: u8 = 0x5D;
}

/// CH422G I/O expander pin mapping (`EXIO0..EXIO7`).
pub mod ch422g_pins {
    /// Touch reset (`TP_RST`, active low).
    pub const TP_RST_BIT: u8 = 1;
    /// LCD backlight enable (`DISP`, active high).
    pub const DISP_BIT: u8 = 2;
    /// LCD reset (`LCD_RST`, active low).
    pub const LCD_RST_BIT: u8 = 3;
    /// microSD card chip-select (`SD_CS`, active low — keep high when idle).
    pub const SD_CS_BIT: u8 = 4;
    /// USB/CAN bus mux select (`USB_SEL`, `0` = USB, `1` = CAN).
    pub const USB_SEL_BIT: u8 = 5;

    /// Initial shadow state with `TP_RST=0`, `DISP=0`, `LCD_RST=0`, `SD_CS=1`, `USB_SEL=0`.
    pub const MASK_RESET_LOW: u8 = 1 << SD_CS_BIT;
    /// Idle state with `TP_RST=1`, `LCD_RST=1`, `SD_CS=1`, `DISP=0` (backlight off).
    pub const MASK_IDLE_DARK: u8 = (1 << TP_RST_BIT) | (1 << LCD_RST_BIT) | (1 << SD_CS_BIT);
    /// Active state with `TP_RST=1`, `LCD_RST=1`, `SD_CS=1`, `DISP=1` (backlight on).
    pub const MASK_ACTIVE: u8 = MASK_IDLE_DARK | (1 << DISP_BIT);
}

/// Capacitive touch (`GT911`) interrupt pin.
pub mod touch {
    /// `TP_IRQ` pin on the ESP32-S3 (also used to strap GT911 address `0x5D` at reset).
    pub const IRQ_GPIO: u8 = 4;
}

/// DPI / RGB565 panel timing constants.
#[cfg(not(feature = "panel-1024x600"))]
pub mod dpi {
    /// Pixel clock frequency in MHz.
    pub const PCLK_MHZ: u32 = 12;
    /// Horizontal active width (pixels).
    pub const H_ACTIVE: usize = 800;
    /// HSYNC pulse width (PCLKs).
    pub const HSYNC_WIDTH: usize = 4;
    /// Horizontal back porch (PCLKs).
    pub const HBP: usize = 8;
    /// Horizontal front porch (PCLKs).
    pub const HFP: usize = 8;
    /// `esp-hal` `horizontal_blank_front_porch` (`HSYNC_WIDTH + HBP`).
    pub const H_BLANK_FRONT_PORCH: usize = HSYNC_WIDTH + HBP;
    /// Total horizontal width (`HSYNC_WIDTH + HBP + H_ACTIVE + HFP`).
    pub const H_TOTAL: usize = HSYNC_WIDTH + HBP + H_ACTIVE + HFP;

    /// Vertical active height (lines).
    pub const V_ACTIVE: usize = 480;
    /// VSYNC pulse width (lines).
    pub const VSYNC_WIDTH: usize = 4;
    /// Vertical back porch (lines).
    pub const VBP: usize = 8;
    /// Vertical front porch (lines).
    pub const VFP: usize = 8;
    /// `esp-hal` `vertical_blank_front_porch` (`VSYNC_WIDTH + VBP`).
    pub const V_BLANK_FRONT_PORCH: usize = VSYNC_WIDTH + VBP;
    /// Total vertical height (`VSYNC_WIDTH + VBP + V_ACTIVE + VFP`).
    pub const V_TOTAL: usize = VSYNC_WIDTH + VBP + V_ACTIVE + VFP;
}

/// DPI / RGB565 panel timing constants for the 5B (`1024×600`) variant.
#[cfg(feature = "panel-1024x600")]
pub mod dpi {
    /// Pixel clock frequency in MHz.
    pub const PCLK_MHZ: u32 = 21;
    /// Horizontal active width (pixels).
    pub const H_ACTIVE: usize = 1024;
    /// HSYNC pulse width (PCLKs).
    pub const HSYNC_WIDTH: usize = 20;
    /// Horizontal back porch (PCLKs).
    pub const HBP: usize = 140;
    /// Horizontal front porch (PCLKs).
    pub const HFP: usize = 160;
    /// `esp-hal` `horizontal_blank_front_porch` (`HSYNC_WIDTH + HBP`).
    pub const H_BLANK_FRONT_PORCH: usize = HSYNC_WIDTH + HBP;
    /// Total horizontal width (`HSYNC_WIDTH + HBP + H_ACTIVE + HFP`).
    pub const H_TOTAL: usize = HSYNC_WIDTH + HBP + H_ACTIVE + HFP;

    /// Vertical active height (lines).
    pub const V_ACTIVE: usize = 600;
    /// VSYNC pulse width (lines).
    pub const VSYNC_WIDTH: usize = 3;
    /// Vertical back porch (lines).
    pub const VBP: usize = 20;
    /// Vertical front porch (lines).
    pub const VFP: usize = 12;
    /// `esp-hal` `vertical_blank_front_porch` (`VSYNC_WIDTH + VBP`).
    pub const V_BLANK_FRONT_PORCH: usize = VSYNC_WIDTH + VBP;
    /// Total vertical height (`VSYNC_WIDTH + VBP + V_ACTIVE + VFP`).
    pub const V_TOTAL: usize = VSYNC_WIDTH + VBP + V_ACTIVE + VFP;
}

/// USB-Serial-JTAG / USB-OTG D+ / D- pins (hardware-fixed on ESP32-S3).
pub mod usb {
    /// D-
    pub const DM_GPIO: u8 = 19;
    /// D+
    pub const DP_GPIO: u8 = 20;
}
