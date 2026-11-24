//! Backlight handle (`DISP` on CH422G `EXIO2`).
//!
//! Because `DISP` is driven through the shared I²C CH422G expander rather than
//! a direct ESP32-S3 GPIO, [`Backlight::on`], [`Backlight::off`], and
//! [`Backlight::set`] are `async` and go through [`SharedCh422g`].

use crate::consts::ch422g_pins;
use crate::expander::{ExpanderError, SharedCh422g};

/// LCD backlight handle (`EXIO2`, active high).
#[derive(Clone, Copy)]
pub struct Backlight {
    ch: &'static SharedCh422g,
}

impl Backlight {
    pub(crate) fn new(ch: &'static SharedCh422g) -> Self {
        Self { ch }
    }

    /// Turn the backlight on (`EXIO2` high).
    pub async fn on(&self) -> Result<(), ExpanderError> {
        self.set(true).await
    }

    /// Turn the backlight off (`EXIO2` low).
    pub async fn off(&self) -> Result<(), ExpanderError> {
        self.set(false).await
    }

    /// Set the backlight on (`true`) or off (`false`).
    pub async fn set(&self, on: bool) -> Result<(), ExpanderError> {
        let mut guard = self.ch.lock().await;
        guard
            .set_pin(ch422g_pins::DISP_BIT, on)
            .await
            .map_err(|_| ExpanderError::Bus)
    }
}
