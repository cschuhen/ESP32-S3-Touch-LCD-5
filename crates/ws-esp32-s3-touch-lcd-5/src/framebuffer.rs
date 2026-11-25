//! PSRAM-backed RGB565 framebuffer for the DPI panel — double-buffered.
//!
//! For 800×480 RGB565, one framebuffer is 768,000 bytes (two = 1,536,000 bytes)
//! in 8 MiB Octal PSRAM. For `panel-1024x600`, one framebuffer is 1,228,800 bytes.
//!
//! On ESP32-S3, Core 0 and Core 1 share a single L1 D-Cache, and GDMA only
//! reads DRAM bounce buffers (`accesses_psram: false`). Core 1's `eof_isr`
//! reads PSRAM through that same shared D-Cache, so `flip()`'s Release/Acquire
//! ordering guarantees visibility without calling `rom_Cache_WriteBack_Addr`
//! (which would stall/race with Core 1's concurrent PSRAM reads).
//!
//! All write methods pace themselves via [`crate::bounce_buffer::pace_psram_write`]
//! so Core 0 never saturates the shared D-Cache / 40 MHz PSRAM bus while
//! Core 1 is refilling a bounce half.

use core::ops::Range;
use core::sync::atomic::{AtomicBool, Ordering};

/// Logical width of the visible framebuffer, in pixels.
pub const WIDTH: usize = crate::consts::dpi::H_ACTIVE;
/// Logical height of the visible framebuffer, in pixels.
pub const HEIGHT: usize = crate::consts::dpi::V_ACTIVE;
/// Bytes in one framebuffer (RGB565 = 2 bytes per pixel).
pub const BYTES: usize = WIDTH * HEIGHT * 2;
/// Total PSRAM bytes required (two buffers, back-to-back).
pub const PSRAM_BYTES_REQUIRED: usize = BYTES * 2;

/// CPU-side write handle to the PSRAM framebuffer pair.
pub struct Framebuffer {
    a: *mut u8,
    b: *mut u8,
    back_is_a: AtomicBool,
}

unsafe impl Send for Framebuffer {}
unsafe impl Sync for Framebuffer {}

impl Framebuffer {
    /// Carve a pair of framebuffers off the start of the PSRAM region.
    ///
    /// # Safety
    ///
    /// `psram_ptr` must point to at least [`PSRAM_BYTES_REQUIRED`] bytes of valid,
    /// CPU-mapped PSRAM, and the caller must guarantee nothing else accesses that
    /// range for the lifetime of the returned values.
    pub unsafe fn split_from_psram(
        psram_ptr: *mut u8,
        psram_size: usize,
    ) -> Option<(*mut u8, Self)> {
        unsafe {
            if psram_size < PSRAM_BYTES_REQUIRED {
                return None;
            }
            let a = psram_ptr;
            let b = psram_ptr.add(BYTES);

            let fb = Self {
                a,
                b,
                back_is_a: AtomicBool::new(false),
            };

            fb.fill_buffer(a, 0x0000);
            fb.fill_buffer(b, 0x0000);

            Some((a, fb))
        }
    }

    fn back_ptr(&self) -> *mut u8 {
        if self.back_is_a.load(Ordering::Acquire) {
            self.a
        } else {
            self.b
        }
    }

    /// Return the raw `(a, b)` PSRAM pointers for low-level diagnostics.
    pub fn raw_buffers(&self) -> (*mut u8, *mut u8) {
        (self.a, self.b)
    }

    /// Write a span of `range.len()` RGB565 pixels into row `y` of the back buffer.
    pub fn write_row(&self, y: usize, range: Range<usize>, pixels: &[u16]) {
        if y >= HEIGHT {
            return;
        }
        let end = range.end.min(WIDTH);
        if range.start >= end {
            return;
        }
        let n = (end - range.start).min(pixels.len());
        crate::bounce_buffer::pace_psram_write(n * 2);
        unsafe {
            let dst = (self.back_ptr() as *mut u16).add(y * WIDTH + range.start);
            for i in 0..n {
                dst.add(i).write_volatile(pixels[i]);
            }
        }
    }

    /// Fill the back buffer with a single RGB565 colour.
    pub fn fill(&self, colour: u16) {
        self.fill_buffer(self.back_ptr(), colour);
    }

    /// Paint a horizontal solid-colour run on row `y`, columns `x_range`.
    pub fn draw_row_solid(&self, y: usize, x_range: Range<usize>, colour: u16) {
        if y >= HEIGHT {
            return;
        }
        let end = x_range.end.min(WIDTH);
        if x_range.start >= end {
            return;
        }
        let n = end - x_range.start;
        crate::bounce_buffer::pace_psram_write(n * 2);
        unsafe {
            let dst = (self.back_ptr() as *mut u16).add(y * WIDTH + x_range.start);
            for i in 0..n {
                dst.add(i).write_volatile(colour);
            }
        }
    }

    /// Paint a vertical solid-colour run at column `x`, rows `y_range`.
    pub fn draw_column(&self, x: usize, y_range: Range<usize>, colour: u16) {
        if x >= WIDTH {
            return;
        }
        let end = y_range.end.min(HEIGHT);
        if y_range.start >= end {
            return;
        }
        unsafe {
            let base = self.back_ptr() as *mut u16;
            let mut y = y_range.start;
            while y < end {
                let chunk_end = (y + 64).min(end);
                // Each vertical pixel dirties a distinct 32-byte D-cache line.
                crate::bounce_buffer::pace_psram_write((chunk_end - y) * 32);
                while y < chunk_end {
                    base.add(y * WIDTH + x).write_volatile(colour);
                    y += 1;
                }
            }
        }
    }

    fn fill_buffer(&self, base: *mut u8, colour: u16) {
        unsafe {
            let lo = (colour & 0xFF) as u8;
            let hi = (colour >> 8) as u8;
            const CHUNK_BYTES: usize = 2048;
            let mut offset = 0usize;
            while offset < BYTES {
                let len = (BYTES - offset).min(CHUNK_BYTES);
                crate::bounce_buffer::pace_psram_write(len);
                if lo == hi {
                    core::ptr::write_bytes(base.add(offset), lo, len);
                } else {
                    let dst = base.add(offset) as *mut u16;
                    let pixels = len / 2;
                    for i in 0..pixels {
                        core::ptr::write(dst.add(i), colour);
                    }
                }
                offset += len;
            }
        }
    }

    /// Tear-free swap of front and back buffers.
    pub fn flip(&self) {
        let was_back_a = self.back_is_a.load(Ordering::Acquire);
        let new_front = if was_back_a { self.a } else { self.b };

        while crate::bounce_buffer::is_flip_pending() {
            core::hint::spin_loop();
        }

        crate::bounce_buffer::request_flip(new_front);

        while crate::bounce_buffer::is_flip_pending() {
            core::hint::spin_loop();
        }

        self.back_is_a.store(!was_back_a, Ordering::Release);
    }

    /// Queue a swap and return immediately without waiting for the ISR to apply it.
    pub fn flip_no_sync(&self) {
        let was_back_a = self.back_is_a.load(Ordering::Acquire);
        let new_front = if was_back_a { self.a } else { self.b };
        crate::bounce_buffer::request_flip(new_front);
        self.back_is_a.store(!was_back_a, Ordering::Release);
    }
}

#[cfg(feature = "embedded-graphics")]
mod eg_support {
    use super::{Framebuffer, HEIGHT, WIDTH};
    use core::convert::Infallible;
    use embedded_graphics_core::{
        Pixel,
        draw_target::DrawTarget,
        geometry::{Dimensions, OriginDimensions, Size},
        pixelcolor::{
            Rgb565,
            raw::{RawData, RawU16},
        },
        primitives::Rectangle,
    };

    impl OriginDimensions for Framebuffer {
        fn size(&self) -> Size {
            Size::new(WIDTH as u32, HEIGHT as u32)
        }
    }

    impl DrawTarget for Framebuffer {
        type Color = Rgb565;
        type Error = Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            let base = self.back_ptr() as *mut u16;
            let mut pending_px = 0usize;
            for Pixel(coord, color) in pixels {
                if coord.x >= 0
                    && coord.y >= 0
                    && (coord.x as usize) < WIDTH
                    && (coord.y as usize) < HEIGHT
                {
                    if pending_px == 0 {
                        crate::bounce_buffer::pace_psram_write(32 * 32);
                    }
                    let x = coord.x as usize;
                    let y = coord.y as usize;
                    let raw = RawU16::from(color).into_inner();
                    unsafe {
                        base.add(y * WIDTH + x).write_volatile(raw);
                    }
                    pending_px += 1;
                    if pending_px >= 32 {
                        pending_px = 0;
                    }
                }
            }
            Ok(())
        }

        fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
            let clipped = area.intersection(&self.bounding_box());
            if clipped.is_zero_sized() {
                return Ok(());
            }
            let raw = RawU16::from(color).into_inner();
            let x0 = clipped.top_left.x as usize;
            let x1 = x0 + clipped.size.width as usize;
            let y0 = clipped.top_left.y as usize;
            let y1 = y0 + clipped.size.height as usize;
            for y in y0..y1 {
                self.draw_row_solid(y, x0..x1, raw);
            }
            Ok(())
        }

        fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
            self.fill(RawU16::from(color).into_inner());
            Ok(())
        }
    }
}
