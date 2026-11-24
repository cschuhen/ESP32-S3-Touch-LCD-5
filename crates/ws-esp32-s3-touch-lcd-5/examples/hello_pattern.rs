//! Calibration test pattern for `ws-esp32-s3-touch-lcd-5`.
//!
//! Draws a 1-pixel white border around the active area, 40×40 coloured corner
//! markers (TL=Red, TR=Green, BL=Blue, BR=Yellow), and vertical RGB gradient
//! bars, then logs `BounceRing::isr_fires()` every second.

#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_5 as bsp;

esp_bootloader_esp_idf::esp_app_desc!();

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;
const RED: u16 = 0xF800;
const GREEN: u16 = 0x07E0;
const BLUE: u16 = 0x001F;
const YELLOW: u16 = 0xFFE0;

#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(
        esp_hal::Config::default()
            .with_cpu_clock(esp_hal::clock::CpuClock::max())
            .with_psram(esp_hal::psram::PsramConfig {
                size: esp_hal::psram::PsramSize::Size(8 * 1024 * 1024),
                ..Default::default()
            }),
    );

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    esp_println::logger::init_logger_from_env();

    let board = bsp::init(bsp::take_resources!(peripherals)).expect("board init");
    let fb = &board.framebuffer;

    let w = bsp::WIDTH;
    let h = bsp::HEIGHT;

    fb.fill(BLACK);

    // 1-px white border at exact active edges
    fb.draw_row_solid(0, 0..w, WHITE);
    fb.draw_row_solid(h - 1, 0..w, WHITE);
    fb.draw_column(0, 0..h, WHITE);
    fb.draw_column(w - 1, 0..h, WHITE);

    // 40x40 corner squares
    for y in 2..42 {
        fb.draw_row_solid(y, 2..42, RED);
        fb.draw_row_solid(y, (w - 42)..(w - 2), GREEN);
    }
    for y in (h - 42)..(h - 2) {
        fb.draw_row_solid(y, 2..42, BLUE);
        fb.draw_row_solid(y, (w - 42)..(w - 2), YELLOW);
    }

    // Horizontal RGB gradient bars in the centre
    let mut row_buf = [0u16; bsp::WIDTH];
    for x in 60..(w - 60) {
        let level = ((x - 60) * 31) / (w - 120);
        row_buf[x] = (level as u16) << 11; // Red ramp
    }
    for y in (h / 2 - 60)..(h / 2 - 20) {
        fb.write_row(y, 60..(w - 60), &row_buf[60..(w - 60)]);
    }
    for x in 60..(w - 60) {
        let level = ((x - 60) * 63) / (w - 120);
        row_buf[x] = (level as u16) << 5; // Green ramp
    }
    for y in (h / 2 - 15)..(h / 2 + 25) {
        fb.write_row(y, 60..(w - 60), &row_buf[60..(w - 60)]);
    }
    for x in 60..(w - 60) {
        let level = ((x - 60) * 31) / (w - 120);
        row_buf[x] = level as u16; // Blue ramp
    }
    for y in (h / 2 + 30)..(h / 2 + 70) {
        fb.write_row(y, 60..(w - 60), &row_buf[60..(w - 60)]);
    }

    fb.flip();
    log::info!("test pattern drawn ({}x{})", w, h);

    let mut prev = bsp::BounceRing::isr_fires();
    loop {
        Timer::after(Duration::from_secs(1)).await;
        let now = bsp::BounceRing::isr_fires();
        log::info!("isr_fires/s = {} (total = {})", now.wrapping_sub(prev), now);
        prev = now;
    }
}
