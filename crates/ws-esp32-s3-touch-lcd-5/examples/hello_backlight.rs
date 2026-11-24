//! Solid colour & backlight bring-up demo for `ws-esp32-s3-touch-lcd-5`.
//!
//! Blinks the CH422G `DISP` backlight once, then cycles solid Red, Green,
//! Blue, White, and Black every 1.5 s to verify RGB565 bit ordering and
//! Octal PSRAM writeback.

#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_5 as bsp;

esp_bootloader_esp_idf::esp_app_desc!();

const COLOURS: &[(&str, u16)] = &[
    ("RED", 0xF800),
    ("GREEN", 0x07E0),
    ("BLUE", 0x001F),
    ("WHITE", 0xFFFF),
    ("BLACK", 0x0000),
];

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
    let (half_bytes, desc_per_half, halves_per_frame) = bsp::BounceRing::dimensions();
    log::info!(
        "board up ({}x{}): bounce half={} B, desc/half={}, halves/frame={}",
        bsp::WIDTH,
        bsp::HEIGHT,
        half_bytes,
        desc_per_half,
        halves_per_frame
    );

    // Blink backlight once to confirm CH422G EXIO2 control.
    log::info!("backlight off");
    let _ = board.backlight.off().await;
    Timer::after(Duration::from_millis(400)).await;
    log::info!("backlight on");
    let _ = board.backlight.on().await;

    loop {
        for &(name, rgb565) in COLOURS {
            log::info!("fill {} (0x{:04X})", name, rgb565);
            board.framebuffer.fill(rgb565);
            board.framebuffer.flip();
            Timer::after(Duration::from_millis(1500)).await;
        }
    }
}
