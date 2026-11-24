# ws-esp32-s3-touch-lcd-5

Board-support package for the [**Waveshare ESP32-S3-Touch-LCD-5**][board] (Rev 1.2)
— a 5.0" 800×480 RGB565 IPS panel (or 1024×600 with the `panel-1024x600` feature
for the 5B variant) with a Goodix GT911 capacitive touch controller and WCH CH422G
I²C I/O expander on an ESP32-S3-WROOM-1-N16R8 (16 MB Flash, 8 MB Octal PSRAM).

[board]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-5

One call brings the board fully alive: CH422G reset sequencing, GT911 `0x5D`
address strapping, `LCD_CAM` DPI peripheral, double-buffered Octal PSRAM
framebuffer, and the DRAM GDMA bounce-buffer ring that avoids ESP32-S3
PSRAM→DPI drift artefacts.

## Quick start

```toml
[dependencies]
ws-esp32-s3-touch-lcd-5 = "0.1"
esp-hal                 = { version = "=1.0.0-rc.0", features = ["esp32s3", "unstable", "psram"] }
esp-hal-embassy         = { version = "0.9.1", features = ["esp32s3"] }
esp-backtrace           = { version = "0.18.1", features = ["esp32s3", "panic-handler", "println"] }
esp-println             = { version = "0.16.1", features = ["esp32s3", "log-04"] }
esp-bootloader-esp-idf  = { version = "0.3", features = ["esp32s3"] }
esp-alloc               = "0.9"
```

```rust,no_run
#![no_std]
#![no_main]

extern crate alloc;
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_5 as bsp;

esp_bootloader_esp_idf::esp_app_desc!();

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

    let mut board = bsp::init(bsp::take_resources!(peripherals))
        .expect("board init");

    // Fill back buffer with solid red and flip tear-free.
    board.framebuffer.fill(0xF800);
    board.framebuffer.flip();

    loop {
        match board.touch.poll().await {
            Ok(Some(point)) => log::info!("touch @ ({}, {})", point.x, point.y),
            Ok(None) | Err(bsp::gt911::Error::NotReady) => {}
            Err(e) => log::warn!("gt911: {:?}", e),
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
    }
}
```

You also need a `.cargo/config.toml` with `target = "xtensa-esp32s3-none-elf"`,
`build-std = ["alloc", "core"]`, and `ESP_HAL_CONFIG_PSRAM_MODE = "octal"` — see
[.cargo/config.toml](.cargo/config.toml) and the [examples](examples/) directory.

## What `init` does

1. Brings up `I2C0` at 400 kHz on `GPIO8` (`SDA`) / `GPIO9` (`SCL`).
2. Holds `TP_IRQ` (`GPIO4`) low, pre-loads CH422G `WR_IO` (`0x38`) with
   `SD_CS=1`, `TP_RST=0`, `LCD_RST=0`, `DISP=0`, enables push-pull outputs via
   `WR_SET` (`0x24 <- 0x01`), then releases `TP_RST` and `LCD_RST` and floats
   `GPIO4` so GT911 latches address `0x5D`.
3. Probes GT911 (`Gt911Blocking::default().init(&mut i2c)`).
4. Configures `LCD_CAM` DPI at 16 MHz PCLK (`Phase::ShiftHigh`, 800×480 active,
   820×500 total; or 21 MHz / 1024×600 with `panel-1024x600`).
5. Allocates two contiguous framebuffers in Octal PSRAM (1.5 MiB total at
   800×480), builds a closed DRAM GDMA bounce-buffer ring (`16` lines/half,
   `51.2 KiB` DRAM), starts continuous DMA, and enables `DMA_OUT_CH0` `OUT_EOF`.
6. Turns on the backlight (`DISP` = `EXIO2` high) and converts `I2C0` to async
   (`SharedI2c` + `SharedCh422g`).

## What you get back (`Board`)

- `framebuffer: Framebuffer` — write RGB565 pixels (`write_row`, `fill`,
  `draw_row_solid`, `draw_column`, or `embedded-graphics` `DrawTarget` with the
  `embedded-graphics` feature) and call `flip()` for tear-free double-buffering.
- `i2c: &'static SharedI2c` — async shared I²C0 bus (`embassy_sync::Mutex`).
- `touch: TouchPoller` — async GT911 poller (`board.touch.poll().await`).
- `backlight: Backlight` — async `on()` / `off()` / `set(bool)` via CH422G `EXIO2`.
- `expander: Expander` — async control of `SD_CS` (`EXIO4`) and `USB_SEL` (`EXIO5`).

## Caveats

- **Run the CPU at 240 MHz (`CpuClock::max()`).** `esp-hal 1.0.0-rc.0` defaults
  to 80 MHz, which is too slow for the bounce-buffer ISR at 16 MHz PCLK.
- **Octal PSRAM mode is mandatory.** Set `ESP_HAL_CONFIG_PSRAM_MODE = "octal"`
  in `.cargo/config.toml`.
- **Call `init` at most once per boot.** The bounce-buffer ring and EOF ISR use
  module-level statics, and the DPI DMA transfer is intentionally leaked.
- **Pinned to `esp-hal = "=1.0.0-rc.0"`** for `esp-hal-embassy 0.9.1`
  compatibility.

## Re-exports

```rust,ignore
pub use ch422g;
pub use gt911;
```

## License

MIT — see the workspace [LICENSE-MIT](../../LICENSE-MIT).
