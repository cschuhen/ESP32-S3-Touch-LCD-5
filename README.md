# ESP32-S3-Touch-LCD-5

Rust drivers and board-support package for the **Waveshare ESP32-S3-Touch-LCD-5** (Rev 1.2, 800×480 RGB565 IPS panel, optional `panel-1024x600` feature for the 5B variant), with a Goodix GT911 capacitive touch controller and WCH CH422G I²C I/O expander on an ESP32-S3-WROOM-1-N16R8.

## Crates

| Crate | Purpose | Dependencies |
|---|---|---|
| [`ch422g`](crates/ch422g) | Minimal `no_std` driver for the WCH CH422G I²C I/O expander (`0x24` / `0x38` single-byte protocol, shadow-register tracking, optional `async`). | `embedded-hal 1.0`, `embedded-hal-async 1.0` |
| [`ws-esp32-s3-touch-lcd-5`](crates/ws-esp32-s3-touch-lcd-5) | Full ESP32-S3 BSP: CH422G reset & backlight (`DISP`), GT911 touch (`0x5D`), `LCD_CAM` DPI RGB565, double-buffered Octal PSRAM framebuffer, and DRAM GDMA bounce-buffer ring. | `esp-hal 1.0.0-rc.0`, `ch422g`, `gt911` |

## Examples & Hardware Verification

From `crates/ws-esp32-s3-touch-lcd-5`:

```sh
# 1. Autonomous hardware-in-the-loop self-test (PSRAM, CH422G, GT911, DPI regs, ISR, flip)
cargo +esp run --release --example selftest_hw --features _example-runtime

# 2. Solid colour & backlight test
cargo +esp run --release --example hello_backlight --features _example-runtime

# 3. 1-px border & RGB gradient calibration pattern
cargo +esp run --release --example hello_pattern --features _example-runtime

# 4. Interactive GT911 touch crosshairs
cargo +esp run --release --example hello_touch --features _example-runtime
```

## License

MIT — see [LICENSE-MIT](LICENSE-MIT).
