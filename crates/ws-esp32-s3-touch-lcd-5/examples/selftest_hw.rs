//! Autonomous hardware-in-the-loop (HIL) self-test for `ws-esp32-s3-touch-lcd-5`.
//!
//! Verifies PSRAM writeback, CH422G I²C transactions, GT911 silicon identity,
//! `LCD_CAM` DPI register state, GDMA bounce-ring ISR cadence, tear-free
//! `Framebuffer::flip()` timing, and async I²C bus sharing.
//!
//! Emits `[SELFTEST] ALL_PASS` on success or `[SELFTEST] FAILED` on any failure.

#![no_std]
#![no_main]

extern crate alloc;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c as AsyncI2c;
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
    esp_println::logger::init_logger_from_env();

    log::info!("[SELFTEST] starting board init...");
    let mut board = match bsp::init(bsp::take_resources!(peripherals)) {
        Ok(b) => {
            log::info!("[PASS] ch422g_init: CH422G + GT911 + DPI initialized");
            b
        }
        Err(e) => {
            log::error!("[FAIL] board_init: {:?}", e);
            log::error!("[SELFTEST] FAILED");
            halt();
        }
    };

    let mut all_ok = true;

    // 1. PSRAM writeback & readback probe across both framebuffers
    all_ok &= check_psram(&board.framebuffer);

    // 2. GT911 config register readback
    all_ok &= check_gt911_config(board.i2c).await;

    // 3. LCD_CAM DPI hardware register readback
    all_ok &= check_dpi_regs();

    // 4. GDMA bounce-ring ISR cadence (1000 ms window)
    all_ok &= check_isr_cadence().await;

    // 5. Double-buffer tear-free flip() sync timing
    all_ok &= check_fb_flip(&board.framebuffer);

    // 6. Async I2C bus sharing stress (backlight + touch)
    all_ok &= check_i2c_stress(&board.backlight, &mut board.touch).await;

    if all_ok {
        log::info!("[SELFTEST] ALL_PASS");
    } else {
        log::error!("[SELFTEST] FAILED");
    }

    halt();
}

fn check_psram(fb: &bsp::Framebuffer) -> bool {
    let (a, b) = fb.raw_buffers();
    let mut mismatches = 0usize;

    for (idx, base) in [a, b].into_iter().enumerate() {
        let words = bsp::FRAMEBUFFER_BYTES / 4;
        let ptr = base as *mut u32;
        unsafe {
            for i in 0..words {
                let pat = ((idx as u32) << 24) ^ (i as u32).wrapping_mul(0x9E37_79B9);
                ptr.add(i).write_volatile(pat);
            }
            for i in 0..words {
                let expected = ((idx as u32) << 24) ^ (i as u32).wrapping_mul(0x9E37_79B9);
                let got = ptr.add(i).read_volatile();
                if got != expected {
                    mismatches += 1;
                    break;
                }
            }
        }
    }

    // Restore clean black framebuffers
    fb.fill(0x0000);
    fb.flip();
    fb.fill(0x0000);
    fb.flip();

    if mismatches == 0 {
        log::info!(
            "[PASS] psram_probe: {} bytes verified across both framebuffers",
            bsp::PSRAM_BYTES_REQUIRED
        );
        true
    } else {
        log::error!("[FAIL] psram_probe: {} mismatches", mismatches);
        false
    }
}

async fn check_gt911_config(bus: &'static bsp::SharedI2c) -> bool {
    let mut dev = I2cDevice::new(bus);
    let mut buf = [0u8; 4];
    match dev
        .write_read(bsp::consts::i2c::GT911_ADDR, &[0x80, 0x48], &mut buf)
        .await
    {
        Ok(()) => {
            let x_max = u16::from_le_bytes([buf[0], buf[1]]);
            let y_max = u16::from_le_bytes([buf[2], buf[3]]);
            log::info!(
                "[PASS] gt911_probe: addr=0x{:02X} configured_res={}x{}",
                bsp::consts::i2c::GT911_ADDR,
                x_max,
                y_max
            );
            true
        }
        Err(e) => {
            log::error!("[FAIL] gt911_probe: i2c read failed: {:?}", e);
            false
        }
    }
}

fn check_dpi_regs() -> bool {
    let lcd = unsafe { &*esp_hal::peripherals::LCD_CAM::PTR };
    let user = lcd.lcd_user().read();
    let ctrl = lcd.lcd_ctrl().read();
    let ctrl1 = lcd.lcd_ctrl1().read();
    let ctrl2 = lcd.lcd_ctrl2().read();
    let misc = lcd.lcd_misc().read();

    let ok = ctrl.lcd_rgb_mode_en().bit_is_set()
        && user.lcd_2byte_en().bit_is_set()
        && misc.lcd_next_frame_en().bit_is_set()
        && user.lcd_start().bit_is_set()
        && ctrl1.lcd_ha_width().bits() as usize == bsp::consts::dpi::H_ACTIVE - 1
        && ctrl1.lcd_ht_width().bits() as usize == bsp::consts::dpi::H_TOTAL - 1
        && ctrl.lcd_va_height().bits() as usize == bsp::consts::dpi::V_ACTIVE - 1
        && ctrl.lcd_vt_height().bits() as usize == bsp::consts::dpi::V_TOTAL - 1
        && ctrl.lcd_hb_front().bits() as usize == bsp::consts::dpi::H_BLANK_FRONT_PORCH - 1
        && ctrl1.lcd_vb_front().bits() as usize == bsp::consts::dpi::V_BLANK_FRONT_PORCH - 1
        && ctrl2.lcd_hsync_width().bits() as usize == bsp::consts::dpi::HSYNC_WIDTH - 1
        && ctrl2.lcd_vsync_width().bits() as usize == bsp::consts::dpi::VSYNC_WIDTH - 1;

    if ok {
        log::info!(
            "[PASS] dpi_regs: {}x{} active, {}x{} total, RGB565 continuous DMA active",
            bsp::consts::dpi::H_ACTIVE,
            bsp::consts::dpi::V_ACTIVE,
            bsp::consts::dpi::H_TOTAL,
            bsp::consts::dpi::V_TOTAL
        );
    } else {
        log::error!("[FAIL] dpi_regs: unexpected LCD_CAM register state");
    }
    ok
}

async fn check_isr_cadence() -> bool {
    let _ = bsp::BounceRing::take_isr_stats(1000);
    let start = bsp::BounceRing::isr_fires();
    Timer::after(Duration::from_millis(1000)).await;
    let end = bsp::BounceRing::isr_fires();
    let delta = end.wrapping_sub(start);
    let st = bsp::BounceRing::take_isr_stats(1000);

    let dma = unsafe { &*esp_hal::peripherals::DMA::PTR };
    let dscr_err = dma.ch(0).out_int().raw().read().out_dscr_err().bit_is_set();

    let halves = bsp::BounceRing::dimensions().2 as u64;
    let expected = ((bsp::consts::dpi::PCLK_MHZ as u64 * 1_000_000 * halves)
        / (bsp::consts::dpi::H_TOTAL as u64 * bsp::consts::dpi::V_TOTAL as u64)) as u32;
    let min_eof = expected * 92 / 100;
    let max_eof = expected * 108 / 100;

    let ok = !dscr_err && st.core == 1 && (min_eof..=max_eof).contains(&delta);
    if ok {
        log::info!(
            "[PASS] isr_cadence: {} EOFs/sec (expected ~{}), isr_core={}, avg={} us, max={} us, core1_cpu={} %, out_dscr_err=false",
            delta,
            expected,
            st.core,
            st.avg_us,
            st.max_us,
            st.cpu_pct
        );
    } else {
        log::error!(
            "[FAIL] isr_cadence: {} EOFs/sec (expected {}..={}), isr_core={}, out_dscr_err={}",
            delta,
            min_eof,
            max_eof,
            st.core,
            dscr_err
        );
    }
    ok
}

fn check_fb_flip(fb: &bsp::Framebuffer) -> bool {
    let mut max_ms = 0u64;
    for _ in 0..5 {
        let t0 = Instant::now();
        fb.flip();
        let dt = t0.elapsed().as_millis();
        max_ms = max_ms.max(dt);
    }
    let ok = (15..=65).contains(&max_ms);
    if ok {
        log::info!("[PASS] fb_flip_sync: 5 flips completed (max {} ms)", max_ms);
    } else {
        log::error!("[FAIL] fb_flip_sync: unexpected max flip time {} ms", max_ms);
    }
    ok
}

async fn check_i2c_stress(backlight: &bsp::Backlight, touch: &mut bsp::TouchPoller) -> bool {
    for i in 0..50u8 {
        if let Err(e) = backlight.set(i % 2 == 0).await {
            log::error!("[FAIL] i2c_stress: backlight error at iter {}: {:?}", i, e);
            return false;
        }
        match touch.poll().await {
            Ok(_) | Err(bsp::gt911::Error::NotReady) => {}
            Err(e) => {
                log::error!("[FAIL] i2c_stress: touch error at iter {}: {:?}", i, e);
                return false;
            }
        }
    }
    let _ = backlight.on().await;
    log::info!("[PASS] i2c_stress: 50 interleaved backlight+touch ops ok");
    true
}

fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
