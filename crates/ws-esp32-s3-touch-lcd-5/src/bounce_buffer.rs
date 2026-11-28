//! Bounce-buffer mode for the DPI panel.
//!
//! Two small DRAM bounce buffers (`BOUNCE_A`, `BOUNCE_B`) sit between the PSRAM
//! master framebuffer and the `LCD_CAM` GDMA channel so GDMA never reads PSRAM
//! directly (which causes diagonal refresh-drift stripes on ESP32-S3).
//!
//! The entire EOF ISR and copy loop are placed in IRAM (`#[esp_hal::ram]`) so
//! SPI Flash cache evictions never stall a bounce-buffer refill.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, AtomicU32, Ordering};

use esp_hal::dma::{
    BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation, TransferDirection,
};
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::Interrupt;
use esp_hal::system::{Cpu, CpuControl, Stack};

use crate::framebuffer::{self, WIDTH};

/// Scanlines per bounce-buffer half.
/// - 800×480: 16 lines -> 30 halves (even), 25,600 B per half (51.2 KiB total DRAM).
/// - 1024×600: 10 lines -> 60 halves (even), 20,480 B per half (40.0 KiB total DRAM).
#[cfg(not(feature = "panel-1024x600"))]
pub const N_LINES_PER_HALF: usize = 16;
#[cfg(feature = "panel-1024x600")]
pub const N_LINES_PER_HALF: usize = 10;

/// Bytes per bounce-buffer half.
pub const HALF_BYTES: usize = N_LINES_PER_HALF * WIDTH * 2;

/// Max DMA chunk size (must be ≤ 4095 on ESP32-S3; 4032 is 64-byte-aligned).
const CHUNK: usize = 4032;

/// Descriptors per half.
const DESC_PER_HALF: usize = HALF_BYTES.div_ceil(CHUNK);
/// Halves per frame.
pub const HALVES_PER_FRAME: usize = framebuffer::BYTES / HALF_BYTES;

/// Coarse startup offset in halves (0 by default; calibrate on hardware if needed).
const STARTUP_OFFSET_HALVES: usize = 0;
/// Fine startup offset in scanlines (0 by default; calibrated with `hello_pattern`).
const STARTUP_OFFSET_LINES: usize = 0;
const STARTUP_OFFSET_BYTES: usize = STARTUP_OFFSET_LINES * WIDTH * 2;

/// Total descriptors in the frame-sized ring.
const N_DESC: usize = DESC_PER_HALF * HALVES_PER_FRAME;

// Compile-time invariants for alternating A/B bounce-buffer wrapping & 16-byte IRAM copy.
const _: () = assert!(framebuffer::BYTES % HALF_BYTES == 0);
const _: () = assert!(HALVES_PER_FRAME % 2 == 0);
const _: () = assert!(HALF_BYTES % 16 == 0);
const _: () = assert!(STARTUP_OFFSET_BYTES % 16 == 0);

#[repr(align(64))]
struct BounceHalf([u8; HALF_BYTES]);

static mut BOUNCE_A: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut BOUNCE_B: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut DESCRIPTORS: [DmaDescriptor; N_DESC] = [DmaDescriptor::EMPTY; N_DESC];

static PSRAM_FB_PTR: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
static PENDING_SWAP: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
static EOF_COUNT: AtomicU32 = AtomicU32::new(0);
static STAT_FIRES: AtomicU32 = AtomicU32::new(0);
static STAT_CYCLES_SUM: AtomicU32 = AtomicU32::new(0);
static STAT_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
static STAT_ISR_CORE: AtomicU8 = AtomicU8::new(0);
static mut CORE1_STACK: Stack<4096> = Stack::new();
static CORE1_READY: AtomicBool = AtomicBool::new(false);
static ISR_ACTIVE: AtomicBool = AtomicBool::new(false);
static PACE_LAST_EOF: AtomicU32 = AtomicU32::new(0);
static PACE_BYTES: AtomicU32 = AtomicU32::new(0);

/// Max Core 0 PSRAM write budget per EOF interval (~64 dirty 64-byte cache lines),
/// keeping Core 1's shared-cache refill comfortably below the 1,093 us deadline.
const MAX_CORE0_BYTES_PER_EOF: u32 = 4096;

/// ISR execution telemetry snapshot returned by [`BounceRing::take_isr_stats`].
#[derive(Debug, Clone, Copy, Default)]
pub struct IsrStats {
    /// CPU core ID (`0` = PRO/Core 0, `1` = APP/Core 1) that serviced the EOF ISR.
    pub core: u8,
    /// Number of EOF interrupts recorded since the previous call.
    pub fires: u32,
    /// Mean ISR execution time in microseconds (assuming 240 MHz CPU clock).
    pub avg_us: u32,
    /// Worst-case single ISR execution time in microseconds (assuming 240 MHz CPU clock).
    pub max_us: u32,
    /// Estimated single-core CPU utilization percentage (`0..=100`, assuming 240 MHz over `window_ms`).
    pub cpu_pct: u32,
}

/// Custom `DmaTxBuffer` wrapping the closed frame-sized DRAM descriptor ring.
pub struct BounceRing {
    head: *mut DmaDescriptor,
}

impl BounceRing {
    /// Build the descriptor ring and prime both bounce halves from `psram_fb_ptr`.
    ///
    /// # Safety
    ///
    /// - `psram_fb_ptr` must remain valid for [`framebuffer::BYTES`] bytes.
    /// - Must be called at most once per boot.
    pub unsafe fn new(psram_fb_ptr: *const u8) -> Self {
        unsafe {
            let descs_base: *mut DmaDescriptor = core::ptr::addr_of_mut!(DESCRIPTORS) as *mut _;
            let buf_a: *mut u8 = core::ptr::addr_of_mut!(BOUNCE_A) as *mut u8;
            let buf_b: *mut u8 = core::ptr::addr_of_mut!(BOUNCE_B) as *mut u8;

            for half in 0..HALVES_PER_FRAME {
                let buf_base = if half % 2 == 0 { buf_a } else { buf_b };
                let desc_offset = half * DESC_PER_HALF;

                for i in 0..DESC_PER_HALF {
                    let byte_start = i * CHUNK;
                    let byte_end = ((i + 1) * CHUNK).min(HALF_BYTES);
                    let chunk_len = byte_end - byte_start;

                    let d = descs_base.add(desc_offset + i);
                    (*d).set_size(chunk_len);
                    (*d).set_length(chunk_len);
                    (*d).buffer = buf_base.add(byte_start);
                    (*d).set_owner(Owner::Dma);

                    let is_last_in_half = i == DESC_PER_HALF - 1;
                    (*d).set_suc_eof(is_last_in_half);

                    let next_global = if is_last_in_half && half == HALVES_PER_FRAME - 1 {
                        0
                    } else {
                        desc_offset + i + 1
                    };
                    (*d).next = descs_base.add(next_global);
                }
            }

            // Enable ESP32-S3 hardware D-Cache sequential autoload across the double PSRAM
            // framebuffer so Core 1's sequential bounce copies prefetch ahead in hardware.
            let extmem = &*esp_hal::peripherals::EXTMEM::PTR;
            extmem
                .dcache_autoload_ctrl()
                .modify(|_, w| w.dcache_autoload_ena().clear_bit());
            while !extmem
                .dcache_autoload_ctrl()
                .read()
                .dcache_autoload_done()
                .bit_is_set()
            {
                core::hint::spin_loop();
            }
            extmem
                .dcache_autoload_sct0_addr()
                .write(|w| w.dcache_autoload_sct0_addr().bits(psram_fb_ptr as u32));
            extmem
                .dcache_autoload_sct0_size()
                .write(|w| w.dcache_autoload_sct0_size().bits(framebuffer::PSRAM_BYTES_REQUIRED as u32));
            extmem.dcache_autoload_ctrl().modify(|_, w| {
                w.dcache_autoload_sct0_ena()
                    .set_bit()
                    .dcache_autoload_sct1_ena()
                    .clear_bit()
                    .dcache_autoload_order()
                    .clear_bit()
                    .dcache_autoload_rqst()
                    .bits(2)
                    .dcache_autoload_size()
                    .bits(3)
                    .dcache_autoload_ena()
                    .set_bit()
            });

            let init_a_chunk = STARTUP_OFFSET_HALVES % HALVES_PER_FRAME;
            let init_b_chunk = (STARTUP_OFFSET_HALVES + 1) % HALVES_PER_FRAME;
            psram_copy_wrapping(psram_fb_ptr, init_a_chunk, buf_a);
            psram_copy_wrapping(psram_fb_ptr, init_b_chunk, buf_b);

            PSRAM_FB_PTR.store(psram_fb_ptr as *mut u8, Ordering::Release);
            EOF_COUNT.store(0, Ordering::Relaxed);
            STAT_FIRES.store(0, Ordering::Relaxed);
            STAT_CYCLES_SUM.store(0, Ordering::Relaxed);
            STAT_CYCLES_MAX.store(0, Ordering::Relaxed);
            ISR_ACTIVE.store(false, Ordering::Release);
            PACE_LAST_EOF.store(0, Ordering::Relaxed);
            PACE_BYTES.store(0, Ordering::Relaxed);

            Self { head: descs_base }
        }
    }

    /// `(half-bytes, descriptors-per-half, halves-per-frame)` for diagnostics.
    pub fn dimensions() -> (usize, usize, usize) {
        (HALF_BYTES, DESC_PER_HALF, HALVES_PER_FRAME)
    }

    /// Total EOF interrupts fired since transfer start.
    pub fn isr_fires() -> u32 {
        EOF_COUNT.load(Ordering::Relaxed)
    }

    /// Read and reset the accumulated ISR cycle stats over `window_ms` (at 240 MHz CPU clock).
    pub fn take_isr_stats(window_ms: u32) -> IsrStats {
        let fires = STAT_FIRES.swap(0, Ordering::Relaxed);
        let sum = STAT_CYCLES_SUM.swap(0, Ordering::Relaxed);
        let max = STAT_CYCLES_MAX.swap(0, Ordering::Relaxed);

        if fires == 0 || window_ms == 0 {
            return IsrStats::default();
        }
        const CYCLES_PER_US: u32 = 240;
        let avg_cycles = sum / fires;
        let window_cycles = (window_ms as u64) * 240_000;
        let cpu_pct = (((sum as u64) * 100) / window_cycles) as u32;
        IsrStats {
            core: STAT_ISR_CORE.load(Ordering::Relaxed),
            fires,
            avg_us: avg_cycles / CYCLES_PER_US,
            max_us: max / CYCLES_PER_US,
            cpu_pct,
        }
    }
}

unsafe impl DmaTxBuffer for BounceRing {
    type View = Self;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.head,
            accesses_psram: false,
            direction: TransferDirection::Out,
            burst_transfer: BurstConfig::default(),
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

/// Reconfigure ESP32-S3 L1 D-Cache from 32-byte to 64-byte lines (keeping
/// 32 KiB / 8-way so internal DRAM is unchanged) to halve Octal PSRAM burst
/// overhead during Core 1 bounce-buffer refills.
///
/// # Safety
///
/// Must be called on Core 0 before Core 1 starts and before PSRAM framebuffer use.
#[esp_hal::ram]
pub(crate) unsafe fn enable_64byte_dcache_lines() {
    unsafe extern "C" {
        fn Cache_WriteBack_All();
        fn Cache_Suspend_DCache() -> u32;
        fn rom_config_data_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );
        fn Cache_Invalidate_DCache_All();
        fn Cache_Resume_DCache(param: u32);
    }
    critical_section::with(|_| unsafe {
        Cache_WriteBack_All();
        let autoload = Cache_Suspend_DCache();
        rom_config_data_cache_mode(0x8000, 8, 64);
        Cache_Invalidate_DCache_All();
        Cache_Resume_DCache(autoload);
    });
}

/// Queue a tear-free front-buffer swap for the next frame boundary.
pub fn request_flip(new_front: *mut u8) {
    PENDING_SWAP.store(new_front, Ordering::Release);
}

/// True if a [`request_flip`] has not yet been consumed by the ISR.
pub fn is_flip_pending() -> bool {
    !PENDING_SWAP.load(Ordering::Acquire).is_null()
}

/// Install the EOF ISR and enable the GDMA channel-0 `OUT_EOF` interrupt.
///
/// # Safety
///
/// Must be called outside interrupt context and after the DPI transfer has started.
pub unsafe fn enable_eof_interrupt() {
    unsafe {
        interrupt::bind_interrupt(Interrupt::DMA_OUT_CH0, eof_isr_trampoline);
        interrupt::enable(Interrupt::DMA_OUT_CH0, Priority::Priority3)
            .expect("DMA_OUT_CH0 enable");

        let dma = &*esp_hal::peripherals::DMA::PTR;
        dma.ch(0)
            .out_int()
            .ena()
            .modify(|_, w| w.out_eof().set_bit());
    }
}

/// Start Core 1 (APP CPU), bind and enable `DMA_OUT_CH0` `OUT_EOF` on Core 1,
/// and park Core 1 in `waiti 0` so all bounce-buffer refills run on Core 1.
///
/// # Safety
///
/// Must be called once outside interrupt context after the DPI transfer has started.
pub unsafe fn enable_eof_interrupt_on_core1(
    cpu_ctrl: esp_hal::peripherals::CPU_CTRL<'static>,
) -> Result<(), ()> {
    CORE1_READY.store(false, Ordering::Release);
    let mut cpu_control = CpuControl::new(cpu_ctrl);
    let guard = cpu_control
        .start_app_core(unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) }, || {
            unsafe {
                interrupt::bind_interrupt(Interrupt::DMA_OUT_CH0, eof_isr_trampoline);
            }
            interrupt::enable(Interrupt::DMA_OUT_CH0, Priority::Priority3)
                .expect("DMA_OUT_CH0 enable on core 1");

            let dma = unsafe { &*esp_hal::peripherals::DMA::PTR };
            dma.ch(0)
                .out_int()
                .ena()
                .modify(|_, w| w.out_eof().set_bit());

            CORE1_READY.store(true, Ordering::Release);
            loop {
                unsafe {
                    core::arch::asm!("waiti 0", options(nomem, nostack));
                }
            }
        })
        .map_err(|_| ())?;

    // Keep Core 1 running permanently; dropping `guard` would park Core 1.
    core::mem::forget(guard);

    while !CORE1_READY.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }
    Ok(())
}

/// Pace Core 0 PSRAM writes so they do not overlap Core 1's `eof_isr` or exceed
/// [`MAX_CORE0_BYTES_PER_EOF`] dirty bytes in a single EOF window.
#[esp_hal::ram]
pub(crate) fn pace_psram_write(bytes: usize) {
    if !CORE1_READY.load(Ordering::Acquire) {
        return;
    }

    while ISR_ACTIVE.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }

    let req = (bytes as u32).min(MAX_CORE0_BYTES_PER_EOF);
    let eof_now = EOF_COUNT.load(Ordering::Acquire);
    let last_eof = PACE_LAST_EOF.load(Ordering::Relaxed);

    if eof_now.wrapping_sub(last_eof) <= 4 {
        PACE_LAST_EOF.store(eof_now, Ordering::Relaxed);
        PACE_BYTES.store(req, Ordering::Relaxed);
        return;
    }

    if eof_now != last_eof {
        // Fresh write burst after idle: align to the start of the quiet window right after eof_isr.
        while EOF_COUNT.load(Ordering::Acquire) == eof_now || ISR_ACTIVE.load(Ordering::Acquire) {
            core::hint::spin_loop();
        }
        PACE_LAST_EOF.store(EOF_COUNT.load(Ordering::Acquire), Ordering::Relaxed);
        PACE_BYTES.store(req, Ordering::Relaxed);
        return;
    }

    let used = PACE_BYTES.load(Ordering::Relaxed);
    if used.saturating_add(req) <= MAX_CORE0_BYTES_PER_EOF {
        PACE_BYTES.store(used + req, Ordering::Relaxed);
        return;
    }

    // Budget for this EOF window is full; wait for the next EOF refill to finish.
    while EOF_COUNT.load(Ordering::Acquire) == eof_now || ISR_ACTIVE.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }
    PACE_LAST_EOF.store(EOF_COUNT.load(Ordering::Acquire), Ordering::Relaxed);
    PACE_BYTES.store(req, Ordering::Relaxed);
}

#[inline(always)]
fn read_ccount() -> u32 {
    let cycles: u32;
    unsafe {
        core::arch::asm!("rsr.ccount {0}", out(reg) cycles, options(nomem, nostack));
    }
    cycles
}

#[esp_hal::ram]
unsafe extern "C" fn eof_isr_trampoline() {
    eof_isr();
}

#[esp_hal::ram]
fn eof_isr() {
    let t0 = read_ccount();
    let dma = unsafe { &*esp_hal::peripherals::DMA::PTR };

    if !dma.ch(0).out_int().st().read().out_eof().bit_is_set() {
        return;
    }

    ISR_ACTIVE.store(true, Ordering::Release);

    let eof_des_addr = dma.ch(0).out_eof_des_addr().read().bits() as usize;
    let descs_base = core::ptr::addr_of!(DESCRIPTORS) as usize;
    let desc_size = core::mem::size_of::<DmaDescriptor>();
    let ring_bytes = N_DESC * desc_size;

    if eof_des_addr < descs_base || eof_des_addr >= descs_base + ring_bytes {
        EOF_COUNT.fetch_add(1, Ordering::Release);
        dma.ch(0)
            .out_int()
            .clr()
            .write(|w| w.out_eof().clear_bit_by_one());
        ISR_ACTIVE.store(false, Ordering::Release);
        return;
    }

    let desc_idx = (eof_des_addr - descs_base) / desc_size;
    let just_finished_half = desc_idx / DESC_PER_HALF;

    let dest: *mut u8 = if just_finished_half % 2 == 0 {
        core::ptr::addr_of_mut!(BOUNCE_A) as *mut u8
    } else {
        core::ptr::addr_of_mut!(BOUNCE_B) as *mut u8
    };

    let next_chunk = (just_finished_half + 2 + STARTUP_OFFSET_HALVES) % HALVES_PER_FRAME;

    if next_chunk == 0 {
        let pending = PENDING_SWAP.swap(core::ptr::null_mut(), Ordering::AcqRel);
        if !pending.is_null() {
            PSRAM_FB_PTR.store(pending, Ordering::Release);
        }
    }

    let psram_base = PSRAM_FB_PTR.load(Ordering::Acquire);
    unsafe {
        psram_copy_wrapping(psram_base, next_chunk, dest);
    }

    EOF_COUNT.fetch_add(1, Ordering::Release);

    dma.ch(0)
        .out_int()
        .clr()
        .write(|w| w.out_eof().clear_bit_by_one());

    let elapsed = read_ccount().wrapping_sub(t0);
    STAT_ISR_CORE.store(Cpu::current() as u8, Ordering::Relaxed);
    STAT_FIRES.fetch_add(1, Ordering::Relaxed);
    STAT_CYCLES_SUM.fetch_add(elapsed, Ordering::Relaxed);
    let _ = STAT_CYCLES_MAX.fetch_max(elapsed, Ordering::Relaxed);
    ISR_ACTIVE.store(false, Ordering::Release);
}

/// 16-byte-unrolled copy so the ISR never calls Flash-resident `memcpy`
/// (inlined into `#[esp_hal::ram] fn eof_isr`).
#[inline(always)]
unsafe fn copy_aligned_16(mut src: *const u32, mut dst: *mut u32, bytes: usize) {
    let mut quads = bytes / 16;
    unsafe {
        while quads > 0 {
            let w0 = src.read();
            let w1 = src.add(1).read();
            let w2 = src.add(2).read();
            let w3 = src.add(3).read();
            dst.write(w0);
            dst.add(1).write(w1);
            dst.add(2).write(w2);
            dst.add(3).write(w3);
            src = src.add(4);
            dst = dst.add(4);
            quads -= 1;
        }
    }
}

#[inline(always)]
unsafe fn psram_copy_wrapping(psram_base: *const u8, chunk_idx: usize, dest: *mut u8) {
    unsafe {
        let natural = chunk_idx * HALF_BYTES;
        let start = (natural + framebuffer::BYTES - STARTUP_OFFSET_BYTES) % framebuffer::BYTES;
        let end = start + HALF_BYTES;

        if end <= framebuffer::BYTES {
            copy_aligned_16(
                psram_base.add(start) as *const u32,
                dest as *mut u32,
                HALF_BYTES,
            );
        } else {
            let first = framebuffer::BYTES - start;
            let second = HALF_BYTES - first;
            copy_aligned_16(
                psram_base.add(start) as *const u32,
                dest as *mut u32,
                first,
            );
            copy_aligned_16(
                psram_base as *const u32,
                dest.add(first) as *mut u32,
                second,
            );
        }
    }
}
