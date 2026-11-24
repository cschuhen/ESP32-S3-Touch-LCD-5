//! Bounce-buffer mode for the DPI panel.
//!
//! Two small DRAM bounce buffers (`BOUNCE_A`, `BOUNCE_B`) sit between the PSRAM
//! master framebuffer and the `LCD_CAM` GDMA channel so GDMA never reads PSRAM
//! directly (which causes diagonal refresh-drift stripes on ESP32-S3).

#![allow(dead_code)]

use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use esp_hal::dma::{
    BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation, TransferDirection,
};
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::Interrupt;

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
/// Fine startup offset in scanlines (0 by default; calibrate with `hello_pattern`).
const STARTUP_OFFSET_LINES: usize = 0;
const STARTUP_OFFSET_BYTES: usize = STARTUP_OFFSET_LINES * WIDTH * 2;

/// Total descriptors in the frame-sized ring.
const N_DESC: usize = DESC_PER_HALF * HALVES_PER_FRAME;

// Compile-time invariants for alternating A/B bounce-buffer wrapping.
const _: () = assert!(framebuffer::BYTES % HALF_BYTES == 0);
const _: () = assert!(HALVES_PER_FRAME % 2 == 0);

#[repr(align(64))]
struct BounceHalf([u8; HALF_BYTES]);

static mut BOUNCE_A: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut BOUNCE_B: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut DESCRIPTORS: [DmaDescriptor; N_DESC] = [DmaDescriptor::EMPTY; N_DESC];

static PSRAM_FB_PTR: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
static PENDING_SWAP: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
static EOF_COUNT: AtomicU32 = AtomicU32::new(0);

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

            let init_a_chunk = STARTUP_OFFSET_HALVES % HALVES_PER_FRAME;
            let init_b_chunk = (STARTUP_OFFSET_HALVES + 1) % HALVES_PER_FRAME;
            psram_copy_wrapping(psram_fb_ptr, init_a_chunk, buf_a);
            psram_copy_wrapping(psram_fb_ptr, init_b_chunk, buf_b);

            PSRAM_FB_PTR.store(psram_fb_ptr as *mut u8, Ordering::Release);
            EOF_COUNT.store(0, Ordering::Relaxed);

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
        interrupt::enable(Interrupt::DMA_OUT_CH0, Priority::Priority2)
            .expect("DMA_OUT_CH0 enable");

        let dma = &*esp_hal::peripherals::DMA::PTR;
        dma.ch(0)
            .out_int()
            .ena()
            .modify(|_, w| w.out_eof().set_bit());
    }
}

unsafe extern "C" fn eof_isr_trampoline() {
    eof_isr();
}

fn eof_isr() {
    let dma = unsafe { &*esp_hal::peripherals::DMA::PTR };

    if !dma.ch(0).out_int().st().read().out_eof().bit_is_set() {
        return;
    }

    let eof_des_addr = dma.ch(0).out_eof_des_addr().read().bits() as usize;
    let descs_base = core::ptr::addr_of!(DESCRIPTORS) as usize;
    let desc_size = core::mem::size_of::<DmaDescriptor>();
    let ring_bytes = N_DESC * desc_size;

    if eof_des_addr < descs_base || eof_des_addr >= descs_base + ring_bytes {
        EOF_COUNT.fetch_add(1, Ordering::Relaxed);
        dma.ch(0)
            .out_int()
            .clr()
            .write(|w| w.out_eof().clear_bit_by_one());
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

    EOF_COUNT.fetch_add(1, Ordering::Relaxed);

    dma.ch(0)
        .out_int()
        .clr()
        .write(|w| w.out_eof().clear_bit_by_one());
}

#[inline]
unsafe fn psram_copy_wrapping(psram_base: *const u8, chunk_idx: usize, dest: *mut u8) {
    unsafe {
        let natural = chunk_idx * HALF_BYTES;
        let start = (natural + framebuffer::BYTES - STARTUP_OFFSET_BYTES) % framebuffer::BYTES;
        let end = start + HALF_BYTES;

        if end <= framebuffer::BYTES {
            core::ptr::copy_nonoverlapping(psram_base.add(start), dest, HALF_BYTES);
        } else {
            let first = framebuffer::BYTES - start;
            let second = HALF_BYTES - first;
            core::ptr::copy_nonoverlapping(psram_base.add(start), dest, first);
            core::ptr::copy_nonoverlapping(psram_base, dest.add(first), second);
        }
    }
}
