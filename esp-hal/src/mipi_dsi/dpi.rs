//! MIPI DSI video-mode (DPI) driver.

use core::{
    marker::PhantomData,
    pin::Pin,
    sync::atomic,
    task::{Context, Poll},
};

use esp_sync::NonReentrantMutex;

use crate::{
    asynch::AtomicWaker,
    interrupt,
    mipi_dsi::{
        ConfigError,
        MipiDsi,
        vdma::{VdmaChannel, VdmaLinkItem},
    },
    peripherals::{Interrupt, MIPI_DSI_BRIDGE, MIPI_DSI_HOST, VDMA},
    soc::clocks::{ClockTree, MipiDsiDpiClkConfig, MipiDsiInstance},
    system::Cpu,
};

/// Channel index used by the static VDMA block-done ISR.
///
/// Set before the channel is started; read by the ISR which cannot capture
/// runtime state.  A single MIPI-DSI instance is the only VDMA user, so
/// this value is stable for the lifetime of `DsiDpi`.
static VDMA_ISR_CHANNEL: atomic::AtomicU8 = atomic::AtomicU8::new(0);

const MAX_FBS: usize = 3;
/// LLIs in the circular DMA ring.  The ISR re-arms each one immediately after
/// the DMA consumes it, so the ring never suspends regardless of ring depth.
/// Two is sufficient; keeping it small reduces ISR overhead.
const NUM_LLIS: usize = 2;
const DMA_BURST_LEN: u32 = 256;
const FIFO_EMPTY_THRESHOLD: u32 = 1024 - DMA_BURST_LEN;

// ── Static link-list storage ──────────────────────────────────────────────────

static LLI_STORAGE: NonReentrantMutex<[VdmaLinkItem; NUM_LLIS]> =
    NonReentrantMutex::new([const { VdmaLinkItem::zeroed() }; NUM_LLIS]);

// ── Public types ──────────────────────────────────────────────────────────────

// DPI pixel clock source is represented by the generated `MipiDsiDpiClkSclk`
// enum from the clock tree. Re-export it so callers don't need to reach into
// `soc::clocks` directly.
pub use crate::soc::clocks::MipiDsiDpiClkSclk as DpiClockSource;

/// Input/output pixel color format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ColorFormat {
    /// RGB 888, 24 bpp.
    Rgb888,
    /// RGB 565, 16 bpp.
    Rgb565,
}

impl ColorFormat {
    pub(crate) fn bits_per_pixel(self) -> u32 {
        match self {
            Self::Rgb888 => 24,
            Self::Rgb565 => 16,
        }
    }

    fn raw_type(self) -> u8 {
        match self {
            Self::Rgb888 => 0,
            Self::Rgb565 => 2,
        }
    }

    fn dpi_type(self) -> u8 {
        match self {
            Self::Rgb888 => 0,
            Self::Rgb565 => 2,
        }
    }

    /// `dpi_color_coding` register value (DSI host).
    fn host_color_coding(self) -> u8 {
        match self {
            Self::Rgb888 => 5, // 24-bit
            Self::Rgb565 => 0, // 16-bit config 1
        }
    }
}

/// Video frame timing (pixel/line counts).
#[derive(Clone, Copy, Debug)]
pub struct FrameTiming {
    /// Active pixels per line.
    pub h_active: u32,
    /// Horizontal sync pulse width (pixels).
    pub hsw: u32,
    /// Horizontal back porch (pixels).
    pub hbp: u32,
    /// Horizontal front porch (pixels).
    pub hfp: u32,
    /// Active lines per frame.
    pub v_active: u32,
    /// Vertical sync pulse width (lines).
    pub vsw: u32,
    /// Vertical back porch (lines).
    pub vbp: u32,
    /// Vertical front porch (lines).
    pub vfp: u32,
}

/// DPI video-mode configuration.
#[derive(Clone, Copy, Debug)]
pub struct DpiConfig {
    /// DSI virtual channel (0–3).
    pub virtual_channel: u8,
    /// Desired pixel clock in MHz.
    pub pixel_clock_mhz: f32,
    /// Pixel clock source (XTAL, PLL_F240M, or PLL_F160M).
    pub dpi_clk_src: DpiClockSource,
    /// Input pixel format (from frame buffer).
    pub in_color_format: ColorFormat,
    /// Output pixel format (sent over DSI lanes).
    pub out_color_format: ColorFormat,
    /// Frame timing parameters.
    pub timing: FrameTiming,
}

// ── VDMA block-done ISR ───────────────────────────────────────────────────────

/// Fired by the DW-GDMA at the end of every frame block transfer.
///
/// Re-arms the LLI that the DMA just consumed (sets `LLI_VALID = 1` and
/// flushes the cache line to SRAM) so the DMA can use it again when the ring
/// comes back around.  This keeps the channel running indefinitely without
/// software ever needing to restart it.
#[crate::ram]
#[crate::handler]
fn vdma_block_done_isr() {
    let channel_id = VDMA_ISR_CHANNEL.load(atomic::Ordering::Relaxed) as usize;
    let ch = VDMA::regs().ch(channel_id);
    ch.intclear0().write(|w| unsafe { w.bits(0xFFFFFFFF) });

    LLI_STORAGE.with(|storage| {
        for lli in storage.iter() {
            lli.rearm();
        }

        // Flush LLIs from CPU cache → SRAM.
        unsafe {
            crate::soc::cache_writeback_addr(
                storage.as_ptr() as u32,
                core::mem::size_of::<VdmaLinkItem>() as u32 * NUM_LLIS as u32,
            );
        }
    });
}

// ── Async waker ───────────────────────────────────────────────────────────────

static VSYNC_WAKER: AtomicWaker = AtomicWaker::new();

#[crate::handler]
fn dsi_bridge_isr() {
    let bridge = MIPI_DSI_BRIDGE::regs();
    let st = bridge.int_st().read();
    if st.vsync().bit_is_set() {
        bridge.int_ena().modify(|_, w| w.vsync().clear_bit());
        VSYNC_WAKER.wake();
    }
}

// ── DsiDpi ────────────────────────────────────────────────────────────────────

/// Video-mode (DPI) streaming handle.
///
/// Consumes the [`MipiDsi`] bus and drives the DSI host in video mode,
/// continuously streaming frame buffers via VDMA.
pub struct DsiDpi<'d> {
    _guard: crate::mipi_dsi::DphyGuard<'d>,
    fb_ptrs: [*mut u8; MAX_FBS],
    fb_size: usize,
    num_fbs: usize,
    current_fb: usize,
    _phantom: PhantomData<&'d mut [u8]>,
}

impl Drop for DsiDpi<'_> {
    fn drop(&mut self) {
        // Disable the block-done interrupt before shutting down.
        let channel_id = self._guard.vdma_channel_id as usize;
        let ch = VDMA::regs().ch(channel_id);
        unsafe {
            ch.intsignal_enable0().write_with_zero(|w| w);
            ch.intstatus_enable0().write_with_zero(|w| w);
        }
        interrupt::disable(Cpu::current(), Interrupt::DMA);

        // Disable the VDMA channel itself.
        let shift = channel_id as u32;
        let val: u32 = 0x0100 << shift; // ch_en_we only (clears ch_en)
        unsafe { VDMA::regs().chen0().write(|w| w.bits(val)) };

        MIPI_DSI_BRIDGE::regs()
            .dpi_misc_config()
            .modify(|_, w| w.dpi_en().clear_bit());
        ClockTree::with(|clocks| MipiDsiInstance::MipiDsi.release_dpi_clk(clocks));
        // _guard is dropped next, which calls dphy_power_down() and releases PHY clocks.
    }
}

impl<'d> DsiDpi<'d> {
    pub(crate) fn new(
        bus: MipiDsi<'d>,
        config: DpiConfig,
        framebuffers: &[&'d mut [u8]],
    ) -> Result<Self, ConfigError> {
        let num_fbs = framebuffers.len();
        debug_assert!((1..=MAX_FBS).contains(&num_fbs));
        let fb_size = framebuffers[0].len();

        // ── DPI clock ──────────────────────────────────────────────────────
        // Compute the divider from the source frequency and the desired pixel
        // clock, then configure and enable through the clock tree so that the
        // upstream PLL reference count is maintained correctly.
        let (_dpi_clk_config, real_dpi_mhz) = ClockTree::with(|clocks| {
            // Obtain source frequency (div_num = 0 → no division applied yet).
            let src_hz = MipiDsiInstance::dpi_clk_config_frequency(
                clocks,
                MipiDsiDpiClkConfig::new(config.dpi_clk_src, 0),
            );
            let src_mhz = src_hz as f32 / 1_000_000.0;
            let div = ((src_mhz / config.pixel_clock_mhz) + 0.5) as u32;
            let div = div.max(1);
            let cfg = MipiDsiDpiClkConfig::new(config.dpi_clk_src, div - 1);
            MipiDsiInstance::MipiDsi.configure_dpi_clk(clocks, cfg);
            MipiDsiInstance::MipiDsi.request_dpi_clk(clocks);
            (cfg, src_mhz / div as f32)
        });

        let host = MIPI_DSI_HOST::regs();
        let bridge = MIPI_DSI_BRIDGE::regs();

        // ── Host: virtual channel & color coding ───────────────────────────
        host.dpi_vcid()
            .modify(|_, w| unsafe { w.dpi_vcid().bits(config.virtual_channel) });
        host.dpi_color_coding().modify(|_, w| unsafe {
            w.dpi_color_coding()
                .bits(config.out_color_format.host_color_coding())
        });

        // All DPI signals active-high (no inversion).
        host.dpi_cfg_pol().write(|w| unsafe { w.bits(0) });

        // Burst mode with sync pulses.  LP transitions are allowed only in
        // blanking intervals; lp_vact_en and frame_bta_ack_en are left clear:
        // - lp_vact_en: going LP during the active pixel burst corrupts video.
        // - frame_bta_ack_en: requesting a per-frame BTA stalls the host if the panel does not
        //   respond, freezing the video stream.
        host.vid_mode_cfg().modify(|_, w| unsafe {
            w.vid_mode_type().bits(2); // burst with sync pulses
            w.lp_vsa_en().set_bit();
            w.lp_vbp_en().set_bit();
            w.lp_vfp_en().set_bit();
            w.lp_hbp_en().set_bit();
            w.lp_hfp_en().set_bit();
            w.lp_cmd_en().set_bit()
        });

        let t = &config.timing;
        host.vid_pkt_size()
            .modify(|_, w| unsafe { w.vid_pkt_size().bits(t.h_active as u16) });
        host.vid_num_chunks()
            .modify(|_, w| unsafe { w.vid_num_chunks().bits(0) });
        host.vid_null_size()
            .modify(|_, w| unsafe { w.vid_null_size().bits(0) });

        // ── Host: horizontal timing (lane byte clock cycles) ───────────────
        let ratio = bus.lane_bit_rate_mbps / config.pixel_clock_mhz / 8.0;
        let htotal = t.hsw + t.hbp + t.h_active + t.hfp;

        let host_hsw = fround_u32(t.hsw as f32 * ratio);
        let host_hbp = fround_u32(t.hbp as f32 * ratio);
        let host_act = fround_u32(t.h_active as f32 * ratio);
        let host_hfp = fround_u32(t.hfp as f32 * ratio);
        let host_htotal = fround_u32(htotal as f32 * ratio);
        let comp = host_htotal as i32 - (host_hsw + host_hbp + host_act + host_hfp) as i32;
        let host_act = (host_act as i32 + comp).max(0) as u32;

        host.vid_hsa_time()
            .modify(|_, w| unsafe { w.vid_hsa_time().bits(host_hsw as u16) });
        host.vid_hbp_time()
            .modify(|_, w| unsafe { w.vid_hbp_time().bits(host_hbp as u16) });
        host.vid_hline_time().modify(|_, w| unsafe {
            w.vid_hline_time()
                .bits((host_hsw + host_hbp + host_act + host_hfp) as u16)
        });

        // ── Host: vertical timing ──────────────────────────────────────────
        host.vid_vsa_lines()
            .modify(|_, w| unsafe { w.vsa_lines().bits(t.vsw as u16) });
        host.vid_vbp_lines()
            .modify(|_, w| unsafe { w.vbp_lines().bits(t.vbp as u16) });
        host.vid_vactive_lines()
            .modify(|_, w| unsafe { w.v_active_lines().bits(t.v_active as u16) });
        host.vid_vfp_lines()
            .modify(|_, w| unsafe { w.vfp_lines().bits(t.vfp as u16) });

        // ── Bridge: timing (HFP compensated for actual DPI clock) ─────────
        let brg_hfp = {
            let c = fround_u32(real_dpi_mhz / config.pixel_clock_mhz * htotal as f32) as i32
                - htotal as i32;
            (t.hfp as i32 + c).max(0) as u32
        };
        bridge.dpi_h_cfg0().modify(|_, w| unsafe {
            w.htotal()
                .bits((t.hsw + t.hbp + t.h_active + brg_hfp) as u16);
            w.hdisp().bits(t.h_active as u16)
        });
        bridge.dpi_h_cfg1().modify(|_, w| unsafe {
            w.hsync().bits(t.hsw as u16);
            w.hbank().bits(t.hbp as u16)
        });
        bridge.dpi_v_cfg0().modify(|_, w| unsafe {
            w.vtotal().bits((t.vsw + t.vbp + t.v_active + t.vfp) as u16);
            w.vdisp().bits(t.v_active as u16)
        });
        bridge.dpi_v_cfg1().modify(|_, w| unsafe {
            w.vsync().bits(t.vsw as u16);
            w.vbank().bits(t.vbp as u16)
        });

        // ── Bridge: pixel format & DMA ─────────────────────────────────────
        let total_bits = t.h_active * t.v_active * config.in_color_format.bits_per_pixel();
        bridge.raw_num_cfg().modify(|_, w| unsafe {
            w.raw_num_total().bits(total_bits.div_ceil(64));
            w.unalign_64bit_en().bit(!total_bits.is_multiple_of(64));
            w.raw_num_total_set().set_bit()
        });
        bridge
            .dpi_misc_config()
            .modify(|_, w| unsafe { w.fifo_underrun_discard_vcnt().bits(t.h_active as u16) });
        bridge.pixel_type().modify(|_, w| unsafe {
            w.raw_type().bits(config.in_color_format.raw_type());
            w.dpi_type().bits(config.out_color_format.dpi_type());
            w.data_in_type().clear_bit()
        });
        bridge.dma_flow_ctrl().modify(|_, w| unsafe {
            w.dsi_dma_flow_controller().clear_bit();
            w.dma_flow_multiblk_num().bits(1)
        });
        bridge
            .dma_frame_interval()
            .modify(|_, w| w.dma_multiblk_en().clear_bit());
        bridge
            .dma_req_cfg()
            .modify(|_, w| unsafe { w.dma_burst_len().bits(DMA_BURST_LEN as u16) });
        bridge.raw_buf_almost_empty_thrd().modify(|_, w| unsafe {
            w.dsi_raw_buf_almost_empty_thrd()
                .bits(FIFO_EMPTY_THRESHOLD as u16)
        });

        bridge.en().modify(|_, w| w.dsi_en().set_bit());
        bridge
            .dpi_config_update()
            .write(|w| w.dpi_config_update().set_bit());

        interrupt::bind_handler(Interrupt::DSI_BRIDGE, dsi_bridge_isr);

        // Flush all frame buffers from CPU cache → PSRAM.
        for fb in framebuffers.iter() {
            unsafe {
                crate::soc::cache_writeback_addr(fb.as_ptr() as u32, fb_size as u32);
            }
        }

        // Build ring: LLI[i] → LLI[(i+1) % NUM_LLIS].
        let fb_ptrs = core::array::from_fn::<_, MAX_FBS, _>(|i| {
            if i < num_fbs {
                framebuffers[i].as_ptr().cast_mut()
            } else {
                core::ptr::null_mut::<u8>()
            }
        });

        let vdma_channel_id = bus.guard.vdma_channel_id;
        VDMA_ISR_CHANNEL.store(vdma_channel_id, atomic::Ordering::Relaxed);

        LLI_STORAGE.with(|storage| {
            for i in 0..NUM_LLIS {
                let next = &storage[(i + 1) % NUM_LLIS] as *const VdmaLinkItem;
                storage[i].configure(fb_ptrs[0] as u32, fb_size, next);
            }

            // Flush the LLI descriptors from CPU cache → SRAM so the DMA sees them.
            unsafe {
                crate::soc::cache_writeback_addr(
                    storage.as_ptr() as u32,
                    (core::mem::size_of::<VdmaLinkItem>() * NUM_LLIS) as u32,
                );
            }

            VdmaChannel::new(vdma_channel_id).start(&storage[0]);
        });

        // ── VDMA block-done interrupt ──────────────────────────────────────
        // Enable the block-transfer-done signal so the ISR fires after every
        // frame block and re-arms the consumed LLI.
        {
            let ch = VDMA::regs().ch(vdma_channel_id as usize);
            ch.intstatus_enable0()
                .write(|w| w.ch1_enable_block_tfr_done_intstat().set_bit());
            ch.intsignal_enable0()
                .write(|w| w.ch1_enable_block_tfr_done_intsignal().set_bit());
        }
        interrupt::bind_handler(Interrupt::DMA, vdma_block_done_isr);

        // ── Enable video mode ──────────────────────────────────────────────
        host.mode_cfg()
            .modify(|_, w| w.cmd_video_mode().clear_bit());
        bridge.dpi_misc_config().modify(|_, w| w.dpi_en().set_bit());
        bridge
            .dpi_config_update()
            .write(|w| w.dpi_config_update().set_bit());

        let MipiDsi { guard, .. } = bus;
        Ok(Self {
            _guard: guard,
            fb_ptrs,
            fb_size,
            num_fbs,
            current_fb: 0,
            _phantom: PhantomData,
        })
    }

    /// Returns a mutable slice to the back (not-currently-displayed) frame buffer.
    ///
    /// With a single frame buffer this is the same buffer the DMA may be
    /// reading — the caller is responsible for synchronisation in that case.
    pub fn framebuffer_mut(&mut self) -> &mut [u8] {
        let back = (self.current_fb + 1) % self.num_fbs;
        unsafe { core::slice::from_raw_parts_mut(self.fb_ptrs[back], self.fb_size) }
    }

    /// Flip the back buffer to the display.
    ///
    /// Flushes the rendered back buffer from CPU cache to PSRAM, then updates
    /// the source address in every LLI so the DMA switches to the new pixels
    /// on its next block boundary.
    ///
    /// `LLI_VALID` re-arming is handled entirely by the `vdma_block_done_isr`
    /// interrupt handler, which fires after every frame block and keeps the
    /// DMA channel running indefinitely without software restarts.
    pub fn commit(&mut self) {
        let back = (self.current_fb + 1) % self.num_fbs;

        // Flush back buffer: CPU cache → PSRAM.
        unsafe {
            crate::soc::cache_writeback_addr(self.fb_ptrs[back] as u32, self.fb_size as u32);
        }

        // Update sar_lo in every LLI to point at the new front buffer.
        // The DMA latches sar_lo at the start of each block, so an in-progress
        // block is unaffected; the new pixels appear on the next block boundary.
        let src = self.fb_ptrs[back] as u32;
        LLI_STORAGE.with(|storage| {
            for lli in storage.iter() {
                lli.set_source(src);
            }
        });

        // The ISR will flush the cache, no need to do it here.

        self.current_fb = back;
    }

    /// Block until the DSI bridge signals the start of the next vertical blank.
    ///
    /// Use this to pace rendering to the display refresh rate.  Any vsync event
    /// that is already pending (i.e. fired while the CPU was busy rendering)
    /// will be returned immediately.
    pub fn wait_for_vsync(&mut self) {
        let bridge = MIPI_DSI_BRIDGE::regs();
        while !bridge.int_raw().read().vsync().bit_is_set() {}
        bridge.int_clr().write(|w| w.vsync().clear_bit_by_one());
    }

    /// Async: yield until the next vsync event.
    pub fn wait_for_vsync_async(&mut self) -> impl Future<Output = ()> {
        VsyncFuture { _dpi: self }
    }
}

struct VsyncFuture<'a, 'b> {
    _dpi: &'a mut DsiDpi<'b>,
}

impl Future for VsyncFuture<'_, '_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let bridge = MIPI_DSI_BRIDGE::regs();
        if bridge.int_raw().read().vsync().bit_is_set() {
            bridge.int_clr().write(|w| w.vsync().clear_bit_by_one());
            Poll::Ready(())
        } else {
            VSYNC_WAKER.register(cx.waker());
            bridge.int_ena().modify(|_, w| w.vsync().set_bit());
            Poll::Pending
        }
    }
}

impl Drop for VsyncFuture<'_, '_> {
    fn drop(&mut self) {
        let bridge = MIPI_DSI_BRIDGE::regs();
        bridge.int_ena().modify(|_, w| w.vsync().clear_bit());
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

#[inline]
fn fround_u32(x: f32) -> u32 {
    (x + 0.5) as u32
}
