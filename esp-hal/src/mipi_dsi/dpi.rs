//! MIPI DSI video-mode (DPI) driver.

use core::marker::PhantomData;

use crate::{
    mipi_dsi::{
        vdma::{VdmaChannel, VdmaLinkItem},
        ConfigError, MipiDsi,
    },
    peripherals::{HP_SYS_CLKRST, MIPI_DSI, MIPI_DSI_BRIDGE, MIPI_DSI_HOST, VDMA},
    system::{GenericPeripheralGuard, Peripheral},
};

const MAX_FBS: usize = 3;
const DMA_BURST_LEN: u32 = 256;
const FIFO_EMPTY_THRESHOLD: u32 = 1024 - DMA_BURST_LEN;

// ── Public types ──────────────────────────────────────────────────────────────

/// Pixel clock source for the DPI interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DpiClockSource {
    /// Crystal oscillator (40 MHz on ESP32-P4).
    Xtal = 0,
    /// 240 MHz PLL.
    PllF240m = 1,
    /// 160 MHz PLL.
    PllF160m = 2,
    /// Audio PLL (variable; treated as 40 MHz reference).
    Apll = 3,
}

impl DpiClockSource {
    fn freq_mhz(self) -> f32 {
        match self {
            Self::Xtal     => 40.0,
            Self::PllF240m => 240.0,
            Self::PllF160m => 160.0,
            Self::Apll     => 40.0,
        }
    }
}

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
    pub hsw:      u32,
    /// Horizontal back porch (pixels).
    pub hbp:      u32,
    /// Horizontal front porch (pixels).
    pub hfp:      u32,
    /// Active lines per frame.
    pub v_active: u32,
    /// Vertical sync pulse width (lines).
    pub vsw:      u32,
    /// Vertical back porch (lines).
    pub vbp:      u32,
    /// Vertical front porch (lines).
    pub vfp:      u32,
}

/// DPI video-mode configuration.
#[derive(Clone, Copy, Debug)]
pub struct DpiConfig {
    /// DSI virtual channel (0–3).
    pub virtual_channel:  u8,
    /// Desired pixel clock in MHz.
    pub pixel_clock_mhz:  f32,
    /// DPI clock source.
    pub dpi_clk_src:      DpiClockSource,
    /// Input pixel format (from frame buffer).
    pub in_color_format:  ColorFormat,
    /// Output pixel format (sent over DSI lanes).
    pub out_color_format: ColorFormat,
    /// Frame timing parameters.
    pub timing:           FrameTiming,
}

// ── Static link-list storage ──────────────────────────────────────────────────

#[repr(align(64))]
struct AlignedLlis([VdmaLinkItem; MAX_FBS]);

static mut LLI_STORAGE: AlignedLlis = AlignedLlis([
    VdmaLinkItem::zeroed(),
    VdmaLinkItem::zeroed(),
    VdmaLinkItem::zeroed(),
]);

// ── DsiDpi ────────────────────────────────────────────────────────────────────

/// Video-mode (DPI) streaming handle.
///
/// Consumes the [`MipiDsi`] bus and drives the DSI host in video mode,
/// continuously streaming frame buffers via VDMA.
pub struct DsiDpi<'d> {
    _dsi:        MIPI_DSI<'d>,
    _vdma:       VDMA<'d>,
    _dsi_guard:  GenericPeripheralGuard<{ Peripheral::MipiDsi as u8 }>,
    _vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>,
    dma:         VdmaChannel,
    fb_ptrs:     [*mut u8; MAX_FBS],
    fb_size:     usize,
    num_fbs:     usize,
    current_fb:  usize,
    _phantom:    PhantomData<&'d mut [u8]>,
}

impl<'d> DsiDpi<'d> {
    pub(crate) fn new(
        bus:          MipiDsi<'d>,
        config:       DpiConfig,
        framebuffers: &[&'d mut [u8]],
    ) -> Result<Self, ConfigError> {
        let num_fbs = framebuffers.len();
        debug_assert!(num_fbs >= 1 && num_fbs <= MAX_FBS);
        let fb_size = framebuffers[0].len();

        // ── DPI clock ──────────────────────────────────────────────────────
        let src_mhz = config.dpi_clk_src.freq_mhz();
        let div = ((src_mhz / config.pixel_clock_mhz) + 0.5) as u32;
        let div = div.max(1);
        let real_dpi_mhz = src_mhz / div as f32;

        HP_SYS_CLKRST::regs().peri_clk_ctrl03().modify(|_, w| unsafe {
            w.mipi_dsi_dpiclk_src_sel()
                .bits(config.dpi_clk_src as u8)
                .mipi_dsi_dpiclk_div_num()
                .bits((div - 1) as u8)
                .mipi_dsi_dpiclk_en()
                .set_bit()
        });

        let host   = MIPI_DSI_HOST::regs();
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

        // LP transitions allowed in all blank periods + commands in LP.
        host.vid_mode_cfg().modify(|_, w| unsafe {
            w.vid_mode_type().bits(2) // burst with sync pulses
                .lp_vsa_en().set_bit()
                .lp_vbp_en().set_bit()
                .lp_vfp_en().set_bit()
                .lp_vact_en().set_bit()
                .lp_hbp_en().set_bit()
                .lp_hfp_en().set_bit()
                .lp_cmd_en().set_bit()
                .frame_bta_ack_en().set_bit()
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

        let host_hsw    = fround_u32(t.hsw     as f32 * ratio);
        let host_hbp    = fround_u32(t.hbp     as f32 * ratio);
        let host_act    = fround_u32(t.h_active as f32 * ratio);
        let host_hfp    = fround_u32(t.hfp     as f32 * ratio);
        let host_htotal = fround_u32(htotal     as f32 * ratio);
        let comp = host_htotal as i32
            - (host_hsw + host_hbp + host_act + host_hfp) as i32;
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
                .bits((t.hsw + t.hbp + t.h_active + brg_hfp) as u16)
                .hdisp()
                .bits(t.h_active as u16)
        });
        bridge.dpi_h_cfg1().modify(|_, w| unsafe {
            w.hsync().bits(t.hsw as u16).hbank().bits(t.hbp as u16)
        });
        bridge.dpi_v_cfg0().modify(|_, w| unsafe {
            w.vtotal()
                .bits((t.vsw + t.vbp + t.v_active + t.vfp) as u16)
                .vdisp()
                .bits(t.v_active as u16)
        });
        bridge.dpi_v_cfg1().modify(|_, w| unsafe {
            w.vsync().bits(t.vsw as u16).vbank().bits(t.vbp as u16)
        });

        // ── Bridge: pixel format & DMA ─────────────────────────────────────
        let total_bits =
            t.h_active * t.v_active * config.in_color_format.bits_per_pixel();
        bridge.raw_num_cfg().modify(|_, w| unsafe {
            w.raw_num_total()
                .bits((total_bits + 63) / 64)
                .unalign_64bit_en()
                .bit(total_bits % 64 != 0)
                .raw_num_total_set()
                .set_bit()
        });
        bridge.dpi_misc_config().modify(|_, w| unsafe {
            w.fifo_underrun_discard_vcnt().bits(t.h_active as u16)
        });
        bridge.pixel_type().modify(|_, w| unsafe {
            w.raw_type()
                .bits(config.in_color_format.raw_type())
                .dpi_type()
                .bits(config.out_color_format.dpi_type())
                .data_in_type()
                .clear_bit()
        });
        bridge.dma_flow_ctrl().modify(|_, w| unsafe {
            w.dsi_dma_flow_controller()
                .clear_bit()
                .dma_flow_multiblk_num()
                .bits(1)
        });
        bridge.dma_frame_interval().modify(|_, w| {
            w.dma_multiblk_en().clear_bit()
        });
        bridge.dma_req_cfg().modify(|_, w| unsafe {
            w.dma_burst_len().bits(DMA_BURST_LEN as u16)
        });
        bridge.raw_buf_almost_empty_thrd().modify(|_, w| unsafe {
            w.dsi_raw_buf_almost_empty_thrd().bits(FIFO_EMPTY_THRESHOLD as u16)
        });

        bridge.en().modify(|_, w| w.dsi_en().set_bit());
        bridge.dpi_config_update()
            .write(|w| w.dpi_config_update().set_bit());

        // ── VDMA: build LLIs and start from framebuffer 0 ─────────────────
        let llis = unsafe { &mut *(&raw mut LLI_STORAGE.0) };
        let mut fb_ptrs = [core::ptr::null_mut::<u8>(); MAX_FBS];
        for i in 0..num_fbs {
            let ptr = framebuffers[i].as_ptr() as *mut u8;
            fb_ptrs[i] = ptr;
            llis[i].configure(ptr as u32, fb_size);
        }

        let mut dma = VdmaChannel::new(0);
        dma.start(&llis[0]);

        // ── Enable video mode ──────────────────────────────────────────────
        host.mode_cfg().modify(|_, w| w.cmd_video_mode().clear_bit());
        bridge.dpi_misc_config().modify(|_, w| w.dpi_en().set_bit());
        bridge.dpi_config_update()
            .write(|w| w.dpi_config_update().set_bit());

        // Enable underrun interrupt (for status monitoring).
        bridge.int_ena().modify(|_, w| w.underrun().set_bit());

        let MipiDsi { _dsi, _vdma, _dsi_guard, _vdma_guard, .. } = bus;
        Ok(Self {
            _dsi,
            _vdma,
            _dsi_guard,
            _vdma_guard,
            dma,
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

    /// Make the back buffer (returned by [`framebuffer_mut`]) the new display
    /// buffer and re-arm the VDMA channel.
    pub fn commit(&mut self) {
        let back = (self.current_fb + 1) % self.num_fbs;
        self.current_fb = back;
        let lli = unsafe { &mut (*(&raw mut LLI_STORAGE.0))[back] };
        self.dma.restart(lli);
    }

    /// Block until the bridge signals the start of the next vertical blank
    /// (vsync).
    pub fn wait_for_vsync(&mut self) {
        let bridge = MIPI_DSI_BRIDGE::regs();
        bridge.int_clr().write(|w| w.vsync().clear_bit_by_one());
        while !bridge.int_raw().read().vsync().bit_is_set() {}
        bridge.int_clr().write(|w| w.vsync().clear_bit_by_one());
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

#[inline]
fn fround_u32(x: f32) -> u32 {
    (x + 0.5) as u32
}
