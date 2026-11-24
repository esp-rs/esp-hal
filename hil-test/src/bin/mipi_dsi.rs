//! MIPI-DSI tests without a panel
//!
//! The tests check the D-PHY rail and video-mode streaming. They do not need a panel: the DSI
//! host drives the lanes also when nothing is connected. Output on a panel is covered by the
//! `lcd-mipi-dsi` QA test in `examples/qa`.

//% CHIP_FILTER: mipi_dsi_driver_supported
//% FEATURES: unstable esp-alloc

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use core::alloc::Layout;

    use esp_hal::{
        clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
        mipi_dsi::{
            Config,
            DataLanes,
            MipiDsi,
            dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
        },
        peripherals::{MIPI_DSI, PMU, Peripherals, VDMA_CH0},
        time::Duration,
    };

    const H_ACTIVE: u32 = 320;
    const V_ACTIVE: u32 = 240;
    const FRAME_BYTES: usize = H_ACTIVE as usize * V_ACTIVE as usize * 2;
    /// More frames than the DMA link-list ring has items, so the ring must be re-armed.
    const FRAMES: usize = 30;
    const COMMIT_TIMEOUT: Duration = Duration::from_millis(100);

    fn dphy_rail_on() -> bool {
        // `xpd` of PMU LDO channel 3, which supplies `VDD_MIPI_DPHY`.
        PMU::regs().ext_ldo_p0_0p2a().read().bits() & (1 << 8) != 0
    }

    fn new_dsi<'d>(dsi: MIPI_DSI<'d>, channel: VDMA_CH0<'d>) -> MipiDsi<'d> {
        MipiDsi::new(
            dsi,
            channel,
            Config::default()
                .with_num_data_lanes(DataLanes::_2)
                .with_lane_bit_rate_mbps(1000.0)
                .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                    MipiDsiPhyPllRefclkSclk::Xtal,
                    0,
                )),
        )
        .unwrap()
    }

    fn framebuffer() -> &'static mut [u8] {
        let layout = Layout::from_size_align(FRAME_BYTES, 64).unwrap();
        let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
        assert!(!ptr.is_null());
        unsafe { core::slice::from_raw_parts_mut(ptr, FRAME_BYTES) }
    }

    fn dpi_config() -> DpiConfig {
        DpiConfig {
            virtual_channel: 0,
            pixel_clock_mhz: 48.0,
            dpi_clk_src: DpiClockSource::PllF240m,
            in_color_format: ColorFormat::Rgb565,
            out_color_format: ColorFormat::Rgb565,
            timing: FrameTiming {
                h_active: H_ACTIVE,
                hsw: 10,
                hbp: 120,
                hfp: 120,
                v_active: V_ACTIVE,
                vsw: 1,
                vbp: 20,
                vfp: 20,
            },
        }
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn dphy_rail_follows_the_driver(mut p: Peripherals) {
        // The PMU keeps the rail state over the reset between tests, so the start state is not
        // checked. `MipiDsi::new` returns only after the D-PHY PLL locks.
        for _ in 0..2 {
            let dsi = new_dsi(p.MIPI_DSI.reborrow(), p.VDMA_CH0.reborrow());
            assert!(dphy_rail_on());
            drop(dsi);
            assert!(!dphy_rail_on());
        }
    }

    #[test]
    fn dpi_streams_until_dropped(mut p: Peripherals) {
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
        let framebuffers = [framebuffer(), framebuffer()];

        // The second run checks that the drop releases the DMA channel and its interrupt.
        for _ in 0..2 {
            let dsi = new_dsi(p.MIPI_DSI.reborrow(), p.VDMA_CH0.reborrow());
            let mut dpi = dsi.dpi(dpi_config(), &framebuffers).unwrap();
            for _ in 0..FRAMES {
                dpi.commit();
                dpi.wait_for_commit(COMMIT_TIMEOUT).unwrap();
            }
            drop(dpi);
            assert!(!dphy_rail_on());
        }
    }
}
