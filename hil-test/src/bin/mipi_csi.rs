//! MIPI-CSI tests without a streaming camera
//!
//! No frame arrives on the CSI lanes: either no camera is connected, or its sensor does not
//! stream. The tests check the D-PHY rail, configuration errors, capture timeouts, the shutdown
//! of an armed capture and stream, and a dropped `next_frame_async` future. Capture with a camera
//! is covered by `examples/qa/src/bin/mipi-csi-sc2336-first-frame.rs`.

//% CHIP_FILTER: mipi_csi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use core::{
        future::Future,
        pin::pin,
        task::{Context, Waker},
    };

    use esp_hal::{
        Blocking,
        mipi_csi::{
            CaptureConfig,
            CaptureConfigError,
            CapturePixelFormat,
            Config,
            ConfigError,
            CsiCapture,
            DmaAxiConfig,
            MipiCsi,
            ReceiveError,
            StreamError,
        },
        peripherals::{MIPI_CSI, PMU, Peripherals, VDMA_CH1},
        time::{Duration, Instant},
    };
    use static_cell::StaticCell;

    const WIDTH: u16 = 64;
    const HEIGHT: u16 = 8;
    const FRAME_BYTES: usize = WIDTH as usize * HEIGHT as usize;
    const TIMEOUT: Duration = Duration::from_millis(20);
    const SENTINEL: u8 = 0xA5;

    #[repr(C, align(64))]
    struct Frame([u8; FRAME_BYTES]);

    static STREAM_BUFFERS: StaticCell<[Frame; 2]> = StaticCell::new();

    fn stream_buffers() -> [&'static mut [u8]; 2] {
        let [a, b] = STREAM_BUFFERS.init([
            Frame([SENTINEL; FRAME_BYTES]),
            Frame([SENTINEL; FRAME_BYTES]),
        ]);
        [&mut a.0, &mut b.0]
    }

    fn untouched(buffer: &[u8]) -> bool {
        buffer.iter().all(|byte| *byte == SENTINEL)
    }

    fn dphy_rail_on() -> bool {
        // `xpd` of PMU LDO channel 3, which supplies `VDD_MIPI_DPHY`.
        PMU::regs().ext_ldo_p0_0p2a().read().bits() & (1 << 8) != 0
    }

    fn capture<'d>(csi: MIPI_CSI<'d>, channel: VDMA_CH1<'d>) -> CsiCapture<'d, Blocking> {
        MipiCsi::new(csi, Config::default())
            .unwrap()
            .with_dma(
                channel,
                CaptureConfig::default()
                    .with_frame_width(WIDTH)
                    .with_frame_height(HEIGHT)
                    .with_pixel_format(CapturePixelFormat::Raw8)
                    .with_timeout(TIMEOUT),
            )
            .unwrap()
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn dphy_rail_follows_the_driver(mut p: Peripherals) {
        // Without PSRAM init, the rail is still on from boot, so the start state is not checked.
        for _ in 0..2 {
            let csi = MipiCsi::new(p.MIPI_CSI.reborrow(), Config::default()).unwrap();
            assert!(dphy_rail_on());
            drop(csi);
            assert!(!dphy_rail_on());
        }
    }

    #[test]
    fn invalid_configurations_are_rejected(mut p: Peripherals) {
        let lane_rate = MipiCsi::new(
            p.MIPI_CSI.reborrow(),
            Config::default().with_lane_bit_rate_mbps(50),
        );
        assert_eq!(lane_rate.err(), Some(ConfigError::InvalidLaneBitRate));
        assert!(!dphy_rail_on());

        let size = MipiCsi::new(p.MIPI_CSI.reborrow(), Config::default())
            .unwrap()
            .with_dma(
                p.VDMA_CH1.reborrow(),
                CaptureConfig::default().with_frame_width(0),
            );
        assert_eq!(size.err(), Some(CaptureConfigError::InvalidFrameDimensions));

        let axi = MipiCsi::new(p.MIPI_CSI.reborrow(), Config::default())
            .unwrap()
            .with_dma(
                p.VDMA_CH1.reborrow(),
                CaptureConfig::default().with_dma_axi_config(DmaAxiConfig {
                    dst_outstanding_requests: 0,
                    ..DmaAxiConfig::DEFAULT
                }),
            );
        assert_eq!(axi.err(), Some(CaptureConfigError::InvalidDmaAxiConfig));
        assert!(!dphy_rail_on());
    }

    #[test]
    fn receive_times_out_without_a_frame(p: Peripherals) {
        let mut capture = capture(p.MIPI_CSI, p.VDMA_CH1);
        let mut buffer = Frame([SENTINEL; FRAME_BYTES]);

        let start = Instant::now();
        let result = capture.receive(&mut buffer.0);
        let waited = start.elapsed();
        assert!(matches!(result, Err(ReceiveError::Timeout(_))));
        assert!(waited >= TIMEOUT && waited < TIMEOUT + Duration::from_millis(10));
        assert!(untouched(&buffer.0));

        let result = capture.receive_with_interrupt(&mut buffer.0, || {});
        assert!(matches!(result, Err(ReceiveError::Timeout(_))));
        assert!(!capture.completion_interrupt_report().fired);
        assert!(untouched(&buffer.0));
    }

    #[test]
    fn stream_times_out_and_restarts(p: Peripherals) {
        let capture = capture(p.MIPI_CSI, p.VDMA_CH1);
        let Ok(mut stream) = capture.into_stream(stream_buffers()) else {
            panic!("stream did not start");
        };

        let start = Instant::now();
        let result = stream.next_frame(TIMEOUT).map(|frame| frame.sequence());
        assert_eq!(result, Err(StreamError::Timeout));
        assert!(start.elapsed() >= TIMEOUT);
        let stats = stream.stats();
        assert_eq!((stats.completed, stats.errored, stats.delivered), (0, 0, 0));

        let (capture, buffers) = stream.stop();
        assert!(buffers.iter().all(|buffer| untouched(buffer)));
        let Ok(stream) = capture.into_stream(buffers) else {
            panic!("stream did not restart");
        };
        let (mut capture, mut buffers) = stream.stop();
        let result = capture.receive(buffers[0]);
        assert!(matches!(result, Err(ReceiveError::Timeout(_))));
        assert!(buffers.iter_mut().all(|buffer| untouched(buffer)));
    }

    #[test]
    fn dropped_next_frame_async_leaves_the_stream(p: Peripherals) {
        let capture = capture(p.MIPI_CSI, p.VDMA_CH1).into_async();
        let Ok(mut stream) = capture.into_stream(stream_buffers()) else {
            panic!("stream did not start");
        };

        {
            let mut future = pin!(stream.next_frame_async());
            let mut cx = Context::from_waker(Waker::noop());
            assert!(future.as_mut().poll(&mut cx).is_pending());
            assert!(future.as_mut().poll(&mut cx).is_pending());
        }
        let result = stream.next_frame(TIMEOUT).map(|frame| frame.sequence());
        assert_eq!(result, Err(StreamError::Timeout));
        assert_eq!(stream.stats().delivered, 0);

        let (capture, buffers) = stream.stop();
        assert!(buffers.iter().all(|buffer| untouched(buffer)));
        let mut capture = capture.into_blocking();
        let result = capture.receive(buffers[0]);
        assert!(matches!(result, Err(ReceiveError::Timeout(_))));
    }
}
