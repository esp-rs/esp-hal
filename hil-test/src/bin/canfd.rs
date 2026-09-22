//! CAN FD Tests
//!
//! The two common test pins are wired together. The single-node tests drive
//! one pin push-pull and listen on the other. Tests with two nodes, or with a
//! fault injector, on the wire use open-drain outputs with the internal
//! pull-ups, which is a minimal CAN bus with no transceivers.

//% CHIP_FILTER: canfd_driver_supported
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    Blocking,
    DriverMode,
    canfd::{
        CanFd,
        CanFdInterrupt,
        ClockSource,
        Config,
        ConfigError,
        Error,
        ExtendedId,
        Frame,
        FrameError,
        FrameKinds,
        Id,
        MaskFilter,
        MaskFilterConfig,
        Mode,
        RangeFilterConfig,
        StandardId,
        Timing,
        TxBufferState,
    },
    rtc_cntl::WakeLock,
    timer::timg::TimerGroup,
};
use hil_test as _;

struct Context<D: DriverMode> {
    canfd: CanFd<'static, D>,
}

// The blocking driver has to be movable into a task and shareable through a
// static. The async driver is `!Send` by design; see `esp_hal::Async`.
const _: () = {
    const fn send_and_sync<T: Send + Sync>() {}
    const fn sync<T: Sync>() {}
    send_and_sync::<CanFd<'static, Blocking>>();
    sync::<CanFd<'static, Async>>();
};

fn std(id: u16) -> StandardId {
    StandardId::new(id).unwrap()
}

fn ext(id: u32) -> ExtendedId {
    ExtendedId::new(id).unwrap()
}

/// Loopback plus self test lets one controller exercise itself without an
/// acknowledging peer.
fn config() -> Config {
    Config::default().with_mode(Mode::LoopbackSelfTest)
}

/// Waits for a queued frame to leave the in-progress states.
fn wait_tx<D: DriverMode>(canfd: &CanFd<'static, D>, index: u8) -> TxBufferState {
    // The slowest bit rate these tests configure needs over a hundred
    // milliseconds for one frame.
    let deadline = esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(500);
    loop {
        let state = canfd.tx_buffer_state(index);
        if !matches!(
            state,
            TxBufferState::Ready | TxBufferState::InProgress | TxBufferState::AbortInProgress
        ) {
            return state;
        }
        assert!(
            esp_hal::time::Instant::now() < deadline,
            "TX buffer {} never finished, stuck in {:?}",
            index,
            state
        );
    }
}

/// Sends `frame` and returns what came back out of the RX buffer.
fn round_trip(canfd: &mut CanFd<'static, Blocking>, frame: &Frame) -> Frame {
    canfd.flush_rx();
    let index = canfd.transmit(frame).unwrap();
    assert_eq!(wait_tx(canfd, index), TxBufferState::Ok, "transmit failed");
    canfd.receive().unwrap()
}

/// Sends a one-byte frame and reports whether the filters let it through.
fn accepted(canfd: &mut CanFd<'static, Blocking>, id: impl Into<Id>) -> bool {
    canfd.flush_rx();
    let frame = Frame::new(id, &[0xAA]).unwrap();
    let index = canfd.transmit(&frame).unwrap();
    assert_eq!(wait_tx(canfd, index), TxBufferState::Ok, "transmit failed");
    canfd.rx_frame_count() > 0
}

#[embedded_test::tests(default_timeout = 3)]
mod blocking_tests {
    use esp_hal::time::{Duration, Instant};

    use super::*;

    /// Checks the timestamp counter advances at `rate` counts per second,
    /// timed against the system timer.
    fn measure_rate(canfd: &CanFd<'static, Blocking>, rate: u32) {
        let rate = u64::from(rate);
        let start = canfd.timestamp();
        let began = Instant::now();
        while began.elapsed() < Duration::from_millis(5) {}
        let elapsed = began.elapsed();
        let ticks = canfd.timestamp() - start;

        let expected = rate * elapsed.as_micros() / 1_000_000;
        assert!(
            ticks.abs_diff(expected) < expected / 10,
            "counter advanced {} in {} us, expected about {}",
            ticks,
            elapsed.as_micros(),
            expected
        );
    }

    #[init]
    fn init() -> Context<Blocking> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let mut canfd = CanFd::new(peripherals.TWAI0, config())
            .unwrap()
            .with_rx(rx)
            .with_tx(tx);
        canfd.start().unwrap();

        Context { canfd }
    }

    #[test]
    fn reports_the_ctu_can_fd_core(ctx: Context<Blocking>) {
        assert!(ctx.canfd.identity().is_ctu_can_fd());
        assert_eq!(ctx.canfd.tx_buffer_count(), 4);
    }

    #[test]
    fn classic_frames_round_trip(mut ctx: Context<Blocking>) {
        for (id, payload) in [
            (Id::from(std(0x123)), &b"12345678"[..]),
            (Id::from(ext(0x1AB_CDEF)), &b"abcd"[..]),
            (Id::from(std(0x7FF)), &[][..]),
        ] {
            let sent = Frame::new(id, payload).unwrap();
            let got = round_trip(&mut ctx.canfd, &sent);

            assert_eq!(got.id(), id);
            assert!(!got.is_fd());
            assert_eq!(got.payload(), payload);
        }
    }

    #[test]
    fn fd_frames_round_trip_at_every_payload_length(mut ctx: Context<Blocking>) {
        for len in [0usize, 1, 8, 12, 16, 20, 24, 32, 48, 64] {
            let mut payload = [0u8; 64];
            for (i, byte) in payload.iter_mut().enumerate().take(len) {
                *byte = i as u8;
            }

            let sent = Frame::new_fd(std(0x100 + len as u16), &payload[..len])
                .unwrap()
                .with_bit_rate_switch(true);
            let got = round_trip(&mut ctx.canfd, &sent);

            assert!(got.is_fd(), "len {}", len);
            assert!(got.is_bit_rate_switched(), "len {}", len);
            assert_eq!(got.payload(), &payload[..len], "len {}", len);
        }
    }

    #[test]
    fn extended_id_fd_frame_round_trips(mut ctx: Context<Blocking>) {
        let sent = Frame::new_fd(ext(0x1FF_FFFF), &[0x5A; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let got = round_trip(&mut ctx.canfd, &sent);

        assert_eq!(got.id(), ext(0x1FF_FFFF).into());
        assert!(got.is_extended());
        assert!(got.is_fd());
        assert_eq!(got.payload(), &[0x5A; 64]);
    }

    #[test]
    fn max_identifiers_round_trip(mut ctx: Context<Blocking>) {
        for id in [Id::from(StandardId::MAX), Id::from(ExtendedId::MAX)] {
            let got = round_trip(&mut ctx.canfd, &Frame::new(id, &[0x5A]).unwrap());
            assert_eq!(got.id(), id);
        }
    }

    #[test]
    fn frames_speak_embedded_can(mut ctx: Context<Blocking>) {
        // Code written against the `embedded-can` traits is entitled to the
        // promise that a frame holds at most 8 bytes.
        use embedded_can::Frame as _;
        use esp_hal::canfd::ClassicFrame;

        fn eight_byte_consumer<F: embedded_can::Frame>(frame: &F) -> [u8; 8] {
            let data = frame.data();
            let mut bytes = [0u8; 8];
            bytes[..data.len()].copy_from_slice(data);
            bytes
        }

        let standard = std(0x123);
        let sent = ClassicFrame::new(standard, &[1, 2, 3]).unwrap();
        let got = ClassicFrame::try_from(round_trip(&mut ctx.canfd, &sent.into())).unwrap();
        assert_eq!(embedded_can::Frame::id(&got), Id::Standard(standard));
        assert!(got.is_standard());
        assert!(got.is_data_frame());
        assert_eq!(got.dlc(), 3);
        assert_eq!(got.data(), &[1, 2, 3]);
        assert_eq!(eight_byte_consumer(&got), [1, 2, 3, 0, 0, 0, 0, 0]);
        assert!(!got.is_fd());

        assert!(ClassicFrame::new(standard, &[0; 9]).is_none());

        let extended = ext(0x1AB_CDEF);
        let request = ClassicFrame::new_remote(extended, 4).unwrap();
        let got = ClassicFrame::try_from(round_trip(&mut ctx.canfd, &request.into())).unwrap();
        assert_eq!(embedded_can::Frame::id(&got), Id::Extended(extended));
        assert!(embedded_can::Frame::is_extended(&got));
        assert!(got.is_remote_frame());
        assert_eq!(got.dlc(), 4);
        assert_eq!(got.data(), &[]);
        assert_eq!(eight_byte_consumer(&got), [0; 8]);
        assert!(ClassicFrame::new_remote(extended, 9).is_none());

        // A received FD frame must never reach that consumer.
        let fd = Frame::new_fd(std(0x124), &[0x5A; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let got = round_trip(&mut ctx.canfd, &fd);
        assert_eq!(
            ClassicFrame::try_from(got).unwrap_err(),
            FrameError::NotClassic
        );
    }

    #[test]
    fn timing_rejects_what_the_hardware_cannot_sample(_ctx: Context<Blocking>) {
        use esp_hal::canfd::{FD_TIMING_LIMITS, NOMINAL_TIMING_LIMITS};

        // TRM 38.3.7.7 states its limits in system clock periods, so each
        // parameter here is in range and the combination is still invalid.
        let too_short_phase2 = Timing {
            baud_rate_prescaler: 1,
            propagation_segment: 5,
            phase_segment_1: 5,
            phase_segment_2: 1,
            sync_jump_width: 1,
        };
        assert!(!too_short_phase2.is_valid(&NOMINAL_TIMING_LIMITS));
        assert!(!too_short_phase2.is_valid(&FD_TIMING_LIMITS));

        let ok = Timing {
            baud_rate_prescaler: 2,
            ..too_short_phase2
        };
        assert!(ok.is_valid(&NOMINAL_TIMING_LIMITS));

        let too_short_before_sample = Timing {
            baud_rate_prescaler: 1,
            propagation_segment: 1,
            phase_segment_1: 0,
            phase_segment_2: 2,
            sync_jump_width: 1,
        };
        assert!(!too_short_before_sample.is_valid(&NOMINAL_TIMING_LIMITS));
    }

    #[test]
    fn frames_report_their_data_length_code(_ctx: Context<Blocking>) {
        for (len, dlc) in [
            (0, 0),
            (8, 8),
            (9, 9),
            (12, 9),
            (13, 10),
            (24, 12),
            (25, 13),
            (32, 13),
            (33, 14),
            (48, 14),
            (49, 15),
            (64, 15),
        ] {
            let frame = Frame::new_fd(std(1), &[0; 64][..len]).unwrap();
            assert_eq!(frame.dlc(), dlc, "len {}", len);
            assert_eq!(frame.len(), len);
        }
        assert_eq!(Frame::new_request(std(1), 5).unwrap().dlc(), 5);
        assert_eq!(
            Frame::new_fd(std(1), &[0; 65]).unwrap_err(),
            FrameError::PayloadTooLong
        );
    }

    #[test]
    fn mask_filter_accepts_and_rejects(mut ctx: Context<Blocking>) {
        // Accept 0x220..=0x22F.
        ctx.canfd
            .set_mask_filter(
                MaskFilter::A,
                &MaskFilterConfig {
                    id: 0x220,
                    mask: 0x7F0,
                    extended: false,
                    accepts: FrameKinds::ALL,
                },
            )
            .unwrap();
        ctx.canfd.disable_mask_filter(MaskFilter::B);
        ctx.canfd.disable_mask_filter(MaskFilter::C);
        ctx.canfd.disable_range_filter();

        assert!(accepted(&mut ctx.canfd, std(0x225)), "0x225 must pass");
        assert!(!accepted(&mut ctx.canfd, std(0x235)), "0x235 must not");

        ctx.canfd.accept_all();
    }

    #[test]
    fn range_filter_accepts_and_rejects(mut ctx: Context<Blocking>) {
        ctx.canfd.disable_mask_filter(MaskFilter::A);
        ctx.canfd
            .set_range_filter(&RangeFilterConfig {
                low: 0x400,
                high: 0x40F,
                extended: false,
                accepts: FrameKinds::ALL,
            })
            .unwrap();

        assert!(accepted(&mut ctx.canfd, std(0x400)), "0x400 must pass");
        assert!(accepted(&mut ctx.canfd, std(0x408)), "0x408 must pass");
        assert!(accepted(&mut ctx.canfd, std(0x40F)), "0x40F must pass");
        assert!(!accepted(&mut ctx.canfd, std(0x3FF)), "0x3FF must not");
        assert!(!accepted(&mut ctx.canfd, std(0x410)), "0x410 must not");

        ctx.canfd.accept_all();
    }

    #[test]
    fn filters_can_reject_fd_frames_alone(mut ctx: Context<Blocking>) {
        ctx.canfd
            .set_mask_filter(
                MaskFilter::A,
                &MaskFilterConfig {
                    id: 0,
                    mask: 0,
                    extended: false,
                    accepts: FrameKinds {
                        classic_standard: true,
                        classic_extended: true,
                        fd_standard: false,
                        fd_extended: false,
                    },
                },
            )
            .unwrap();
        ctx.canfd.disable_range_filter();

        assert!(accepted(&mut ctx.canfd, std(0x123)), "classic must pass");

        ctx.canfd.flush_rx();
        let fd = Frame::new_fd(std(0x123), &[0; 12]).unwrap();
        let index = ctx.canfd.transmit(&fd).unwrap();
        assert_eq!(wait_tx(&ctx.canfd, index), TxBufferState::Ok);
        assert_eq!(ctx.canfd.rx_frame_count(), 0, "FD frame must be filtered");

        ctx.canfd.accept_all();
    }

    #[test]
    fn the_timestamp_counter_runs_at_the_rate_it_reports(mut ctx: Context<Blocking>) {
        let resolution = ctx.canfd.start_timestamp_timer(1_000_000).unwrap();
        assert_eq!(resolution, 1_000_000);
        assert_eq!(ctx.canfd.timestamp_bit_width(), 32);
        measure_rate(&ctx.canfd, resolution);

        // A restart at a lower divider can stall the prescaler for up to
        // 65536 clock periods, about a third of the time; alternate to catch it.
        for round in 0..6 {
            let wanted = if round % 2 == 0 { 2_000_000 } else { 1_000_000 };

            ctx.canfd.stop_timestamp_timer();
            let got = ctx.canfd.start_timestamp_timer(wanted).unwrap();
            assert_eq!(got, wanted);
            assert!(
                ctx.canfd.timestamp() < u64::from(wanted) / 100,
                "restarting the timer did not clear the counter"
            );
            measure_rate(&ctx.canfd, got);
        }

        ctx.canfd.stop_timestamp_timer();
    }

    #[test]
    fn timestamps_are_captured(mut ctx: Context<Blocking>) {
        ctx.canfd.start_timestamp_timer(1_000_000).unwrap();

        // Let the counter climb past anything another frame field could hold.
        let deadline = Instant::now() + Duration::from_millis(500);
        while ctx.canfd.timestamp() < 0x1_0000 {
            assert!(
                Instant::now() < deadline,
                "timestamp counter never reached a usable value"
            );
        }

        let before = ctx.canfd.timestamp();
        let got = round_trip(
            &mut ctx.canfd,
            &Frame::new(std(0x555), &[1, 2, 3, 4]).unwrap(),
        );
        let after = ctx.canfd.timestamp();

        assert!(
            got.timestamp() > before && got.timestamp() <= after,
            "timestamp {} outside the window [{}, {}] of the transfer",
            got.timestamp(),
            before,
            after
        );

        ctx.canfd.stop_timestamp_timer();
    }

    #[test]
    fn a_clean_bus_reports_no_errors(mut ctx: Context<Blocking>) {
        round_trip(&mut ctx.canfd, &Frame::new(std(0x321), &[7; 8]).unwrap());

        assert_eq!(ctx.canfd.error_counters(), (0, 0));
        assert_eq!(ctx.canfd.error_state(), esp_hal::canfd::ErrorState::Active);
    }

    #[test]
    fn leaving_the_bus_waits_out_a_frame_at_a_low_bit_rate(mut ctx: Context<Blocking>) {
        // 5 kbit/s: a 64-byte frame takes over a hundred milliseconds.
        let slow = Timing {
            baud_rate_prescaler: 200,
            propagation_segment: 40,
            phase_segment_1: 24,
            phase_segment_2: 15,
            sync_jump_width: 8,
        };
        ctx.canfd
            .apply_config(&config().with_nominal_timing(slow))
            .unwrap();
        ctx.canfd.start().unwrap();

        let frame = Frame::new_fd(std(0x100), &[0xA5; 64]).unwrap();
        let index = ctx.canfd.transmit(&frame).unwrap();

        let deadline = Instant::now() + Duration::from_millis(100);
        while ctx.canfd.tx_buffer_state(index) != TxBufferState::InProgress {
            assert!(Instant::now() < deadline, "the frame never started");
        }

        let began = Instant::now();
        ctx.canfd
            .stop()
            .expect("leaving the bus cut the frame short");
        let waited = began.elapsed();

        assert!(
            waited > Duration::from_millis(60),
            "stopped after {} us, which is too soon to have waited for the frame",
            waited.as_micros()
        );
        assert!(
            !matches!(
                ctx.canfd.tx_buffer_state(index),
                TxBufferState::Ready | TxBufferState::InProgress | TxBufferState::AbortInProgress
            ),
            "the buffer was still in flight after stopping"
        );
    }

    #[test]
    fn the_chip_stays_awake_exactly_while_the_controller_is_on_the_bus(mut ctx: Context<Blocking>) {
        assert!(ctx.canfd.is_started());
        assert!(WakeLock::is_active(), "started in init, yet asleep");

        ctx.canfd.stop().unwrap();
        assert!(!ctx.canfd.is_started());
        assert!(!WakeLock::is_active(), "stopped, yet held awake");

        ctx.canfd.start().unwrap();
        assert!(ctx.canfd.is_started());
        assert!(WakeLock::is_active(), "restarted, yet asleep");

        ctx.canfd.apply_config(&config()).unwrap();
        assert!(!ctx.canfd.is_started());
        assert!(!WakeLock::is_active(), "reconfigured, yet held awake");

        ctx.canfd.start().unwrap();
        ctx.canfd.start().unwrap();
        ctx.canfd.stop().unwrap();
        assert!(
            !WakeLock::is_active(),
            "a repeated start left a lock behind"
        );

        ctx.canfd.start().unwrap();
        drop(ctx.canfd);
        assert!(!WakeLock::is_active(), "dropped, yet held awake");
    }

    #[test]
    fn the_transceiver_mode_cannot_change_once_the_pins_are_bound(mut ctx: Context<Blocking>) {
        assert_eq!(
            ctx.canfd.apply_config(&config().with_no_transceiver(true)),
            Err(ConfigError::TransceiverModeLocked)
        );
        assert!(!ctx.canfd.config().no_transceiver());

        ctx.canfd
            .apply_config(&config().with_retransmit_limit(1))
            .unwrap();
    }

    #[test]
    fn the_slowest_bit_rate_still_joins_the_bus(mut ctx: Context<Blocking>) {
        // Every field at its maximum from the 40 MHz crystal is about 741
        // bit/s, where joining takes over a hundred milliseconds.
        let slowest = Timing {
            baud_rate_prescaler: 255,
            propagation_segment: 127,
            phase_segment_1: 63,
            phase_segment_2: 63,
            sync_jump_width: 31,
        };
        ctx.canfd
            .apply_config(
                &config()
                    .with_clock_source(ClockSource::Xtal)
                    .with_nominal_timing(slowest),
            )
            .unwrap();

        let began = Instant::now();
        ctx.canfd
            .start()
            .expect("the controller gave up before the bus could integrate");

        assert!(
            began.elapsed() > Duration::from_millis(10),
            "joined in {} us, which is faster than this bit rate allows",
            began.elapsed().as_micros()
        );
    }

    #[test]
    fn changing_the_clock_source_stops_the_timestamp_timer(mut ctx: Context<Blocking>) {
        let resolution = ctx.canfd.start_timestamp_timer(1_000_000).unwrap();
        measure_rate(&ctx.canfd, resolution);

        ctx.canfd
            .apply_config(&config().with_clock_source(ClockSource::Xtal))
            .unwrap();

        let stopped_at = ctx.canfd.timestamp();
        let began = Instant::now();
        while began.elapsed() < Duration::from_millis(5) {}
        assert_eq!(
            ctx.canfd.timestamp(),
            stopped_at,
            "the timer kept counting across a clock source change"
        );

        let resolution = ctx.canfd.start_timestamp_timer(1_000_000).unwrap();
        measure_rate(&ctx.canfd, resolution);
        ctx.canfd.stop_timestamp_timer();
    }

    #[test]
    fn transmitting_while_off_the_bus_is_refused(mut ctx: Context<Blocking>) {
        let frame = Frame::new(std(0x1A5), &[9]).unwrap();

        // A stopped controller ignores the arming command, so success here
        // would be a lost frame. Every path off the bus is covered.
        ctx.canfd.stop().unwrap();
        assert_eq!(ctx.canfd.transmit(&frame), Err(Error::ControllerStopped));

        let (_, mut tx) = ctx.canfd.split();
        assert_eq!(tx.transmit(&frame), Err(Error::ControllerStopped));

        ctx.canfd.apply_config(&config()).unwrap();
        assert_eq!(ctx.canfd.transmit(&frame), Err(Error::ControllerStopped));

        ctx.canfd.start().unwrap();
        let got = round_trip(&mut ctx.canfd, &frame);
        assert_eq!(got.id(), std(0x1A5).into());
        assert_eq!(got.payload(), &[9]);
    }

    #[test]
    fn filter_identifiers_that_do_not_fit_the_format_are_refused(mut ctx: Context<Blocking>) {
        // The filter registers hold 11 or 29 bits; a wider value would be
        // masked on the way in and match something else.
        let too_large = MaskFilterConfig {
            id: 0x800,
            mask: 0x7FF,
            extended: false,
            accepts: FrameKinds::ALL,
        };
        assert_eq!(
            ctx.canfd.set_mask_filter(MaskFilter::A, &too_large),
            Err(ConfigError::FilterIdTooLarge)
        );
        assert_eq!(
            ctx.canfd.set_mask_filter(
                MaskFilter::A,
                &MaskFilterConfig {
                    mask: 0x800,
                    ..too_large
                }
            ),
            Err(ConfigError::FilterIdTooLarge)
        );
        assert_eq!(
            ctx.canfd.set_range_filter(&RangeFilterConfig {
                low: 0x400,
                high: 0x2000_0000,
                extended: true,
                accepts: FrameKinds::ALL,
            }),
            Err(ConfigError::FilterIdTooLarge)
        );

        ctx.canfd
            .set_mask_filter(
                MaskFilter::A,
                &MaskFilterConfig {
                    id: 0x7FF,
                    mask: 0x7FF,
                    extended: false,
                    accepts: FrameKinds::ALL,
                },
            )
            .unwrap();
        ctx.canfd.disable_mask_filter(MaskFilter::B);
        ctx.canfd.disable_mask_filter(MaskFilter::C);
        ctx.canfd.disable_range_filter();

        assert!(accepted(&mut ctx.canfd, std(0x7FF)), "0x7FF must pass");
        assert!(!accepted(&mut ctx.canfd, std(0x000)), "0x000 must not");

        ctx.canfd.accept_all();
    }

    #[test]
    fn tx_buffer_indices_the_hardware_does_not_have_are_refused(mut ctx: Context<Blocking>) {
        let frame = Frame::new(std(0x111), &[1]).unwrap();
        let index = ctx.canfd.transmit(&frame).unwrap();
        assert_eq!(wait_tx(&ctx.canfd, index), TxBufferState::Ok);

        let count = ctx.canfd.tx_buffer_count();
        for index in [count, count + 1, 2 * count, u8::MAX] {
            assert_eq!(
                ctx.canfd.tx_buffer_state(index),
                TxBufferState::NotExist,
                "index {} reported a buffer that does not exist",
                index
            );

            // The commands must not land on a real buffer either.
            ctx.canfd.abort_transmit(index);
            ctx.canfd.release_tx_buffer(index);
            ctx.canfd.set_tx_priority(index, 7);
            assert_eq!(
                ctx.canfd.tx_buffer_state(0),
                TxBufferState::Ok,
                "a command with index {} reached buffer 0",
                index
            );
        }
    }

    #[test]
    fn the_secondary_sample_point_leaves_room_for_the_measured_delay(mut ctx: Context<Blocking>) {
        // The default data phase is four quanta of ten clock periods, so an
        // offset of 16 quanta lands exactly on the four-bit-time limit of
        // TRM 38.3.7.3, and the measured delay comes on top.
        assert_eq!(
            ctx.canfd
                .apply_config(&config().with_secondary_sample_point_offset(16)),
            Err(ConfigError::UnsupportedSecondarySamplePoint)
        );
        assert_eq!(ctx.canfd.config().secondary_sample_point_offset(), None);

        ctx.canfd
            .apply_config(&config().with_secondary_sample_point_offset(15))
            .unwrap();
        assert_eq!(ctx.canfd.config().secondary_sample_point_offset(), Some(15));
    }
}

/// Two controllers on one wire, so a peer acknowledges frames and arbitrates.
#[embedded_test::tests(default_timeout = 5)]
mod two_node_tests {
    use super::*;

    struct Nodes {
        node0: CanFd<'static, Blocking>,
        node1: CanFd<'static, Blocking>,
    }

    #[init]
    fn init() -> Nodes {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        // Each controller needs both signals on its own end of the wire.
        let (pin0, pin1) = hil_test::common_test_pins!(peripherals);
        let (rx0, tx0) = unsafe { pin0.split() };
        let (rx1, tx1) = unsafe { pin1.split() };

        let config = Config::default().with_no_transceiver(true);

        let mut node0 = CanFd::new(peripherals.TWAI0, config)
            .unwrap()
            .with_rx(rx0)
            .with_tx(tx0);
        let mut node1 = CanFd::new(peripherals.TWAI1, config)
            .unwrap()
            .with_rx(rx1)
            .with_tx(tx1);

        node0.start().unwrap();
        node1.start().unwrap();

        Nodes { node0, node1 }
    }

    /// Sends from `from`, expects it on `to`, and returns what arrived.
    fn cross(
        from: &mut CanFd<'static, Blocking>,
        to: &mut CanFd<'static, Blocking>,
        frame: &Frame,
    ) -> Frame {
        to.flush_rx();
        let index = from.transmit(frame).unwrap();
        assert_eq!(wait_tx(from, index), TxBufferState::Ok, "transmit failed");
        to.receive().unwrap()
    }

    #[test]
    fn frames_cross_in_both_directions(mut ctx: Nodes) {
        // TX OK here means the peer acknowledged.
        let sent = Frame::new(std(0x301), &[9, 8, 7, 6, 5, 4, 3, 2]).unwrap();
        let got = cross(&mut ctx.node0, &mut ctx.node1, &sent);
        assert_eq!(got.id(), std(0x301).into());
        assert_eq!(got.payload(), sent.payload());

        let sent = Frame::new(std(0x401), &[0xC0, 0xFF, 0xEE]).unwrap();
        let got = cross(&mut ctx.node1, &mut ctx.node0, &sent);
        assert_eq!(got.id(), std(0x401).into());
        assert_eq!(got.payload(), sent.payload());
    }

    #[test]
    fn fd_frames_cross_at_the_data_bit_rate(mut ctx: Nodes) {
        let sent = Frame::new_fd(ext(0x1AB_CDEF), &[0x3C; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let got = cross(&mut ctx.node0, &mut ctx.node1, &sent);

        assert_eq!(got.id(), ext(0x1AB_CDEF).into());
        assert!(got.is_fd());
        assert!(got.is_bit_rate_switched());
        assert_eq!(got.payload(), &[0x3C; 64]);
    }

    #[test]
    fn simultaneous_transmissions_arbitrate(mut ctx: Nodes) {
        ctx.node0.flush_rx();
        ctx.node1.flush_rx();

        // No handler is registered, so listening only latches the flag.
        ctx.node0.clear_interrupts(CanFdInterrupt::ArbitrationLost);
        ctx.node1.clear_interrupts(CanFdInterrupt::ArbitrationLost);
        ctx.node0.listen(CanFdInterrupt::ArbitrationLost);
        ctx.node1.listen(CanFdInterrupt::ArbitrationLost);

        // Both nodes become ready while the filler occupies the bus, so they
        // start together at the next intermission and arbitrate.
        let filler = Frame::new(std(0x7FF), &[0; 8]).unwrap();
        let filler_idx = ctx.node0.transmit(&filler).unwrap();

        let low = Frame::new(std(0x100), &[1]).unwrap();
        let high = Frame::new(std(0x200), &[2]).unwrap();
        let i0 = ctx.node0.transmit(&low).unwrap();
        let i1 = ctx.node1.transmit(&high).unwrap();

        assert_eq!(wait_tx(&ctx.node0, filler_idx), TxBufferState::Ok);
        assert_eq!(wait_tx(&ctx.node0, i0), TxBufferState::Ok);
        assert_eq!(wait_tx(&ctx.node1, i1), TxBufferState::Ok);

        assert_eq!(ctx.node1.receive().unwrap().id(), std(0x7FF).into());
        assert_eq!(ctx.node1.receive().unwrap().id(), std(0x100).into());
        assert_eq!(ctx.node0.receive().unwrap().id(), std(0x200).into());

        // The higher identifier is the one that backed off.
        assert!(
            ctx.node1
                .interrupts()
                .contains(CanFdInterrupt::ArbitrationLost),
            "0x200 should have lost arbitration to 0x100"
        );
        assert!(
            !ctx.node0
                .interrupts()
                .contains(CanFdInterrupt::ArbitrationLost),
            "0x100 has the lower identifier and should have won"
        );

        assert_eq!(ctx.node0.error_counters(), (0, 0), "node0");
        assert_eq!(ctx.node1.error_counters(), (0, 0), "node1");
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod async_tests {
    use core::{
        sync::atomic::{AtomicUsize, Ordering},
        task::{Poll, RawWaker, RawWakerVTable, Waker},
    };

    use embassy_time::{Duration, Instant, Timer};

    use super::*;

    static WAKES: AtomicUsize = AtomicUsize::new(0);

    /// A waker belonging to no executor, so a wake can only come from the
    /// driver.
    fn counting_waker() -> Waker {
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        fn wake(_: *const ()) {
            WAKES.fetch_add(1, Ordering::SeqCst);
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, |_| {});

        WAKES.store(0, Ordering::SeqCst);

        unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
    }

    /// The async driver plus a peer that delivers a frame on demand.
    struct AsyncContext {
        canfd: CanFd<'static, Async>,
        peer: CanFd<'static, Blocking>,
    }

    #[init]
    async fn init() -> AsyncContext {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (pin0, pin1) = hil_test::common_test_pins!(peripherals);
        let (rx0, tx0) = unsafe { pin0.split() };
        let (rx1, tx1) = unsafe { pin1.split() };

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut canfd = CanFd::new(peripherals.TWAI0, config().with_no_transceiver(true))
            .unwrap()
            .with_rx(rx0)
            .with_tx(tx0)
            .into_async();
        canfd.start().unwrap();

        let peer_config = Config::default()
            .with_mode(Mode::SelfTest)
            .with_no_transceiver(true);
        let mut peer = CanFd::new(peripherals.TWAI1, peer_config)
            .unwrap()
            .with_rx(rx1)
            .with_tx(tx1);
        peer.start().unwrap();

        AsyncContext { canfd, peer }
    }

    #[test]
    async fn frames_round_trip(mut ctx: AsyncContext) {
        for frame in [
            Frame::new(std(0x201), &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap(),
            Frame::new_fd(std(0x202), &[0x77; 64])
                .unwrap()
                .with_bit_rate_switch(true),
        ] {
            ctx.canfd.flush_rx();
            ctx.canfd.transmit_async(&frame).await.unwrap();

            let got = ctx.canfd.receive_async().await;
            assert_eq!(got.id(), frame.id());
            assert_eq!(got.is_fd(), frame.is_fd());
            assert_eq!(got.payload(), frame.payload());
        }
    }

    #[test]
    async fn receive_async_waits_instead_of_spinning(mut ctx: AsyncContext) {
        ctx.canfd.flush_rx();

        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);
        let mut receive = core::pin::pin!(ctx.canfd.receive_async());

        assert!(
            receive.as_mut().poll(&mut cx).is_pending(),
            "received a frame from an idle bus"
        );
        assert!(WakeLock::is_active(), "waiting for a frame while asleep");

        // A future that wakes itself on every poll is still Pending, so only
        // the wake count tells spinning apart from parking.
        Timer::after(Duration::from_millis(50)).await;

        assert_eq!(
            WAKES.load(Ordering::SeqCst),
            0,
            "the future woke itself while the bus was idle"
        );
        assert!(
            receive.as_mut().poll(&mut cx).is_pending(),
            "received a frame from an idle bus"
        );
    }

    #[test]
    async fn receive_async_wakes_on_the_rx_interrupt(mut ctx: AsyncContext) {
        ctx.canfd.flush_rx();

        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);

        let mut receive = core::pin::pin!(ctx.canfd.receive_async());

        assert!(
            receive.as_mut().poll(&mut cx).is_pending(),
            "receive_async returned before any frame was sent"
        );
        assert_eq!(
            WAKES.load(Ordering::SeqCst),
            0,
            "nothing should have woken the future yet"
        );

        ctx.peer
            .transmit(&Frame::new(std(0x203), &[0xEE; 2]).unwrap())
            .unwrap();

        // This waker belongs to no executor, so the wake can only come from
        // the RX interrupt.
        let deadline = Instant::now() + Duration::from_millis(500);
        while WAKES.load(Ordering::SeqCst) == 0 {
            assert!(
                Instant::now() < deadline,
                "the RX interrupt never woke the future"
            );
        }

        match receive.as_mut().poll(&mut cx) {
            Poll::Ready(frame) => assert_eq!(frame.id(), std(0x203).into()),
            Poll::Pending => panic!("the frame arrived but the future did not complete"),
        }
    }
}

/// Two nodes exchanging real traffic, with the driver under test in async
/// mode.
#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod contention_tests {
    use core::{
        sync::atomic::{AtomicUsize, Ordering},
        task::{Poll, RawWaker, RawWakerVTable, Waker},
    };

    use embassy_time::Timer;
    use esp_hal::time::{Duration, Instant};

    use super::*;

    static WAKES: AtomicUsize = AtomicUsize::new(0);

    fn counting_waker() -> Waker {
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        fn wake(_: *const ()) {
            WAKES.fetch_add(1, Ordering::SeqCst);
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, |_| {});

        WAKES.store(0, Ordering::SeqCst);

        unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
    }

    struct Nodes {
        node: CanFd<'static, Async>,
        peer: CanFd<'static, Blocking>,
    }

    #[init]
    async fn init() -> Nodes {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (pin0, pin1) = hil_test::common_test_pins!(peripherals);
        let (rx0, tx0) = unsafe { pin0.split() };
        let (rx1, tx1) = unsafe { pin1.split() };

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);

        let config = Config::default().with_no_transceiver(true);
        let mut node = CanFd::new(peripherals.TWAI0, config)
            .unwrap()
            .with_rx(rx0)
            .with_tx(tx0)
            .into_async();
        let mut peer = CanFd::new(peripherals.TWAI1, config)
            .unwrap()
            .with_rx(rx1)
            .with_tx(tx1);
        node.start().unwrap();
        peer.start().unwrap();

        Nodes { node, peer }
    }

    /// 5 kbit/s in the arbitration phase: a 64-byte frame takes over a hundred
    /// milliseconds, leaving room to interrupt it.
    fn slow_nominal() -> Config {
        Config::default()
            .with_no_transceiver(true)
            .with_nominal_timing(Timing {
                baud_rate_prescaler: 200,
                propagation_segment: 40,
                phase_segment_1: 24,
                phase_segment_2: 15,
                sync_jump_width: 8,
            })
    }

    /// The same rate in the data phase only. A data phase slower than the
    /// arbitration phase is unusual but legal (TRM 38.3.7.1).
    fn slow_data() -> Config {
        Config::default()
            .with_no_transceiver(true)
            .with_fd_timing(Timing {
                baud_rate_prescaler: 200,
                propagation_segment: 40,
                phase_segment_1: 24,
                phase_segment_2: 15,
                sync_jump_width: 8,
            })
    }

    #[test]
    async fn rx_and_tx_halves_work_at_the_same_time(mut ctx: Nodes) {
        let (mut rx, mut tx) = ctx.node.split();

        let outgoing = Frame::new_fd(std(0x150), &[0x5A; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let mut receive = core::pin::pin!(rx.receive_async());
        let mut transmit = core::pin::pin!(tx.transmit_async(&outgoing));

        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);

        assert!(receive.as_mut().poll(&mut cx).is_pending());
        assert!(transmit.as_mut().poll(&mut cx).is_pending());

        let deadline = Instant::now() + Duration::from_millis(500);
        let mut sent = false;
        loop {
            if !sent && transmit.as_mut().poll(&mut cx).is_ready() {
                sent = true;
                assert_eq!(ctx.peer.receive().unwrap().id(), std(0x150).into());
                let index = ctx
                    .peer
                    .transmit(&Frame::new(std(0x151), &[0xC3; 8]).unwrap())
                    .unwrap();
                assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);
            }
            if sent && let Poll::Ready(frame) = receive.as_mut().poll(&mut cx) {
                assert_eq!(frame.id(), std(0x151).into());
                assert_eq!(frame.payload(), &[0xC3; 8]);
                break;
            }
            assert!(
                Instant::now() < deadline,
                "split halves did not both complete"
            );
        }
    }

    #[test]
    async fn tx_buffer_priority_controls_the_local_queue(mut ctx: Nodes) {
        // Buffer zero goes first because it was armed on an idle bus; the rest
        // follow in descending priority. Buffer one is ranked above the
        // three-bit field, which must saturate rather than wrap to zero.
        let priorities = [0u8, 9, 2, 3];
        for (index, &priority) in priorities.iter().enumerate() {
            ctx.node.set_tx_priority(index as u8, priority);
        }

        let first = Frame::new_fd(std(0x777), &[0; 64]).unwrap();
        assert_eq!(ctx.node.transmit(&first).unwrap(), 0);
        for index in 1..4u8 {
            assert_eq!(
                ctx.node
                    .transmit(&Frame::new(std(0x100), &[index]).unwrap())
                    .unwrap(),
                index
            );
        }
        for index in 0..4 {
            assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
        }

        assert_eq!(ctx.peer.receive().unwrap().id(), std(0x777).into());
        for index in [1u8, 3, 2] {
            assert_eq!(
                ctx.peer.receive().unwrap().payload(),
                &[index],
                "buffers left in the wrong order"
            );
        }
    }

    #[test]
    async fn a_full_rx_buffer_drains_in_order_and_rearms(mut ctx: Nodes) {
        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);

        {
            let mut receive = core::pin::pin!(ctx.node.receive_async());
            assert!(receive.as_mut().poll(&mut cx).is_pending());

            // More frames than the 128-word buffer can hold, never read.
            for seq in 0..40u8 {
                let frame = Frame::new_fd(std(0x123), &[seq; 64])
                    .unwrap()
                    .with_bit_rate_switch(true);
                let index = ctx.peer.transmit(&frame).unwrap();
                assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);
            }
            assert!(WAKES.load(Ordering::SeqCst) > 0, "no frame ever woke RX");
        }

        assert!(ctx.node.rx_overrun(), "overrun was not reported");
        let queued = ctx.node.rx_frame_count();
        assert!(queued > 0 && queued < 40, "queued {} frames", queued);

        // An overrun drops what does not fit, not what is already held.
        for seq in 0..queued {
            let mut receive = core::pin::pin!(ctx.node.receive_async());
            match receive.as_mut().poll(&mut cx) {
                Poll::Ready(frame) => assert_eq!(frame.payload(), &[seq as u8; 64]),
                Poll::Pending => panic!("a queued frame was lost"),
            }
        }
        assert_eq!(ctx.node.rx_frame_count(), 0);

        ctx.node.clear_rx_overrun();
        assert!(!ctx.node.rx_overrun());

        WAKES.store(0, Ordering::SeqCst);
        let mut receive = core::pin::pin!(ctx.node.receive_async());
        assert!(receive.as_mut().poll(&mut cx).is_pending());

        // One spurious wake left over from the drained buffer is allowed.
        Timer::after_millis(10).await;
        WAKES.store(0, Ordering::SeqCst);
        assert!(receive.as_mut().poll(&mut cx).is_pending());
        Timer::after_millis(10).await;
        assert_eq!(
            WAKES.load(Ordering::SeqCst),
            0,
            "the future kept waking itself on an idle bus"
        );

        let index = ctx
            .peer
            .transmit(&Frame::new(std(0x456), &[42]).unwrap())
            .unwrap();
        assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);

        let deadline = Instant::now() + Duration::from_millis(100);
        while WAKES.load(Ordering::SeqCst) == 0 {
            assert!(Instant::now() < deadline, "RX never rearmed");
        }
        match receive.as_mut().poll(&mut cx) {
            Poll::Ready(frame) => assert_eq!(frame.id(), std(0x456).into()),
            Poll::Pending => panic!("the fresh frame did not complete the future"),
        }
    }

    #[test]
    async fn a_cancelled_transmission_reuses_its_buffer_without_losing_a_wake(mut ctx: Nodes) {
        ctx.node.apply_config(&slow_nominal()).unwrap();
        ctx.peer.apply_config(&slow_nominal()).unwrap();
        ctx.node.start().unwrap();
        ctx.peer.start().unwrap();

        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);

        // Drop a transmit future while its frame is on the wire; the hardware
        // keeps sending it.
        let first = Frame::new_fd(std(0x100), &[0xA5; 64]).unwrap();
        {
            let mut transmit = core::pin::pin!(ctx.node.transmit_async(&first));
            assert!(transmit.as_mut().poll(&mut cx).is_pending());
            Timer::after_millis(5).await;
        }
        assert_eq!(ctx.node.tx_buffer_state(0), TxBufferState::InProgress);
        assert!(
            WakeLock::is_active(),
            "cancelling the future let the chip sleep on a frame in flight"
        );
        ctx.node.abort_transmit(0);
        assert_eq!(wait_tx(&ctx.node, 0), TxBufferState::Ok);
        assert_eq!(ctx.peer.receive().unwrap().id(), std(0x100).into());

        // The next future takes the buffer the cancelled one left behind.
        WAKES.store(0, Ordering::SeqCst);
        let second = Frame::new_fd(std(0x101), &[0x3C; 64]).unwrap();
        {
            let mut transmit = core::pin::pin!(ctx.node.transmit_async(&second));
            assert!(transmit.as_mut().poll(&mut cx).is_pending());

            let deadline = Instant::now() + Duration::from_millis(500);
            loop {
                if WAKES.swap(0, Ordering::SeqCst) > 0
                    && let Poll::Ready(result) = transmit.as_mut().poll(&mut cx)
                {
                    result.unwrap();
                    break;
                }
                assert!(
                    Instant::now() < deadline,
                    "the reused buffer never delivered its wake"
                );
                Timer::after_millis(1).await;
            }
        }

        assert_eq!(ctx.peer.receive().unwrap().payload(), second.payload());
        assert_eq!(ctx.node.error_counters(), (0, 0));
        assert_eq!(ctx.peer.error_counters(), (0, 0));
    }

    #[test]
    async fn dropping_an_async_driver_with_a_pending_interrupt_returns(mut ctx: Nodes) {
        let waker = counting_waker();
        let mut cx = core::task::Context::from_waker(&waker);

        // Arm the RX interrupt and cancel the future.
        {
            let mut receive = core::pin::pin!(ctx.node.receive_async());
            assert!(receive.as_mut().poll(&mut cx).is_pending());
        }

        // Tear the driver down while its interrupt is pending. The critical
        // section keeps the request pending until teardown is over.
        critical_section::with(|_| {
            let index = ctx
                .peer
                .transmit(&Frame::new(std(0x2A), &[7]).unwrap())
                .unwrap();
            assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);

            let deadline = Instant::now() + Duration::from_millis(100);
            while ctx.node.rx_frame_count() == 0 {
                assert!(Instant::now() < deadline, "the frame never arrived");
            }

            drop(ctx.node);
        });

        // Reaching this line is the assertion.
        assert_eq!(ctx.peer.error_counters(), (0, 0));
    }

    #[test]
    async fn joining_the_bus_waits_out_a_frame_in_flight(mut ctx: Nodes) {
        ctx.node.apply_config(&slow_nominal()).unwrap();
        ctx.peer.apply_config(&slow_nominal()).unwrap();
        ctx.node.start().unwrap();
        ctx.peer.start().unwrap();

        ctx.peer.stop().unwrap();

        let frame = Frame::new_fd(ext(0x1234567), &[0x55; 64]).unwrap();
        let index = ctx.node.transmit(&frame).unwrap();
        let deadline = Instant::now() + Duration::from_millis(100);
        while ctx.node.tx_buffer_state(index) != TxBufferState::InProgress {
            assert!(Instant::now() < deadline, "the frame never started");
        }

        let began = Instant::now();
        ctx.peer
            .start()
            .expect("joining the bus during a legal frame timed out");
        let waited = began.elapsed();

        assert!(
            waited > Duration::from_millis(10),
            "joined in {} us, which is too soon to have waited for the frame",
            waited.as_micros()
        );

        assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
        assert_eq!(ctx.peer.receive().unwrap().payload(), frame.payload());
    }

    #[test]
    async fn joining_the_bus_waits_out_a_slow_data_phase(mut ctx: Nodes) {
        // 50 kbit/s nominal with 6.25 kbit/s data makes a 64-byte frame last
        // about 105 ms, while 2048 nominal bit times is 41 ms.
        let slow_payload = Config::default()
            .with_no_transceiver(true)
            .with_nominal_timing(Timing {
                baud_rate_prescaler: 100,
                propagation_segment: 7,
                phase_segment_1: 5,
                phase_segment_2: 3,
                sync_jump_width: 3,
            })
            .with_fd_timing(Timing {
                baud_rate_prescaler: 128,
                propagation_segment: 63,
                phase_segment_1: 15,
                phase_segment_2: 21,
                sync_jump_width: 3,
            });

        ctx.node.apply_config(&slow_payload).unwrap();
        // Self test on the sender: the peer is off the bus and cannot ACK.
        ctx.peer
            .apply_config(&slow_payload.with_mode(Mode::SelfTest))
            .unwrap();
        ctx.node.start().unwrap();
        ctx.peer.start().unwrap();

        ctx.node.stop().unwrap();

        let frame = Frame::new_fd(std(0x321), &[0; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let index = ctx.peer.transmit(&frame).unwrap();
        let deadline = Instant::now() + Duration::from_millis(100);
        while ctx.peer.tx_buffer_state(index) != TxBufferState::InProgress {
            assert!(Instant::now() < deadline, "the frame never started");
        }
        Timer::after_millis(5).await;

        let began = Instant::now();
        ctx.node
            .start()
            .expect("joining timed out inside a frame with a slow data phase");
        let waited = began.elapsed();

        assert!(
            waited > Duration::from_millis(10),
            "joined in {} us, too soon to have waited for the frame",
            waited.as_micros()
        );
        assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);
    }

    #[test]
    async fn leaving_the_bus_waits_out_a_slow_data_phase(mut ctx: Nodes) {
        ctx.node.apply_config(&slow_data()).unwrap();
        ctx.peer.apply_config(&slow_data()).unwrap();
        ctx.node.start().unwrap();
        ctx.peer.start().unwrap();

        let frame = Frame::new_fd(std(0x321), &[0x55; 64])
            .unwrap()
            .with_bit_rate_switch(true);

        let index = ctx.node.transmit(&frame).unwrap();
        assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
        assert_eq!(ctx.peer.receive().unwrap().payload(), frame.payload());

        let index = ctx.node.transmit(&frame).unwrap();
        Timer::after_millis(5).await;
        assert_eq!(ctx.node.tx_buffer_state(index), TxBufferState::InProgress);

        let began = Instant::now();
        let result = ctx.node.stop();
        let waited = began.elapsed();

        assert_eq!(
            result,
            Ok(()),
            "stopping gave up after {} us, so the bound ignored the data phase",
            waited.as_micros()
        );
        assert!(
            waited > Duration::from_millis(50),
            "stopped after {} us, too soon to have waited for the frame",
            waited.as_micros()
        );
    }

    #[test]
    async fn dropping_a_transmitter_does_not_corrupt_a_slow_data_frame(mut ctx: Nodes) {
        ctx.node.apply_config(&slow_data()).unwrap();
        ctx.peer.apply_config(&slow_data()).unwrap();
        ctx.node.start().unwrap();
        ctx.peer.start().unwrap();

        let frame = Frame::new_fd(std(0x321), &[0x55; 64])
            .unwrap()
            .with_bit_rate_switch(true);
        let index = ctx.node.transmit(&frame).unwrap();
        Timer::after_millis(5).await;
        assert_eq!(ctx.node.tx_buffer_state(index), TxBufferState::InProgress);

        drop(ctx.node);
        Timer::after_millis(150).await;

        assert_eq!(
            ctx.peer.error_counters(),
            (0, 0),
            "dropping the transmitter corrupted the frame the peer was receiving"
        );
        assert_eq!(ctx.peer.receive().unwrap().payload(), &[0x55; 64]);
    }
}

/// Faults injected on the wire: the second pin drives a dominant pulse into a
/// frame being transmitted, which is a bit error against the controller's own
/// output.
#[embedded_test::tests(default_timeout = 5)]
mod fault_tests {
    use esp_hal::{
        canfd::ErrorState,
        delay::Delay,
        gpio::{DriveMode, Level, Output, OutputConfig, Pull},
        time::{Duration, Instant},
    };

    use super::*;

    struct Context {
        node: CanFd<'static, Blocking>,
        injector: Output<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (pin0, pin1) = hil_test::common_test_pins!(peripherals);
        let injector = Output::new(
            pin1,
            Level::High,
            OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        let (rx, tx) = unsafe { pin0.split() };

        // One shot, so each corrupted frame counts exactly once.
        let mut node = CanFd::new(
            peripherals.TWAI0,
            Config::default()
                .with_mode(Mode::SelfTest)
                .with_retransmit_limit(0)
                .with_no_transceiver(true),
        )
        .unwrap()
        .with_rx(rx)
        .with_tx(tx);
        node.start().unwrap();

        Context { node, injector }
    }

    /// Corrupts frames until the transmit error counter reaches `target`.
    fn drive_to(ctx: &mut Context, target_tec: u16) {
        let frame = Frame::new(ext(0x1234567), &[0xFF; 8]).unwrap();
        let mut seen_passive = false;

        for _ in 0..40 {
            let index = ctx.node.transmit(&frame).unwrap();

            let deadline = Instant::now() + Duration::from_millis(5);
            while ctx.node.tx_buffer_state(index) != TxBufferState::InProgress {
                assert!(Instant::now() < deadline, "the frame never started");
            }

            // 120 us into a 500 kbit/s frame is inside its payload.
            Delay::new().delay_micros(120);
            ctx.injector.set_low();
            Delay::new().delay_micros(20);
            ctx.injector.set_high();

            assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Failed);

            let state = ctx.node.error_state();
            let (_, tec) = ctx.node.error_counters();

            if state == ErrorState::Passive {
                seen_passive = true;
            }
            if state == ErrorState::BusOff {
                assert!(seen_passive, "went bus-off without passing through passive");
                assert!(tec >= 256, "bus-off at TEC {}", tec);
                return;
            }
            if tec >= target_tec {
                return;
            }
        }

        panic!(
            "injection never reached TEC {}: {:?}",
            target_tec,
            ctx.node.error_counters()
        );
    }

    #[test]
    fn injected_faults_reach_bus_off_and_recover(mut ctx: Context) {
        drive_to(&mut ctx, 256);

        let capture = ctx.node.error_capture();
        assert_eq!(capture.kind, esp_hal::canfd::BusErrorKind::Bit);
        assert_eq!(
            embedded_can::Error::kind(&capture.kind),
            embedded_can::ErrorKind::Bit
        );

        // Bus-off is not left on its own.
        Delay::new().delay_millis(10);
        assert_eq!(ctx.node.error_state(), ErrorState::BusOff);

        let began = Instant::now();
        ctx.node.request_bus_off_recovery();
        while ctx.node.error_state() != ErrorState::Active {
            assert!(
                began.elapsed() < Duration::from_millis(100),
                "the controller never rejoined the bus"
            );
        }

        // 128 occurrences of 11 recessive bits at 500 kbit/s is about 2.8 ms.
        assert!(
            began.elapsed() > Duration::from_micros(2500),
            "rejoined in {} us, too fast to have waited out reintegration",
            began.elapsed().as_micros()
        );
        assert_eq!(ctx.node.error_counters(), (0, 0));

        let index = ctx
            .node
            .transmit(&Frame::new(std(0x123), &[0xAB; 8]).unwrap())
            .unwrap();
        assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
    }

    #[test]
    fn a_recovery_request_made_while_active_does_nothing(mut ctx: Context) {
        // The hardware would remember an early request and rejoin on its own.
        ctx.node.request_bus_off_recovery();

        drive_to(&mut ctx, 256);
        Delay::new().delay_millis(10);

        assert_eq!(
            ctx.node.error_state(),
            ErrorState::BusOff,
            "an early recovery request armed an automatic recovery"
        );
    }
}
