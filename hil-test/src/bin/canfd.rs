//! CAN FD Tests
//!
//! The two common test pins are wired together, and the controllers drive
//! them open-drain, so the pins form a single recessive-high net: a real, if
//! minimal, CAN bus between the two controllers of the chip. The single-node
//! tests loop one controller back to itself and work with the pins unwired.
//!
//! No external pull-up is needed. The two-node tests run the data phase at the
//! default 2 Mbit/s, which the two internal pull-ups of the wired pins handle
//! together. The single-node tests have one pull-up and run their data phase
//! at 1 Mbit/s; at 2 Mbit/s the recessive edge of one pin is too slow.

//% CHIP_FILTER: canfd_driver_supported
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use hil_test as _;

mod canfd {
    use esp_hal::{
        Async,
        Blocking,
        DriverMode,
        canfd::{
            CANFD_DEVICE_ID,
            CanFd,
            CanFdInterrupt,
            ClockSource,
            Config,
            ConfigError,
            Error,
            Frame,
            FrameError,
            FrameKinds,
            MaskFilter,
            MaskFilterConfig,
            Mode,
            RangeFilterConfig,
            Timing,
            TxBufferState,
            dlc_to_len,
            len_to_dlc,
        },
        rtc_cntl::WakeLock,
        timer::timg::TimerGroup,
    };

    struct Context<D: DriverMode> {
        canfd: CanFd<'static, D>,
    }

    // A driver that is not `Send` cannot be moved into a task, and one that is
    // not `Sync` cannot be shared with an interrupt handler through a `static`.
    // The register pointer inside would take both away without this check.
    // An async driver is `!Send` by design, because its interrupt handler is
    // bound to the core it was created on; see `esp_hal::Async`.
    const _: () = {
        const fn send_and_sync<T: Send + Sync>() {}
        const fn sync<T: Sync>() {}
        send_and_sync::<CanFd<'static, Blocking>>();
        sync::<CanFd<'static, Async>>();
    };

    /// Loopback plus self test lets one controller exercise itself: the frame is
    /// routed back into the RX buffer, and self test keeps the missing external
    /// acknowledgement from failing the transmission.
    ///
    /// The data phase runs at 1 Mbit/s rather than the default 2 Mbit/s. A
    /// single pin has only its own internal pull-up to pull the wire recessive,
    /// and on the runner that edge is too slow for 2 Mbit/s; two pins wired
    /// together pull twice as hard, which is why the two-node tests keep the
    /// default.
    fn config() -> Config {
        Config::default()
            .with_mode(Mode::LoopbackSelfTest)
            .with_no_transceiver(true)
            .with_fd_timing(Timing {
                baud_rate_prescaler: 20,
                propagation_segment: 1,
                phase_segment_1: 1,
                phase_segment_2: 1,
                sync_jump_width: 1,
            })
    }

    /// Waits for a queued frame to leave the in-progress states.
    ///
    /// Bounded so a stuck transmission fails the test rather than hanging it
    /// until the harness timeout.
    fn wait_tx<D: DriverMode>(canfd: &CanFd<'static, D>, index: u8) -> TxBufferState {
        // A deadline rather than a spin count, because the slowest bit rate
        // these tests configure needs over a hundred milliseconds for one frame.
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
    fn accepted(canfd: &mut CanFd<'static, Blocking>, id: u32, extended: bool) -> bool {
        canfd.flush_rx();
        let frame = Frame::new(id, extended, &[0xAA]).unwrap();
        let index = canfd.transmit(&frame).unwrap();
        assert_eq!(wait_tx(canfd, index), TxBufferState::Ok, "transmit failed");
        canfd.rx_frame_count() > 0
    }

    #[embedded_test::tests(default_timeout = 3)]
    mod blocking_tests {
        use esp_hal::time::{Duration, Instant};

        use super::*;

        /// Checks the timestamp counter really advances at `rate` counts per
        /// second, timed against the system timer.
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

            let (loopback_pin, _) = hil_test::common_test_pins!(peripherals);
            let (rx, tx) = unsafe { loopback_pin.split() };

            let mut canfd = CanFd::new(peripherals.TWAI0, config())
                .unwrap()
                .with_rx(rx)
                .with_tx(tx);
            canfd.start().unwrap();

            Context { canfd }
        }

        #[test]
        fn reports_the_ctu_can_fd_core(ctx: Context<Blocking>) {
            assert_eq!(ctx.canfd.identity().device_id, CANFD_DEVICE_ID);
            assert_eq!(ctx.canfd.tx_buffer_count(), 4);
        }

        #[test]
        fn classic_frames_round_trip(mut ctx: Context<Blocking>) {
            for (id, extended, payload) in [
                (0x123, false, &b"12345678"[..]),
                (0x1AB_CDEF, true, &b"abcd"[..]),
                (0x7FF, false, &[][..]),
            ] {
                let sent = Frame::new(id, extended, payload).unwrap();
                let got = round_trip(&mut ctx.canfd, &sent);

                assert_eq!(got.id(), id);
                assert_eq!(got.is_extended(), extended);
                assert!(!got.is_fd());
                assert_eq!(got.payload(), payload);
            }
        }

        #[test]
        fn fd_frames_round_trip_at_every_payload_length(mut ctx: Context<Blocking>) {
            // The lengths a CAN FD data length code can express, so both
            // directions of the DLC mapping are exercised.
            for len in [0usize, 1, 8, 12, 16, 20, 24, 32, 48, 64] {
                let mut payload = [0u8; 64];
                for (i, byte) in payload.iter_mut().enumerate().take(len) {
                    *byte = i as u8;
                }

                let sent = Frame::new_fd(0x100 + len as u32, false, true, &payload[..len]).unwrap();
                let got = round_trip(&mut ctx.canfd, &sent);

                assert!(got.is_fd(), "len {}", len);
                assert!(got.is_bit_rate_switched(), "len {}", len);
                assert_eq!(got.payload(), &payload[..len], "len {}", len);
            }
        }

        #[test]
        fn extended_id_fd_frame_round_trips(mut ctx: Context<Blocking>) {
            let sent = Frame::new_fd(0x1FF_FFFF, true, true, &[0x5A; 64]).unwrap();
            let got = round_trip(&mut ctx.canfd, &sent);

            assert_eq!(got.id(), 0x1FF_FFFF);
            assert!(got.is_extended());
            assert!(got.is_fd());
            assert_eq!(got.payload(), &[0x5A; 64]);
        }

        #[test]
        fn frames_speak_embedded_can(mut ctx: Context<Blocking>) {
            // Code written against the `embedded-can` traits has to be able to
            // build, send and read classic frames without knowing this driver,
            // and it is entitled to the trait's promise that a frame holds at
            // most 8 bytes. This is what such code looks like.
            use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
            use esp_hal::canfd::ClassicFrame;

            fn eight_byte_consumer<F: embedded_can::Frame>(frame: &F) -> [u8; 8] {
                // Sized by the data, not by the code: a request frame has a
                // code and no data.
                let data = frame.data();
                let mut bytes = [0u8; 8];
                bytes[..data.len()].copy_from_slice(data);
                bytes
            }

            let standard = StandardId::new(0x123).unwrap();
            let sent = ClassicFrame::new(standard, &[1, 2, 3]).unwrap();
            let got = ClassicFrame::try_from(round_trip(&mut ctx.canfd, &sent.into())).unwrap();
            assert_eq!(embedded_can::Frame::id(&got), Id::Standard(standard));
            assert!(got.is_standard());
            assert!(got.is_data_frame());
            assert_eq!(got.dlc(), 3);
            assert_eq!(got.data(), &[1, 2, 3]);
            assert_eq!(eight_byte_consumer(&got), [1, 2, 3, 0, 0, 0, 0, 0]);
            // The wrapped frame is still there, timestamp and all.
            assert!(!got.is_fd());

            // The trait is classic CAN: nine bytes are not a frame it can make.
            assert!(ClassicFrame::new(standard, &[0; 9]).is_none());

            let extended = ExtendedId::new(0x1AB_CDEF).unwrap();
            let request = ClassicFrame::new_remote(extended, 4).unwrap();
            let got = ClassicFrame::try_from(round_trip(&mut ctx.canfd, &request.into())).unwrap();
            assert_eq!(embedded_can::Frame::id(&got), Id::Extended(extended));
            assert!(embedded_can::Frame::is_extended(&got));
            assert!(got.is_remote_frame());
            // A request carries no data, and its code is the length it asks for.
            assert_eq!(got.dlc(), 4);
            assert_eq!(got.data(), &[]);
            assert_eq!(eight_byte_consumer(&got), [0; 8]);
            assert!(ClassicFrame::new_remote(extended, 9).is_none());

            // A received FD frame must never reach that consumer: the only way
            // to the trait is a conversion that refuses it. Handing over the
            // frame anyway used to be accepted, and the consumer then panicked
            // on a 64-byte payload.
            let fd = Frame::new_fd(0x124, false, true, &[0x5A; 64]).unwrap();
            let got = round_trip(&mut ctx.canfd, &fd);
            assert_eq!(
                ClassicFrame::try_from(got).unwrap_err(),
                FrameError::NotClassic
            );
        }

        #[test]
        fn timing_rejects_what_the_hardware_cannot_sample(_ctx: Context<Blocking>) {
            use esp_hal::canfd::{FD_TIMING_LIMITS, NOMINAL_TIMING_LIMITS};

            // TRM 38.3.7.7 states both limits in minimal time quanta, so the
            // prescaler is part of them: each parameter is individually in
            // range here, and the combination is still invalid.
            let too_short_phase2 = Timing {
                baud_rate_prescaler: 1,
                propagation_segment: 5,
                phase_segment_1: 5,
                phase_segment_2: 1,
                sync_jump_width: 1,
            };
            assert!(!too_short_phase2.is_valid(&NOMINAL_TIMING_LIMITS));
            assert!(!too_short_phase2.is_valid(&FD_TIMING_LIMITS));

            // The same segments with a prescaler of 2 give phase segment 2 two
            // system clocks, which the hardware can sample.
            let ok = Timing {
                baud_rate_prescaler: 2,
                ..too_short_phase2
            };
            assert!(ok.is_valid(&NOMINAL_TIMING_LIMITS));

            // Sync_Seg + Prop_Seg + Phase_Seg1 must exceed two minimal quanta.
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
        fn dlc_maps_both_ways(_ctx: Context<Blocking>) {
            for len in [0u8, 1, 8, 12, 16, 20, 24, 32, 48, 64] {
                assert_eq!(dlc_to_len(len_to_dlc(len)), len, "len {}", len);
            }
            // A length the format cannot express is rounded up to the next code.
            assert_eq!(dlc_to_len(len_to_dlc(9)), 12);
            assert_eq!(dlc_to_len(len_to_dlc(33)), 48);
        }

        #[test]
        fn mask_filter_accepts_and_rejects(mut ctx: Context<Blocking>) {
            // Accept 0x220..=0x22F: the low nibble is a don't-care.
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

            assert!(accepted(&mut ctx.canfd, 0x225, false), "0x225 must pass");
            assert!(!accepted(&mut ctx.canfd, 0x235, false), "0x235 must not");

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

            // Both ends, and both sides of both ends: a filter that only ever
            // checks its upper bound passes a test that only probes above it.
            assert!(accepted(&mut ctx.canfd, 0x400, false), "0x400 must pass");
            assert!(accepted(&mut ctx.canfd, 0x408, false), "0x408 must pass");
            assert!(accepted(&mut ctx.canfd, 0x40F, false), "0x40F must pass");
            assert!(!accepted(&mut ctx.canfd, 0x3FF, false), "0x3FF must not");
            assert!(!accepted(&mut ctx.canfd, 0x410, false), "0x410 must not");

            ctx.canfd.accept_all();
        }

        #[test]
        fn filters_can_reject_fd_frames_alone(mut ctx: Context<Blocking>) {
            // Same identifier, different frame format: only the classic one is
            // accepted. This gating has no equivalent on the classic peripheral.
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

            assert!(accepted(&mut ctx.canfd, 0x123, false), "classic must pass");

            ctx.canfd.flush_rx();
            let fd = Frame::new_fd(0x123, false, false, &[0; 12]).unwrap();
            let index = ctx.canfd.transmit(&fd).unwrap();
            assert_eq!(wait_tx(&ctx.canfd, index), TxBufferState::Ok);
            assert_eq!(ctx.canfd.rx_frame_count(), 0, "FD frame must be filtered");

            ctx.canfd.accept_all();
        }

        #[test]
        fn the_timestamp_counter_runs_at_the_rate_it_reports(mut ctx: Context<Blocking>) {
            // Timing the counter against the system timer is what makes the
            // reported resolution worth anything. Without it the prescaler can
            // be off by its whole divider — the register is documented as a
            // count step rather than a divider, so getting it backwards is a
            // live possibility — and every assertion about a timestamp being
            // non-zero still holds.
            let resolution = ctx.canfd.start_timestamp_timer(1_000_000).unwrap();
            assert_eq!(resolution, 1_000_000);
            assert_eq!(ctx.canfd.timestamp_bit_width(), 32);
            measure_rate(&ctx.canfd, resolution);

            // Restarting at another resolution has to take effect, has to start
            // from a known value rather than leaving counts made in the old unit
            // behind, and has to be counting by the time it is handed back: the
            // prescaler stalls for up to 65536 clock periods when a new divider
            // falls below the phase it happens to hold. Alternated several times
            // because that stall depends on the phase, so one change catches it
            // only about a third of the time.
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

            // Let the counter climb past anything a frame could carry in a
            // field of its own, so that decoding the wrong word cannot land
            // inside the window checked below. Bounded, so a timer that never
            // ticks fails with a clear message instead of running into the
            // harness timeout.
            let deadline = Instant::now() + Duration::from_millis(500);
            while ctx.canfd.timestamp() < 0x1_0000 {
                assert!(
                    Instant::now() < deadline,
                    "timestamp counter never reached a usable value"
                );
            }

            // Bracket the transfer: the stamp is taken at the end of the frame,
            // so it has to land inside the window the transfer occupied. That is
            // what distinguishes a timestamp from any other field of the buffer
            // — an identifier, or a length, would sit far outside a window the
            // counter has already run past.
            let before = ctx.canfd.timestamp();
            let got = round_trip(
                &mut ctx.canfd,
                &Frame::new(0x555, false, &[1, 2, 3, 4]).unwrap(),
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
            round_trip(&mut ctx.canfd, &Frame::new(0x321, false, &[7; 8]).unwrap());

            assert_eq!(ctx.canfd.error_counters(), (0, 0));
            assert_eq!(ctx.canfd.error_state(), esp_hal::canfd::ErrorState::Active);
        }

        #[test]
        fn leaving_the_bus_waits_out_a_frame_at_a_low_bit_rate(mut ctx: Context<Blocking>) {
            // 80 MHz / (200 * 80 quanta) = 5 kbit/s, where a 64-byte frame takes
            // well over a hundred milliseconds. A wait bounded by any constant
            // short enough to be useful at 1 Mbit/s gives up in the middle of a
            // legal frame, and the controller then leaves the bus with a partial
            // frame on the wire for its peers to report as an error.
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

            let frame = Frame::new_fd(0x100, false, false, &[0xA5; 64]).unwrap();
            let index = ctx.canfd.transmit(&frame).unwrap();

            // Wait until the frame is really on the wire: aborting one that has
            // not started yet is immediate and would prove nothing.
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
            // Whatever the buffer settled as, it must have settled: that is what
            // leaving the bus waits for.
            assert!(
                !matches!(
                    ctx.canfd.tx_buffer_state(index),
                    TxBufferState::Ready
                        | TxBufferState::InProgress
                        | TxBufferState::AbortInProgress
                ),
                "the buffer was still in flight after stopping"
            );
        }

        #[test]
        fn identifiers_that_do_not_fit_the_format_are_refused(mut ctx: Context<Blocking>) {
            // The frame buffer keeps 11 or 29 identifier bits, so a wider one
            // used to be accepted and then sent under a different identifier —
            // matching different filters and arbitrating differently than the
            // caller checked.
            assert_eq!(
                Frame::new(0x800, false, &[0]).unwrap_err(),
                FrameError::IdTooLarge
            );
            assert_eq!(
                Frame::new_fd(0x2000_0000, true, false, &[0]).unwrap_err(),
                FrameError::IdTooLarge
            );
            assert_eq!(
                Frame::new_request(0x800, false, 8).unwrap_err(),
                FrameError::IdTooLarge
            );

            // The largest identifier of each format is still accepted, and
            // arrives as itself.
            for (id, extended) in [(0x7FF, false), (0x1FFF_FFFF, true)] {
                let frame = Frame::new(id, extended, &[0x5A]).unwrap();
                let got = round_trip(&mut ctx.canfd, &frame);
                assert_eq!(got.id(), id, "identifier changed on the wire");
                assert_eq!(got.is_extended(), extended);
            }
        }

        #[test]
        fn the_chip_stays_awake_exactly_while_the_controller_is_on_the_bus(
            mut ctx: Context<Blocking>,
        ) {
            // Light sleep gates the function clock, which would freeze a frame
            // in flight and lose whatever arrives, so a controller on the bus
            // has to hold the system awake. Not a future: a frame keeps going
            // after the future that queued it is dropped, and a frame can
            // arrive while nothing is waiting. Every way onto and off the bus
            // is walked here, because the lock has to follow all of them.
            assert!(ctx.canfd.is_started());
            assert!(WakeLock::is_active(), "started in init, yet asleep");

            ctx.canfd.stop().unwrap();
            assert!(!ctx.canfd.is_started());
            assert!(!WakeLock::is_active(), "stopped, yet held awake");

            ctx.canfd.start().unwrap();
            assert!(ctx.canfd.is_started());
            assert!(WakeLock::is_active(), "restarted, yet asleep");

            // Reconfiguring leaves the bus too, and starting twice must not
            // take a second lock that the next stop would leave behind.
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
            // It decides how the pins are driven, and that was committed when
            // they were assigned. Accepting it would leave `config()` claiming
            // an output stage the pin does not have.
            assert_eq!(
                ctx.canfd.apply_config(&config().with_no_transceiver(false)),
                Err(ConfigError::TransceiverModeLocked)
            );
            assert!(ctx.canfd.config().no_transceiver());

            // Applying a configuration that leaves it alone still works.
            ctx.canfd
                .apply_config(&config().with_retransmit_limit(1))
                .unwrap();
        }

        #[test]
        fn the_slowest_bit_rate_still_joins_the_bus(mut ctx: Context<Blocking>) {
            // Every field at its maximum from the 40 MHz crystal is about 741
            // bit/s, where joining takes over a hundred milliseconds. A bound
            // counted in polling iterations rather than time would be spent long
            // before that, and how early depends on the CPU clock, so the same
            // driver would join on one board and refuse on another.
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

            // The measurement is the point: a start that returned immediately
            // would pass the assertion above while proving nothing.
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

            // The prescaler divides the function clock, so pointing that clock
            // elsewhere would leave the counter running at a resolution nobody
            // asked for and no longer the one that was reported.
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

            // Restarting reports a resolution the new clock really produces.
            let resolution = ctx.canfd.start_timestamp_timer(1_000_000).unwrap();
            measure_rate(&ctx.canfd, resolution);
            ctx.canfd.stop_timestamp_timer();
        }

        #[test]
        fn transmitting_while_off_the_bus_is_refused(mut ctx: Context<Blocking>) {
            let frame = Frame::new(0x1A5, false, &[9]).unwrap();

            // With the controller off the bus the hardware ignores the command
            // that arms a buffer and nothing replays it later, so a success
            // here would be a frame silently lost. Every path that leaves the
            // bus is covered: stop, the TX half of a stopped controller, and a
            // reconfiguration.
            ctx.canfd.stop().unwrap();
            assert_eq!(ctx.canfd.transmit(&frame), Err(Error::ControllerStopped));

            let (_, mut tx) = ctx.canfd.split();
            assert_eq!(tx.transmit(&frame), Err(Error::ControllerStopped));

            ctx.canfd.apply_config(&config()).unwrap();
            assert_eq!(ctx.canfd.transmit(&frame), Err(Error::ControllerStopped));

            // Back on the bus, the very same frame goes through.
            ctx.canfd.start().unwrap();
            let got = round_trip(&mut ctx.canfd, &frame);
            assert_eq!(got.id(), 0x1A5);
            assert_eq!(got.payload(), &[9]);
        }

        #[test]
        fn filter_identifiers_that_do_not_fit_the_format_are_refused(mut ctx: Context<Blocking>) {
            // The filter registers hold 11 or 29 bits, exactly like the frame
            // buffer, so an out-of-range value used to be masked on the way in:
            // a filter asked to match 0x800 quietly became a filter for 0x000,
            // accepting traffic the caller believed it was rejecting.
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

            // The mask is held in the same field and is checked the same way.
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

            // The largest identifier of the format is accepted, and filters on
            // itself rather than on a truncated value.
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

            assert!(accepted(&mut ctx.canfd, 0x7FF, false), "0x7FF must pass");
            assert!(!accepted(&mut ctx.canfd, 0x000, false), "0x000 must not");

            ctx.canfd.accept_all();
        }

        #[test]
        fn tx_buffer_indices_the_hardware_does_not_have_are_refused(mut ctx: Context<Blocking>) {
            // Put a known state into buffer zero, so reporting a neighbour's
            // field instead of refusing is visible.
            let frame = Frame::new(0x111, false, &[1]).unwrap();
            let index = ctx.canfd.transmit(&frame).unwrap();
            assert_eq!(wait_tx(&ctx.canfd, index), TxBufferState::Ok);

            let count = ctx.canfd.tx_buffer_count();
            for index in [count, count + 1, 2 * count, u8::MAX] {
                // The four-bit fields share one register, so index 8 shifts a
                // 32-bit word by 32 and used to read buffer zero back.
                assert_eq!(
                    ctx.canfd.tx_buffer_state(index),
                    TxBufferState::NotExist,
                    "index {} reported a buffer that does not exist",
                    index
                );

                // And the commands must not land on a real buffer either.
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
        fn the_secondary_sample_point_leaves_room_for_the_measured_delay(
            mut ctx: Context<Blocking>,
        ) {
            // The default data phase is four quanta of ten clock periods, so an
            // offset of 16 quanta lands exactly on the four-bit-time limit of
            // TRM 38.3.7.3 — and the hardware adds the delay it measures on top,
            // which is never zero. Accepting this configuration means every FD
            // transmission fails instead. Nothing is transmitted here, so the
            // default data phase is used rather than the slower loopback one.
            let default_data_phase = config().with_fd_timing(Config::default().fd_timing());
            assert_eq!(
                ctx.canfd
                    .apply_config(&default_data_phase.with_secondary_sample_point_offset(16)),
                Err(ConfigError::UnsupportedSecondarySamplePoint)
            );

            // A rejected configuration must not be reported as the live one.
            assert_eq!(ctx.canfd.config().secondary_sample_point_offset(), None);

            // One quantum lower leaves room for the core's own input delay.
            ctx.canfd
                .apply_config(&default_data_phase.with_secondary_sample_point_offset(15))
                .unwrap();
            assert_eq!(ctx.canfd.config().secondary_sample_point_offset(), Some(15));
        }
    }

    /// Two controllers on one wire.
    ///
    /// The runners connect the two common test pins (see `hil-test/README.md`),
    /// so driving both open-drain with pull-ups makes them a single recessive-high
    /// net — a real, if minimal, CAN bus. Unlike the loopback tests above, these
    /// exercise the parts that need a second node: a peer acknowledging a frame,
    /// and arbitration between two nodes that start transmitting together.
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

            let (pin0, pin1) = hil_test::common_test_pins!(peripherals);
            let (rx0, tx0) = unsafe { pin0.split() };
            let (rx1, tx1) = unsafe { pin1.split() };

            // Normal mode on both: each frame is acknowledged by the other node,
            // so no self-test crutch is needed.
            let config = Config::default()
                .with_mode(Mode::Normal)
                .with_no_transceiver(true);

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
            // A frame only reaches "TX OK" here if the peer acknowledged it, so
            // these assertions cover acknowledgement as well as transfer.
            let sent = Frame::new(0x301, false, &[9, 8, 7, 6, 5, 4, 3, 2]).unwrap();
            let got = cross(&mut ctx.node0, &mut ctx.node1, &sent);
            assert_eq!(got.id(), 0x301);
            assert_eq!(got.payload(), sent.payload());

            let sent = Frame::new(0x401, false, &[0xC0, 0xFF, 0xEE]).unwrap();
            let got = cross(&mut ctx.node1, &mut ctx.node0, &sent);
            assert_eq!(got.id(), 0x401);
            assert_eq!(got.payload(), sent.payload());
        }

        #[test]
        fn fd_frames_cross_at_the_data_bit_rate(mut ctx: Nodes) {
            let sent = Frame::new_fd(0x1AB_CDEF, true, true, &[0x3C; 64]).unwrap();
            let got = cross(&mut ctx.node0, &mut ctx.node1, &sent);

            assert_eq!(got.id(), 0x1AB_CDEF);
            assert!(got.is_extended());
            assert!(got.is_fd());
            assert!(got.is_bit_rate_switched());
            assert_eq!(got.payload(), &[0x3C; 64]);
        }

        #[test]
        fn simultaneous_transmissions_arbitrate(mut ctx: Nodes) {
            ctx.node0.flush_rx();
            ctx.node1.flush_rx();

            // Latch arbitration-lost on both. The CPU interrupt is never enabled
            // for a blocking driver, so this only makes the status bit record
            // what happened.
            ctx.node0.clear_interrupts(CanFdInterrupt::ArbitrationLost);
            ctx.node1.clear_interrupts(CanFdInterrupt::ArbitrationLost);
            ctx.node0.listen(CanFdInterrupt::ArbitrationLost);
            ctx.node1.listen(CanFdInterrupt::ArbitrationLost);

            // Arming one node and then the other does not create contention:
            // the first starts as soon as the bus is idle, and by the time the
            // second is armed it can only wait for the frame in flight. Both
            // must instead become ready *while the bus is busy*, so they start
            // together at the following intermission and genuinely arbitrate.
            let filler = Frame::new(0x7FF, false, &[0; 8]).unwrap();
            let filler_idx = ctx.node0.transmit(&filler).unwrap();

            let low = Frame::new(0x100, false, &[1]).unwrap();
            let high = Frame::new(0x200, false, &[2]).unwrap();
            let i0 = ctx.node0.transmit(&low).unwrap();
            let i1 = ctx.node1.transmit(&high).unwrap();

            assert_eq!(wait_tx(&ctx.node0, filler_idx), TxBufferState::Ok);
            assert_eq!(wait_tx(&ctx.node0, i0), TxBufferState::Ok);
            assert_eq!(wait_tx(&ctx.node1, i1), TxBufferState::Ok);

            // node1 saw the filler and then the winner; node0 only ever sees its
            // peer's frame, never its own.
            assert_eq!(ctx.node1.receive().unwrap().id(), 0x7FF);
            assert_eq!(ctx.node1.receive().unwrap().id(), 0x100);
            assert_eq!(ctx.node0.receive().unwrap().id(), 0x200);

            // Both frames arriving proves transfer, but not that the two nodes
            // actually contended: the second could simply have waited for an
            // idle bus. The arbitration-lost flag is what distinguishes them,
            // and CAN resolves arbitration by identifier, so the higher one must
            // be the side that backed off.
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

            // Backing off and retrying is not an error, so the counters stay at
            // zero. Had the two corrupted each other, this is where it shows.
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

        /// Wakes counted by [`counting_waker`].
        static WAKES: AtomicUsize = AtomicUsize::new(0);

        /// A waker belonging to no executor, so a wake can only have come from
        /// the driver, and resting at zero is observable.
        fn counting_waker() -> Waker {
            fn clone(_: *const ()) -> RawWaker {
                RawWaker::new(core::ptr::null(), &VTABLE)
            }
            fn wake(_: *const ()) {
                WAKES.fetch_add(1, Ordering::SeqCst);
            }
            static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, |_| {});

            WAKES.store(0, Ordering::SeqCst);

            // SAFETY: the vtable's operations only touch a static counter and
            // never dereference the null data pointer.
            unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
        }

        /// The async driver plus a second controller on the other pin, used to
        /// deliver a frame at a moment of the test's choosing.
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
            esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

            // The driver under test loops back to itself, so it works whether or
            // not the pins are wired. The peer needs the jumper, and only the
            // wake test uses it.
            let mut canfd = CanFd::new(peripherals.TWAI0, config())
                .unwrap()
                .with_rx(rx0)
                .with_tx(tx0)
                .into_async();
            canfd.start().unwrap();

            // Same wire, so the same data phase as the loopback node: a peer
            // that expects the default 2 Mbit/s would flag the node's 1 Mbit/s
            // payload as an error and destroy the frame.
            let peer_config = Config::default()
                .with_mode(Mode::SelfTest)
                .with_no_transceiver(true)
                .with_fd_timing(config().fd_timing());
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
                Frame::new(0x201, false, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap(),
                Frame::new_fd(0x202, false, true, &[0x77; 64]).unwrap(),
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

            // Nothing is on the bus, so the future must park...
            assert!(
                receive.as_mut().poll(&mut cx).is_pending(),
                "received a frame from an idle bus"
            );
            // ...without letting the chip sleep through the frame it waits for.
            assert!(WakeLock::is_active(), "waiting for a frame while asleep");

            // ...and stay parked. Racing the future against a timer cannot see
            // the difference: a future that wakes itself on every poll is still
            // Pending, so the timer still wins and the test still passes while
            // the executor spins. The wake count is what tells them apart.
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
            // Polled by hand against a waker of our own, because racing the
            // future against a timer proves nothing: both share the task's
            // waker, so the timer's wake alone gets the future polled again, it
            // finds the frame already queued, and the test passes even if the
            // driver never wakes anything. Counting wakes on a private waker is
            // what makes the interrupt path observable.
            ctx.canfd.flush_rx();

            let waker = counting_waker();
            let mut cx = core::task::Context::from_waker(&waker);

            let mut receive = core::pin::pin!(ctx.canfd.receive_async());

            // Nothing has been sent, so the future must park.
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
                .transmit(&Frame::new(0x203, false, &[0xEE; 2]).unwrap())
                .unwrap();

            // The wake can only come from the RX interrupt: this waker belongs
            // to no executor and no timer.
            let deadline = Instant::now() + Duration::from_millis(500);
            while WAKES.load(Ordering::SeqCst) == 0 {
                assert!(
                    Instant::now() < deadline,
                    "the RX interrupt never woke the future"
                );
            }

            match receive.as_mut().poll(&mut cx) {
                Poll::Ready(frame) => assert_eq!(frame.id(), 0x203),
                Poll::Pending => panic!("the frame arrived but the future did not complete"),
            }
        }
    }

    /// Two nodes exchanging real traffic, with the driver under test in async
    /// mode.
    ///
    /// Unlike [`async_tests`], which loops one controller back to itself, these
    /// need the peer: they are about what happens between two nodes when a
    /// buffer overflows, a transmission is cancelled, or a controller leaves the
    /// bus in the middle of a frame.
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

        /// A waker belonging to no executor, so a wake can only have come from
        /// the driver.
        fn counting_waker() -> Waker {
            fn clone(_: *const ()) -> RawWaker {
                RawWaker::new(core::ptr::null(), &VTABLE)
            }
            fn wake(_: *const ()) {
                WAKES.fetch_add(1, Ordering::SeqCst);
            }
            static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, |_| {});

            WAKES.store(0, Ordering::SeqCst);

            // SAFETY: the vtable's operations only touch a static counter and
            // never dereference the null data pointer.
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
            esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

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

        /// 5 kbit/s in the arbitration phase, where a 64-byte frame takes over a
        /// hundred milliseconds and there is room to interrupt one mid-flight.
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

        /// The same rate, but in the data phase only, which bit rate switching
        /// then uses for the payload.
        ///
        /// A data phase slower than the arbitration phase is unusual and still
        /// legal: the two prescalers are independent (TRM 38.3.7.1), the
        /// configuration is accepted, and it transfers on hardware.
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
            // The whole-controller futures each borrow everything, so this is
            // the only way to hold a receive and a transmit at once.
            let (mut rx, mut tx) = ctx.node.split();

            let outgoing = Frame::new_fd(0x150, false, true, &[0x5A; 64]).unwrap();
            let mut receive = core::pin::pin!(rx.receive_async());
            let mut transmit = core::pin::pin!(tx.transmit_async(&outgoing));

            let waker = counting_waker();
            let mut cx = core::task::Context::from_waker(&waker);

            // Both are live at once: the transmission is in flight while the
            // receive future waits for the peer's answer.
            assert!(receive.as_mut().poll(&mut cx).is_pending());
            assert!(transmit.as_mut().poll(&mut cx).is_pending());

            let deadline = Instant::now() + Duration::from_millis(500);
            let mut sent = false;
            loop {
                if !sent && transmit.as_mut().poll(&mut cx).is_ready() {
                    sent = true;
                    // Only once the frame is out can the peer answer it.
                    assert_eq!(ctx.peer.receive().unwrap().id(), 0x150);
                    let index = ctx
                        .peer
                        .transmit(&Frame::new(0x151, false, &[0xC3; 8]).unwrap())
                        .unwrap();
                    assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);
                }
                if sent && let Poll::Ready(frame) = receive.as_mut().poll(&mut cx) {
                    assert_eq!(frame.id(), 0x151);
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
            // Priorities order the buffers against each other, not against the
            // bus: buffer zero goes first because it was armed first, and the
            // rest follow in descending priority (TRM 38.3.8.1). Buffer one is
            // ranked above the three-bit field: it has to go first among the
            // rest, not last, which is what masking the value down to zero
            // would make of it.
            let priorities = [0u8, 9, 2, 3];
            for (index, &priority) in priorities.iter().enumerate() {
                ctx.node.set_tx_priority(index as u8, priority);
            }

            let first = Frame::new_fd(0x777, false, false, &[0; 64]).unwrap();
            assert_eq!(ctx.node.transmit(&first).unwrap(), 0);
            for index in 1..4u8 {
                assert_eq!(
                    ctx.node
                        .transmit(&Frame::new(0x100, false, &[index]).unwrap())
                        .unwrap(),
                    index
                );
            }
            for index in 0..4 {
                assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
            }

            assert_eq!(ctx.peer.receive().unwrap().id(), 0x777);
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
                    let frame = Frame::new_fd(0x123, false, true, &[seq; 64]).unwrap();
                    let index = ctx.peer.transmit(&frame).unwrap();
                    assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);
                }
                assert!(WAKES.load(Ordering::SeqCst) > 0, "no frame ever woke RX");
            }

            assert!(ctx.node.rx_overrun(), "overrun was not reported");
            let queued = ctx.node.rx_frame_count();
            assert!(queued > 0 && queued < 40, "queued {} frames", queued);

            // What survived must be the oldest frames, in order: an overrun
            // drops what does not fit, not what is already held.
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

            // And the future must rearm: a fresh frame still wakes it.
            WAKES.store(0, Ordering::SeqCst);
            let mut receive = core::pin::pin!(ctx.node.receive_async());
            assert!(receive.as_mut().poll(&mut cx).is_pending());

            // One wake left over from the drained buffer is allowed; a future
            // may be woken spuriously. What matters is that it settles.
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
                .transmit(&Frame::new(0x456, false, &[42]).unwrap())
                .unwrap();
            assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);

            let deadline = Instant::now() + Duration::from_millis(100);
            while WAKES.load(Ordering::SeqCst) == 0 {
                assert!(Instant::now() < deadline, "RX never rearmed");
            }
            match receive.as_mut().poll(&mut cx) {
                Poll::Ready(frame) => assert_eq!(frame.id(), 0x456),
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

            // Drop a transmit future while its frame is on the wire. The frame
            // is not cancel-safe by design, so the hardware keeps sending it.
            let first = Frame::new_fd(0x100, false, false, &[0xA5; 64]).unwrap();
            {
                let mut transmit = core::pin::pin!(ctx.node.transmit_async(&first));
                assert!(transmit.as_mut().poll(&mut cx).is_pending());
                Timer::after_millis(5).await;
            }
            assert_eq!(ctx.node.tx_buffer_state(0), TxBufferState::InProgress);
            // The frame is still on the wire, so the chip must still be held
            // awake: a lock that belonged to the future went away with it.
            assert!(
                WakeLock::is_active(),
                "cancelling the future let the chip sleep on a frame in flight"
            );
            ctx.node.abort_transmit(0);
            assert_eq!(wait_tx(&ctx.node, 0), TxBufferState::Ok);
            assert_eq!(ctx.peer.receive().unwrap().id(), 0x100);

            // The next future takes the first writable buffer, which is the one
            // the cancelled future left behind. Its wake must still arrive.
            WAKES.store(0, Ordering::SeqCst);
            let second = Frame::new_fd(0x101, false, false, &[0x3C; 64]).unwrap();
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

            // Arm the RX interrupt and cancel the future, which leaves the
            // source enabled with nothing waiting on it.
            {
                let mut receive = core::pin::pin!(ctx.node.receive_async());
                assert!(receive.as_mut().poll(&mut cx).is_pending());
            }

            // Deliver a frame and tear the driver down while its interrupt is
            // still pending. The critical section holds the request off until
            // teardown is over, which is what makes the ordering deterministic:
            // releasing the clocks with a source still armed leaves the request
            // line asserted and no way for a handler to retire it, and the CPU
            // never leaves the trap.
            critical_section::with(|_| {
                let index = ctx
                    .peer
                    .transmit(&Frame::new(0x2A, false, &[7]).unwrap())
                    .unwrap();
                assert_eq!(wait_tx(&ctx.peer, index), TxBufferState::Ok);

                let deadline = Instant::now() + Duration::from_millis(100);
                while ctx.node.rx_frame_count() == 0 {
                    assert!(Instant::now() < deadline, "the frame never arrived");
                }

                drop(ctx.node);
            });

            // Reaching this line is the assertion. The peer is checked too, so
            // the test cannot pass by having gone quiet some other way.
            assert_eq!(ctx.peer.error_counters(), (0, 0));
        }

        #[test]
        async fn joining_the_bus_waits_out_a_frame_in_flight(mut ctx: Nodes) {
            // 5 kbit/s, where a 64-byte frame takes well over a hundred
            // milliseconds. Integration waits for eleven consecutive recessive
            // bits, which cannot appear until that frame ends, so a bound
            // shorter than one frame turns an ordinary exchange between other
            // nodes into a failure to start.
            ctx.node.apply_config(&slow_nominal()).unwrap();
            ctx.peer.apply_config(&slow_nominal()).unwrap();
            ctx.node.start().unwrap();
            ctx.peer.start().unwrap();

            // Take the peer off the bus, so it has to join during the frame.
            ctx.peer.stop().unwrap();

            let frame = Frame::new_fd(0x1234567, true, false, &[0x55; 64]).unwrap();
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

            // The frame the peer joined during is acknowledged and arrives.
            assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
            assert_eq!(ctx.peer.receive().unwrap().payload(), frame.payload());
        }

        #[test]
        async fn joining_the_bus_waits_out_a_slow_data_phase(mut ctx: Nodes) {
            // The frame on the wire belongs to the node transmitting it, and its
            // payload runs at that node's data bit rate. A wait derived from the
            // arbitration timing alone is far too short whenever the data phase
            // is the slower one: 50 kbit/s nominal with 6.25 kbit/s data makes a
            // 64-byte frame last about 105 ms, while 2048 nominal bit times is
            // 41 ms.
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
            // Self test on the sender, so the missing acknowledgement of a peer
            // that has stepped off the bus is not itself a failure.
            ctx.peer
                .apply_config(&slow_payload.with_mode(Mode::SelfTest))
                .unwrap();
            ctx.node.start().unwrap();
            ctx.peer.start().unwrap();

            ctx.node.stop().unwrap();

            let frame = Frame::new_fd(0x321, false, true, &[0; 64]).unwrap();
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

            let frame = Frame::new_fd(0x321, false, true, &[0x55; 64]).unwrap();

            // Establish first that this timing really does carry a frame, so a
            // failure below is about the teardown and not about the timing.
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

            let index = ctx
                .node
                .transmit(&Frame::new_fd(0x321, false, true, &[0x55; 64]).unwrap())
                .unwrap();
            Timer::after_millis(5).await;
            assert_eq!(ctx.node.tx_buffer_state(index), TxBufferState::InProgress);

            drop(ctx.node);
            Timer::after_millis(150).await;

            // Cutting the frame short would show up on the peer as a receive
            // error and a frame that never arrived.
            assert_eq!(
                ctx.peer.error_counters(),
                (0, 0),
                "dropping the transmitter corrupted the frame the peer was receiving"
            );
            assert_eq!(ctx.peer.receive().unwrap().payload(), &[0x55; 64]);
        }
    }

    /// Faults injected on the wire, to reach the states a healthy bus never
    /// enters.
    ///
    /// The second test pin drives a dominant pulse into the middle of a frame
    /// the controller is transmitting, which is a bit error against its own
    /// output and moves the transmit error counter by eight each time.
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
            // Open-drain, like the controller itself: the two share a wire.
            let injector = Output::new(
                pin1,
                Level::High,
                OutputConfig::default()
                    .with_drive_mode(DriveMode::OpenDrain)
                    .with_pull(Pull::Up),
            );
            let (rx, tx) = unsafe { pin0.split() };

            // Self test, so the missing acknowledgement of a lone node does not
            // fail transmissions, and one shot, so each corrupted frame counts
            // exactly once.
            let mut node = CanFd::new(
                peripherals.TWAI0,
                config().with_mode(Mode::SelfTest).with_retransmit_limit(0),
            )
            .unwrap()
            .with_rx(rx)
            .with_tx(tx);
            node.start().unwrap();

            Context { node, injector }
        }

        /// Corrupts frames until the transmit error counter reaches `target`,
        /// checking the fault confinement states on the way.
        fn drive_to(ctx: &mut Context, target_tec: u16) {
            let frame = Frame::new(0x1234567, true, &[0xFF; 8]).unwrap();
            let mut seen_passive = false;

            for _ in 0..40 {
                let index = ctx.node.transmit(&frame).unwrap();

                let deadline = Instant::now() + Duration::from_millis(5);
                while ctx.node.tx_buffer_state(index) != TxBufferState::InProgress {
                    assert!(Instant::now() < deadline, "the frame never started");
                }

                // 120 us into a 500 kbit/s frame is inside its payload, where a
                // dominant pulse contradicts the recessive bits being sent.
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

            // The pulse contradicted the controller's own output, which is a
            // bit error, and code written against `embedded-can` sees it as
            // one too.
            let capture = ctx.node.error_capture();
            assert_eq!(capture.kind, esp_hal::canfd::BusErrorKind::Bit);
            assert_eq!(
                embedded_can::Error::kind(&capture.kind),
                embedded_can::ErrorKind::Bit
            );

            // Bus-off is not left on its own: recovery is the caller's decision.
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

            // Reintegration waits for 128 occurrences of 11 recessive bits,
            // which at 500 kbit/s cannot be faster than about 2.8 ms.
            assert!(
                began.elapsed() > Duration::from_micros(2500),
                "rejoined in {} us, too fast to have waited out reintegration",
                began.elapsed().as_micros()
            );
            assert_eq!(ctx.node.error_counters(), (0, 0));

            let index = ctx
                .node
                .transmit(&Frame::new(0x123, false, &[0xAB; 8]).unwrap())
                .unwrap();
            assert_eq!(wait_tx(&ctx.node, index), TxBufferState::Ok);
        }

        #[test]
        fn a_recovery_request_made_while_active_does_nothing(mut ctx: Context) {
            // The hardware remembers this command even when it is not bus-off,
            // and would then rejoin on its own later — TRM 38.3.4 calls the
            // error state sticky, while the register description of ERCRST says
            // the command has no effect outside bus-off. The driver keeps the
            // documented behaviour by checking the state itself.
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
}
