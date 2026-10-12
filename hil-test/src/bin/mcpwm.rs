//! MCPWM test

//% CHIP_FILTER: mcpwm_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(mcpwm_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod mcpwm {
    use esp_hal::{
        self,
        delay::Delay,
        gpio::{AnyPin, Level, NoPin, Output, OutputConfig, Pin},
        mcpwm::{
            McPwm,
            PeripheralClockConfig,
            capture::{CaptureChannelConfig, CaptureEdge, CaptureMode, CaptureTimerConfig},
            timer::{ConfigError, CounterDirection, PwmWorkingMode, SyncOutSelect},
        },
        time::Rate,
    };

    struct Context<'d> {
        mcpwm: McPwm<'static>,
        input: AnyPin<'d>,
        output: AnyPin<'d>,
        delay: Delay,
        clock_cfg: PeripheralClockConfig,
    }

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (din, dout) = hil_test::common_test_pins!(peripherals);

        let din = din.degrade();
        let dout = dout.degrade();

        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(1));
        assert!(clock_cfg.is_ok(), "Failed to set MCPWM clock frequency");
        let clock_cfg = clock_cfg.unwrap();

        Context {
            mcpwm: McPwm::new(peripherals.MCPWM0, clock_cfg),
            input: din,
            output: dout,
            delay: Delay::new(),
            clock_cfg,
        }
    }

    #[test]
    fn test_capture_channels_disabled_does_not_capture_software_trigger(ctx: Context<'static>) {
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];
        let mut timer = ctx.mcpwm.capture_timer;

        timer.start();

        // Trigger a capture event on each capture channel and verify that the captured phase is
        // zero
        for mut capture in captures {
            capture.set_enable(false);
            capture.clear_interrupt();
            capture.listen(CaptureMode::AnyEdge);

            capture.trigger_capture();
            ctx.delay.delay_micros(1);

            // The capture channel is disabled, so the captured phase should be zero and no
            // interrupt should be set
            assert_eq!(0, capture.events().phase());
            assert!(!capture.is_interrupt_set());
        }
    }

    #[test]
    fn test_capture_channels_not_listening_capture_software_trigger(ctx: Context<'static>) {
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];
        let mut timer = ctx.mcpwm.capture_timer;

        timer.start();

        // Trigger a capture event on each capture channel and verify that the captured phase is not
        // zero
        for mut capture in captures {
            capture.set_enable(true);
            capture.clear_interrupt();
            capture.listen(CaptureMode::None);

            capture.trigger_capture();
            ctx.delay.delay_micros(1);

            // The capture channel is not listening to events
            // however software trigger should still capture the phase
            assert_ne!(0, capture.events().phase());
            assert!(!capture.is_interrupt_set()); // No interrupt should be set since the channel is not listening to events
        }
    }

    #[test]
    fn test_capture_channels_disabled_does_not_capture_gpio_input(ctx: Context<'static>) {
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];
        let mut timer = ctx.mcpwm.capture_timer;

        timer.start();

        // Clear any pending interrupts on the capture channels
        for capture in captures {
            let input = unsafe { ctx.input.clone_unchecked() };
            let mut capture = capture.with_signal_input(input);

            capture.set_enable(false);
            capture.clear_interrupt();
            capture.listen(CaptureMode::AnyEdge);

            output.set_high();
            ctx.delay.delay_micros(1);
            output.set_low();
            ctx.delay.delay_micros(1);

            // The capture channel is disabled, so the captured phase should be zero and no
            // interrupt should be set
            assert_eq!(0, capture.events().phase());
            assert!(!capture.is_interrupt_set());

            capture.with_signal_input(NoPin);
        }
    }

    #[test]
    fn test_capture_channels_not_listening_does_not_capture_gpio_input(ctx: Context<'static>) {
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];
        let mut timer = ctx.mcpwm.capture_timer;

        timer.start();

        // Clear any pending interrupts on the capture channels
        for capture in captures {
            let input = unsafe { ctx.input.clone_unchecked() };
            let mut capture = capture.with_signal_input(input);

            capture.set_enable(true);
            capture.clear_interrupt();
            capture.listen(CaptureMode::None);

            output.set_high();
            ctx.delay.delay_micros(1);
            output.set_low();
            ctx.delay.delay_micros(1);

            // The capture channel is not listening, so the captured phase should be zero and no
            // interrupt should be set
            assert_eq!(0, capture.events().phase());
            assert!(!capture.is_interrupt_set());

            capture.with_signal_input(NoPin);
        }
    }

    #[test]
    fn test_capture_channels_trigger_capture(ctx: Context<'static>) {
        let mut timer = ctx.mcpwm.capture_timer;
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];

        // Apply a sync phase of 1000000 to the capture timer
        timer.start();
        timer.apply_config(CaptureTimerConfig::default().with_sync_phase(1000000));
        timer.trigger_sync();
        ctx.delay.delay_micros(5);

        // Trigger a capture event on each capture channel and verify that the captured phase is not
        // zero
        for mut capture in captures {
            capture.set_enable(true);
            capture.clear_interrupt();
            capture.listen(CaptureMode::AnyEdge);
            capture.trigger_capture();
            ctx.delay.delay_micros(5);
            // As we have waited for 5 microseconds, the capture phase should be greater than the
            // sync phase of 1,000,000
            assert!(
                capture.events().phase() > 1000000,
                "Capture phase should be greater than sync phase"
            );
        }
    }

    #[test]
    fn test_capture_timer_test_reset(ctx: Context<'static>) {
        let mut timer = ctx.mcpwm.capture_timer;

        // Apply a sync phase of 1234 to the capture timer
        // If test_capture_timer_test_apply_phase passes then these 2 lines
        // should work
        // Apply a sync phase of 1234 to the capture timer
        timer.apply_config(CaptureTimerConfig::default().with_sync_phase(1000000));
        timer.start();
        timer.trigger_sync();
        ctx.delay.delay_micros(5);

        // Reset the capture timer
        timer.reset();
        timer.start(); // Restart the timer after reset
        ctx.delay.delay_micros(1);

        // To read capture timer value, we need to trigger a capture event
        let captures = [ctx.mcpwm.capture0, ctx.mcpwm.capture1, ctx.mcpwm.capture2];
        // Trigger a capture event on each capture channel and verify that the captured phase is
        // equal to the sync phase
        for mut capture in captures {
            capture.set_enable(true);
            capture.trigger_capture();
            ctx.delay.delay_micros(1);

            // We don't know exactly what the capture phase will be after reset, but it should be
            // less than the sync phase.
            assert!(
                capture.events().phase() < 1000000,
                "Capture phase should be less than sync phase after reset"
            );
            // Timer is running we waited >1 microsecond so the capture phase should not be zero
            assert_ne!(
                0,
                capture.events().phase(),
                "Capture phase should not be zero"
            );
        }
    }

    #[test]
    fn test_capture_any_edge(mut ctx: Context<'static>) {
        // Setup capture 0 to capture either falling or rising edges
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());

        ctx.mcpwm.capture_timer.start();
        let mut capture = ctx.mcpwm.capture0.with_signal_input(ctx.input);
        capture.set_enable(true);
        capture.listen(CaptureMode::AnyEdge);

        output.set_high();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Rising, capture.events().edge());

        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Falling, capture.events().edge());
    }

    #[test]
    fn test_capture_unlisten_during_running(mut ctx: Context<'static>) {
        // Setup capture 0 to capture either falling or rising edges
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());

        ctx.mcpwm.capture_timer.start();
        let mut capture = ctx.mcpwm.capture0.with_signal_input(ctx.input);
        capture.set_enable(true);
        capture.listen(CaptureMode::AnyEdge);

        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert!(capture.is_interrupt_set());

        // Unlisten and clear the interrupt, then verify that the interrupt is no longer set
        capture.clear_interrupt();
        capture.unlisten();

        // Trigger a capture event and verify that the interrupt is not set
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert!(!capture.is_interrupt_set());
    }

    #[test]
    fn test_capture_set_disable_during_running(mut ctx: Context<'static>) {
        // Setup capture 0 to capture either falling or rising edges
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());

        ctx.mcpwm.capture_timer.start();
        let mut capture = ctx.mcpwm.capture0.with_signal_input(ctx.input);
        capture.set_enable(true);
        capture.listen(CaptureMode::AnyEdge);

        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert!(capture.is_interrupt_set());

        // Disable the capture channel and clear the interrupt, then verify that the interrupt is no
        // longer set
        capture.clear_interrupt();
        capture.set_enable(false);

        // Trigger a capture event and verify that the interrupt is not set
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert!(!capture.is_interrupt_set());
    }

    #[test]
    fn test_capture_with_invert(mut ctx: Context<'static>) {
        // Setup capture 0 to capture either falling or rising edges
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        ctx.mcpwm.capture_timer.start();

        // Setup capture 0 with invert enabled
        let mut capture = ctx.mcpwm.capture0.with_signal_input(ctx.input);
        capture.apply_config(CaptureChannelConfig::default().with_invert(true));
        capture.set_enable(true);
        capture.listen(CaptureMode::AnyEdge);

        // With invert enabled, the first edge should be falling
        output.set_high();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Falling, capture.events().edge());

        // The next edge should be rising
        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Rising, capture.events().edge());
    }

    #[test]
    fn test_capture_with_prescaler(mut ctx: Context<'static>) {
        // Setup capture 0 to capture either falling or rising edges
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        ctx.mcpwm.capture_timer.start();

        // Setup capture 0 with invert enabled
        let mut capture = ctx.mcpwm.capture0.with_signal_input(ctx.input);
        // Only capture the 256th edge
        capture.apply_config(CaptureChannelConfig::default().with_prescaler(255));
        capture.set_enable(true);
        capture.listen(CaptureMode::RisingEdge);

        let first_capture = capture.events();

        // The next 254 edges should be ignored, so the edge should still be rising
        for _ in 0..254 {
            output.set_low();
            ctx.delay.delay_micros(1);
            output.set_high(); // Rising edge
            ctx.delay.delay_micros(1);
            // These should be ignored, so the capture events should still be the same as the first
            // capture
            assert_eq!(first_capture, capture.events());
        }

        // This edge should be detected
        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high(); // Rising edge
        ctx.delay.delay_micros(1);
    }

    #[test]
    fn test_timer_set_counter(ctx: Context<'static>) {
        let mut timer = ctx.mcpwm.timer0; // Don't start the timer

        timer.set_counter(0, CounterDirection::Increasing);
        ctx.delay.delay_micros(1);
        assert_eq!((0, CounterDirection::Increasing), timer.status());

        timer.set_counter(1234, CounterDirection::Increasing);
        ctx.delay.delay_micros(1);
        assert_eq!((1234, CounterDirection::Increasing), timer.status());

        timer.set_counter(5553, CounterDirection::Increasing);
        ctx.delay.delay_micros(1);
        assert_eq!((5553, CounterDirection::Increasing), timer.status());
    }

    #[test]
    fn test_timer_sync_phase_from_sync_line(ctx: Context<'static>) {
        // setup sync line to be controlled by output pin
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        ctx.mcpwm.sync0.set_signal(ctx.input);

        let timer_cfg = ctx
            .clock_cfg
            .timer_clock_with_prescaler(u16::MAX, PwmWorkingMode::Increase, 0)
            .with_phase(25000);

        // create timer but don't start it
        let mut timer = ctx.mcpwm.timer0;
        timer.set_sync_in(ctx.mcpwm.sync0.get_sync_out());
        timer.set_counter(0, CounterDirection::Increasing);
        assert!(timer.apply_config(timer_cfg).is_ok());

        // before sync
        assert_eq!(0, timer.status().0);

        // sync the timer
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(25000, timer.status().0);

        // apply new sync phase
        let timer_cfg = timer_cfg.with_phase(12345);
        assert!(timer.apply_config(timer_cfg).is_ok());

        // sync the timer
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(12345, timer.status().0);
    }

    #[test]
    fn test_cap_timer_sync_phase_from_sync_line(ctx: Context<'static>) {
        // setup sync line to be controlled by output pin
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        ctx.mcpwm.sync0.set_signal(ctx.input);

        let mut cap_timer = ctx.mcpwm.capture_timer;
        cap_timer.set_sync_in(ctx.mcpwm.sync0.get_sync_out());

        let mut capture = ctx.mcpwm.capture0;
        capture.set_enable(true);

        // before sync initial phase == 0
        capture.trigger_capture();
        ctx.delay.delay_micros(1);
        assert_eq!(0, capture.events().phase());

        cap_timer.apply_config(CaptureTimerConfig::default().with_sync_phase(1234));

        // sync the timer
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);

        // Compare capture phase
        capture.trigger_capture();
        ctx.delay.delay_micros(1);
        assert_eq!(1234, capture.events().phase());

        // apply a new sync phase
        cap_timer.apply_config(CaptureTimerConfig::default().with_sync_phase(5324));

        // sync the timer
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);

        // Compare capture phase
        capture.trigger_capture();
        ctx.delay.delay_micros(1);
        assert_eq!(5324, capture.events().phase());
    }

    #[test]
    fn test_timer_sync_out_propagating_sync(ctx: Context<'static>) {
        // setup sync line to be controlled by output pin
        ctx.mcpwm.sync0.set_signal(ctx.input);

        // Small prescaler to ensure sync will fire often ~ 5uS
        let timer0_cfg = ctx
            .clock_cfg
            .timer_clock_with_prescaler(5, PwmWorkingMode::Increase, 0)
            .with_sync_out(SyncOutSelect::WhenEqualPeriod);

        // Create timer0 with sync out and start it
        let mut timer0 = ctx.mcpwm.timer0;
        assert!(timer0.apply_config(timer0_cfg).is_ok());
        timer0.start();

        let timer1_cfg = ctx
            .clock_cfg
            .timer_clock_with_prescaler(u16::MAX, PwmWorkingMode::Increase, 0)
            .with_phase(25000);

        // timer 1 listens to sync out of timer 0
        let mut timer1 = ctx.mcpwm.timer1;
        assert!(timer1.apply_config(timer1_cfg).is_ok());

        // before sync
        assert_eq!(0, timer1.status().0);

        // timer 1 should be synced from timer 0
        timer1.set_sync_in(timer0.get_sync_out());
        ctx.delay.delay_millis(1);
        assert_eq!(25000, timer1.status().0);

        let timer1_cfg = timer1_cfg.with_phase(12345);
        assert!(timer1.apply_config(timer1_cfg).is_ok());

        // timer 1 should be synced from timer 0 with new phase
        ctx.delay.delay_millis(1);
        assert_eq!(12345, timer1.status().0);
    }

    #[test]
    fn test_timer_apply_config_rejects_invalid_phase(ctx: Context<'static>) {
        let mut timer = ctx.mcpwm.timer0;

        let invalid_increase = ctx
            .clock_cfg
            .timer_clock_with_prescaler(10, PwmWorkingMode::Increase, 0)
            .with_phase(12);
        assert_eq!(
            Err(ConfigError::InvalidPhaseRange),
            timer.apply_config(invalid_increase)
        );

        let invalid_updown_increasing = ctx
            .clock_cfg
            .timer_clock_with_prescaler(10, PwmWorkingMode::UpDown, 0)
            .with_phase(11);
        assert_eq!(
            Err(ConfigError::InvalidPhaseRange),
            timer.apply_config(invalid_updown_increasing)
        );

        let invalid_updown_decreasing = ctx
            .clock_cfg
            .timer_clock_with_prescaler(10, PwmWorkingMode::UpDown, 0)
            .with_direction(CounterDirection::Decreasing)
            .with_phase(0);
        assert_eq!(
            Err(ConfigError::InvalidPhaseRange),
            timer.apply_config(invalid_updown_decreasing)
        );
    }

    #[test]
    fn test_sync_line_invert_triggers_on_falling_edge(ctx: Context<'static>) {
        // testing sync line invert
        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());
        ctx.mcpwm.sync0.set_signal(ctx.input);
        ctx.mcpwm.sync0.set_invert(true);

        // configure timer with sync phase of 2468
        let timer_cfg = ctx
            .clock_cfg
            .timer_clock_with_prescaler(u16::MAX, PwmWorkingMode::Increase, 0)
            .with_phase(2468);

        // setup timer but not running
        let mut timer = ctx.mcpwm.timer1;
        timer.set_sync_in(ctx.mcpwm.sync0.get_sync_out());
        timer.set_counter(0, CounterDirection::Increasing);
        assert!(timer.apply_config(timer_cfg).is_ok());

        assert_eq!(0, timer.status().0);

        // sync generated on falling edge
        // should stay zero
        output.set_high();
        ctx.delay.delay_micros(1);
        assert_eq!(0, timer.status().0);

        // should update to sync phase of 2468
        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(2468, timer.status().0);

        // Same test just different phase
        let timer_cfg = timer_cfg.with_phase(1357);
        assert!(timer.apply_config(timer_cfg).is_ok());

        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1357, timer.status().0);
    }
}
