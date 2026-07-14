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
        gpio::{AnyPin, Level, Output, OutputConfig, Pin},
        mcpwm::{
            AnyMcPwm,
            McPwm,
            PeripheralClockConfig,
            capture::{CaptureEdge, CaptureMode, CaptureTimerConfig},
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
            mcpwm: McPwm::new(AnyMcPwm::from(peripherals.MCPWM0), clock_cfg),
            input: din,
            output: dout,
            delay: Delay::new(),
            clock_cfg,
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

        output.set_high();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Rising, capture.events().edge());

        output.set_low();
        ctx.delay.delay_micros(1);
        assert_eq!(CaptureEdge::Falling, capture.events().edge());
    }

    #[test]
    fn test_timer_set_counter(ctx: Context<'static>) {
        // create timer but don't start it
        let mut timer = ctx.mcpwm.timer0;

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
        assert_eq!(0, capture.events().time());

        cap_timer.apply_config(CaptureTimerConfig::default().with_sync_phase(1234));

        // sync the timer
        output.set_high();
        ctx.delay.delay_micros(1);
        output.set_low();
        ctx.delay.delay_micros(1);

        // Compare capture phase
        capture.trigger_capture();
        ctx.delay.delay_micros(1);
        assert_eq!(1234, capture.events().time());

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
        assert_eq!(5324, capture.events().time());
    }

    #[test]
    fn test_timer_sync_out_propagating_sync(ctx: Context<'static>) {
        // setup sync line to be controlled by output pin
        ctx.mcpwm.sync0.set_signal(ctx.input);

        // Small prescaler to ensure sync will fire often ~ 5uS
        let timer0_cfg = ctx
            .clock_cfg
            .timer_clock_with_prescaler(5, PwmWorkingMode::Increase, 0)
            .with_sync_out(SyncOutSelect::SyncWhenEqualPeriod);

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
