//! TIMG Watchdog Tests

//% CHIP_FILTER: timergroup_driver_supported

#![no_std]
#![no_main]

use hil_test as _;

/// The watchdog tests arm the MWDT and rely on it *not* firing: a reset fails
/// the run. Long timeouts keep the margin comfortable on every chip.
#[embedded_test::tests(default_timeout = 30)]
mod wdt {
    use esp_hal::{
        delay::Delay,
        peripherals::TIMG0,
        time::Duration,
        timer::timg::{MwdtStage, MwdtStageAction, TimerGroup, Wdt},
    };

    struct Context {
        wdt: Wdt<TIMG0<'static>>,
        delay: Delay,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        Context {
            wdt: timg0.wdt,
            delay: Delay::new(),
        }
    }

    /// The requested timeout must be applied. On chips that have the
    /// `WDT_CONF_UPDATE_EN` bit the configuration only latches after the
    /// update strobe; without it the watchdog keeps its previous hold (a
    /// sub-second default after boot) and resets the chip long before the
    /// requested timeout elapses.
    #[test]
    fn test_timeout_is_applied(mut ctx: Context) {
        ctx.wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_secs(10));
        ctx.wdt.enable();

        // Surviving this delay proves the 10 s hold is in effect: the
        // default hold would have reset the chip after well under a second.
        ctx.delay.delay_millis(3_000);

        ctx.wdt.disable();
    }

    /// Stage actions must be applied. Turning stage 0 off stops the watchdog
    /// from resetting the system even while it stays enabled and unfed. On
    /// chips that have the `WDT_CONF_UPDATE_EN` bit the stage write needs
    /// the update strobe to take effect. (The stage is configured after
    /// `enable`, because enabling rewrites the stage actions.)
    #[test]
    fn test_stage_action_is_applied(mut ctx: Context) {
        ctx.wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_secs(2));
        ctx.wdt.enable();
        ctx.wdt
            .set_stage_action(MwdtStage::Stage0, MwdtStageAction::Off);

        // Surviving this delay proves the stage action took effect: stage 0
        // would still reset the system after the 2 s hold otherwise.
        ctx.delay.delay_millis(4_000);

        ctx.wdt.disable();
    }

    /// Disabling the watchdog must take effect: an enabled watchdog that is
    /// disabled and then left unfed must not reset the system.
    #[test]
    fn test_disable_is_applied(mut ctx: Context) {
        ctx.wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_secs(2));
        ctx.wdt.enable();

        ctx.delay.delay_millis(100);

        ctx.wdt.disable();

        // Surviving this delay proves the disable latched: the 2 s hold
        // would have reset the system otherwise.
        ctx.delay.delay_millis(4_000);
    }
}
