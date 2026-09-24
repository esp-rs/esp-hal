//! The hardware wakeup-enable mask, and the hooks that run around a sleep.
//!
//! The driver that owns the hardware enables a wakeup source. The caller of a sleep function does
//! not. The hardware wakeup-enable mask is the record of the request. It is a register in the
//! always-on domain, and it keeps its value while the chip is awake, through a light sleep, and
//! through a deep-sleep wake. Sleep entry reads the mask back, and calculates everything else from
//! it.
//!
//! A driver can also register hooks in the call that sets its mask bit. Use them to do work at
//! sleep entry, or to restore state after a light sleep.

use enumset::EnumSet;
use esp_sync::NonReentrantMutex;

use crate::{
    rtc_cntl::{WakeupSource, sleep::RtcSleepConfig},
    soc::clocks::ClockSource,
    time::Duration,
};

/// Which sleep the chip is entering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SleepKind {
    /// The digital domain keeps its state, and the sleep returns to the caller.
    Light,
    /// The chip resets when it wakes.
    Deep,
}

/// A power domain or an analog block that a wakeup source needs powered while the chip sleeps.
///
/// The names are the same for all chips. A request for a domain that the target chip does not
/// have, or that it cannot power down, does nothing.
///
/// A clock is not a domain. Ask for one with [`WrappedSleepConfig::keep_clock_running`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// The names are the same for all chips, and each chip uses a different subset.
#[allow(dead_code, reason = "the names are the same for all chips")]
pub(crate) enum SleepResource {
    /// The low-power peripherals, including the RTC IO pads.
    LpPeripherals,
    /// The low-power memory, which holds the program and the data of a low-power core.
    LpMemory,
    /// The high-performance peripherals, including the digital GPIO pads.
    HpPeripherals,
    /// The BBPLL and the analog I2C buses that configure it.
    Bbpll,
}

/// The sleep configuration, as the entry hook of a wakeup source can see it.
///
/// A hook can only relax the sleep. It can keep a power domain powered, keep a clock running,
/// refuse a light sleep, or shorten the sleep. The caller of the sleep function selects how much
/// of the chip to power down, and it arms the wake timer. A wakeup source only adds the resources
/// that it needs, refuses the sleep, or sets a shorter limit. It cannot request a power-down, stop
/// a clock, cancel a refusal, or lengthen a limit.
///
/// Each request is idempotent. The order of the hooks cannot change the result. The source with
/// the strongest request wins. One refusal is enough. The shortest limit wins.
///
/// Keep this property. A method that requests a power-down, stops a clock, cancels a refusal, or
/// lengthens a limit removes it.
#[instability::unstable]
pub struct WrappedSleepConfig<'a> {
    config: &'a mut RtcSleepConfig,
    clocks: EnumSet<ClockSource>,
    refused: bool,
    limit: Option<Duration>,
}

impl<'a> WrappedSleepConfig<'a> {
    pub(crate) fn new(config: &'a mut RtcSleepConfig) -> Self {
        Self {
            config,
            clocks: EnumSet::empty(),
            refused: false,
            limit: None,
        }
    }

    /// Returns whether the chip is entering deep sleep, which resets it when it wakes.
    #[instability::unstable]
    pub fn is_deep_sleep(&self) -> bool {
        self.config.is_deep_sleep()
    }

    /// Keeps `resource` powered during the sleep.
    pub(crate) fn keep_alive(&mut self, resource: SleepResource) {
        // One function for all chips. It keeps the code of one resource together for all chips, so
        // that a reader can compare the chips, and a new resource needs one change only.
        let _config = &mut self.config;
        match resource {
            SleepResource::LpPeripherals => {
                cfg_select! {
                    // esp32h2 powers its low-power peripherals from the top domain, so it has no
                    // separate flag to clear.
                    esp32h2 => {}
                    soc_has_pmu => _config.pd_flags.set_pd_lp_periph(false),
                    _ => _config.set_rtc_peri_pd_en(false),
                }
            }
            SleepResource::LpMemory => {
                cfg_select! {
                    // The PMU chips keep the contents of the low-power memory through a sleep in
                    // retention mode, so there is no power-down to prevent.
                    any(esp32, esp32s2, esp32s3) => _config.set_rtc_slowmem_pd_en(false),
                    _ => {}
                }
            }
            SleepResource::HpPeripherals => {
                cfg_select! {
                    // esp32, esp32s2 and esp32h2 cannot power down the high-performance peripherals
                    // separately, so there is nothing to prevent.
                    any(esp32, esp32s2, esp32h2) => {}
                    soc_has_pmu => _config.pd_flags.set_pd_hp_periph(false),
                    _ => _config.set_dig_peri_pd_en(false),
                }
            }
            SleepResource::Bbpll => {
                cfg_select! {
                    esp32c6 => _config.pd_flags.set_pd_bbpll(false),
                    _ => {}
                }
            }
        }
    }

    /// Keeps the BBPLL, and the analog I2C buses that configure it, powered during a light sleep.
    ///
    /// A radio that needs the PLL immediately after the wake asks for this. It costs about 2 mA
    /// of sleep current on the ESP32-C6. Only the ESP32-C6 honors the request. Other chips ignore
    /// it.
    #[instability::unstable]
    pub fn keep_bbpll_powered(&mut self) {
        self.keep_alive(SleepResource::Bbpll);
    }

    /// Keeps `source` running during the sleep.
    ///
    /// Ask a clock tree node which source it runs on, and name the answer. A source that the chip
    /// keeps running anyway, or that it cannot power down, needs nothing.
    #[instability::unstable]
    pub fn keep_clock_running(&mut self, source: ClockSource) {
        self.clocks.insert(source);
    }

    /// Refuses a light sleep.
    ///
    /// Call this when software state makes the sleep unsafe and the hardware cannot see that
    /// state. One call is enough. A later hook cannot cancel it.
    ///
    /// A deep sleep ignores the call. The wake resets the chip, so there is no caller to report
    /// the refusal to. Read [`Self::is_deep_sleep`] when other work in the hook depends on the
    /// kind of sleep.
    ///
    /// The hardware also rejects a sleep when an enabled source is already asserted at sleep
    /// entry. This call is the software refusal.
    #[instability::unstable]
    pub fn reject_sleep(&mut self) {
        self.refused = true;
    }

    /// Limits this sleep to `duration`.
    ///
    /// The duration starts when the driver programs the wake timer, after every hook has run. The
    /// shortest request wins. A later hook cannot make it longer.
    ///
    /// The driver clamps the wake timer to the limit. When the timer wakeup source is not enabled,
    /// the limit enables it for this sleep. Nothing else wakes the chip at the limit. After a
    /// light sleep the driver restores the timer. A deadline the caller armed stays armed. A timer
    /// that only the limit enabled is disabled again.
    ///
    /// The limit applies to deep sleep as well. The wake resets the chip. Read
    /// [`Self::is_deep_sleep`] when the hook should bound only one kind of sleep.
    ///
    /// A duration that the sleep transition cannot catch does not start the sleep. A light sleep
    /// then returns as if it had ended. A deep sleep without rejection panics, because it cannot
    /// return and the chip does not wake. This is not a refusal. Use [`Self::reject_sleep`] to
    /// refuse a light sleep. A duration of zero is too short to sleep.
    #[instability::unstable]
    pub fn limit_sleep(&mut self, duration: Duration) {
        let already_shorter = match self.limit {
            Some(current) => current.as_micros() <= duration.as_micros(),
            None => false,
        };
        if !already_shorter {
            self.limit = Some(duration);
        }
    }

    /// Prevents the power-down of every clock source a hook asked for.
    ///
    /// The set is what makes the requests of the hooks independent of their order.
    fn apply_clock_requests(&mut self) {
        // One function for all chips, like `keep_alive`.
        let _config = &mut self.config;
        for source in self.clocks {
            match source {
                ClockSource::XtalClk => {
                    cfg_select! {
                        soc_has_pmu => _config.pd_flags.set_pd_xtal(false),
                        // This flag has the opposite sense. It forces the crystal on.
                        _ => _config.set_xtal_fpu(true),
                    }
                }
                ClockSource::RcFastClk => {
                    cfg_select! {
                        soc_has_pmu => _config.pd_flags.set_pd_rc_fast(false),
                        _ => _config.set_int_8m_pd_en(false),
                    }
                }
                #[cfg(use_xtal32k)]
                ClockSource::Xtal32kClk => {
                    cfg_select! {
                        soc_has_pmu => _config.pd_flags.set_pd_xtal32k(false),
                        _ => {}
                    }
                }
                #[cfg(soc_has_clock_node_rc32k_clk)]
                ClockSource::Rc32kClk => {
                    cfg_select! {
                        // esp32h2 has no separate flag for the 32 kHz RC oscillator.
                        esp32h2 => {}
                        _ => _config.pd_flags.set_pd_rc32k(false),
                    }
                }
                // The slow RC oscillator runs in the always-on domain, and the external 32 kHz
                // oscillator arrives on a pad, so neither has a power-down to prevent.
                #[cfg(soc_has_clock_node_osc_slow_clk)]
                ClockSource::OscSlowClk => {}
                ClockSource::RcSlowClk => {}
            }
        }
    }
}

/// Runs at sleep entry, before the sleep configuration reaches hardware.
///
/// The configuration already holds the kind of the sleep, so a hook that needs it asks
/// [`WrappedSleepConfig::is_deep_sleep`]. A hook keeps a clock running with
/// [`WrappedSleepConfig::keep_clock_running`]. A hook refuses a light sleep with
/// [`WrappedSleepConfig::reject_sleep`].
///
/// The hook can run with interrupts disabled, for example from automatic light sleep. Do not
/// allocate, take a blocking lock, or log in it.
#[instability::unstable]
pub type SleepEntryHook = fn(&mut WrappedSleepConfig<'_>);

/// Runs after a light sleep.
///
/// It also runs when a hook refuses a light sleep. An entry hook may already have changed a pad,
/// and the exit hook puts that pad back. A deep sleep resets the chip, which runs the
/// initialization again.
#[instability::unstable]
pub type SleepExitHook = fn();

for_each_wakeup_source! {
    (all $( ($variant:ident, $bit:literal) ),*) => {
        /// One slot for each bit of the mask, so that a source can use its bit as the index.
        const HOOK_SLOTS: usize = {
            let mut highest = 0;
            $(
                if $bit > highest {
                    highest = $bit;
                }
            )*
            highest + 1
        };
    };
}

struct Hooks {
    entry: [Option<SleepEntryHook>; HOOK_SLOTS],
    exit: [Option<SleepExitHook>; HOOK_SLOTS],
}

/// The same lock protects the hooks and the mask bits, so the two always agree.
static HOOKS: NonReentrantMutex<Hooks> = NonReentrantMutex::new(Hooks {
    entry: [None; HOOK_SLOTS],
    exit: [None; HOOK_SLOTS],
});

impl WakeupSource {
    /// Enables this source, so that it can end a sleep.
    ///
    /// The source stays enabled until [`Self::disable`] is called. It also stays enabled through a
    /// sleep and through a deep-sleep wake. While the chip is awake, an enabled source does
    /// nothing.
    // On a chip where all the implemented sources need hooks, nothing calls this function.
    #[allow(dead_code, reason = "not every chip has such a source yet")]
    pub(crate) fn enable(self) {
        self.enable_with_hooks(None, None)
    }

    /// Enables this source and registers the hooks that run around a sleep.
    ///
    /// A source has one slot for each kind of hook, so a second call replaces the hooks of the
    /// first call. One driver owns each source, so only that driver can replace its own hooks.
    /// A call for a source that is already enabled does this.
    ///
    /// Both hooks run with the flash accessible, so they need no [`ram`][crate::ram] attribute. The
    /// entry hook runs before esp-hal writes the sleep configuration to hardware, and the exit hook
    /// runs after the wake sequence restores it. Both hooks are part of sleep entry, so keep them
    /// short.
    ///
    /// Only the driver that owns the source calls this. esp-hal owns the sources of its own
    /// drivers, for example the timer source. A call for such a source replaces the hooks of that
    /// driver, and the driver then does not work through a sleep.
    #[instability::unstable]
    pub fn enable_with_hooks(self, entry: Option<SleepEntryHook>, exit: Option<SleepExitHook>) {
        HOOKS.with(|hooks| {
            hooks.entry[self as usize] = entry;
            hooks.exit[self as usize] = exit;

            set_mask_bit(self, true);
        })
    }

    /// Disables this source, and removes its hooks.
    ///
    /// Only the driver that owns the source calls this, as for [`Self::enable_with_hooks`].
    #[instability::unstable]
    pub fn disable(self) {
        HOOKS.with(|hooks| {
            hooks.entry[self as usize] = None;
            hooks.exit[self as usize] = None;

            set_mask_bit(self, false);
        })
    }
}

/// Returns the enabled wakeup sources, as the hardware mask records them.
pub(crate) fn enabled_sources() -> enumset::EnumSet<WakeupSource> {
    enumset::EnumSet::from_u32_truncated(mask())
}

/// Returns the sources that reject the next sleep.
///
/// A source rejects a sleep if the source is already asserted when the sleep starts. Without the
/// rejection, the chip sleeps through the event that the caller wants to wake on. The reject
/// sources are the enabled wake sources that the chip can also reject on. A chip cannot reject on
/// every source that it can wake from. esp32 rejects on GPIO and SDIO only, and esp32c2, esp32c3,
/// esp32s2 and esp32s3 do not reject on a UART.
pub(crate) fn reject_mask() -> u32 {
    mask() & property!("sleep.rejectable_mask")
}

/// What the entry hooks asked for, other than power domains and clocks.
///
/// The clock requests reach the configuration before this value is returned, and so does the
/// wake timer. The refusal does not, because only the caller knows what to do with it.
pub(crate) struct SleepEntry {
    /// A hook called [`WrappedSleepConfig::reject_sleep`].
    pub(crate) refused: bool,

    /// How [`WrappedSleepConfig::limit_sleep`] changed the wake timer.
    ///
    /// `None` when no hook set a limit. The caller restores the timer on every path that does not
    /// sleep, and after a light sleep.
    #[cfg(sleep_has_wakeup_source_timer)]
    pub(crate) clamp: Option<super::timer::LimitClamp>,
}

/// Runs the sleep-entry hook of every enabled source.
///
/// One [`WrappedSleepConfig::reject_sleep`] is enough. The caller ignores the refusal for a deep
/// sleep. The wake resets the chip, so there is no caller to report the refusal to. The shortest
/// [`WrappedSleepConfig::limit_sleep`] wins.
///
/// The mask as read at sleep entry selects the hooks. A hook can enable another source. The GPIO
/// hook does this, because it selects between the `ext0`, `ext1` and per-pin paths. The caller
/// therefore reads the mask again after the hooks. It does not run the hooks again until the mask
/// stops to change.
///
/// The caller writes the kind of the sleep to the configuration before this call, so that the hooks
/// can read it.
pub(crate) fn run_entry_hooks(config: &mut RtcSleepConfig) -> SleepEntry {
    let mut wrapped = WrappedSleepConfig::new(config);

    for source in enabled_sources() {
        let hook = HOOKS.with(|hooks| hooks.entry[source as usize]);
        if let Some(hook) = hook {
            hook(&mut wrapped);
        }
    }

    // The clamp writes the comparator, so it runs after every hook has asked for its limit. A
    // source that the clamp enables has not run its entry hook. It runs here, where the requests
    // of a hook still reach the configuration.
    #[cfg(sleep_has_wakeup_source_timer)]
    let clamp = wrapped.limit.map(|limit| {
        let clamp = super::timer::clamp_to_limit(limit);
        clamp.apply_entry_hook(&mut wrapped);
        clamp
    });

    wrapped.apply_clock_requests();

    SleepEntry {
        refused: wrapped.refused,
        #[cfg(sleep_has_wakeup_source_timer)]
        clamp,
    }
}

/// Runs the post-wake hook of every enabled source.
///
/// A light sleep calls this after the wake. A refused light sleep calls it too, so an entry hook
/// can undo a pad change. A sleep that a limit ends before it starts calls it for the same reason.
/// A deep sleep that resets the chip does not call this.
pub(crate) fn run_exit_hooks() {
    for source in enabled_sources() {
        let hook = HOOKS.with(|hooks| hooks.exit[source as usize]);
        if let Some(hook) = hook {
            hook();
        }
    }
}

/// Reads the wakeup-enable mask back from hardware.
pub(crate) fn mask() -> u32 {
    let reg = cfg_select! {
        soc_has_pmu => crate::peripherals::PMU::regs().slp_wakeup_cntl2(),
        _ => crate::peripherals::LPWR::regs().wakeup_state(),
    };
    reg.read().wakeup_ena().bits() as _
}

/// Writes the wakeup-enable mask.
///
/// The function writes the field and not the register, because on esp32 the mask shares its
/// register with `gpio_wakeup_filter` and with the read-only `wakeup_cause`.
pub(crate) fn set_mask(mask: u32) {
    let reg = cfg_select! {
        soc_has_pmu => crate::peripherals::PMU::regs().slp_wakeup_cntl2(),
        _ => crate::peripherals::LPWR::regs().wakeup_state(),
    };
    reg.modify(|_, w| unsafe { w.wakeup_ena().bits(mask as _) });
}

fn set_mask_bit(source: WakeupSource, enable: bool) {
    let bit = 1 << source as u32;
    let current = mask();

    set_mask(if enable {
        current | bit
    } else {
        current & !bit
    });
}
