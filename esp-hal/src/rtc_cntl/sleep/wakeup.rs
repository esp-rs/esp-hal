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

use esp_sync::NonReentrantMutex;

use crate::rtc_cntl::{WakeupSource, sleep::RtcSleepConfig};

/// Which sleep the chip is entering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SleepKind {
    /// The digital domain keeps its state, and the sleep returns to the caller.
    Light,
    /// The chip resets when it wakes.
    Deep,
}

/// A resource that a wakeup source needs powered while the chip sleeps.
///
/// The names are the same for all chips. A request for a resource that the target chip does not
/// have, or that it cannot power down, does nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// The requested resources are different on each chip, and no wakeup source requests an oscillator
// yet.
#[allow(dead_code, reason = "the names are the same for all chips")]
pub(crate) enum SleepResource {
    /// The low-power peripherals, including the RTC IO pads.
    LpPeripherals,
    /// The low-power memory, which holds the program and the data of a low-power core.
    LpMemory,
    /// The high-performance peripherals, including the digital GPIO pads.
    HpPeripherals,
    /// The main crystal oscillator.
    Xtal,
    /// The fast RC oscillator.
    RcFast,
    /// The 32 kHz crystal oscillator.
    Xtal32k,
    /// The 32 kHz RC oscillator.
    Rc32k,
}

/// The sleep configuration, as the entry hook of a wakeup source can see it.
///
/// A hook can only prevent a power-down. It cannot request one, because the caller of the sleep
/// function selects how much of the chip to power down, and a wakeup source only adds the resources
/// that it needs. A hook thus clears bits, which is idempotent and commutative. The order of the
/// hooks cannot change the result, and the source with the highest demand wins.
///
/// Keep this property. A method that sets a power-down bit removes it.
pub(crate) struct WrappedSleepConfig<'a> {
    config: &'a mut RtcSleepConfig,
}

impl<'a> WrappedSleepConfig<'a> {
    pub(crate) fn new(config: &'a mut RtcSleepConfig) -> Self {
        Self { config }
    }

    /// Returns whether the chip is entering deep sleep, which resets it when it wakes.
    pub(crate) fn is_deep_sleep(&self) -> bool {
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
            SleepResource::Xtal => {
                cfg_select! {
                    soc_has_pmu => _config.pd_flags.set_pd_xtal(false),
                    // This flag has the opposite sense. It forces the crystal on.
                    _ => _config.set_xtal_fpu(true),
                }
            }
            SleepResource::RcFast => {
                cfg_select! {
                    soc_has_pmu => _config.pd_flags.set_pd_rc_fast(false),
                    _ => _config.set_int_8m_pd_en(false),
                }
            }
            SleepResource::Xtal32k => {
                cfg_select! {
                    soc_has_pmu => _config.pd_flags.set_pd_xtal32k(false),
                    _ => {}
                }
            }
            SleepResource::Rc32k => {
                cfg_select! {
                    esp32h2 => {}
                    soc_has_pmu => _config.pd_flags.set_pd_rc32k(false),
                    _ => {}
                }
            }
        }
    }
}

/// Runs at sleep entry, before the sleep configuration reaches hardware.
///
/// The configuration already holds the kind of the sleep, so a hook that needs it asks
/// [`WrappedSleepConfig::is_deep_sleep`]
pub(crate) type SleepEntryHook = fn(&mut WrappedSleepConfig<'_>);

/// Runs after a light sleep. A deep sleep resets the chip, which runs the initialization again.
pub(crate) type SleepExitHook = fn();

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
    /// Both hooks run with the flash accessible. The entry hook runs before esp-hal writes the
    /// sleep configuration to hardware, and the exit hook runs after the wake sequence restores
    /// it. Both hooks are part of sleep entry, so keep them short. Give them the
    /// [`ram`][crate::ram] attribute, to keep the flash out of the sleep path
    pub(crate) fn enable_with_hooks(
        self,
        entry: Option<SleepEntryHook>,
        exit: Option<SleepExitHook>,
    ) {
        HOOKS.with(|hooks| {
            hooks.entry[self as usize] = entry;
            hooks.exit[self as usize] = exit;

            set_mask_bit(self, true);
        })
    }

    /// Disables this source, and removes its hooks.
    pub(crate) fn disable(self) {
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

/// Runs the sleep-entry hook of every enabled source.
///
/// The mask as read at sleep entry selects the hooks. A hook can enable another source. The GPIO
/// hook does this, because it selects between the `ext0`, `ext1` and per-pin paths. The caller
/// therefore reads the mask again after the hooks. It does not run the hooks again until the mask
/// stops to change.
///
/// The caller writes the kind of the sleep to the configuration before this call, so that the hooks
/// can read it.
pub(crate) fn run_entry_hooks(config: &mut RtcSleepConfig) {
    let mut wrapped = WrappedSleepConfig::new(config);

    for source in enabled_sources() {
        let hook = HOOKS.with(|hooks| hooks.entry[source as usize]);
        if let Some(hook) = hook {
            hook(&mut wrapped);
        }
    }
}

/// Runs the post-wake hook of every enabled source. Only a light sleep calls this.
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
/// register with `gpio_wakeup_filter` and with the read-only `wakeup_cause`
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
