//! The hardware wakeup-enable mask, and the hooks that run around a sleep.
//!
//! A wakeup source is enabled by the driver that owns the hardware, not by the caller of a sleep
//! function. The record of that intent is the hardware wakeup-enable mask itself: a register in the
//! always-on domain that keeps its value while the chip is awake, across a light sleep and across a
//! deep-sleep wake. Sleep entry reads the mask back and derives everything else from it.
//!
//! A driver that needs work done at sleep entry, or state restored after a light sleep, registers
//! hooks in the same call that sets its mask bit.

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
/// The vocabulary is chip-agnostic. Asking for a resource that the target chip does not have, or
/// that it cannot power down in the first place, does nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// Which resources are asked for varies per chip, and no wakeup source votes for the oscillators
// yet.
#[allow(dead_code, reason = "the vocabulary is chip-agnostic")]
pub(crate) enum SleepResource {
    /// The low-power peripherals, including the RTC IO pads.
    LpPeripherals,
    /// The low-power memory, which holds a low-power core's program and data.
    LpMemory,
    /// The high-performance peripherals, including the digital GPIO pads.
    HpPeripherals,
    /// The main crystal oscillator.
    Xtal,
    /// The fast RC oscillator.
    RcFast,
    /// The 32kHz crystal oscillator.
    Xtal32k,
    /// The 32kHz RC oscillator.
    Rc32k,
}

/// The sleep configuration, as a wakeup source's entry hook sees it.
///
/// A hook may only *prevent* a power-down, never request one: how deeply to power down is a
/// property of the sleep the caller asked for, while keeping a resource alive is a requirement of
/// the wakeup source. Clearing bits is idempotent and commutative, so the order in which the hooks
/// run cannot change the outcome, and the most conservative source wins.
///
/// Keep it that way. A method that sets a power-down bit would destroy that property.
pub(crate) struct WrappedSleepConfig<'a> {
    config: &'a mut RtcSleepConfig,
}

impl<'a> WrappedSleepConfig<'a> {
    pub(crate) fn new(config: &'a mut RtcSleepConfig) -> Self {
        Self { config }
    }

    /// Keeps `resource` powered during the sleep.
    pub(crate) fn keep_alive(&mut self, resource: SleepResource) {
        // One implementation for every chip, with the per-chip answers for one resource kept
        // together, so that a reader sees them side by side and a new resource is one edit.
        let _config = &mut self.config;
        match resource {
            SleepResource::LpPeripherals => {
                cfg_select! {
                    // esp32h2 powers its low-power peripherals with the top domain, so it has no
                    // flag of its own to clear here.
                    esp32h2 => {}
                    soc_has_pmu => _config.pd_flags.set_pd_lp_periph(false),
                    _ => _config.set_rtc_peri_pd_en(false),
                }
            }
            SleepResource::LpMemory => {
                cfg_select! {
                    // The PMU chips retain the low-power memory through a sleep instead of keeping
                    // it powered, so there is no power-down to prevent.
                    any(esp32, esp32s2, esp32s3) => _config.set_rtc_slowmem_pd_en(false),
                    _ => {}
                }
            }
            SleepResource::HpPeripherals => {
                cfg_select! {
                    // esp32, esp32s2 and esp32h2 cannot power down the high-performance
                    // peripherals on their own, so there is nothing to prevent.
                    any(esp32, esp32s2, esp32h2) => {}
                    soc_has_pmu => _config.pd_flags.set_pd_hp_periph(false),
                    _ => _config.set_dig_peri_pd_en(false),
                }
            }
            SleepResource::Xtal => {
                cfg_select! {
                    soc_has_pmu => _config.pd_flags.set_pd_xtal(false),
                    // Inverted here: the flag forces the crystal on rather than off.
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
pub(crate) type SleepEntryHook = fn(SleepKind, &mut WrappedSleepConfig<'_>);

/// Runs after a light sleep. Deep sleep resets the chip, so it re-runs initialisation instead.
pub(crate) type SleepExitHook = fn();

for_each_wakeup_source! {
    (all $( ($variant:ident, $bit:literal) ),*) => {
        /// One slot per bit the mask can hold, so that a source indexes its own slot.
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

/// The hooks are guarded by the same lock as the mask bits, so the two can never disagree.
static HOOKS: NonReentrantMutex<Hooks> = NonReentrantMutex::new(Hooks {
    entry: [None; HOOK_SLOTS],
    exit: [None; HOOK_SLOTS],
});

impl WakeupSource {
    /// Enables this source, so that it can end a sleep.
    ///
    /// The source stays enabled until [`Self::disable`] is called, including across sleeps and
    /// across a deep-sleep wake. Enabling a source is inert while the chip is awake.
    // A chip whose only implemented sources need hooks has no caller for this.
    #[allow(dead_code, reason = "not every chip has such a source yet")]
    pub(crate) fn enable(self) {
        self.enable_with_hooks(None, None)
    }

    /// Enables this source and registers the hooks that run around a sleep.
    ///
    /// A source has one slot per hook kind, so a second owner of the same source replaces the
    /// hooks of the first. Since a source belongs to one driver, that only happens when a driver
    /// re-registers its own hooks, which is what enabling an already enabled source does.
    ///
    /// Both hooks run with the flash accessible: the entry hook before the sleep configuration
    /// reaches hardware, the exit hook after the wake sequence has restored it. A hook and what it
    /// calls therefore need no [`ram`][crate::ram] attribute. They do run inside sleep entry, so
    /// keep them short.
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

    /// Disables this source, and forgets its hooks.
    pub(crate) fn disable(self) {
        HOOKS.with(|hooks| {
            hooks.entry[self as usize] = None;
            hooks.exit[self as usize] = None;

            set_mask_bit(self, false);
        })
    }
}

/// Returns the enabled wakeup sources, as recorded by the hardware.
pub(crate) fn enabled_sources() -> enumset::EnumSet<WakeupSource> {
    enumset::EnumSet::from_u32_truncated(mask())
}

/// Returns the sources that reject the sleep about to be requested.
///
/// A source rejects a sleep that is requested while the source is already asserted, which would
/// otherwise sleep through the event the caller wants to wake on. The reject sources are the
/// enabled wake sources, narrowed to those the chip can reject on, which is not every source it
/// can wake from: esp32 rejects on GPIO and SDIO only, and esp32c2, esp32c3, esp32s2 and esp32s3
/// do not reject on a UART.
pub(crate) fn reject_mask() -> u32 {
    mask() & property!("sleep.rejectable_mask")
}

/// Returns whether any wakeup source that configures pads is enabled.
///
/// Read this before the mask is cleared at init: it decides whether the pads a previous run armed
/// need releasing, the way ESP-IDF gates `esp_deep_sleep_wakeup_io_reset`.
#[cfg(any(sleep_ext1_version = "2", sleep_ext1_version = "3"))]
pub(crate) fn io_wake_enabled() -> bool {
    let sources = enabled_sources();

    #[allow(unused_mut)]
    let mut enabled = sources.contains(WakeupSource::Gpio);

    #[cfg(sleep_has_wakeup_source_ext0)]
    {
        enabled |= sources.contains(WakeupSource::Ext0);
    }
    #[cfg(sleep_has_wakeup_source_ext1)]
    {
        enabled |= sources.contains(WakeupSource::Ext1);
    }

    enabled
}

/// Runs every enabled source's sleep-entry hook.
///
/// Hook selection uses the mask as it reads at entry. A hook may enable a source of its own — GPIO
/// does, because it decides between the `ext0`, `ext1` and per-pin paths here — and the caller
/// re-reads the mask afterwards rather than looking for a fixed point.
pub(crate) fn run_entry_hooks(kind: SleepKind, config: &mut RtcSleepConfig) {
    let mut wrapped = WrappedSleepConfig::new(config);

    for source in enabled_sources() {
        let hook = HOOKS.with(|hooks| hooks.entry[source as usize]);
        if let Some(hook) = hook {
            hook(kind, &mut wrapped);
        }
    }
}

/// Runs every enabled source's post-wake hook. Light sleep only.
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
/// The write is field-level, because on esp32 the mask shares its register with
/// `gpio_wakeup_filter` and the read-only `wakeup_cause`.
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
