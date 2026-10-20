//! The sleep alarm.
//!
//! The alarm is the first comparator of the LP timer. ESP-IDF also reserves that comparator for the
//! sleep wakeup. An armed alarm is a standing request, like every other wakeup source. It stays
//! armed through the wake that it causes, and it ends only when the caller clears it.

use crate::{
    peripherals::RTC_TIMER,
    rtc_cntl::{WakeupSource, sleep::WrappedSleepConfig},
    time::{Duration, Instant},
};

/// The time that ESP-IDF reserves between the arming of the alarm and the start of the sleep, in
/// slow-clock ticks.
///
/// This is `SLEEP_TIMER_ALARM_TO_SLEEP_TICKS`, and its comment gives 80 µs for it. The sleep
/// transition cannot catch a nearer deadline, and the 48-bit counter needs decades to reach the
/// deadline again.
const ALARM_TO_SLEEP_TICKS: u64 = 16;

/// Arms the alarm for `deadline`, and enables the timer wakeup source.
pub(crate) fn set_deadline(deadline: Instant) {
    let now = Instant::now();

    // A deadline in the past must stay in the past. That is how the caller learns about it, as a
    // rejected light sleep or as a panic in `sleep_deep`.
    let ticks = if deadline >= now {
        let ahead = crate::clock::us_to_rtc_ticks((deadline - now).as_micros());
        crate::rtc_cntl::time_since_boot_raw().saturating_add(ahead)
    } else {
        let behind = crate::clock::us_to_rtc_ticks((now - deadline).as_micros());
        crate::rtc_cntl::time_since_boot_raw().saturating_sub(behind)
    };

    arm(ticks);

    WakeupSource::Timer.enable_with_hooks(Some(entry_hook), None);
}

/// Disarms the alarm, and disables the timer wakeup source.
pub(crate) fn clear_deadline() {
    WakeupSource::Timer.disable();
    disarm();
}

/// Returns whether the armed deadline is too near for the sleep transition to catch it.
///
/// A deep sleep that misses the alarm does not wake. The transition is not rejected, and the
/// counter needs decades to reach the deadline again. A light sleep on most chips rejects, because
/// the timer is a reject source. esp32 cannot reject on the timer, so a missed alarm is not safe
/// there either.
///
/// The comparator holds the deadline, so esp-hal keeps no copy of it. The target of the comparator
/// is readable on every chip, also where the alarm enable beside it is not readable. Only a caller
/// that finds the timer source enabled calls this function, so the target always comes from this
/// run.
pub(crate) fn deadline_missed() -> bool {
    target_ticks() < crate::rtc_cntl::time_since_boot_raw() + ALARM_TO_SLEEP_TICKS
}

/// How [`clamp_to_limit`] changed the wake timer.
///
/// A limit applies to one sleep. It must not replace a deadline the caller armed. Restore it with
/// [`restore_after_limit`] on every path that returns to the caller.
#[must_use = "restore the wake timer after the sleep"]
pub(crate) enum LimitClamp {
    /// The armed deadline is at or before the limit, and the transition can catch it.
    Unchanged,
    /// The limit replaced the armed deadline. `previous` is that deadline, in RTC ticks.
    Shortened {
        /// The comparator value from before the clamp.
        previous: u64,
    },
    /// The timer source was off, so the limit enabled it.
    Enabled,
    /// The sleep does not wake at or before the limit.
    ///
    /// The limit is already due, the transition cannot catch the limit, or the armed deadline is
    /// sooner and the transition cannot catch that deadline. The comparator is unchanged. The
    /// sleep must not start.
    TooShort,
}

impl LimitClamp {
    /// Runs the timer entry hook when this limit enabled the source.
    ///
    /// The hook keeps the low-power peripherals powered on chips whose comparator is not in the
    /// always-on domain. The hook of a source that was already enabled has run. Hooks do not run
    /// again, so a source that this limit enabled would otherwise sleep without that request.
    pub(crate) fn apply_entry_hook(&self, config: &mut WrappedSleepConfig<'_>) {
        if matches!(self, Self::Enabled) {
            entry_hook(config);
        }
    }
}

/// Shortens the wake timer so the sleep lasts at most `limit`.
///
/// The duration is measured from this call, on the RTC counter the comparator uses. An armed
/// deadline that is sooner than the limit stays, when the transition can catch that deadline.
///
/// When the timer wakeup source is off, the limit enables it. Nothing else wakes the chip at the
/// limit.
///
/// Returns [`LimitClamp::TooShort`] when the sleep does not wake at or before the limit. The
/// timer is unchanged. The caller must not start the sleep. esp32 cannot reject on the timer, so a
/// missed alarm does not wake the chip.
pub(crate) fn clamp_to_limit(limit: Duration) -> LimitClamp {
    let now = crate::rtc_cntl::time_since_boot_raw();
    let ahead = crate::clock::us_to_rtc_ticks(limit.as_micros());
    let limit_ticks = now.saturating_add(ahead);

    // Same window as `deadline_missed`. Check it before any write. A later armed deadline must
    // not hide a limit the transition cannot catch, and arming that limit clears the status
    // of the caller's deadline.
    if limit_ticks < now + ALARM_TO_SLEEP_TICKS {
        return LimitClamp::TooShort;
    }

    if super::enabled_sources().contains(WakeupSource::Timer) {
        let current = target_ticks();
        if current <= limit_ticks {
            // The armed deadline is sooner. When the transition cannot catch it, the chip does
            // not wake at the limit either. esp32 cannot reject on the timer.
            if current < now + ALARM_TO_SLEEP_TICKS {
                return LimitClamp::TooShort;
            }
            LimitClamp::Unchanged
        } else {
            arm(limit_ticks);
            LimitClamp::Shortened { previous: current }
        }
    } else {
        arm(limit_ticks);
        WakeupSource::Timer.enable_with_hooks(Some(entry_hook), None);
        LimitClamp::Enabled
    }
}

/// Puts the wake timer back to the state from before [`clamp_to_limit`].
pub(crate) fn restore_after_limit(clamp: LimitClamp) {
    match clamp {
        LimitClamp::Unchanged | LimitClamp::TooShort => {}
        LimitClamp::Shortened { previous } => arm(previous),
        LimitClamp::Enabled => clear_deadline(),
    }
}

/// Reads the comparator target, in RTC counter ticks.
fn target_ticks() -> u64 {
    let regs = RTC_TIMER::regs();

    let (low, high) = cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4, esp32s31) => (
            regs.tar0_low().read().main_timer_tar_low0().bits(),
            regs.tar0_high().read().main_timer_tar_high0().bits(),
        ),
        _ => (
            regs.slp_timer0().read().slp_val_lo().bits(),
            regs.slp_timer1().read().slp_val_hi().bits(),
        ),
    };

    (u64::from(high) << 32) | u64::from(low)
}

#[crate::ram]
fn entry_hook(config: &mut WrappedSleepConfig<'_>) {
    // The PMU chips run the comparator from the always-on domain. ESP-IDF also powers their
    // low-power peripherals down while a timer wake is armed.
    if !cfg!(soc_has_pmu) {
        config.keep_alive(super::SleepResource::LpPeripherals);
    }
}

/// Writes the comparator, and clears the status that the previous deadline can leave set.
///
/// Only this function clears the status. Sleep entry must not clear it on its own, because the
/// status is the only report of an expired deadline. Arming a new deadline clears it here.
fn arm(ticks: u64) {
    let low = (ticks & 0xffff_ffff) as u32;
    let high = ((ticks >> 32) & 0xffff) as u16;
    let regs = RTC_TIMER::regs();

    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4, esp32s31) => {
            regs.int_clr().write(|w| w.soc_wakeup().clear_bit_by_one());
            regs.tar0_low()
                .write(|w| unsafe { w.main_timer_tar_low0().bits(low) });
            regs.tar0_high()
                .write(|w| unsafe { w.main_timer_tar_high0().bits(high) });
            regs.tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().set_bit());
        }
        _ => {
            regs.int_clr().write(|w| w.main_timer().clear_bit_by_one());
            regs.slp_timer0()
                .write(|w| unsafe { w.slp_val_lo().bits(low) });
            regs.slp_timer1().write(|w| unsafe {
                w.slp_val_hi().bits(high);
                w.main_timer_alarm_en().set_bit()
            });
        }
    }
}

fn disarm() {
    let regs = RTC_TIMER::regs();

    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4, esp32s31) => {
            regs.tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().clear_bit());
            regs.int_clr().write(|w| w.soc_wakeup().clear_bit_by_one());
        }
        _ => {
            // The alarm enable is in the same register as the high half of the target. A write of
            // the target therefore arms the alarm again.
            regs.slp_timer1()
                .write(|w| unsafe { w.slp_val_hi().bits(0) });
            regs.int_clr().write(|w| w.main_timer().clear_bit_by_one());
        }
    }
}
