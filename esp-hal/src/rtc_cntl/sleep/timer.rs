//! The sleep alarm.
//!
//! The alarm is the first comparator of the LP timer. ESP-IDF also reserves that comparator for the
//! sleep wakeup. An armed alarm is a standing request, like every other wakeup source. It stays
//! armed through the wake that it causes, and it ends only when the caller clears it.

use crate::{
    peripherals::RTC_TIMER,
    rtc_cntl::{WakeupSource, sleep::WrappedSleepConfig},
    time::Instant,
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
/// Only a deep sleep needs this result. The hardware rejects a light sleep that misses the alarm,
/// because the enabled sources are also the reject sources, and the alarm keeps its status.
///
/// The comparator holds the deadline, so esp-hal keeps no copy of it. The target of the comparator
/// is readable on every chip, also where the alarm enable beside it is not readable. Only a caller
/// that finds the timer source enabled calls this function, so the target always comes from this
/// run.
pub(crate) fn deadline_missed() -> bool {
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
    let deadline = (u64::from(high) << 32) | u64::from(low);

    deadline < crate::rtc_cntl::time_since_boot_raw() + ALARM_TO_SLEEP_TICKS
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
/// Only this function clears the status. Sleep entry must not clear it, because the status is the
/// only report of an expired deadline.
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
