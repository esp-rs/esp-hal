use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{Rtc, WakeLock},
    system::Cpu,
    time::{Duration, Instant},
};
use esp_sync::NonReentrantMutex;

use crate::{SCHEDULER, task::IdleFn};

const LIGHT_SLEEP_MIN_US: u64 =
    esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US") as u64;

static RTC: NonReentrantMutex<Option<Rtc<'static>>> = NonReentrantMutex::new(None);

/// Builds an idle hook that automatically enters light sleep when the system is
/// idle, no [`WakeLock`] is held, and the next scheduled wakeup is far enough in
/// the future.
///
/// Pass the returned hook to [`start_with_idle_hook`]. With auto light-sleep
/// enabled, esp-rtos takes ownership of the RTC peripheral.
///
/// Only available on chips that support auto light-sleep (currently ESP32-C6).
///
/// [`start_with_idle_hook`]: crate::start_with_idle_hook
pub fn auto_light_sleep(lpwr: LPWR<'static>) -> IdleFn {
    let rtc = Rtc::new(lpwr);
    RTC.with(|slot| *slot = Some(rtc));
    auto_light_sleep_hook
}

extern "C" fn auto_light_sleep_hook() -> ! {
    loop {
        let slept = SCHEDULER.with(|scheduler| {
            if WakeLock::is_active() {
                return false;
            }

            // MVP: only commit to sleep on the primary core, and never while a second
            // core scheduler is running (it would keep running while the chip sleeps).
            if Cpu::current() != Cpu::ProCpu {
                return false;
            }
            #[cfg(multi_core)]
            if scheduler.per_cpu[1].initialized {
                return false;
            }

            let Some(time_driver) = scheduler.time_driver.as_ref() else {
                return false;
            };
            let next_wakeup = time_driver.next_wakeup();
            if next_wakeup == u64::MAX {
                return false;
            }

            let now = crate::now();
            if next_wakeup.saturating_sub(now) < LIGHT_SLEEP_MIN_US {
                return false;
            }

            RTC.with(|slot| {
                let rtc = slot.as_mut().unwrap();
                rtc.light_sleep_until(Instant::EPOCH + Duration::from_micros(next_wakeup));
            });
            true
        });

        if !slept {
            esp_hal::interrupt::wait_for_interrupt();
        }
    }
}
