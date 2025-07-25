//! Test we get an error when attempting to initialize esp-wifi with interrupts
//! disabled in common ways

//% CHIPS: esp32 esp32s2 esp32c2 esp32c3 esp32c6 esp32s3
//% FEATURES: unstable esp-wifi esp-alloc esp-wifi/wifi embassy

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(target_arch = "riscv32")]
use esp_hal::riscv::interrupt::free as interrupt_free;
#[cfg(target_arch = "xtensa")]
use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
use esp_hal::{
    clock::CpuClock,
    interrupt::{Priority, software::SoftwareInterruptControl},
    peripherals::{Peripherals, TIMG0},
    timer::timg::TimerGroup,
};
use esp_hal_embassy::InterruptExecutor;
use esp_wifi::InitializationError;
use hil_test::mk_static;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(unused)] // compile test
fn baremetal_preempt_can_be_initialized_with_any_timer(timer: esp_hal::timer::AnyTimer<'static>) {
    esp_radio_preempt_baremetal::init(timer);
}

#[embassy_executor::task]
async fn try_init(
    signal: &'static Signal<CriticalSectionRawMutex, Option<InitializationError>>,
    timer: TIMG0<'static>,
) {
    let timg0 = TimerGroup::new(timer);
    esp_radio_preempt_baremetal::init(timg0.timer0);

    match esp_wifi::init() {
        Ok(_) => signal.signal(None),
        Err(err) => signal.signal(Some(err)),
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(size: 72 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_init_fails_without_scheduler(_peripherals: Peripherals) {
        // esp-radio-preempt-baremetal must be initialized before esp-wifi.
        let init = esp_wifi::init();

        assert!(matches!(
            init,
            Err(esp_wifi::InitializationError::SchedulerNotInitialized),
        ));
    }

    #[test]
    fn test_init_fails_cs(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_radio_preempt_baremetal::init(timg0.timer0);

        let init = critical_section::with(|_| esp_wifi::init());

        assert!(matches!(
            init,
            Err(esp_wifi::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    fn test_init_fails_interrupt_free(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_radio_preempt_baremetal::init(timg0.timer0);

        let init = interrupt_free(|| esp_wifi::init());

        assert!(matches!(
            init,
            Err(esp_wifi::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    async fn test_init_fails_in_interrupt_executor_task(peripherals: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt1);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);

        let signal =
            mk_static!(Signal<CriticalSectionRawMutex, Option<InitializationError>>, Signal::new());

        spawner.spawn(try_init(signal, peripherals.TIMG0)).ok();

        let res = signal.wait().await;

        assert!(matches!(
            res,
            Some(esp_wifi::InitializationError::InterruptsDisabled),
        ));
    }
}
