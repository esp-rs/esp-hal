//! Test we get an error when attempting to initialize esp-wifi with interrupts
//! disabled in common ways

//% CHIPS: esp32 esp32s2 esp32c2 esp32c3 esp32c6 esp32s3
//% FEATURES: unstable esp-wifi esp-alloc esp-wifi/wifi embassy

#![no_std]
#![no_main]

#[cfg(target_arch = "riscv32")]
use esp_hal::riscv::interrupt::free as interrupt_free;
#[cfg(target_arch = "xtensa")]
use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
use esp_hal::{
    clock::CpuClock,
    interrupt::{Priority, software::SoftwareInterruptControl},
    peripherals::{Peripherals, RADIO_CLK, RNG, TIMG0},
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;
use portable_atomic::AtomicUsize;
use static_cell::StaticCell;

static ASYNC_TEST_STATE: AtomicUsize = AtomicUsize::new(0);

#[embassy_executor::task]
async fn try_init(timer: TIMG0<'static>, rng: RNG<'static>, radio_clk: RADIO_CLK<'static>) {
    let timg0 = TimerGroup::new(timer);

    defmt::info!("before");
    let init = esp_wifi::init(timg0.timer0, Rng::new(rng), radio_clk);
    defmt::info!("after {}", init.is_ok());

    match init {
        Ok(_) => ASYNC_TEST_STATE.store(1, core::sync::atomic::Ordering::Relaxed),
        Err(_) => ASYNC_TEST_STATE.store(2, core::sync::atomic::Ordering::Relaxed),
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(size: 72 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_init_fails_cs(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let init = critical_section::with(|_| {
            esp_wifi::init(
                timg0.timer0,
                Rng::new(peripherals.RNG),
                peripherals.RADIO_CLK,
            )
        });

        assert!(matches!(
            init,
            Err(esp_wifi::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    fn test_init_fails_interrupt_free(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let init = interrupt_free(|| {
            esp_wifi::init(
                timg0.timer0,
                Rng::new(peripherals.RNG),
                peripherals.RADIO_CLK,
            )
        });

        assert!(matches!(
            init,
            Err(esp_wifi::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    fn test_init_fails_in_interrupt_executor_task(peripherals: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt1);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);
        spawner
            .spawn(try_init(
                peripherals.TIMG0,
                peripherals.RNG,
                peripherals.RADIO_CLK,
            ))
            .ok();

        loop {
            if ASYNC_TEST_STATE.load(core::sync::atomic::Ordering::Relaxed) != 0 {
                break;
            }
        }

        assert!(ASYNC_TEST_STATE.load(core::sync::atomic::Ordering::Relaxed) == 2);
    }
}
