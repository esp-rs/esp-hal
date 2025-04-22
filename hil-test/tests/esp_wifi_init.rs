//! Test we get an error when attempting to initialize esp-wifi with interrupts
//! disabled in common ways

//% CHIPS: esp32 esp32s2 esp32c2 esp32c3 esp32c6 esp32s3
//% FEATURES: unstable esp-wifi esp-alloc esp-wifi/wifi

#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use hil_test as _;

#[cfg(target_arch = "riscv32")]
use esp_hal::riscv::interrupt::free as interrupt_free;

#[cfg(target_arch = "xtensa")]
use esp_hal::xtensa_lx::interrupt::free as interrupt_free;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{peripherals::Peripherals, rng::Rng, timer::timg::TimerGroup};

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
}
