//! Test that the interrupt executor correctly gives back control to thread mode
//! code.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: integrated-timers

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
async fn interrupt_driven_task(signal: &'static Signal<CriticalSectionRawMutex, ()>) {
    loop {
        signal.wait().await;
        defmt::info!("Received");
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod test {
    use defmt_rtt as _;
    use esp_backtrace as _;
    use esp_hal::{
        clock::ClockControl,
        interrupt::Priority,
        peripherals::Peripherals,
        system::{SoftwareInterrupt, SystemControl},
    };
    use esp_hal_embassy::InterruptExecutor;

    use super::*;

    #[init]
    fn init() -> SoftwareInterrupt<1> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        system.software_interrupt_control.software_interrupt1
    }

    #[test]
    #[timeout(3)]
    fn run_interrupt_executor_test(interrupt: SoftwareInterrupt<1>) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(interrupt));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);

        spawner.spawn(interrupt_driven_task(signal)).unwrap();

        signal.signal(());
        defmt::info!("Returned");
    }
}
