//! Test that the interrupt executor correctly gives back control to thread mode
//! code.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: integrated-timers
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{
    interrupt::{
        software::{SoftwareInterrupt, SoftwareInterruptControl},
        Priority,
    },
    timer::timg::TimerGroup,
};
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;

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
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use hil_test as _;

    use super::*;

    #[init]
    fn init() -> SoftwareInterrupt<1> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        sw_ints.software_interrupt1
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
    }
}
