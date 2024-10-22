//! Test that the interrupt executor correctly gives back control to thread mode
//! code.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: integrated-timers
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(multi_core)]
use esp_hal::cpu_control::{CpuControl, Stack};
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
async fn interrupt_driven_task(
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
    response: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    loop {
        signal.wait().await;
        response.signal(());
    }
}

struct Context {
    interrupt: SoftwareInterrupt<1>,
    #[cfg(multi_core)]
    cpu_control: CpuControl<'static>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        Context {
            interrupt: sw_ints.software_interrupt1,
            #[cfg(multi_core)]
            cpu_control: CpuControl::new(peripherals.CPU_CTRL),
        }
    }

    #[test]
    #[timeout(3)]
    async fn run_interrupt_executor_test(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);

        spawner
            .spawn(interrupt_driven_task(signal, response))
            .unwrap();

        for _ in 0..3 {
            signal.signal(());
            response.wait().await;
        }
    }

    #[test]
    #[cfg(multi_core)]
    #[timeout(3)]
    async fn run_interrupt_executor_test_on_core_1(mut ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let response = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let signal = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let cpu1_fnctn = {
            move || {
                let interrupt_executor =
                    mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));

                let spawner = interrupt_executor.start(Priority::Priority3);

                spawner
                    .spawn(interrupt_driven_task(signal, response))
                    .unwrap();

                loop {}
            }
        };

        #[allow(static_mut_refs)]
        let _guard = ctx
            .cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        for _ in 0..3 {
            signal.signal(());
            response.wait().await;
        }
    }
}
