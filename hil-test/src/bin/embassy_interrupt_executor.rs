//! Test that the interrupt executor correctly gives back control to thread mode
//! code.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{
    interrupt::{
        Priority,
        software::{SoftwareInterrupt, SoftwareInterruptControl},
    },
    timer::timg::TimerGroup,
};
#[cfg(multi_core)]
use esp_hal::{
    peripherals::CPU_CTRL,
    system::{Cpu, CpuControl, Stack},
};
use esp_rtos::embassy::{Executor, InterruptExecutor};
use hil_test::mk_static;

#[embassy_executor::task]
async fn responder_task(
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
    response: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    response.signal(());
    loop {
        signal.wait().await;
        response.signal(());
    }
}

#[embassy_executor::task]
async fn tester_task(
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
    response: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    response.wait().await;
    for _ in 0..3 {
        signal.signal(());
        response.wait().await;
    }
    embedded_test::export::check_outcome(());
}

#[embassy_executor::task]
#[cfg(multi_core)]
async fn tester_task_multi_core(
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
    response: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    response.wait().await;
    for _ in 0..3 {
        signal.signal(());
        response.wait().await;
    }

    unsafe {
        // Park the second core, we don't need it anymore
        CpuControl::new(CPU_CTRL::steal()).park_core(Cpu::AppCpu);
    }
    embedded_test::export::check_outcome(());
}

struct Context {
    #[cfg(xtensa)]
    sw_int0: SoftwareInterrupt<'static, 0>,
    #[cfg(multi_core)]
    sw_int1: SoftwareInterrupt<'static, 1>,
    sw_int2: SoftwareInterrupt<'static, 2>,
    #[cfg(multi_core)]
    cpu_control: CPU_CTRL<'static>,
}

#[embedded_test::tests(default_timeout = 3)]
mod test {
    use esp_rtos::embassy::Callbacks;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        Context {
            #[cfg(xtensa)]
            sw_int0: sw_int.software_interrupt0,
            #[cfg(multi_core)]
            sw_int1: sw_int.software_interrupt1,
            sw_int2: sw_int.software_interrupt2,
            #[cfg(multi_core)]
            cpu_control: peripherals.CPU_CTRL,
        }
    }

    #[test]
    fn run_test_with_callbacks_api(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);
        spawner.must_spawn(responder_task(signal, response));

        let thread_executor = mk_static!(Executor, Executor::new());

        struct NoCallbacks;

        impl Callbacks for NoCallbacks {
            fn before_poll(&mut self) {}
            fn on_idle(&mut self) {}
        }

        let callbacks = NoCallbacks;

        thread_executor.run_with_callbacks(
            |spawner| {
                spawner.must_spawn(tester_task(signal, response));
            },
            callbacks,
        )
    }

    #[test]
    fn run_interrupt_executor_test(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);
        spawner.must_spawn(responder_task(signal, response));

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task(signal, response));
        })
    }

    #[test]
    #[cfg(multi_core)]
    fn run_interrupt_executor_test_on_core_1(ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let response = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let signal = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        esp_rtos::start_second_core(
            ctx.cpu_control,
            #[cfg(xtensa)]
            ctx.sw_int0,
            ctx.sw_int1,
            app_core_stack,
            || {
                let interrupt_executor =
                    mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));

                let spawner = interrupt_executor.start(Priority::Priority3);

                spawner.spawn(responder_task(signal, response)).unwrap();
            },
        );

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response));
        })
    }

    #[test]
    #[cfg(multi_core)]
    fn run_thread_executor_test_on_core_1(ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        esp_rtos::start_second_core(
            ctx.cpu_control,
            #[cfg(xtensa)]
            ctx.sw_int0,
            ctx.sw_int1,
            app_core_stack,
            || {
                let executor = mk_static!(Executor, Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(responder_task(signal, response)).ok();
                });
            },
        );

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response));
        })
    }
}
