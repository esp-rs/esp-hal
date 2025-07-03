//! Test that the interrupt executor correctly gives back control to thread mode
//! code.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy
//% ENV(single_integrated):   ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = single-integrated
//% ENV(multiple_integrated): ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = multiple-integrated
//% ENV(generic_queue):       ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = generic
//% ENV(generic_queue):       ESP_HAL_EMBASSY_CONFIG_GENERIC_QUEUE_SIZE = 16
//% ENV(default_with_waiti):  ESP_HAL_EMBASSY_CONFIG_LOW_POWER_WAIT = true
//% ENV(default_no_waiti):    ESP_HAL_EMBASSY_CONFIG_LOW_POWER_WAIT = false

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::interrupt::{
    Priority,
    software::{SoftwareInterrupt, SoftwareInterruptControl},
};
#[cfg(multi_core)]
use esp_hal::system::{AppCoreGuard, CpuControl, Stack};
use esp_hal_embassy::{Executor, InterruptExecutor};
use hil_test::mk_static;

esp_bootloader_esp_idf::esp_app_desc!();

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
    core_guard: AppCoreGuard<'static>,
) {
    response.wait().await;
    for _ in 0..3 {
        signal.signal(());
        response.wait().await;
    }

    core::mem::drop(core_guard);
    embedded_test::export::check_outcome(());
}

struct Context {
    interrupt: SoftwareInterrupt<'static, 1>,
    #[cfg(multi_core)]
    cpu_control: CpuControl<'static>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod test {
    use esp_hal_embassy::Callbacks;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        hil_test::init_embassy!(peripherals, 2);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        Context {
            interrupt: sw_ints.software_interrupt1,
            #[cfg(multi_core)]
            cpu_control: CpuControl::new(peripherals.CPU_CTRL),
        }
    }

    #[test]
    fn run_test_with_callbacks_api(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));
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
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));
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
    fn run_interrupt_executor_test_on_core_1(mut ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let response = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let signal = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let cpu1_fnctn = {
            move || {
                let interrupt_executor =
                    mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));

                let spawner = interrupt_executor.start(Priority::Priority3);

                spawner.spawn(responder_task(signal, response)).unwrap();

                loop {}
            }
        };

        let guard = ctx
            .cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        let thread_executor = mk_static!(Executor, Executor::new());

        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response, guard));
        })
    }

    #[test]
    #[cfg(multi_core)]
    fn run_thread_executor_test_on_core_1(mut ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let cpu1_fnctn = || {
            let executor = mk_static!(Executor, Executor::new());
            executor.run(|spawner| {
                spawner.spawn(responder_task(signal, response)).ok();
            });
        };

        let guard = ctx
            .cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        let thread_executor = mk_static!(Executor, Executor::new());

        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response, guard));
        })
    }
}
