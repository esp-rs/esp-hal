//! Test we get an error when attempting to initialize esp-radio with interrupts
//! disabled in common ways

//% CHIPS: esp32 esp32s2 esp32c2 esp32c3 esp32c6 esp32s3
//% FEATURES: unstable esp-radio esp-alloc esp-radio/wifi esp-radio/unstable embassy

#![no_std]
#![no_main]

use core::ffi::c_void;

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(multi_core)]
use esp_hal::system::{Cpu, CpuControl, Stack};
#[cfg(xtensa)]
use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
use esp_hal::{
    clock::CpuClock,
    interrupt::{Priority, software::SoftwareInterruptControl},
    peripherals::{Peripherals, TIMG0},
    ram,
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
#[cfg(riscv)]
use esp_hal::{interrupt::software::SoftwareInterrupt, riscv::interrupt::free as interrupt_free};
use esp_radio::InitializationError;
use esp_radio_rtos_driver::{
    self as preempt,
    semaphore::{SemaphoreHandle, SemaphoreKind},
};
use esp_rtos::{CurrentThreadHandle, embassy::InterruptExecutor, semaphore::Semaphore};
use hil_test::mk_static;
use portable_atomic::{AtomicBool, AtomicUsize, Ordering};
use static_cell::StaticCell;

#[allow(unused)] // compile test
fn baremetal_preempt_can_be_initialized_with_any_timer(timer: esp_hal::timer::AnyTimer<'static>) {
    esp_rtos::start(
        timer,
        #[cfg(riscv)]
        unsafe {
            SoftwareInterrupt::<'static, 0>::steal()
        },
    );
}

#[embassy_executor::task]
async fn try_init(
    signal: &'static Signal<CriticalSectionRawMutex, Option<InitializationError>>,
    timer: TIMG0<'static>,
) {
    let timg0 = TimerGroup::new(timer);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(riscv)]
        unsafe {
            SoftwareInterrupt::<'static, 0>::steal()
        },
    );

    match esp_radio::init() {
        Ok(_) => signal.signal(None),
        Err(err) => signal.signal(Some(err)),
    }
}

#[inline(never)]
#[cfg(xtensa)]
fn run_float_calc(x: f32) -> f32 {
    let result = core::hint::black_box(x) * 2.0;
    defmt::info!("{}", defmt::Display2Format(&result));
    result
}

#[cfg(multi_core)]
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
        esp_alloc::heap_allocator!(size: 36 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_init_fails_without_scheduler(_peripherals: Peripherals) {
        // esp-rtos must be initialized before esp-radio.
        let init = esp_radio::init();

        assert!(matches!(
            init,
            Err(InitializationError::SchedulerNotInitialized),
        ));
    }

    #[test]
    fn test_init_fails_cs(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            unsafe {
                SoftwareInterrupt::<'static, 0>::steal()
            },
        );

        let init = critical_section::with(|_| esp_radio::init());

        assert!(matches!(init, Err(InitializationError::InterruptsDisabled),));
    }

    #[test]
    fn test_init_fails_interrupt_free(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            unsafe {
                SoftwareInterrupt::<'static, 0>::steal()
            },
        );

        let init = interrupt_free(|| esp_radio::init());

        assert!(matches!(init, Err(InitializationError::InterruptsDisabled),));
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
            Some(esp_radio::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_wifi_can_be_initialized(mut p: Peripherals) {
        #[cfg(riscv)]
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        let esp_radio_ctrl =
            &*mk_static!(esp_radio::Controller<'static>, esp_radio::init().unwrap());

        // Initialize, then de-initialize wifi
        let wifi =
            esp_radio::wifi::new(&esp_radio_ctrl, p.WIFI.reborrow(), Default::default()).unwrap();
        drop(wifi);

        // Now, can we do it again?
        let _wifi =
            esp_radio::wifi::new(&esp_radio_ctrl, p.WIFI.reborrow(), Default::default()).unwrap();
    }

    #[test]
    fn test_esp_rtos_sleep_wakes_up(p: Peripherals) {
        #[cfg(riscv)]
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        let now = Instant::now();

        CurrentThreadHandle::get().delay(Duration::from_millis(10));

        hil_test::assert!(now.elapsed() >= Duration::from_millis(10));
    }

    #[test]
    #[timeout(2)]
    fn test_esp_rtos_time_slicing(p: Peripherals) {
        #[cfg(riscv)]
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        struct TestContext {
            time_slice_observed: SemaphoreHandle,
            counter: AtomicUsize,
        }
        let mut test_context = TestContext {
            // This semaphore signals the end of the test
            time_slice_observed: SemaphoreHandle::new(SemaphoreKind::Counting {
                max: 2,
                initial: 0,
            }),
            counter: AtomicUsize::new(0),
        };

        // Spawn tasks
        extern "C" fn task_fn(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            let mut expected_value = None;

            loop {
                let was = context.counter.fetch_add(1, Ordering::SeqCst);

                if let Some(expected) = expected_value {
                    // Not the first iteration. Check that the counter matches the expected value.
                    if was == expected {
                        expected_value = Some(was + 1);
                    } else {
                        break;
                    }
                } else {
                    // First iteration, just grab the initial value.
                    expected_value = Some(was + 1);
                }
            }

            context.time_slice_observed.give();
        }

        unsafe {
            preempt::task_create(
                "task",
                task_fn,
                (&raw mut test_context).cast::<c_void>(),
                0,
                None,
                2048,
            );
            preempt::task_create(
                "task",
                task_fn,
                (&raw mut test_context).cast::<c_void>(),
                0,
                None,
                2048,
            );
        }

        test_context.time_slice_observed.take(None);
        test_context.time_slice_observed.take(None);
    }

    #[test]
    fn test_esp_rtos_priority_inheritance(p: Peripherals) {
        #[cfg(riscv)]
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        // We need three tasks to test priority inheritance:
        // - A high priority task that will attempt to acquire the mutex.
        // - A medium priority task that will do some unrelated work.
        // - A low priority task that will hold the mutex before the high priority task could
        //   acquire it.
        //
        // Priority inversion is a situation where the higher priority task is being blocked, and a
        // medium priority task is ready to run while the low priority task holds the mutex. The
        // issue is that in this case the medium priority task is effectively prioritized over the
        // high priority task.
        //
        // The test will be successful if the high priority task is able to acquire the mutex
        // before the medium priority task runs.

        // The main task serves as the low priority task.
        // The main task will spawn the high and medium priority tasks after obtaining the mutex.
        // The medium priority task will assert that the high priority task has finished.

        struct TestContext {
            ready_semaphore: Semaphore,
            mutex: Semaphore,
            high_priority_task_finished: AtomicBool,
        }
        let mut test_context = TestContext {
            // This semaphore signals the end of the test
            ready_semaphore: Semaphore::new_counting(0, 1),
            // We'll use this mutex to test priority inheritance
            mutex: Semaphore::new_mutex(false),
            high_priority_task_finished: AtomicBool::new(false),
        };

        test_context.mutex.take(None);
        info!("Low: mutex obtained");

        // Spawn tasks
        extern "C" fn high_priority_task(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            info!("High: acquiring mutex");
            context.mutex.take(None);

            info!("High: acquired mutex, mark finished");

            context
                .high_priority_task_finished
                .store(true, Ordering::SeqCst);

            info!("High: released mutex");
            context.mutex.give();
        }
        extern "C" fn medium_priority_task(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            info!("Medium: asserting high-priority task finished");
            assert!(context.high_priority_task_finished.load(Ordering::SeqCst));

            info!("Medium: marking test finished");
            context.ready_semaphore.give();
        }

        unsafe {
            info!("Low: spawning high priority task");
            preempt::task_create(
                "high_priority_task",
                high_priority_task,
                (&raw mut test_context).cast::<c_void>(),
                3,
                None,
                4096,
            );
            info!("Low: spawning medium priority task");
            preempt::task_create(
                "medium_priority_task",
                medium_priority_task,
                (&raw mut test_context).cast::<c_void>(),
                2,
                None,
                4096,
            );
        }

        // Priority inheritance means this runs before the medium priority task
        info!("Low: tasks spawned, returning mutex");
        test_context.mutex.give();

        info!("Low: wait for tasks to finish");
        test_context.ready_semaphore.take(None);
    }

    #[test]
    #[cfg(multi_core)]
    fn test_esp_rtos_smp(p: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);

        let timg0 = TimerGroup::new(p.TIMG0);

        // In this test, we run two tasks that are pinned to each of the cores. They will each
        // increment a counter, if they are scheduled to run on their specific core.

        struct TestContext {
            ready_semaphore: Semaphore,
        }
        let test_context = TestContext {
            // This semaphore signals the end of the test. Each test case will give it once it is
            // done.
            ready_semaphore: Semaphore::new_counting(0, 2),
        };

        fn count_impl(context: &TestContext, core: Cpu) {
            let mut counter = 0;
            loop {
                if Cpu::current() == core {
                    counter += 1;
                } else {
                    preempt::yield_task();
                }
                if counter == 1000 {
                    context.ready_semaphore.give();
                    break;
                }
            }
        }

        // Spawn tasks
        extern "C" fn count_on_app_core(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            count_impl(context, Cpu::AppCpu);
        }
        extern "C" fn count_on_pro_core(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            count_impl(context, Cpu::ProCpu);
        }

        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        esp_rtos::start_second_core(
            p.CPU_CTRL,
            #[cfg(xtensa)]
            sw_ints.software_interrupt0,
            sw_ints.software_interrupt1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut APP_CORE_STACK
            },
            || {},
        );

        unsafe {
            preempt::task_create(
                "CPU 1",
                count_on_app_core,
                (&raw const test_context).cast::<c_void>().cast_mut(),
                1,
                Some(1),
                4096,
            );
            preempt::task_create(
                "CPU 0",
                count_on_pro_core,
                (&raw const test_context).cast::<c_void>().cast_mut(),
                1,
                Some(0),
                4096,
            );
        }

        info!("Wait for tasks to finish");
        test_context.ready_semaphore.take(None);
        test_context.ready_semaphore.take(None);

        unsafe {
            // Park the second core, we don't need it anymore
            esp_hal::system::CpuControl::new(esp_hal::peripherals::CPU_CTRL::steal())
                .park_core(Cpu::AppCpu);
        }
    }

    // Cp0Disable exception regression tests

    #[test]
    #[cfg(xtensa)]
    fn fpu_is_enabled() {
        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(all(multi_core, xtensa))]
    #[test]
    fn fpu_is_enabled_on_core1(peripherals: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        static DONE: AtomicBool = AtomicBool::new(false);

        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

        let guard = cpu_control
            .start_app_core(
                #[allow(static_mut_refs)]
                unsafe {
                    &mut APP_CORE_STACK
                },
                || {
                    let result = super::run_float_calc(2.0);
                    assert_eq!(result, 4.0);
                    DONE.store(true, Ordering::Relaxed);
                    loop {}
                },
            )
            .unwrap();

        while !DONE.load(Ordering::Relaxed) {}
        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);

        core::mem::drop(guard);
    }

    #[cfg(all(multi_core, xtensa))]
    #[test]
    fn fpu_is_enabled_on_core1_with_preempt(p: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        static DONE: AtomicBool = AtomicBool::new(false);

        let timg0 = TimerGroup::new(p.TIMG0);
        #[cfg(riscv)]
        let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            software_interrupt,
        );

        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start_second_core::<8192>(
            p.CPU_CTRL,
            sw_ints.software_interrupt0,
            sw_ints.software_interrupt1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut APP_CORE_STACK
            },
            || {
                preempt::usleep(10);

                let result = super::run_float_calc(2.0);
                assert_eq!(result, 4.0);
                DONE.store(true, Ordering::Relaxed);
                loop {}
            },
        );

        while !DONE.load(Ordering::Relaxed) {}
        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }
}
