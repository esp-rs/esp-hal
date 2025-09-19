//! Test we get an error when attempting to initialize esp-radio with interrupts
//! disabled in common ways

//% CHIPS: esp32 esp32s2 esp32c2 esp32c3 esp32c6 esp32s3
//% FEATURES: unstable esp-radio esp-alloc esp-radio/wifi esp-radio/unstable embassy

#![no_std]
#![no_main]

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(xtensa)]
use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
use esp_hal::{
    clock::CpuClock,
    interrupt::{Priority, software::SoftwareInterruptControl},
    peripherals::{Peripherals, TIMG0},
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
#[cfg(riscv)]
use esp_hal::{interrupt::software::SoftwareInterrupt, riscv::interrupt::free as interrupt_free};
use esp_hal_embassy::InterruptExecutor;
use esp_radio::InitializationError;
use hil_test::mk_static;
use static_cell::StaticCell;

#[allow(unused)] // compile test
fn baremetal_preempt_can_be_initialized_with_any_timer(timer: esp_hal::timer::AnyTimer<'static>) {
    esp_preempt::start(
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
    esp_preempt::start(
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

#[embedded_test::tests(default_timeout = 3, executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);
        esp_alloc::heap_allocator!(size: 36 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_init_fails_without_scheduler(_peripherals: Peripherals) {
        // esp-preempt must be initialized before esp-radio.
        let init = esp_radio::init();

        assert!(matches!(
            init,
            Err(InitializationError::SchedulerNotInitialized),
        ));
    }

    #[test]
    fn test_init_fails_cs(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_preempt::start(
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
        esp_preempt::start(
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
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_preempt::start(
            timg0.timer0,
            #[cfg(riscv)]
            unsafe {
                SoftwareInterrupt::<'static, 0>::steal()
            },
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
    fn test_esp_preempt_sleep_wakes_up(p: Peripherals) {
        use esp_radio_preempt_driver as preempt;
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_preempt::start(
            timg0.timer0,
            #[cfg(riscv)]
            unsafe {
                SoftwareInterrupt::<'static, 0>::steal()
            },
        );

        let now = Instant::now();

        preempt::usleep(10_000);

        assert!(now.elapsed() >= Duration::from_millis(10));
    }

    #[test]
    fn test_esp_preempt_priority_inheritance(p: Peripherals) {
        use core::ffi::c_void;

        use esp_radio_preempt_driver as preempt;
        use portable_atomic::{AtomicBool, Ordering};
        use preempt::semaphore::{SemaphoreHandle, SemaphoreKind};

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_preempt::start(
            timg0.timer0,
            #[cfg(riscv)]
            unsafe {
                SoftwareInterrupt::<'static, 0>::steal()
            },
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
            ready_semaphore: SemaphoreHandle,
            mutex: SemaphoreHandle,
            high_priority_task_finished: AtomicBool,
        }
        let test_context = TestContext {
            // This semaphore signals the end of the test
            ready_semaphore: SemaphoreHandle::new(SemaphoreKind::Counting { max: 1, initial: 0 }),
            // We'll use this mutex to test priority inheritance
            mutex: SemaphoreHandle::new(SemaphoreKind::Mutex),
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

            // TODO: support one-shot tasks in esp-preempt
            unsafe {
                preempt::schedule_task_deletion(core::ptr::null_mut());
            }
        }
        extern "C" fn medium_priority_task(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            info!("Medium: asserting high-priority task finished");
            assert!(context.high_priority_task_finished.load(Ordering::SeqCst));

            info!("Medium: marking test finished");
            context.ready_semaphore.give();

            // TODO: support one-shot tasks in esp-preempt
            unsafe {
                preempt::schedule_task_deletion(core::ptr::null_mut());
            }
        }

        unsafe {
            info!("Low: spawning high priority task");
            preempt::task_create(
                high_priority_task,
                (&raw const test_context).cast::<c_void>().cast_mut(),
                3,
                None,
                4096,
            );
            info!("Low: spawning medium priority task");
            preempt::task_create(
                medium_priority_task,
                (&raw const test_context).cast::<c_void>().cast_mut(),
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
    fn test_esp_preempt_smp(p: Peripherals) {
        use core::ffi::c_void;

        use esp_hal::system::Cpu;
        use esp_radio_preempt_driver as preempt;
        use preempt::semaphore::{SemaphoreHandle, SemaphoreKind};

        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);

        let timg0 = TimerGroup::new(p.TIMG0);

        // In this test, we run two tasks on the same priority level. They will each increment a
        // counter, if they are scheduled to run on their specific core. The tasks have equal
        // priority, which means the scheduler should be able to prioritize them above the
        // main task, and run both of them simultaneously.

        struct TestContext {
            ready_semaphore: SemaphoreHandle,
        }
        let test_context = TestContext {
            // This semaphore signals the end of the test. Each test case will give it once it is
            // done.
            ready_semaphore: SemaphoreHandle::new(SemaphoreKind::Counting { max: 2, initial: 0 }),
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

            // TODO: support one-shot tasks in esp-preempt
            unsafe {
                preempt::schedule_task_deletion(core::ptr::null_mut());
            }
        }
        extern "C" fn count_on_pro_core(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            count_impl(context, Cpu::ProCpu);

            // TODO: support one-shot tasks in esp-preempt
            unsafe {
                preempt::schedule_task_deletion(core::ptr::null_mut());
            }
        }

        esp_preempt::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_ints.software_interrupt0,
        );

        esp_preempt::start_second_core::<8192>(
            p.CPU_CTRL,
            #[cfg(xtensa)]
            sw_ints.software_interrupt0,
            sw_ints.software_interrupt1,
        );

        unsafe {
            preempt::task_create(
                count_on_app_core,
                (&raw const test_context).cast::<c_void>().cast_mut(),
                1,
                Some(1),
                4096,
            );
            preempt::task_create(
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
}
