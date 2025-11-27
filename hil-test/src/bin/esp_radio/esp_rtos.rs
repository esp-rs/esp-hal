#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::ffi::c_void;

    use defmt::info;
    use esp_hal::{
        clock::CpuClock,
        interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl},
        time::{Duration, Instant},
        timer::timg::TimerGroup,
    };
    #[cfg(multi_core)]
    use esp_hal::{peripherals::CPU_CTRL, system::Cpu};
    use esp_radio_rtos_driver::{
        self as preempt,
        queue::QueueHandle,
        semaphore::{SemaphoreHandle, SemaphoreKind},
    };
    use esp_rtos::{CurrentThreadHandle, semaphore::Semaphore};
    use portable_atomic::{AtomicBool, AtomicUsize, Ordering};

    struct Context {
        #[cfg(multi_core)]
        sw_int1: SoftwareInterrupt<'static, 1>,
        sw_int2: SoftwareInterrupt<'static, 2>,
        #[cfg(multi_core)]
        cpu_cntl: CPU_CTRL<'static>,
    }

    #[allow(unused)] // compile test
    fn baremetal_preempt_can_be_initialized_with_any_timer(
        timer: esp_hal::timer::AnyTimer<'static>,
    ) {
        esp_rtos::start(timer, unsafe { SoftwareInterrupt::<'static, 0>::steal() });
    }

    #[init]
    fn init() -> Context {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        Context {
            #[cfg(multi_core)]
            sw_int1: sw_ints.software_interrupt1,
            sw_int2: sw_ints.software_interrupt2,
            #[cfg(multi_core)]
            cpu_cntl: p.CPU_CTRL,
        }
    }

    #[test]
    fn sleep_wakes_up() {
        let now = Instant::now();

        CurrentThreadHandle::get().delay(Duration::from_millis(10));

        hil_test::assert!(now.elapsed() >= Duration::from_millis(10));
    }

    #[test]
    #[timeout(2)]
    fn time_slicing() {
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
    fn priority_inheritance() {
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
    fn task_deletion_does_not_crash() {
        // Spawn tasks
        extern "C" fn high_priority_task(context: *mut c_void) {
            info!("High: spawning medium priority task");

            let context = unsafe { &*(context as *const TestContext) };

            context.mutex.take(None);
            info!("High: mutex obtained, exiting");
        }

        struct TestContext {
            mutex: Semaphore,
        }
        let mut test_context = TestContext {
            mutex: Semaphore::new_mutex(false),
        };

        test_context.mutex.take(None);
        info!("Low: mutex obtained");

        let handle = unsafe {
            info!("Low: spawning high priority task");
            preempt::task_create(
                "high_priority_task",
                high_priority_task,
                (&raw mut test_context).cast::<c_void>(),
                3,
                None,
                4096,
            )
        };

        unsafe { preempt::schedule_task_deletion(Some(handle)) };
        test_context.mutex.give();

        info!("Low: exiting");
    }

    #[test]
    fn interrupt_handler_is_not_preempted_by_context_switch(mut ctx: Context) {
        // In this test, we start a thread, and make it wait for a signal. We then trigger a
        // low-priority interrupt, which sets the signal and exits the test. The test must not time
        // out.

        static SEM: Semaphore = Semaphore::new_counting(0, 1);

        extern "C" fn helper_thread(_context: *mut c_void) {
            SEM.take(None);
            // This thread must never be scheduled. Doing so may mean the interrupt handler never
            // completes.
            panic!();
        }

        unsafe {
            preempt::task_create(
                "helper_thread",
                helper_thread,
                core::ptr::null_mut(),
                3,
                None,
                4096,
            )
        };

        #[esp_hal::handler]
        fn sw_handler() {
            unsafe { SoftwareInterrupt::<'static, 2>::steal() }.reset();
            SEM.give();
            embedded_test::export::check_outcome(());
        }

        ctx.sw_int2.set_interrupt_handler(sw_handler);
        ctx.sw_int2.raise();

        loop {}
    }

    #[test]
    async fn timers_dont_stop_when_timer_is_cancelled() {
        extern "C" fn helper_thread(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            loop {
                info!("Helper Task: try take mutex");
                // Put the thread to sleep with a timeout. Waking this thread
                // must not cause the timer to stop.
                context.mutex.take(Some(Duration::from_millis(10)));
            }
        }

        struct TestContext {
            mutex: Semaphore,
        }
        let mut test_context = TestContext {
            mutex: Semaphore::new_mutex(false),
        };

        test_context.mutex.take(None);

        unsafe {
            preempt::task_create(
                "helper_thread",
                helper_thread,
                (&raw mut test_context).cast::<c_void>(),
                1,
                None,
                4096,
            )
        };

        // Raise our priority so that giving the mutex does not cause a context switch.
        CurrentThreadHandle::get().set_priority(2);

        embassy_futures::join::join(
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)),
            async {
                // Give the mutex AFTER the timer has been scheduler.
                test_context.mutex.give();
                info!("Low: mutex given");
            },
        )
        .await;

        info!("Low: exiting");
    }

    #[test]
    #[cfg(multi_core)]
    fn smp(ctx: Context) {
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
                    // Let's also test that the delay works on both cores.
                    CurrentThreadHandle::get().delay(Duration::from_micros(100));
                } else {
                    preempt::yield_task();
                }
                if counter == 10 {
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

        esp_rtos::start_second_core(
            unsafe { ctx.cpu_cntl.clone_unchecked() },
            ctx.sw_int1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
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
            esp_hal::system::CpuControl::new(ctx.cpu_cntl).park_core(Cpu::AppCpu);
        }
    }

    #[test]
    #[cfg(multi_core)]
    async fn embassy_cross_core(ctx: Context) {
        // This is a regression test verifying that waking an embassy task does not run into a
        // deadlock. This is a bit fragile, but the test should not produce false positives,
        // only false negatives.
        use embassy_sync::signal::Signal;
        use esp_rtos::embassy::Executor;
        use esp_sync::RawMutex;
        use static_cell::StaticCell;

        static SIGNAL: Signal<RawMutex, ()> = Signal::new();

        esp_rtos::start_second_core(
            unsafe { ctx.cpu_cntl.clone_unchecked() },
            ctx.sw_int1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            || {
                #[embassy_executor::task]
                async fn task() {
                    // Spam timer events on the second core. This arms a timer on core 0, which
                    // wakes up the task on core 1 shortly after.
                    for _ in 0..10000 {
                        embassy_time::Timer::after(embassy_time::Duration::from_micros(1)).await;
                    }
                    SIGNAL.signal(());
                }

                static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
                let executor = CORE1_EXECUTOR.init(Executor::new());
                executor.run(|spawner| {
                    spawner.must_spawn(task());
                });
            },
        );

        SIGNAL.wait().await;

        unsafe {
            // Park the second core, we don't need it anymore
            esp_hal::system::CpuControl::new(ctx.cpu_cntl).park_core(Cpu::AppCpu);
        }
    }

    #[test]
    async fn primitives_time_out() {
        let mutex = Semaphore::new_mutex(false);
        let success = mutex.take(Some(Duration::ZERO));
        hil_test::assert!(success); // mutex is originally untaken
        let success = mutex.take(Some(Duration::ZERO));
        hil_test::assert!(!success);

        let sem = Semaphore::new_counting(0, 1);
        let success = sem.take(Some(Duration::ZERO));
        hil_test::assert!(!success);

        let sem_ptr = esp_radio_rtos_driver::current_task_thread_semaphore();
        let sem = unsafe { SemaphoreHandle::ref_from_ptr(&sem_ptr) };
        let success = sem.take(Some(0));
        hil_test::assert!(!success);

        let q = QueueHandle::new(1, 1);
        let mut item = [0u8; 1];
        let success = unsafe { q.receive(item.as_mut_ptr(), Some(0)) };
        hil_test::assert!(!success);
        let success = unsafe { q.send_to_back(item.as_ptr(), Some(0)) };
        hil_test::assert!(success); // queue is originally empty
        let success = unsafe { q.send_to_back(item.as_ptr(), Some(0)) };
        hil_test::assert!(!success); // queue is now full
    }

    #[test]
    async fn queue_basics() {
        let q = QueueHandle::new(3, 1);

        let enqueue = |item| unsafe { q.send_to_back(&raw const item, Some(0)) };
        let dequeue = || -> Option<u8> {
            let mut item = 0;
            unsafe { q.receive(&raw mut item, Some(0)) }.then_some(item)
        };

        hil_test::assert_eq!(q.messages_waiting(), 0);

        hil_test::assert!(enqueue(42));
        hil_test::assert_eq!(q.messages_waiting(), 1);

        // insert an element that will be removed
        hil_test::assert!(enqueue(0));
        hil_test::assert_eq!(q.messages_waiting(), 2);

        hil_test::assert!(enqueue(24));
        hil_test::assert_eq!(q.messages_waiting(), 3);

        // Verify removing removes an item. The item is in the middle of the queue, so that we can
        // verify that the order of elements is kept.
        let item = [0; 1];
        unsafe { q.remove(item.as_ptr()) };
        hil_test::assert_eq!(q.messages_waiting(), 2);

        hil_test::assert!(enqueue(66));
        hil_test::assert_eq!(q.messages_waiting(), 3);

        hil_test::assert!(!enqueue(99)); // queue is full
        hil_test::assert_eq!(q.messages_waiting(), 3);

        // Verify items are dequeued in the correct order
        hil_test::assert_eq!(dequeue(), Some(42));
        hil_test::assert_eq!(q.messages_waiting(), 2);

        hil_test::assert_eq!(dequeue(), Some(24));
        hil_test::assert_eq!(q.messages_waiting(), 1);

        hil_test::assert_eq!(dequeue(), Some(66));
        hil_test::assert_eq!(q.messages_waiting(), 0);

        hil_test::assert_eq!(dequeue(), None);
        hil_test::assert_eq!(q.messages_waiting(), 0);
    }

    #[test]
    fn esp_radio_timer_delete() {
        unsafe extern "C" fn my_func(_ptr: *mut core::ffi::c_void) {
            esp_radio_rtos_driver::usleep(5_000);
        }

        // delayed timer deletion
        let mut low_watermark = usize::MAX;
        for i in 0..30 {
            unsafe {
                let handle1 = esp_radio_rtos_driver::timer::TimerHandle::new(
                    my_func,
                    core::ptr::null::<()>() as _,
                );
                handle1.arm(10, false);
                let handle2 = esp_radio_rtos_driver::timer::TimerHandle::new(
                    my_func,
                    core::ptr::null::<()>() as _,
                );
                handle2.arm(1000, false);

                core::mem::drop(handle2);
                core::mem::drop(handle1);

                esp_radio_rtos_driver::usleep(5_000);

                if i < 15 {
                    low_watermark = usize::min(low_watermark, esp_alloc::HEAP.free());
                } else {
                    assert!(esp_alloc::HEAP.free() >= low_watermark);
                }
            }
        }

        // non-delayed timer deletion
        let mut low_watermark = usize::MAX;
        for i in 0..30 {
            unsafe {
                let handle1 = esp_radio_rtos_driver::timer::TimerHandle::new(
                    my_func,
                    core::ptr::null::<()>() as _,
                );
                handle1.arm(1000, false);
                let handle2 = esp_radio_rtos_driver::timer::TimerHandle::new(
                    my_func,
                    core::ptr::null::<()>() as _,
                );
                handle2.arm(1000, false);

                esp_radio_rtos_driver::usleep(15_000);

                core::mem::drop(handle2);
                core::mem::drop(handle1);

                esp_radio_rtos_driver::usleep(5_000);

                if i < 15 {
                    low_watermark = usize::min(low_watermark, esp_alloc::HEAP.free());
                } else {
                    assert!(esp_alloc::HEAP.free() >= low_watermark);
                }
            }
        }
    }
}
