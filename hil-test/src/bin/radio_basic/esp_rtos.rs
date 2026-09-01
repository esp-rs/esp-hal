#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use alloc::boxed::Box;
    use core::ffi::c_void;

    use defmt::info;
    use esp_hal::{
        clock::CpuClock,
        interrupt::{Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR0, FROM_CPU_INTR2, TIMG0},
        time::{Duration, Instant},
        timer::timg::TimerGroup,
    };
    #[cfg(multi_core)]
    use esp_hal::{
        peripherals::{CPU_CTRL, FROM_CPU_INTR1},
        system::Cpu,
    };
    use esp_radio_rtos_driver::{
        self as preempt,
        queue::QueueHandle,
        semaphore::{SemaphoreHandle, SemaphoreKind},
    };
    use esp_rtos::{
        CurrentThreadHandle,
        embassy::InterruptExecutor,
        thread::{Stack as ThreadStack, ThreadSpawner},
    };
    use portable_atomic::{AtomicBool, AtomicPtr, AtomicUsize, Ordering};
    use static_cell::{ConstStaticCell, StaticCell};

    struct Context {
        #[cfg(multi_core)]
        sw_int1: FROM_CPU_INTR1<'static>,
        sw_int2: FROM_CPU_INTR2<'static>,
        #[cfg(multi_core)]
        cpu_cntl: CPU_CTRL<'static>,
    }

    #[allow(unused)] // compile test
    fn baremetal_preempt_can_be_initialized_with_any_timer(
        timer: esp_hal::timer::AnyTimer<'static>,
        int: FROM_CPU_INTR0<'static>,
    ) {
        esp_rtos::start(timer, int);
    }

    #[init]
    fn init() -> Context {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, p.FROM_CPU_INTR0);

        Context {
            #[cfg(multi_core)]
            sw_int1: p.FROM_CPU_INTR1,
            sw_int2: p.FROM_CPU_INTR2,
            #[cfg(multi_core)]
            cpu_cntl: p.CPU_CTRL,
        }
    }

    fn no_init() {}

    #[test(init = no_init)]
    #[should_panic]
    fn panics_in_interrupt_context() {
        #[embassy_executor::task]
        async fn try_init(timer: TIMG0<'static>, sw_int0: FROM_CPU_INTR0<'static>) {
            let timg0 = TimerGroup::new(timer);
            esp_rtos::start(timg0.timer0, sw_int0);
        }

        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(p.FROM_CPU_INTR1);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);

        spawner.spawn(try_init(p.TIMG0, p.FROM_CPU_INTR0).unwrap());
    }

    #[test]
    fn sleep_wakes_up() {
        let now = Instant::now();

        CurrentThreadHandle::get().delay(Duration::from_millis(10));

        hil_test::assert!(now.elapsed() >= Duration::from_millis(10));
    }

    #[test]
    fn thread_runs_a_function_and_returns_its_value() {
        fn worker() -> u32 {
            42
        }

        let handle = ThreadSpawner::new(4096).with_name("worker").spawn(worker);

        hil_test::assert_eq!(handle.name(), Some("worker"));

        let (value, _spawner) = handle.join();
        hil_test::assert_eq!(value, 42);
    }

    #[test]
    fn thread_returns_a_value_that_is_not_copy() {
        let captured = Box::new(3_u32);

        let handle = ThreadSpawner::new(4096).spawn(move || {
            let mut items = alloc::vec::Vec::new();
            items.push(*captured);
            items.push(*captured);
            items
        });

        let (value, _spawner) = handle.join();
        hil_test::assert_eq!(value.len(), 2);
        hil_test::assert_eq!(value[0], 3);
        hil_test::assert_eq!(value[1], 3);
    }

    #[test]
    fn a_static_stack_can_be_reused_after_a_join() {
        static STACK: ConstStaticCell<ThreadStack<4096>> = ConstStaticCell::new(ThreadStack::new());

        let spawner = ThreadSpawner::from_static(STACK.take()).with_priority(2);

        let (first, spawner) = spawner.spawn(|| 1_u32).join();
        let (second, _spawner) = spawner.spawn(|| 2_u32).join();

        hil_test::assert_eq!(first, 1);
        hil_test::assert_eq!(second, 2);
    }

    #[test]
    fn a_detached_thread_keeps_running() {
        static FINISHED: AtomicBool = AtomicBool::new(false);

        ThreadSpawner::new(4096)
            .spawn(|| {
                CurrentThreadHandle::get().delay(Duration::from_millis(10));
                FINISHED.store(true, Ordering::SeqCst);
            })
            .detach();

        while !FINISHED.load(Ordering::SeqCst) {
            CurrentThreadHandle::get().delay(Duration::from_millis(1));
        }
    }

    #[test]
    fn a_detached_thread_drops_its_return_value() {
        static DROPPED: AtomicBool = AtomicBool::new(false);

        struct Observed;
        impl Drop for Observed {
            fn drop(&mut self) {
                DROPPED.store(true, Ordering::SeqCst);
            }
        }

        ThreadSpawner::new(4096).spawn(|| Observed).detach();

        while !DROPPED.load(Ordering::SeqCst) {
            CurrentThreadHandle::get().delay(Duration::from_millis(1));
        }
    }

    #[test]
    #[cfg(multi_core)]
    fn a_thread_can_be_pinned_to_the_second_core(ctx: Context) {
        esp_rtos::start_second_core(
            unsafe { ctx.cpu_cntl.clone_unchecked() },
            ctx.sw_int1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            || {},
        );

        let (cpu, _spawner) = ThreadSpawner::new(4096)
            .with_pinned_to(Cpu::AppCpu)
            .with_priority(2)
            .spawn(Cpu::current)
            .join();

        hil_test::assert!(cpu == Cpu::AppCpu);

        unsafe {
            // Park the second core, we don't need it anymore
            esp_hal::system::CpuControl::new(ctx.cpu_cntl).park_core(Cpu::AppCpu);
        }
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
            ready_semaphore: SemaphoreHandle,
            mutex: SemaphoreHandle,
            high_priority_task_finished: AtomicBool,
        }
        let mut test_context = TestContext {
            // This semaphore signals the end of the test
            ready_semaphore: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }),
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
            mutex: SemaphoreHandle,
        }
        let mut test_context = TestContext {
            mutex: SemaphoreHandle::new(SemaphoreKind::Mutex),
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
    fn task_can_delete_itself_when_nothing_else_is_ready() {
        // The task deletes itself while the main task sleeps. The run queue is empty at that point,
        // so the scheduler has to switch from the deleted task to the idle context.
        extern "C" fn self_deleting_task(_context: *mut c_void) {
            // Wait for the main task to go to sleep, so that this task is the last ready one.
            CurrentThreadHandle::get().delay(Duration::from_millis(10));

            info!("Task: deleting itself");
            unsafe { preempt::schedule_task_deletion(None) };

            unreachable!("A deleted task must not run again");
        }

        unsafe {
            preempt::task_create(
                "self_deleting_task",
                self_deleting_task,
                core::ptr::null_mut(),
                3,
                None,
                4096,
            )
        };

        // Sleeping takes the main task out of the run queue.
        CurrentThreadHandle::get().delay(Duration::from_millis(50));

        info!("Main: done");
    }

    #[test]
    fn interrupt_handler_is_not_preempted_by_context_switch(ctx: Context) {
        // In this test, we start a thread, and make it wait for a signal. We then trigger a
        // low-priority interrupt, which sets the signal and exits the test. The test must not time
        // out.

        static SEM: AtomicPtr<SemaphoreHandle> = AtomicPtr::new(core::ptr::null_mut());

        let mut sem = SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 });
        SEM.store(&raw mut sem, Ordering::Relaxed);

        extern "C" fn helper_thread(_context: *mut c_void) {
            let sem = unsafe { &*SEM.load(Ordering::Relaxed) };
            sem.take(None);
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
            SoftwareInterrupt::<'static, 2>::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            let sem = unsafe { &*SEM.load(Ordering::Relaxed) };
            sem.give();
            embedded_test::export::check_outcome(());
        }

        let mut sw_int2 = SoftwareInterrupt::new(ctx.sw_int2);
        sw_int2.set_interrupt_handler(sw_handler);
        sw_int2.raise();

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
                context.mutex.take(Some(10_000));
            }
        }

        struct TestContext {
            mutex: SemaphoreHandle,
        }
        // Leak the context to prevent it from being dropped
        let test_context = Box::leak(Box::new(TestContext {
            mutex: SemaphoreHandle::new(SemaphoreKind::Mutex),
        }));

        test_context.mutex.take(None);

        unsafe {
            preempt::task_create(
                "helper_thread",
                helper_thread,
                (&raw mut *test_context).cast::<c_void>(),
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
            ready_semaphore: SemaphoreHandle,
        }
        let test_context = TestContext {
            // This semaphore signals the end of the test. Each test case will give it once it is
            // done.
            ready_semaphore: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 2 }),
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
    fn deleting_a_task_on_another_core_keeps_other_tasks_alive(ctx: Context) {
        // Core 0 deletes a task that is running on core 1. The task is freed while core 1 still
        // holds it as its current task, so the allocator can hand the same address to a task
        // created afterwards. Core 1 then acts on the new task instead of the deleted one.
        struct TestContext {
            victim_runs: SemaphoreHandle,
            successor_alive: SemaphoreHandle,
        }

        let test_context = TestContext {
            victim_runs: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }),
            successor_alive: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }),
        };

        extern "C" fn victim(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            context.victim_runs.give();

            // Stay the current task of core 1, and keep entering the scheduler.
            loop {
                preempt::yield_task();
            }
        }

        extern "C" fn successor(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };

            // Report being alive, and sleep in between so that the main task can run.
            loop {
                context.successor_alive.give();
                CurrentThreadHandle::get().delay(Duration::from_millis(10));
            }
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

        // The spinning tasks must not starve the main task, otherwise this test can only time out.
        CurrentThreadHandle::get().set_priority(5);

        let context_ptr = (&raw const test_context).cast::<c_void>().cast_mut();

        // Both tasks use the same stack size, so that the successor can reuse the memory of the
        // victim.
        const STACK_SIZE: usize = 4096;

        let victim_handle =
            unsafe { preempt::task_create("victim", victim, context_ptr, 1, Some(1), STACK_SIZE) };

        info!("Wait for the victim to run on core 1");
        hil_test::assert!(test_context.victim_runs.take(Some(1_000_000)));

        info!("Delete the victim from core 0");
        unsafe { preempt::schedule_task_deletion(Some(victim_handle)) };

        let successor_handle = unsafe {
            preempt::task_create("successor", successor, context_ptr, 1, Some(0), STACK_SIZE)
        };
        info!(
            "victim: {:?}, successor: {:?}",
            victim_handle.as_ptr(),
            successor_handle.as_ptr()
        );

        // The successor must keep running. If core 1 still treats the freed address as its current
        // task, it corrupts or deletes the successor instead.
        for round in 0..5 {
            hil_test::assert!(
                test_context.successor_alive.take(Some(500_000)),
                "The task created after the deletion stopped running in round {}",
                round
            );
        }

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
                    spawner.spawn(task().unwrap());
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
    #[cfg(multi_core)]
    async fn moving_data_to_second_core(ctx: Context) {
        // This is a regression test for https://github.com/esp-rs/esp-hal/issues/4912.
        // It doesn't necessarily need RTOS, but RTOS uses the affected multi-core APIs
        // so we might as well test the whole chain.
        use embassy_sync::signal::Signal;
        use esp_sync::RawMutex;

        static SIGNAL: Signal<RawMutex, Result<(), (usize, u32)>> = Signal::new();

        // We rely on ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4 here
        let data = [0xabad1dea_u32; 20];

        esp_rtos::start_second_core(
            unsafe { ctx.cpu_cntl.clone_unchecked() },
            ctx.sw_int1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            move || {
                let result = if let Some(mismatch) = data
                    .iter()
                    .copied()
                    .enumerate()
                    .find(|(_, v)| *v != 0xabad1dea_u32)
                {
                    Err(mismatch)
                } else {
                    Ok(())
                };

                SIGNAL.signal(result);
            },
        );

        let result = SIGNAL.wait().await;

        if let Err((index, read)) = result {
            defmt::panic!(
                "Data corrupted at index {} (got {:x} instead of {:x})",
                index,
                read,
                data[index]
            );
        }

        unsafe {
            // Park the second core, we don't need it anymore
            esp_hal::system::CpuControl::new(ctx.cpu_cntl).park_core(Cpu::AppCpu);
        }
    }

    #[test]
    #[cfg(multi_core)]
    async fn embassy_cross_core_bare_metal(ctx: Context) {
        use embassy_sync::signal::Signal;
        use esp_hal::delay::Delay;
        use esp_sync::RawMutex;

        static SIGNAL: Signal<RawMutex, ()> = Signal::new();

        let _gaurd = esp_hal::system::CpuControl::new(ctx.cpu_cntl)
            .start_app_core(
                #[allow(static_mut_refs)]
                unsafe {
                    &mut crate::APP_CORE_STACK
                },
                move || {
                    Delay::new().delay_millis(100);
                    SIGNAL.signal(());
                    loop {}
                },
            )
            .unwrap();

        SIGNAL.wait().await;
    }

    #[test]
    async fn primitives_time_out() {
        let mutex = SemaphoreHandle::new(SemaphoreKind::Mutex);
        let success = mutex.take(Some(0));
        hil_test::assert!(success); // mutex is originally untaken
        let success = mutex.take(Some(0));
        hil_test::assert!(!success);

        let sem = SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 });
        let success = sem.take(Some(0));
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
                    hil_test::assert!(
                        esp_alloc::HEAP.free() >= low_watermark,
                        "Heap free space is less than expected: {} < {}",
                        esp_alloc::HEAP.free(),
                        low_watermark
                    );
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
                    hil_test::assert!(
                        esp_alloc::HEAP.free() >= low_watermark,
                        "Heap free space is less than expected: {} < {}",
                        esp_alloc::HEAP.free(),
                        low_watermark
                    );
                }
            }
        }
    }
}

/// Tests for the configuration where the scheduler runs on the second core only, and the first core
/// stays bare-metal.
///
/// The test functions run on the first core, so they must not use any esp-rtos API. They observe
/// the second core through atomics instead.
#[cfg(multi_core)]
#[embedded_test::tests(default_timeout = 3)]
mod second_core_only {
    use core::ffi::c_void;

    use esp_hal::{
        clock::CpuClock,
        peripherals::{CPU_CTRL, FROM_CPU_INTR1},
        system::Cpu,
        time::Duration,
        timer::timg::{Timer, TimerGroup},
    };
    use esp_radio_rtos_driver as preempt;
    use esp_rtos::CurrentThreadHandle;
    use portable_atomic::{AtomicBool, AtomicUsize, Ordering};

    struct Context {
        cpu_control: CPU_CTRL<'static>,
        sw_int1: FROM_CPU_INTR1<'static>,
        timer: Timer<'static>,
    }

    #[init]
    fn init() -> Context {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        let timg0 = TimerGroup::new(p.TIMG0);

        Context {
            cpu_control: p.CPU_CTRL,
            sw_int1: p.FROM_CPU_INTR1,
            timer: timg0.timer0,
        }
    }

    fn start_on_second_core(ctx: Context, func: impl FnOnce() + Send + 'static) {
        esp_rtos::start_on_second_core_only(
            ctx.cpu_control,
            ctx.sw_int1,
            ctx.timer,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            func,
        );
    }

    fn spawn(name: &str, task: extern "C" fn(*mut c_void), priority: u32) {
        unsafe { preempt::task_create(name, task, core::ptr::null_mut(), priority, None, 4096) };
    }

    #[test]
    fn task_runs_on_the_second_core(ctx: Context) {
        static FINISHED: AtomicBool = AtomicBool::new(false);
        static RAN_ON_SECOND_CORE: AtomicBool = AtomicBool::new(false);

        extern "C" fn task(_: *mut c_void) {
            RAN_ON_SECOND_CORE.store(Cpu::current() == Cpu::AppCpu, Ordering::SeqCst);
            FINISHED.store(true, Ordering::SeqCst);
        }

        start_on_second_core(ctx, || spawn("task", task, 1));

        while !FINISHED.load(Ordering::SeqCst) {}

        hil_test::assert!(RAN_ON_SECOND_CORE.load(Ordering::SeqCst));
    }

    #[test]
    fn time_slicing_on_the_second_core(ctx: Context) {
        // Two tasks of the same priority must both make progress. Each task counts up until the
        // counter jumps, which means the other task ran in between.
        static FINISHED: AtomicUsize = AtomicUsize::new(0);
        static COUNTER: AtomicUsize = AtomicUsize::new(0);

        extern "C" fn task(_: *mut c_void) {
            let mut expected_value = None;

            loop {
                let was = COUNTER.fetch_add(1, Ordering::SeqCst);

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

            FINISHED.fetch_add(1, Ordering::SeqCst);
        }

        start_on_second_core(ctx, || {
            // The tasks run at the priority of the main thread, so that the main thread can create
            // the second task.
            spawn("task1", task, 0);
            spawn("task2", task, 0);
        });

        while FINISHED.load(Ordering::SeqCst) < 2 {}
    }

    #[test]
    fn the_first_core_keeps_running(ctx: Context) {
        static FIRST_CORE_COUNTER: AtomicUsize = AtomicUsize::new(0);
        static FIRST_CORE_ADVANCED: AtomicBool = AtomicBool::new(false);
        static FINISHED: AtomicBool = AtomicBool::new(false);

        extern "C" fn observer(_: *mut c_void) {
            let before = FIRST_CORE_COUNTER.load(Ordering::Relaxed);

            // Sleeping proves that the scheduler of the second core works while the first core runs
            // its own code.
            CurrentThreadHandle::get().delay(Duration::from_millis(50));

            let after = FIRST_CORE_COUNTER.load(Ordering::Relaxed);
            FIRST_CORE_ADVANCED.store(after > before, Ordering::SeqCst);
            FINISHED.store(true, Ordering::SeqCst);
        }

        start_on_second_core(ctx, || spawn("observer", observer, 1));

        while !FINISHED.load(Ordering::SeqCst) {
            FIRST_CORE_COUNTER.fetch_add(1, Ordering::Relaxed);
        }

        hil_test::assert!(FIRST_CORE_ADVANCED.load(Ordering::SeqCst));
    }

    #[test]
    #[should_panic]
    fn thread_mode_executor_on_the_first_core_panics(ctx: Context) {
        use esp_rtos::embassy::Executor;
        use static_cell::StaticCell;

        start_on_second_core(ctx, || {});

        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        EXECUTOR.init(Executor::new()).run(|_| {});
    }
}
