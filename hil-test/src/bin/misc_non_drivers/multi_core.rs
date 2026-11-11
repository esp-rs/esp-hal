#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        interrupt::{
            InterruptHandler,
            Priority,
            RunLevel,
            ipc::{Ipc, IpcError, IpcRequest, Ticket},
            software::SoftwareInterrupt,
        },
        peripherals::{CPU_CTRL, FROM_CPU_INTR2, IPC, Peripherals},
        system::{AppCoreGuard, Cpu, CpuControl, Stack, is_running},
        time::{Duration, Instant},
    };
    use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
    use hil_test as _;
    use portable_atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

    static mut IPC_APP_CORE_STACK: Stack<4096> = Stack::new();
    static MASK_APP_CORE: AtomicBool = AtomicBool::new(false);
    static APP_CORE_MASKED: AtomicBool = AtomicBool::new(false);

    #[cfg(xtensa)]
    const ELEVATED_PRIORITIES: &[Priority] = &[Priority::Priority2, Priority::Priority3];
    #[cfg(riscv)]
    const ELEVATED_PRIORITIES: &[Priority] = &[
        Priority::Priority2,
        Priority::Priority3,
        Priority::Priority4,
        Priority::Priority5,
        Priority::Priority6,
        Priority::Priority7,
        Priority::Priority8,
    ];

    fn deadline(ms: u64) -> Instant {
        Instant::now() + Duration::from_millis(ms)
    }

    fn ipc() -> Ipc {
        Ipc::new(unsafe { IPC::steal() })
    }

    /// Starts the APP core in a loop that masks its interrupts while `MASK_APP_CORE` is set.
    fn start_ipc_app_core(cpu_control: &mut CpuControl<'_>) -> AppCoreGuard<'static> {
        let stack = unsafe { &mut *core::ptr::addr_of_mut!(IPC_APP_CORE_STACK) };
        cpu_control
            .start_app_core(stack, || {
                loop {
                    if MASK_APP_CORE.load(Ordering::Acquire) {
                        let token = unsafe { SingleCoreInterruptLock.enter() };
                        APP_CORE_MASKED.store(true, Ordering::Release);
                        while MASK_APP_CORE.load(Ordering::Acquire) {}
                        APP_CORE_MASKED.store(false, Ordering::Release);
                        unsafe { SingleCoreInterruptLock.exit(token) };
                    }
                }
            })
            .unwrap()
    }

    /// Returns the stack of the APP core.
    ///
    /// Each test starts after a chip reset, and each start resets the APP core, so no earlier
    /// reference to the stack is used again.
    fn app_core_stack() -> &'static mut Stack<2048> {
        static mut APP_CORE_STACK: Stack<2048> = Stack::new();
        unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) }
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn app_core_starts_again(p: Peripherals) {
        static STARTED: AtomicU32 = AtomicU32::new(0);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);

        // The closure returns, which stops the APP core.
        let _guard = cpu_control
            .start_app_core(app_core_stack(), || {
                STARTED.store(1, Ordering::Release);
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 1 {}
        while is_running(Cpu::AppCpu) {}

        // The closure runs until the guard drops.
        let guard = cpu_control
            .start_app_core(app_core_stack(), || {
                STARTED.store(2, Ordering::Release);
                loop {}
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 2 {}
        drop(guard);

        let _guard = cpu_control
            .start_app_core(app_core_stack(), || {
                STARTED.store(3, Ordering::Release);
                loop {}
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 3 {}
    }

    #[test]
    fn ipc_post_runs_on_the_other_core(p: Peripherals) {
        static CORE: AtomicUsize = AtomicUsize::new(usize::MAX);
        extern "C" fn record() {
            CORE.store(Cpu::current() as usize, Ordering::Release);
        }
        static REQUEST: IpcRequest = IpcRequest::new(record);

        let _guard = start_ipc_app_core(&mut CpuControl::new(p.CPU_CTRL));

        let ticket = Ipc::new(p.IPC).post(Cpu::AppCpu, &REQUEST).unwrap();
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(CORE.load(Ordering::Acquire), Cpu::AppCpu as usize);
    }

    #[test]
    fn ipc_post_from_interrupt_handlers(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        static PRIORITY: AtomicUsize = AtomicUsize::new(0);
        static RESULT: AtomicUsize = AtomicUsize::new(0);
        static HANDLED: AtomicBool = AtomicBool::new(false);
        const RESULT_OK: usize = 1;
        const RESULT_WRONG_LEVEL: usize = 2;
        const RESULT_ERROR: usize = 3;

        extern "C" fn count() {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        extern "C" fn post_from_handler() {
            SoftwareInterrupt::<'static, 2>::new(unsafe { FROM_CPU_INTR2::steal() }).reset();

            let priority = ELEVATED_PRIORITIES[PRIORITY.load(Ordering::Relaxed)];
            let result = if RunLevel::current() != priority {
                RESULT_WRONG_LEVEL
            } else {
                match ipc()
                    .post(Cpu::AppCpu, &REQUEST)
                    .and_then(|ticket| ticket.wait(deadline(100)))
                {
                    Ok(()) => RESULT_OK,
                    Err(_) => RESULT_ERROR,
                }
            };
            RESULT.store(result, Ordering::Relaxed);
            HANDLED.store(true, Ordering::Release);
        }

        let _guard = start_ipc_app_core(&mut CpuControl::new(p.CPU_CTRL));
        let mut interrupt = SoftwareInterrupt::new(p.FROM_CPU_INTR2);

        for (index, priority) in ELEVATED_PRIORITIES.iter().enumerate() {
            PRIORITY.store(index, Ordering::Relaxed);
            HANDLED.store(false, Ordering::Relaxed);
            interrupt.set_interrupt_handler(InterruptHandler::new(post_from_handler, *priority));
            interrupt.raise();

            let end = deadline(500);
            while !HANDLED.load(Ordering::Acquire) {
                assert!(Instant::now() < end);
            }
            assert_eq!(RESULT.load(Ordering::Relaxed), RESULT_OK);
            assert_eq!(RUNS.load(Ordering::Relaxed), index as u32 + 1);
        }
    }

    #[test]
    fn ipc_self_post_runs_before_wait_returns(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count() {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &REQUEST).unwrap();
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_masked_self_wait_would_deadlock(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count() {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let token = unsafe { SingleCoreInterruptLock.enter() };
        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &REQUEST).unwrap();
        let masked_result = ticket.wait(deadline(10));
        let masked_runs = RUNS.load(Ordering::Relaxed);
        unsafe { SingleCoreInterruptLock.exit(token) };

        assert_eq!(masked_result, Err(IpcError::WouldDeadlock));
        assert_eq!(masked_runs, 0);
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_wait_inside_func_would_deadlock(p: Peripherals) {
        static INNER_RUNS: AtomicU32 = AtomicU32::new(0);
        static INNER_RUNS_IN_OUTER: AtomicU32 = AtomicU32::new(u32::MAX);
        static OUTER_GOT_WOULD_DEADLOCK: AtomicBool = AtomicBool::new(false);

        extern "C" fn inner() {
            INNER_RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static INNER: IpcRequest = IpcRequest::new(inner);

        extern "C" fn outer() {
            let result = ipc()
                .post(Cpu::current(), &INNER)
                .map(|ticket| ticket.wait(deadline(10)));
            OUTER_GOT_WOULD_DEADLOCK.store(
                result == Ok(Err(IpcError::WouldDeadlock)),
                Ordering::Relaxed,
            );
            INNER_RUNS_IN_OUTER.store(INNER_RUNS.load(Ordering::Relaxed), Ordering::Relaxed);
        }
        static OUTER: IpcRequest = IpcRequest::new(outer);

        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &OUTER).unwrap();
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert!(OUTER_GOT_WOULD_DEADLOCK.load(Ordering::Relaxed));
        assert_eq!(INNER_RUNS_IN_OUTER.load(Ordering::Relaxed), 0);

        let end = deadline(100);
        while INNER_RUNS.load(Ordering::Relaxed) == 0 {
            assert!(Instant::now() < end);
        }
        assert_eq!(INNER_RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_masked_target_times_out(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count() {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let _guard = start_ipc_app_core(&mut CpuControl::new(p.CPU_CTRL));
        MASK_APP_CORE.store(true, Ordering::Release);
        while !APP_CORE_MASKED.load(Ordering::Acquire) {}

        let ticket = Ipc::new(p.IPC).post(Cpu::AppCpu, &REQUEST).unwrap();
        assert_eq!(ticket.wait(deadline(10)), Err(IpcError::Timeout));
        assert_eq!(RUNS.load(Ordering::Relaxed), 0);

        MASK_APP_CORE.store(false, Ordering::Release);
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_post_does_not_wait_for_a_busy_core(p: Peripherals) {
        static RELEASE: AtomicBool = AtomicBool::new(false);
        static LONG_STARTED: AtomicBool = AtomicBool::new(false);
        static OTHER_RUNS: AtomicU32 = AtomicU32::new(0);

        extern "C" fn long() {
            LONG_STARTED.store(true, Ordering::Release);
            while !RELEASE.load(Ordering::Acquire) {}
        }
        static LONG: IpcRequest = IpcRequest::new(long);

        extern "C" fn other() {
            OTHER_RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static OTHERS: [IpcRequest; 4] = [
            IpcRequest::new(other),
            IpcRequest::new(other),
            IpcRequest::new(other),
            IpcRequest::new(other),
        ];

        let _guard = start_ipc_app_core(&mut CpuControl::new(p.CPU_CTRL));
        let ipc = Ipc::new(p.IPC);

        let long_ticket = ipc.post(Cpu::AppCpu, &LONG).unwrap();
        let end = deadline(100);
        while !LONG_STARTED.load(Ordering::Acquire) {
            assert!(Instant::now() < end);
        }

        let mut tickets: [Option<Ticket>; 4] = [None; 4];
        for (index, request) in OTHERS.iter().enumerate() {
            let start = Instant::now();
            tickets[index] = Some(ipc.post(Cpu::AppCpu, request).unwrap());
            assert!(start.elapsed() < Duration::from_millis(1));
        }
        assert_eq!(OTHER_RUNS.load(Ordering::Relaxed), 0);
        for ticket in tickets.iter().flatten() {
            assert_eq!(ticket.poll(), None);
        }

        RELEASE.store(true, Ordering::Release);
        assert_eq!(long_ticket.wait(deadline(100)), Ok(()));
        for ticket in tickets.iter().flatten() {
            assert_eq!(ticket.wait(deadline(100)), Ok(()));
        }
        assert_eq!(OTHER_RUNS.load(Ordering::Relaxed), 4);
    }

    #[test]
    fn parks_add_up(p: Peripherals) {
        static TICKS: AtomicU32 = AtomicU32::new(0);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let _guard = cpu_control
            .start_app_core(app_core_stack(), || {
                loop {
                    TICKS.fetch_add(1, Ordering::Relaxed);
                }
            })
            .unwrap();

        while TICKS.load(Ordering::Relaxed) == 0 {}
        hil_test::assert!(is_running(Cpu::AppCpu));

        let first = cpu_control.park_core(Cpu::AppCpu);
        let second = cpu_control.park_core(Cpu::AppCpu);
        hil_test::assert!(!is_running(Cpu::AppCpu));

        drop(first);
        hil_test::assert!(!is_running(Cpu::AppCpu));
        let ticks = TICKS.load(Ordering::Relaxed);
        for _ in 0..10_000 {
            core::hint::spin_loop();
        }
        hil_test::assert_eq!(TICKS.load(Ordering::Relaxed), ticks);

        drop(second);
        hil_test::assert!(is_running(Cpu::AppCpu));
        while TICKS.load(Ordering::Relaxed) == ticks {}
    }

    #[test]
    fn interrupts_run_while_park_waits(p: Peripherals) {
        static IN_SECTION: AtomicBool = AtomicBool::new(false);
        static PARKING: AtomicBool = AtomicBool::new(false);
        static RELEASE: AtomicBool = AtomicBool::new(false);

        extern "C" fn release() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            RELEASE.store(true, Ordering::Release);
        }

        let mut interrupt = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        interrupt.set_interrupt_handler(InterruptHandler::new(release, Priority::Priority2));

        // The APP core takes the park request only after an interrupt handler on the PRO core
        // runs. The park thus waits forever if the wait masks the interrupts of the PRO core.
        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let _guard = cpu_control
            .start_app_core(app_core_stack(), || {
                critical_section::with(|_| {
                    IN_SECTION.store(true, Ordering::Release);
                    while !PARKING.load(Ordering::Acquire) {}
                    // The PRO core starts to wait for the park in this time.
                    esp_hal::delay::Delay::new().delay_millis(10);
                    SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).raise();
                    while !RELEASE.load(Ordering::Acquire) {}
                });
                loop {}
            })
            .unwrap();

        // The APP core starts with its interrupts enabled. A park before the critical section
        // thus does not test the wait.
        while !IN_SECTION.load(Ordering::Acquire) {}
        PARKING.store(true, Ordering::Release);
        let parked = cpu_control.park_core(Cpu::AppCpu);
        hil_test::assert!(RELEASE.load(Ordering::Acquire));
        drop(parked);
    }

    #[test]
    fn cores_can_park_each_other_at_the_same_time(p: Peripherals) {
        static START: AtomicBool = AtomicBool::new(false);
        static DONE: AtomicBool = AtomicBool::new(false);

        const ROUNDS: u32 = 1000;

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let _guard = cpu_control
            .start_app_core(app_core_stack(), || {
                let cpu_control = CpuControl::new(unsafe { CPU_CTRL::steal() });
                while !START.load(Ordering::Relaxed) {}
                for _ in 0..ROUNDS {
                    drop(cpu_control.park_core(Cpu::ProCpu));
                }
                DONE.store(true, Ordering::Release);
                loop {}
            })
            .unwrap();

        START.store(true, Ordering::Relaxed);
        for _ in 0..ROUNDS {
            drop(cpu_control.park_core(Cpu::AppCpu));
        }

        while !DONE.load(Ordering::Acquire) {}
    }

    #[test]
    fn ipc_post_after_app_core_stops_is_offline(p: Peripherals) {
        extern "C" fn nothing() {}
        static REQUEST: IpcRequest = IpcRequest::new(nothing);

        let guard = start_ipc_app_core(&mut CpuControl::new(p.CPU_CTRL));
        drop(guard);

        hil_test::assert!(matches!(
            Ipc::new(p.IPC).post(Cpu::AppCpu, &REQUEST),
            Err(IpcError::Offline)
        ));
    }

    #[test]
    fn ipc_run_pending_at_app_core_stop_ends_offline(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count() {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let ipc = Ipc::new(p.IPC);
        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let guard = start_ipc_app_core(&mut cpu_control);

        // The parked APP core waits in its IPC handler, so the post stays queued.
        let park = cpu_control.park_core(Cpu::AppCpu);
        let ticket = ipc.post(Cpu::AppCpu, &REQUEST).unwrap();
        drop(guard);
        drop(park);

        hil_test::assert_eq!(ticket.wait(deadline(100)), Err(IpcError::Offline));
        hil_test::assert_eq!(RUNS.load(Ordering::Relaxed), 0);

        let _guard = start_ipc_app_core(&mut cpu_control);
        let ticket = ipc.post(Cpu::AppCpu, &REQUEST).unwrap();
        hil_test::assert_eq!(ticket.wait(deadline(100)), Ok(()));
        hil_test::assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_second_start_fails_while_the_app_core_runs(p: Peripherals) {
        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let _guard = cpu_control
            .start_app_core(app_core_stack(), || loop {})
            .unwrap();

        let second =
            cpu_control.start_app_core(unsafe { &mut *(&raw mut IPC_APP_CORE_STACK) }, || loop {});
        hil_test::assert!(matches!(
            second,
            Err(esp_hal::system::Error::CoreAlreadyRunning)
        ));
    }
}
