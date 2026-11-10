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
    fn start_ipc_app_core(cpu_ctrl: CPU_CTRL<'static>) -> AppCoreGuard<'static> {
        let stack = unsafe { &mut *core::ptr::addr_of_mut!(IPC_APP_CORE_STACK) };
        CpuControl::new(cpu_ctrl)
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

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn app_core_starts_again(p: Peripherals) {
        static STARTED: AtomicU32 = AtomicU32::new(0);
        static mut APP_CORE_STACK: Stack<2048> = Stack::new();

        // Each start resets the APP core, so no earlier reference to the stack is used again.
        let stack = || unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) };

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);

        // The closure returns, which parks the APP core.
        let _guard = cpu_control
            .start_app_core(stack(), || {
                STARTED.store(1, Ordering::Release);
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 1 {}
        while is_running(Cpu::AppCpu) {}

        // The closure runs until the guard drops.
        let guard = cpu_control
            .start_app_core(stack(), || {
                STARTED.store(2, Ordering::Release);
                loop {}
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 2 {}
        drop(guard);

        let _guard = cpu_control
            .start_app_core(stack(), || {
                STARTED.store(3, Ordering::Release);
                loop {}
            })
            .unwrap();
        while STARTED.load(Ordering::Acquire) != 3 {}
    }

    #[test]
    fn ipc_post_runs_on_the_other_core(p: Peripherals) {
        static ARG: AtomicUsize = AtomicUsize::new(0);
        static CORE: AtomicUsize = AtomicUsize::new(usize::MAX);
        extern "C" fn record(arg: usize) {
            ARG.store(arg, Ordering::Relaxed);
            CORE.store(Cpu::current() as usize, Ordering::Release);
        }
        static REQUEST: IpcRequest = IpcRequest::new(record);

        let _guard = start_ipc_app_core(p.CPU_CTRL);

        let ticket = Ipc::new(p.IPC).post(Cpu::AppCpu, &REQUEST, 42).unwrap();
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(CORE.load(Ordering::Acquire), Cpu::AppCpu as usize);
        assert_eq!(ARG.load(Ordering::Relaxed), 42);
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

        extern "C" fn count(_: usize) {
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
                    .post(Cpu::AppCpu, &REQUEST, 0)
                    .and_then(|ticket| ticket.wait(deadline(100)))
                {
                    Ok(()) => RESULT_OK,
                    Err(_) => RESULT_ERROR,
                }
            };
            RESULT.store(result, Ordering::Relaxed);
            HANDLED.store(true, Ordering::Release);
        }

        let _guard = start_ipc_app_core(p.CPU_CTRL);
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
        extern "C" fn count(_: usize) {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &REQUEST, 7).unwrap();
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_masked_self_wait_would_deadlock(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count(_: usize) {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let token = unsafe { SingleCoreInterruptLock.enter() };
        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &REQUEST, 0).unwrap();
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

        extern "C" fn inner(_: usize) {
            INNER_RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static INNER: IpcRequest = IpcRequest::new(inner);

        extern "C" fn outer(_: usize) {
            let result = ipc()
                .post(Cpu::current(), &INNER, 0)
                .map(|ticket| ticket.wait(deadline(10)));
            OUTER_GOT_WOULD_DEADLOCK.store(
                result == Ok(Err(IpcError::WouldDeadlock)),
                Ordering::Relaxed,
            );
            INNER_RUNS_IN_OUTER.store(INNER_RUNS.load(Ordering::Relaxed), Ordering::Relaxed);
        }
        static OUTER: IpcRequest = IpcRequest::new(outer);

        let ticket = Ipc::new(p.IPC).post(Cpu::current(), &OUTER, 0).unwrap();
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
        extern "C" fn count(_: usize) {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let _guard = start_ipc_app_core(p.CPU_CTRL);
        MASK_APP_CORE.store(true, Ordering::Release);
        while !APP_CORE_MASKED.load(Ordering::Acquire) {}

        let ticket = Ipc::new(p.IPC).post(Cpu::AppCpu, &REQUEST, 0).unwrap();
        assert_eq!(ticket.wait(deadline(10)), Err(IpcError::Timeout));
        assert_eq!(RUNS.load(Ordering::Relaxed), 0);

        MASK_APP_CORE.store(false, Ordering::Release);
        assert_eq!(ticket.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_posts_with_the_same_arg_join(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn count(_: usize) {
            RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(count);

        let ipc = Ipc::new(p.IPC);
        let token = unsafe { SingleCoreInterruptLock.enter() };
        let first = ipc.post(Cpu::current(), &REQUEST, 3);
        let second = ipc.post(Cpu::current(), &REQUEST, 3);
        unsafe { SingleCoreInterruptLock.exit(token) };

        let (first, second) = (first.unwrap(), second.unwrap());
        assert_eq!(first.wait(deadline(100)), Ok(()));
        assert_eq!(second.wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_post_with_another_arg_is_busy(p: Peripherals) {
        static RUNS: AtomicU32 = AtomicU32::new(0);
        static ARG: AtomicUsize = AtomicUsize::new(0);
        extern "C" fn record(arg: usize) {
            RUNS.fetch_add(1, Ordering::Relaxed);
            ARG.store(arg, Ordering::Relaxed);
        }
        static REQUEST: IpcRequest = IpcRequest::new(record);

        let ipc = Ipc::new(p.IPC);
        let token = unsafe { SingleCoreInterruptLock.enter() };
        let first = ipc.post(Cpu::current(), &REQUEST, 1);
        let second = ipc.post(Cpu::current(), &REQUEST, 2);
        unsafe { SingleCoreInterruptLock.exit(token) };

        assert!(matches!(second, Err(IpcError::Busy)));
        assert_eq!(first.unwrap().wait(deadline(100)), Ok(()));
        assert_eq!(RUNS.load(Ordering::Relaxed), 1);
        assert_eq!(ARG.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ipc_post_does_not_wait_for_a_busy_core(p: Peripherals) {
        static RELEASE: AtomicBool = AtomicBool::new(false);
        static LONG_STARTED: AtomicBool = AtomicBool::new(false);
        static OTHER_RUNS: AtomicU32 = AtomicU32::new(0);

        extern "C" fn long(_: usize) {
            LONG_STARTED.store(true, Ordering::Release);
            while !RELEASE.load(Ordering::Acquire) {}
        }
        static LONG: IpcRequest = IpcRequest::new(long);

        extern "C" fn other(_: usize) {
            OTHER_RUNS.fetch_add(1, Ordering::Relaxed);
        }
        static OTHERS: [IpcRequest; 4] = [
            IpcRequest::new(other),
            IpcRequest::new(other),
            IpcRequest::new(other),
            IpcRequest::new(other),
        ];

        let _guard = start_ipc_app_core(p.CPU_CTRL);
        let ipc = Ipc::new(p.IPC);

        let long_ticket = ipc.post(Cpu::AppCpu, &LONG, 0).unwrap();
        let end = deadline(100);
        while !LONG_STARTED.load(Ordering::Acquire) {
            assert!(Instant::now() < end);
        }

        let mut tickets: [Option<Ticket>; 4] = [None; 4];
        let mut work = 0u32;
        for (index, request) in OTHERS.iter().enumerate() {
            let start = Instant::now();
            tickets[index] = Some(ipc.post(Cpu::AppCpu, request, index).unwrap());
            assert!(start.elapsed() < Duration::from_millis(1));
            work += 1;
        }
        assert_eq!(work, 4);
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
}
