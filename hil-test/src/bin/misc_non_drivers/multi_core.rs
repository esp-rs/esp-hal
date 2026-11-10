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
        static CORE: AtomicUsize = AtomicUsize::new(usize::MAX);
        extern "C" fn record() {
            CORE.store(Cpu::current() as usize, Ordering::Release);
        }
        static REQUEST: IpcRequest = IpcRequest::new(record);

        let _guard = start_ipc_app_core(p.CPU_CTRL);

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

        let _guard = start_ipc_app_core(p.CPU_CTRL);
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

        let _guard = start_ipc_app_core(p.CPU_CTRL);
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

    /// The number of FP registers that an interrupt entry must keep, and the words after them
    /// that hold the FP control and status registers.
    #[cfg(all(riscv, target_feature = "f"))]
    const FP_REGS: usize = 20;
    #[cfg(all(riscv, target_feature = "f"))]
    const FP_WORDS: usize = FP_REGS + 1;
    #[cfg(all(xtensa, target_feature = "fp"))]
    const FP_REGS: usize = 16;
    #[cfg(all(xtensa, target_feature = "fp"))]
    const FP_WORDS: usize = FP_REGS + 2;

    #[cfg(any(target_feature = "f", target_feature = "fp"))]
    const fn fp_pattern(invert: u32) -> [u32; FP_REGS] {
        let mut pattern = [0; FP_REGS];
        let mut i = 0;
        while i < FP_REGS {
            pattern[i] = (0x3f80_0000 + i as u32 * 0x0101_0101) ^ invert;
            i += 1;
        }
        pattern
    }

    /// Loads `pattern` into the FP registers, and `control` into the FP control register.
    ///
    /// # Safety
    ///
    /// The caller must declare the FP registers as clobbered.
    #[cfg(any(target_feature = "f", target_feature = "fp"))]
    #[inline(always)]
    unsafe fn load_fp(pattern: &[u32; FP_REGS], control: u32) {
        cfg_select! {
            riscv => unsafe {
                core::arch::asm!(
                    "flw ft0, 0({p})",
                    "flw ft1, 4({p})",
                    "flw ft2, 8({p})",
                    "flw ft3, 12({p})",
                    "flw ft4, 16({p})",
                    "flw ft5, 20({p})",
                    "flw ft6, 24({p})",
                    "flw ft7, 28({p})",
                    "flw ft8, 32({p})",
                    "flw ft9, 36({p})",
                    "flw ft10, 40({p})",
                    "flw ft11, 44({p})",
                    "flw fa0, 48({p})",
                    "flw fa1, 52({p})",
                    "flw fa2, 56({p})",
                    "flw fa3, 60({p})",
                    "flw fa4, 64({p})",
                    "flw fa5, 68({p})",
                    "flw fa6, 72({p})",
                    "flw fa7, 76({p})",
                    "fscsr {c}",
                    p = in(reg) pattern.as_ptr(),
                    c = in(reg) control,
                    out("ft0") _,
                    out("ft1") _,
                    out("ft2") _,
                    out("ft3") _,
                    out("ft4") _,
                    out("ft5") _,
                    out("ft6") _,
                    out("ft7") _,
                    out("ft8") _,
                    out("ft9") _,
                    out("ft10") _,
                    out("ft11") _,
                    out("fa0") _,
                    out("fa1") _,
                    out("fa2") _,
                    out("fa3") _,
                    out("fa4") _,
                    out("fa5") _,
                    out("fa6") _,
                    out("fa7") _,
                )
            },
            xtensa => unsafe {
                core::arch::asm!(
                    "lsi f0, {p}, 0",
                    "lsi f1, {p}, 4",
                    "lsi f2, {p}, 8",
                    "lsi f3, {p}, 12",
                    "lsi f4, {p}, 16",
                    "lsi f5, {p}, 20",
                    "lsi f6, {p}, 24",
                    "lsi f7, {p}, 28",
                    "lsi f8, {p}, 32",
                    "lsi f9, {p}, 36",
                    "lsi f10, {p}, 40",
                    "lsi f11, {p}, 44",
                    "lsi f12, {p}, 48",
                    "lsi f13, {p}, 52",
                    "lsi f14, {p}, 56",
                    "lsi f15, {p}, 60",
                    "wur.fcr {c}",
                    p = in(reg) pattern.as_ptr(),
                    c = in(reg) control,
                    out("f0") _,
                    out("f1") _,
                    out("f2") _,
                    out("f3") _,
                    out("f4") _,
                    out("f5") _,
                    out("f6") _,
                    out("f7") _,
                    out("f8") _,
                    out("f9") _,
                    out("f10") _,
                    out("f11") _,
                    out("f12") _,
                    out("f13") _,
                    out("f14") _,
                    out("f15") _,
                )
            },
        }
    }

    #[test]
    #[cfg(any(target_feature = "f", target_feature = "fp"))]
    fn ipc_func_keeps_the_fpu_state_of_the_interrupted_code(p: Peripherals) {
        // The rounding mode "toward zero", with no exception flags.
        #[cfg(riscv)]
        const CONTROL: u32 = 0b001 << 5;
        #[cfg(xtensa)]
        const CONTROL: u32 = 1;

        static READY: AtomicU32 = AtomicU32::new(0);
        static DONE: AtomicU32 = AtomicU32::new(0);
        static RESULT: AtomicU32 = AtomicU32::new(0);

        extern "C" fn use_fpu() {
            let x = core::hint::black_box(1.5f32);
            RESULT.store((x * x + 0.25).to_bits(), Ordering::Relaxed);
            unsafe { load_fp(&fp_pattern(u32::MAX), 0) };
            DONE.store(1, Ordering::Release);
        }
        static USE_FPU: IpcRequest = IpcRequest::new(use_fpu);

        static mut APP_CORE_STACK: Stack<4096> = Stack::new();
        let _guard = CpuControl::new(p.CPU_CTRL)
            .start_app_core(
                unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
                || {
                    while READY.load(Ordering::Acquire) == 0 {}
                    let _ = ipc().post(Cpu::ProCpu, &USE_FPU);
                    loop {}
                },
            )
            .unwrap();

        let pattern = fp_pattern(0);
        let mut saved = [0u32; FP_WORDS];
        // One block keeps the compiler away from the FP registers between the load and the store.
        cfg_select! {
            riscv => unsafe {
                core::arch::asm!(
                    "flw ft0, 0({p})",
                    "flw ft1, 4({p})",
                    "flw ft2, 8({p})",
                    "flw ft3, 12({p})",
                    "flw ft4, 16({p})",
                    "flw ft5, 20({p})",
                    "flw ft6, 24({p})",
                    "flw ft7, 28({p})",
                    "flw ft8, 32({p})",
                    "flw ft9, 36({p})",
                    "flw ft10, 40({p})",
                    "flw ft11, 44({p})",
                    "flw fa0, 48({p})",
                    "flw fa1, 52({p})",
                    "flw fa2, 56({p})",
                    "flw fa3, 60({p})",
                    "flw fa4, 64({p})",
                    "flw fa5, 68({p})",
                    "flw fa6, 72({p})",
                    "flw fa7, 76({p})",
                    "fscsr {c}",
                    "sw {t}, 0({ready})",
                    "2:",
                    "lw {t}, 0({done})",
                    "beqz {t}, 2b",
                    "fsw ft0, 0({s})",
                    "fsw ft1, 4({s})",
                    "fsw ft2, 8({s})",
                    "fsw ft3, 12({s})",
                    "fsw ft4, 16({s})",
                    "fsw ft5, 20({s})",
                    "fsw ft6, 24({s})",
                    "fsw ft7, 28({s})",
                    "fsw ft8, 32({s})",
                    "fsw ft9, 36({s})",
                    "fsw ft10, 40({s})",
                    "fsw ft11, 44({s})",
                    "fsw fa0, 48({s})",
                    "fsw fa1, 52({s})",
                    "fsw fa2, 56({s})",
                    "fsw fa3, 60({s})",
                    "fsw fa4, 64({s})",
                    "fsw fa5, 68({s})",
                    "fsw fa6, 72({s})",
                    "fsw fa7, 76({s})",
                    "frcsr {t}",
                    "sw {t}, 80({s})",
                    p = in(reg) pattern.as_ptr(),
                    s = in(reg) saved.as_mut_ptr(),
                    c = in(reg) CONTROL,
                    ready = in(reg) READY.as_ptr(),
                    done = in(reg) DONE.as_ptr(),
                    t = inout(reg) 1u32 => _,
                    out("ft0") _,
                    out("ft1") _,
                    out("ft2") _,
                    out("ft3") _,
                    out("ft4") _,
                    out("ft5") _,
                    out("ft6") _,
                    out("ft7") _,
                    out("ft8") _,
                    out("ft9") _,
                    out("ft10") _,
                    out("ft11") _,
                    out("fa0") _,
                    out("fa1") _,
                    out("fa2") _,
                    out("fa3") _,
                    out("fa4") _,
                    out("fa5") _,
                    out("fa6") _,
                    out("fa7") _,
                )
            },
            xtensa => unsafe {
                core::arch::asm!(
                    "lsi f0, {p}, 0",
                    "lsi f1, {p}, 4",
                    "lsi f2, {p}, 8",
                    "lsi f3, {p}, 12",
                    "lsi f4, {p}, 16",
                    "lsi f5, {p}, 20",
                    "lsi f6, {p}, 24",
                    "lsi f7, {p}, 28",
                    "lsi f8, {p}, 32",
                    "lsi f9, {p}, 36",
                    "lsi f10, {p}, 40",
                    "lsi f11, {p}, 44",
                    "lsi f12, {p}, 48",
                    "lsi f13, {p}, 52",
                    "lsi f14, {p}, 56",
                    "lsi f15, {p}, 60",
                    "wur.fcr {c}",
                    "wur.fsr {z}",
                    "s32i {t}, {ready}, 0",
                    "memw",
                    "2:",
                    "memw",
                    "l32i {t}, {done}, 0",
                    "beqz {t}, 2b",
                    "ssi f0, {s}, 0",
                    "ssi f1, {s}, 4",
                    "ssi f2, {s}, 8",
                    "ssi f3, {s}, 12",
                    "ssi f4, {s}, 16",
                    "ssi f5, {s}, 20",
                    "ssi f6, {s}, 24",
                    "ssi f7, {s}, 28",
                    "ssi f8, {s}, 32",
                    "ssi f9, {s}, 36",
                    "ssi f10, {s}, 40",
                    "ssi f11, {s}, 44",
                    "ssi f12, {s}, 48",
                    "ssi f13, {s}, 52",
                    "ssi f14, {s}, 56",
                    "ssi f15, {s}, 60",
                    "rur.fcr {t}",
                    "s32i {t}, {s}, 64",
                    "rur.fsr {t}",
                    "s32i {t}, {s}, 68",
                    p = in(reg) pattern.as_ptr(),
                    s = in(reg) saved.as_mut_ptr(),
                    c = in(reg) CONTROL,
                    z = in(reg) 0u32,
                    ready = in(reg) READY.as_ptr(),
                    done = in(reg) DONE.as_ptr(),
                    t = inout(reg) 1u32 => _,
                    out("f0") _,
                    out("f1") _,
                    out("f2") _,
                    out("f3") _,
                    out("f4") _,
                    out("f5") _,
                    out("f6") _,
                    out("f7") _,
                    out("f8") _,
                    out("f9") _,
                    out("f10") _,
                    out("f11") _,
                    out("f12") _,
                    out("f13") _,
                    out("f14") _,
                    out("f15") _,
                )
            },
        }

        let x = 1.5f32;
        assert_eq!(RESULT.load(Ordering::Relaxed), (x * x + 0.25).to_bits());
        assert_eq!(saved[..FP_REGS], pattern[..]);
        assert_eq!(saved[FP_REGS], CONTROL);
        #[cfg(xtensa)]
        assert_eq!(saved[FP_REGS + 1], 0);
    }
}
