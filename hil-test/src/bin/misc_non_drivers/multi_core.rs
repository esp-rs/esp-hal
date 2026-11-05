#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use core::sync::atomic::AtomicBool;

    use esp_hal::{
        peripherals::{CPU_CTRL, Peripherals},
        system::{Cpu, CpuControl, Stack, is_running},
    };
    use hil_test as _;
    use portable_atomic::{AtomicU32, Ordering};

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
}
