#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        peripherals::Peripherals,
        system::{Cpu, CpuControl, Stack, is_running},
    };
    use hil_test as _;
    use portable_atomic::{AtomicU32, Ordering};

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
}
