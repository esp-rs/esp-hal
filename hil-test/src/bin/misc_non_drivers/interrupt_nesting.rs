#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use defmt::info;
    use esp_hal::{
        handler,
        interrupt::{self, Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR2, FROM_CPU_INTR3, Peripherals},
        system::Cpu,
    };
    use portable_atomic::{AtomicU32, Ordering};

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn different_priority_levels_can_interrupt_each_other(p: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        #[handler(priority = Priority::min())]
        fn handler_low_prio() {
            info!("Low runs");
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();

            // Trigger high priority interrupt before incrementing counter
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).raise();
            // See that high priority interrupted us
            hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), 1);

            COUNTER.fetch_add(1, Ordering::Relaxed);
            info!("Low returns");
        }

        #[handler(priority = Priority::max())]
        fn handler_high_prio() {
            info!("High runs");
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();

            // See if we interrupted the low priority handler as we should
            assert_eq!(COUNTER.load(Ordering::Relaxed), 0);
            COUNTER.fetch_add(1, Ordering::Relaxed);
            info!("High returns");
        }

        let mut interrupt_low = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        let mut interrupt_high = SoftwareInterrupt::new(p.FROM_CPU_INTR3);

        interrupt_low.set_interrupt_handler(handler_low_prio);
        interrupt_high.set_interrupt_handler(handler_high_prio);

        interrupt_low.raise();
        hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), 2);
    }

    fn stack_pointer() -> u32 {
        let sp: u32;
        #[cfg(target_arch = "xtensa")]
        unsafe {
            core::arch::asm!("mov {0}, sp", out(reg) sp)
        };
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("mv {0}, sp", out(reg) sp)
        };
        sp
    }

    #[test]
    fn handlers_run_on_the_isr_stack(p: Peripherals) {
        static LOW_SP: AtomicU32 = AtomicU32::new(0);
        static HIGH_SP: AtomicU32 = AtomicU32::new(0);

        #[handler(priority = Priority::min())]
        fn handler_low_prio() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            LOW_SP.store(stack_pointer(), Ordering::Relaxed);
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).raise();
        }

        #[handler(priority = Priority::max())]
        fn handler_high_prio() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();
            HIGH_SP.store(stack_pointer(), Ordering::Relaxed);
        }

        unsafe extern "C" {
            static _isr_stack_bottom_cpu0: u32;
            static _isr_stack_top_cpu0: u32;
        }
        let bottom = (&raw const _isr_stack_bottom_cpu0) as u32;
        let top = (&raw const _isr_stack_top_cpu0) as u32;

        let mut interrupt_low = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        let mut interrupt_high = SoftwareInterrupt::new(p.FROM_CPU_INTR3);

        interrupt_low.set_interrupt_handler(handler_low_prio);
        interrupt_high.set_interrupt_handler(handler_high_prio);

        interrupt_low.raise();

        let low = LOW_SP.load(Ordering::Relaxed);
        let high = HIGH_SP.load(Ordering::Relaxed);
        hil_test::assert!((bottom..top).contains(&low));
        hil_test::assert!((bottom..low).contains(&high));
    }

    #[test]
    fn unused_isr_stack_shrinks_when_a_handler_uses_the_stack(p: Peripherals) {
        const USED: usize = 1024;

        #[handler]
        fn use_stack() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            let buffer = [0x5a_u8; USED];
            core::hint::black_box(&buffer);
        }

        let before = interrupt::unused_isr_stack(Cpu::ProCpu);

        let mut sw_int = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        sw_int.set_interrupt_handler(use_stack);
        sw_int.raise();

        let after = interrupt::unused_isr_stack(Cpu::ProCpu);
        hil_test::assert!(
            before >= after + USED,
            "unused before: {}, after: {}",
            before,
            after
        );
    }

    #[test]
    #[should_panic]
    #[cfg(not(soc_cpu_watchpoints = "2"))]
    fn writing_the_isr_stack_guard_panics() {
        // The default `ESP_HAL_CONFIG_STACK_GUARD_OFFSET`.
        const GUARD_OFFSET: usize = 60;

        unsafe extern "C" {
            static mut _isr_stack_bottom_cpu0: u32;
        }
        let guard = unsafe { (&raw mut _isr_stack_bottom_cpu0).byte_add(GUARD_OFFSET) };
        unsafe { guard.write_volatile(0) };
    }

    #[test]
    #[should_panic]
    #[cfg(all(target_arch = "riscv32", not(any(esp32c2, esp32c3))))]
    fn code_on_the_isr_stack_does_not_run(p: Peripherals) {
        // PMP makes the stacks, including the interrupt stacks, not executable.
        #[handler]
        fn run_code_from_the_stack() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();

            // `ret`
            let code = core::hint::black_box([0x0000_8067_u32]);
            unsafe { core::arch::asm!("fence.i") };
            let function: extern "C" fn() = unsafe { core::mem::transmute(code.as_ptr()) };
            function();
        }

        let mut sw_int = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        sw_int.set_interrupt_handler(run_code_from_the_stack);
        sw_int.raise();
    }

    #[test]
    #[cfg(all(multi_core, soc_has_cpu_ctrl))]
    fn app_cpu_started_again_runs_handlers_on_its_isr_stack(p: Peripherals) {
        // The APP CPU stops inside a handler, then starts again. Its next handler must still run
        // on its interrupt stack.
        use esp_hal::system::{CpuControl, Stack};
        use portable_atomic::AtomicBool;

        static mut APP_CORE_STACK: Stack<8192> = Stack::new();
        static IN_HANDLER: AtomicBool = AtomicBool::new(false);
        static SP: AtomicU32 = AtomicU32::new(0);

        #[handler]
        fn never_returns() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            IN_HANDLER.store(true, Ordering::Release);
            loop {}
        }

        #[handler]
        fn record_stack_pointer() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();
            SP.store(stack_pointer(), Ordering::Release);
        }

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);

        let guard = cpu_control
            .start_app_core(unsafe { &mut *(&raw mut APP_CORE_STACK) }, || {
                let mut sw_int = SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() });
                sw_int.set_interrupt_handler(never_returns);
                sw_int.raise();
                loop {}
            })
            .unwrap();
        while !IN_HANDLER.load(Ordering::Acquire) {}
        drop(guard);

        let _guard = cpu_control
            .start_app_core(unsafe { &mut *(&raw mut APP_CORE_STACK) }, || {
                let mut sw_int = SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() });
                sw_int.set_interrupt_handler(record_stack_pointer);
                sw_int.raise();
                loop {}
            })
            .unwrap();

        let sp = loop {
            let sp = SP.load(Ordering::Acquire);
            if sp != 0 {
                break sp;
            }
        };

        unsafe extern "C" {
            static _isr_stack_bottom_cpu1: u32;
            static _isr_stack_top_cpu1: u32;
        }
        let bottom = (&raw const _isr_stack_bottom_cpu1) as u32;
        let top = (&raw const _isr_stack_top_cpu1) as u32;
        hil_test::assert!(
            (bottom..top).contains(&sp),
            "sp: {:x}, interrupt stack: {:x}..{:x}",
            sp,
            bottom,
            top
        );
    }
}
