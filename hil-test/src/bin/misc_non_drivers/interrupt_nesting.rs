#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use defmt::info;
    use esp_hal::{
        handler,
        interrupt::{Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR2, FROM_CPU_INTR3, Peripherals},
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

    #[test]
    fn handlers_run_on_the_isr_stack(p: Peripherals) {
        static LOW_SP: AtomicU32 = AtomicU32::new(0);
        static HIGH_SP: AtomicU32 = AtomicU32::new(0);

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
}
