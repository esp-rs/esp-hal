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
    #[should_panic]
    #[cfg(riscv)]
    fn stack_guard_is_monitored_after_nested_interrupts(p: Peripherals) {
        unsafe extern "C" {
            static mut __stack_chk_guard: u32;
        }

        #[handler(priority = Priority::min())]
        fn handler_low_prio() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).raise();
        }

        #[handler(priority = Priority::max())]
        fn handler_high_prio() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();
        }

        let mut interrupt_low = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        let mut interrupt_high = SoftwareInterrupt::new(p.FROM_CPU_INTR3);

        interrupt_low.set_interrupt_handler(handler_low_prio);
        interrupt_high.set_interrupt_handler(handler_high_prio);

        // The high priority interrupt nests in the low priority one
        interrupt_low.raise();

        unsafe { (&raw mut __stack_chk_guard).write_volatile(0) };
    }
}
