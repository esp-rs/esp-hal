#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use defmt::info;
    use esp_hal::{
        handler,
        interrupt::{Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR0, FROM_CPU_INTR1, FROM_CPU_INTR2, Peripherals},
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
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).reset();

            // Fire the checker interrupt. It must run after this one and the higher prio one are
            // done.
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).raise();

            // Trigger high priority interrupt before incrementing counter
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).raise();
            // Se that high priority interrupted us
            hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), 1);

            COUNTER.fetch_add(1, Ordering::Relaxed);
            info!("Low returns");
        }

        #[handler(priority = Priority::min())]
        fn handler_low_prio_checker() {
            info!("Checker runs");
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();

            // Se that we didn't interrupt anything
            hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), 2);

            COUNTER.fetch_add(1, Ordering::Relaxed);
            info!("Checker returns");
        }

        #[handler(priority = Priority::max())]
        fn handler_high_prio() {
            info!("High runs");
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).reset();

            // See if we interrupted the low priority handler as we should
            assert_eq!(COUNTER.load(Ordering::Relaxed), 0);
            COUNTER.fetch_add(1, Ordering::Relaxed);
            info!("High returns");
        }

        let mut interrupt0 = SoftwareInterrupt::new(p.FROM_CPU_INTR0);
        let mut interrupt1 = SoftwareInterrupt::new(p.FROM_CPU_INTR1);
        let mut interrupt2 = SoftwareInterrupt::new(p.FROM_CPU_INTR2);

        interrupt0.set_interrupt_handler(handler_low_prio);
        interrupt1.set_interrupt_handler(handler_high_prio);
        interrupt2.set_interrupt_handler(handler_low_prio_checker);

        interrupt0.raise();
        hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), 3);
    }
}
