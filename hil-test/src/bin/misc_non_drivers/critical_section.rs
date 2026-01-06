#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        clock::CpuClock,
        interrupt::{
            InterruptHandler,
            Priority,
            software::{SoftwareInterrupt, SoftwareInterruptControl},
        },
        peripherals::Peripherals,
        sync::RawPriorityLimitedMutex,
    };
    use esp_sync::NonReentrantMutex;
    use portable_atomic::{AtomicU32, Ordering};

    fn test_access_at_priority(peripherals: Peripherals, priority: Priority) {
        static LOCK: RawPriorityLimitedMutex = RawPriorityLimitedMutex::new(Priority::Priority1);

        extern "C" fn access<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            LOCK.lock(|| {});
            embedded_test::export::check_outcome(());
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut prio_2_interrupt = sw_ints.software_interrupt1;

        prio_2_interrupt.set_interrupt_handler(InterruptHandler::new(access::<1>, priority));

        prio_2_interrupt.raise();
        loop {}
    }

    fn software_interrupt_fires_before_returning(p: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        extern "C" fn increment<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            COUNTER.fetch_add(1, Ordering::AcqRel);
        }

        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);

        let mut interrupt = sw_ints.software_interrupt0;
        interrupt.set_interrupt_handler(InterruptHandler::new(increment::<0>, Priority::Priority1));

        const ITERATIONS: u32 = 100;

        for _ in 0..ITERATIONS {
            interrupt.raise();
            interrupt.raise();
        }

        hil_test::assert_eq!(COUNTER.load(Ordering::Relaxed), ITERATIONS * 2);
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    fn init_max_cpu_speed() -> Peripherals {
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()))
    }

    #[test]
    fn critical_section_is_reentrant() {
        let mut flag = false;

        critical_section::with(|_| {
            critical_section::with(|_| {
                flag = true;
            });
        });

        assert!(flag);
    }

    #[test]
    fn non_reentrant_mutex_can_provide_mutable_access() {
        let flag = NonReentrantMutex::new(false);

        flag.with(|f| {
            *f = true;
        });
        flag.with(|f| {
            assert!(*f);
        });
    }

    #[test]
    #[should_panic]
    fn non_reentrant_mutex_is_not_reentrant() {
        let flag = NonReentrantMutex::new(false);

        flag.with(|_f| {
            flag.with(|f| {
                *f = true;
            });
        });
    }

    #[test]
    fn priority_lock_tests(peripherals: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        extern "C" fn increment<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            COUNTER.fetch_add(1, Ordering::AcqRel);
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut prio_1_interrupt = sw_ints.software_interrupt0;
        let mut prio_2_interrupt = sw_ints.software_interrupt1;

        prio_1_interrupt
            .set_interrupt_handler(InterruptHandler::new(increment::<0>, Priority::Priority1));
        prio_2_interrupt
            .set_interrupt_handler(InterruptHandler::new(increment::<1>, Priority::Priority2));

        let lock = RawPriorityLimitedMutex::new(Priority::Priority1);

        // Lock does nothing unless taken

        prio_1_interrupt.raise();
        assert_eq!(COUNTER.load(Ordering::Acquire), 1);

        // Taking the lock masks the lower priority interrupt
        lock.lock(|| {
            prio_1_interrupt.raise();
            assert_eq!(COUNTER.load(Ordering::Acquire), 1); // not incremented

            // Taken lock does not mask higher priority interrupts
            prio_2_interrupt.raise();
            assert_eq!(COUNTER.load(Ordering::Acquire), 2);
        });

        // Releasing the lock unmasks the lower priority interrupt
        assert_eq!(COUNTER.load(Ordering::Acquire), 3);
    }

    #[test]
    fn priority_lock_allows_access_from_equal_priority(peripherals: Peripherals) {
        test_access_at_priority(peripherals, Priority::Priority1);
    }

    #[test]
    #[should_panic]
    fn priority_lock_panics_on_higher_priority_access(peripherals: Peripherals) {
        test_access_at_priority(peripherals, Priority::Priority2);
    }

    #[test]
    fn max_priority_lock_is_masking_interrupt(peripherals: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        extern "C" fn increment<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            COUNTER.fetch_add(1, Ordering::AcqRel);
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut interrupt = sw_ints.software_interrupt0;
        interrupt.set_interrupt_handler(InterruptHandler::new(increment::<0>, Priority::Priority1));

        let lock = RawPriorityLimitedMutex::new(Priority::max());

        // Taking the lock masks the lower priority interrupt
        lock.lock(|| {
            interrupt.raise();
            assert_eq!(COUNTER.load(Ordering::Acquire), 0); // not incremented
        });

        // Releasing the lock unmasks the lower priority interrupt
        assert_eq!(COUNTER.load(Ordering::Acquire), 1);
    }

    #[test]
    #[cfg(multi_core)]
    fn critical_section_on_multi_core(p: Peripherals) {
        // TODO: test other locks, too
        use core::{cell::Cell, sync::atomic::AtomicBool};

        use critical_section::Mutex;
        use esp_hal::system::{CpuControl, Stack};
        use hil_test::mk_static;

        static COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
        static START_COUNTING: AtomicBool = AtomicBool::new(false);
        static DONE_COUNTING: AtomicBool = AtomicBool::new(false);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());

        let cpu1_fnctn = || {
            while !START_COUNTING.load(Ordering::Relaxed) {}
            for _ in 0..1000 {
                critical_section::with(|cs| {
                    let data_ref = COUNTER.borrow(cs);
                    data_ref.set(data_ref.get() + 1);
                });
            }
            DONE_COUNTING.store(true, Ordering::Relaxed);
            loop {}
        };

        let _guard = cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        START_COUNTING.store(true, Ordering::Relaxed);
        for _ in 0..1000 {
            critical_section::with(|cs| {
                let data_ref = COUNTER.borrow(cs);
                data_ref.set(data_ref.get() + 2);
            });
        }

        while !DONE_COUNTING.load(Ordering::Relaxed) {}

        critical_section::with(|cs| {
            let data_ref = COUNTER.borrow(cs);
            assert_eq!(data_ref.get(), 3000);
        });
    }

    #[test]
    fn software_interrupts_fire_immediately(p: Peripherals) {
        software_interrupt_fires_before_returning(p);
    }

    #[test(init = init_max_cpu_speed)]
    fn software_interrupts_fire_immediately_max_speed(p: Peripherals) {
        software_interrupt_fires_before_returning(p);
    }
}
