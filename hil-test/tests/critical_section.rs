//! Ensure invariants of locks are upheld.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

// TODO: add multi-core tests

#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    interrupt::{
        software::{SoftwareInterrupt, SoftwareInterruptControl},
        InterruptHandler,
        Priority,
    },
    peripherals::Peripherals,
    sync::{Locked, RawPriorityLimitedMutex},
};
use hil_test as _;

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

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
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
    fn locked_can_provide_mutable_access() {
        let flag = Locked::new(false);

        flag.with(|f| {
            *f = true;
        });
        flag.with(|f| {
            assert!(*f);
        });
    }

    #[test]
    #[should_panic]
    fn locked_is_not_reentrant() {
        let flag = Locked::new(false);

        flag.with(|_f| {
            flag.with(|f| {
                *f = true;
            });
        });
    }

    #[test]
    fn priority_lock_tests(peripherals: Peripherals) {
        use portable_atomic::{AtomicU32, Ordering};

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

        let delay = Delay::new();

        // Lock does nothing unless taken

        prio_1_interrupt.raise();
        // Software interrupts may not trigger immediately and there may be some
        // instructions executed after `raise`. We need to wait a short while
        // to ensure that the interrupt has been serviced before reading the counter.
        delay.delay_millis(1);
        assert_eq!(COUNTER.load(Ordering::Acquire), 1);

        // Taking the lock masks the lower priority interrupt
        lock.lock(|| {
            prio_1_interrupt.raise();
            delay.delay_millis(1);
            assert_eq!(COUNTER.load(Ordering::Acquire), 1); // not incremented

            // Taken lock does not mask higher priority interrupts
            prio_2_interrupt.raise();
            delay.delay_millis(1);
            assert_eq!(COUNTER.load(Ordering::Acquire), 2);
        });

        // Releasing the lock unmasks the lower priority interrupt
        delay.delay_millis(1);
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
}
