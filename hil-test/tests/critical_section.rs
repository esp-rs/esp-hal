//! Ensure invariants of locks are upheld.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

// TODO: add multi-core tests

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::sync::Locked;

    #[init]
    fn init() {
        esp_hal::init(esp_hal::Config::default());
    }

    #[test]
    #[timeout(3)]
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
    #[timeout(3)]
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
}
