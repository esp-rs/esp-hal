//! IEEE 802.15.4 driver tests.

//% CHIP_FILTER: ieee802154_driver_supported && rng_trng_supported
//% FEATURES: unstable esp-alloc esp-radio/ieee802154 esp-radio/unstable

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{peripherals::Peripherals, rng::Trng};
    use esp_radio::ieee802154::Ieee802154;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(size: 24 * 1024);
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn trng_is_available_while_radio_is_enabled(p: Peripherals) {
        assert!(Trng::try_new().is_err());

        let radio = Ieee802154::new(p.IEEE802154);
        let trng = Trng::try_new().unwrap();

        let mut values = [0u8; 16];
        trng.read(&mut values);
        assert!(values.windows(2).any(|w| w[0] != w[1]));

        core::mem::drop(trng);
        core::mem::drop(radio);

        assert!(Trng::try_new().is_err());
    }
}
