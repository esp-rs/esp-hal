//% FEATURES: unstable

#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use esp_hal::efuse::{self, InterfaceMacAddress, MacAddress};
    use hil_test as _;

    #[init]
    fn init() {
        _ = esp_hal::init(esp_hal::Config::default());
    }

    /// All available interface MACs must be distinct from each other.
    #[test]
    fn interface_macs_are_distinct() {
        #[cfg(soc_has_wifi)]
        let sta = efuse::interface_mac_address(InterfaceMacAddress::Station);
        #[cfg(soc_has_wifi)]
        let ap = efuse::interface_mac_address(InterfaceMacAddress::AccessPoint);
        #[cfg(soc_has_bt)]
        let bt = efuse::interface_mac_address(InterfaceMacAddress::Bluetooth);

        #[cfg(soc_has_wifi)]
        assert_ne!(sta, ap);
        #[cfg(all(soc_has_wifi, soc_has_bt))]
        {
            assert_ne!(sta, bt);
            assert_ne!(ap, bt);
        }
    }

    /// Derived interfaces (AP, BT) must have the locally-administered bit set.
    #[test]
    fn derived_macs_have_local_admin_bit() {
        #[cfg(soc_has_wifi)]
        {
            let ap = efuse::interface_mac_address(InterfaceMacAddress::AccessPoint);
            assert_ne!(ap.as_bytes()[0] & 0x02, 0);
        }
        #[cfg(soc_has_bt)]
        {
            let bt = efuse::interface_mac_address(InterfaceMacAddress::Bluetooth);
            assert_ne!(bt.as_bytes()[0] & 0x02, 0);
        }
    }

    /// Exercises the override flow using unstable helpers:
    /// 1. Station defaults to the eFuse base MAC.
    /// 2. After `override_mac_address`, Station reflects the override.
    /// 3. A second `override_mac_address` returns `AlreadySet`.
    #[cfg(soc_has_wifi)]
    #[test]
    fn set_mac_override_updates_interface_mac() {
        let base = efuse::base_mac_address();
        let sta_before = efuse::interface_mac_address(InterfaceMacAddress::Station);
        assert_eq!(sta_before, base);

        let custom = MacAddress::new_eui48([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
        efuse::override_mac_address(custom).expect("first override_mac_address should succeed");

        // Station must now return the overridden MAC
        let sta_after = efuse::interface_mac_address(InterfaceMacAddress::Station);
        assert_eq!(sta_after, custom);
        assert_ne!(sta_after, base);

        // mac_address() must still return the factory eFuse value
        assert_eq!(efuse::base_mac_address(), base);

        // Second set must fail
        let another = MacAddress::new_eui48([0x02, 0x00, 0x00, 0x00, 0x00, 0x02]);
        assert!(efuse::override_mac_address(another).is_err());
    }
}
