#[cfg(soc_has_bt)]
use esp_hal::peripherals::BT;
#[cfg(all(feature = "unstable", soc_has_ieee802154))]
use esp_hal::peripherals::IEEE802154;
#[cfg(soc_has_wifi)]
use esp_hal::peripherals::WIFI;

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c5, path = "clocks_ll/esp32c5.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
#[allow(unused)]
pub(crate) mod clocks_ll;

#[inline]
#[instability::unstable]
/// Do any common initial initialization needed for the radio clocks
pub fn init_radio_clocks() {
    clocks_ll::init_clocks();
}

// #[instability::unstable]
/// This trait provides common clock functionality for all modem peripherals.
pub trait ModemClockController<'d> {
    /// Enable the modem clock for this controller.
    fn enable_modem_clock(&mut self, enable: bool);
}

#[cfg(soc_has_wifi)]
// #[instability::unstable]
impl<'d> ModemClockController<'d> for WIFI<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_wifi(enable);
    }
}

#[cfg(soc_has_bt)]
// #[instability::unstable]
impl<'d> ModemClockController<'d> for BT<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_bt(enable);
    }
}

#[cfg(all(feature = "unstable", soc_has_ieee802154))]
// #[instability::unstable]
impl<'d> ModemClockController<'d> for IEEE802154<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_ieee802154(enable);
    }
}
