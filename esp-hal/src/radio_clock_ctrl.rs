//! Control over the radio clocs.
use core::cell::RefCell;

use critical_section::Mutex;
use portable_atomic::{AtomicBool, Ordering};

use crate::peripherals::RADIO_CLK;

/// Enumeration of the available radio peripherals for this chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioPeripherals {
    /// Represents the PHY (Physical Layer) peripheral.
    Phy,
    /// Represents the Bluetooth peripheral.
    #[cfg(bt)]
    Bt,
    /// Represents the WiFi peripheral.
    #[cfg(wifi)]
    Wifi,
    /// Represents the IEEE 802.15.4 peripheral.
    #[cfg(ieee802154)]
    Ieee802154,
}

/// Control the radio peripheral clocks
///
/// NOTE: We don't really want the user messing with the radio clocks, so this
/// is hidden. In the future, we could just make it private.
#[doc(hidden)]
pub trait RadioClockController {
    /// Enable the peripheral
    fn enable(&mut self, peripheral: RadioPeripherals);

    /// Disable the peripheral
    fn disable(&mut self, peripheral: RadioPeripherals);

    /// Reset the MAC
    fn reset_mac(&mut self);

    /// Do any common initial initialization needed
    fn init_clocks(&mut self);

    /// Initialize BLE RTC clocks
    fn ble_rtc_clk_init(&mut self);

    /// Reset the Resolvable Private Address (RPA).
    fn reset_rpa(&mut self);
}

static PHY_CLOCK_ENABLED: AtomicBool = AtomicBool::new(false);
/// The PHY clock couldn't be disabled, because some modem clocks are still
/// active.
pub struct ModemClocksEnabledError;

/// This struct allows shared access to the radio clocks.
pub struct SharedRadioClockControl {
    radio_clock: Mutex<RefCell<RADIO_CLK>>,

    #[cfg(bt)]
    bt_clock_enabled: AtomicBool,
    #[cfg(wifi)]
    wifi_clock_enabled: AtomicBool,
    #[cfg(ieee802154)]
    ieee802154_clock_enabled: AtomicBool,
}
impl SharedRadioClockControl {
    /// Create a new [SharedRadioClockControl].
    pub const fn new(radio_clock: RADIO_CLK) -> Self {
        Self {
            radio_clock: Mutex::new(RefCell::new(radio_clock)),
            #[cfg(bt)]
            bt_clock_enabled: AtomicBool::new(false),
            #[cfg(wifi)]
            wifi_clock_enabled: AtomicBool::new(false),
            #[cfg(ieee802154)]
            ieee802154_clock_enabled: AtomicBool::new(false),
        }
    }
    /// Check if any modem clocks are enabled.
    pub fn any_modem_clocks_enabled(&self) -> bool {
        let mut any_clocks_enabled = false;

        #[cfg(bt)]
        {
            any_clocks_enabled |= self.bt_clock_enabled.load(Ordering::Relaxed);
        }
        #[cfg(wifi)]
        {
            any_clocks_enabled |= self.wifi_clock_enabled.load(Ordering::Relaxed);
        }
        #[cfg(ieee802154)]
        {
            any_clocks_enabled |= self.ieee802154_clock_enabled.load(Ordering::Relaxed);
        }

        any_clocks_enabled
    }
    /// Enable or disable the clock, without changing the PHY clock status.
    fn set_clock_status_raw(&self, enabled: bool, radio_peripheral: RadioPeripherals) {
        critical_section::with(|cs| {
            let mut radio_clock = self.radio_clock.borrow_ref_mut(cs);
            if enabled {
                radio_clock.enable(radio_peripheral)
            } else {
                radio_clock.disable(radio_peripheral)
            }
        })
    }
    /// Enable or disable the PHY clock.
    ///
    /// If the PHY clock state was successfully changed, or the current state
    /// already matched the specified one `Ok(())` is returned.
    /// If the PHY clock is being disabled, but other modem clocks are still
    /// active `Err(ModemClocksEnabledError)` is returned.
    fn set_phy_clock_status(&self, enabled: bool) -> Result<(), ModemClocksEnabledError> {
        // We shouldn't disable the PHY clock, if other clocks are still active.
        if !enabled && self.any_modem_clocks_enabled() {
            return Err(ModemClocksEnabledError);
        }
        // If the current clock status is already the same as the user provided one, we
        // return. Otherwise we store the new value.
        if PHY_CLOCK_ENABLED
            .compare_exchange(!enabled, enabled, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return Ok(());
        }
        self.set_clock_status_raw(enabled, RadioPeripherals::Phy);
        Ok(())
    }
    /// Set the status of the modem clock, enabling or disabling the PHY if
    /// needed.
    fn set_modem_clock_status_internal(&self, enabled: bool, radio_peripheral: RadioPeripherals) {
        // Depending on if we're enabling or disabling, we need to set the PHY and modem
        // clock status in a different order.
        if enabled {
            let _ = self.set_phy_clock_status(true);
            self.set_clock_status_raw(true, radio_peripheral);
        } else {
            self.set_clock_status_raw(false, radio_peripheral);
            let _ = self.set_phy_clock_status(false);
        }
    }
    /// Enable or disable a modem clock.
    ///
    /// If required, this will also enabled or disable the PHY clock.
    /// # Panics
    /// This panics, if [RadioPeripherals::Phy] is passed in, since the PHY
    /// clock is controlled internally.
    pub fn set_modem_clock_status(&self, enabled: bool, radio_peripheral: RadioPeripherals) {
        // We first assert, that the clock we're touching isn't the PHY clock, since
        // that's special.
        assert_ne!(radio_peripheral, RadioPeripherals::Phy);

        // We obtain a reference to the AtomicBool tracking the state of the clock.
        let modem_clock_enabled = match radio_peripheral {
            #[cfg(bt)]
            RadioPeripherals::Bt => &self.bt_clock_enabled,
            #[cfg(wifi)]
            RadioPeripherals::Wifi => &self.wifi_clock_enabled,
            #[cfg(ieee802154)]
            RadioPeripherals::Ieee802154 => &self.ieee802154_clock_enabled,
            _ => unreachable!(),
        };
        // If the states already match, we return. Otherwise the new state is stored.
        if modem_clock_enabled
            .compare_exchange(!enabled, enabled, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return;
        }
        self.set_modem_clock_status_internal(enabled, radio_peripheral);
    }
}
