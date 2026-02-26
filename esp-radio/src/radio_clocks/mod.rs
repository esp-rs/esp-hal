use core::{cell::Cell, marker::PhantomData};

use esp_sync::RawMutex;

use super::*;
#[cfg(soc_has_bt)]
use esp_hal::peripherals::BT;
#[cfg(all(feature = "unstable", soc_has_ieee802154))]
use esp_hal::peripherals::IEEE802154;
#[cfg(soc_has_wifi)]
use esp_hal::peripherals::WIFI;
use crate::private::Sealed;

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c5, path = "clocks_ll/esp32c5.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
pub(crate) mod clocks_ll;

/// Tracks the number of references to the PHY clock.
static PHY_CLOCK_REF_COUNTER: embassy_sync::blocking_mutex::Mutex<RawMutex, Cell<u8>> =
    embassy_sync::blocking_mutex::Mutex::new(Cell::new(0));

fn increase_phy_clock_ref_count_internal() {
    PHY_CLOCK_REF_COUNTER.lock(|phy_clock_ref_counter| {
        let phy_clock_ref_count = phy_clock_ref_counter.get();

        if phy_clock_ref_count == 0 {
            clocks_ll::enable_phy(true);
        }
        let new_phy_clock_ref_count = unwrap!(
            phy_clock_ref_count.checked_add(1),
            "PHY clock ref count overflowed."
        );

        phy_clock_ref_counter.set(new_phy_clock_ref_count);
    })
}

fn decrease_phy_clock_ref_count_internal() {
    PHY_CLOCK_REF_COUNTER.lock(|phy_clock_ref_counter| {
        let new_phy_clock_ref_count = unwrap!(
            phy_clock_ref_counter.get().checked_sub(1),
            "PHY clock ref count underflowed. Either you forgot a PhyClockGuard, or used ModemClockController::decrease_phy_clock_ref_count incorrectly."
        );

        if new_phy_clock_ref_count == 0 {
            clocks_ll::enable_phy(false);
        }

        phy_clock_ref_counter.set(new_phy_clock_ref_count);
    })
}

#[inline]
#[instability::unstable]
/// Do any common initial initialization needed for the radio clocks
pub fn init_radio_clocks() {
    clocks_ll::init_clocks();
}

#[instability::unstable]
#[derive(Debug)]
/// Prevents the PHY clock from being disabled.
///
/// As long as at least one [PhyClockGuard] exists, the PHY clock will remain
/// active. To release this guard, you can either let it go out of scope or use
/// [PhyClockGuard::release] to explicitly release it.
pub struct PhyClockGuard<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl PhyClockGuard<'_> {
    #[instability::unstable]
    #[inline]
    /// Release the clock guard.
    ///
    /// The PHY clock will be disabled, if this is the last clock guard.
    pub fn release(self) {}
}

impl Drop for PhyClockGuard<'_> {
    fn drop(&mut self) {
        decrease_phy_clock_ref_count_internal();
    }
}

#[instability::unstable]
/// This trait provides common clock functionality for all modem peripherals.
pub trait ModemClockController<'d>: Sealed + 'd {
    /// Enable the modem clock for this controller.
    fn enable_modem_clock(&mut self, enable: bool);

    /// Enable the PHY clock and acquire a [PhyClockGuard].
    ///
    /// The PHY clock will only be disabled, once all [PhyClockGuard]'s of all
    /// modems were dropped.
    fn enable_phy_clock(&self) -> PhyClockGuard<'d> {
        increase_phy_clock_ref_count_internal();
        PhyClockGuard {
            _phantom: PhantomData,
        }
    }

    /// Decreases the PHY clock reference count for this modem ignoring
    /// currently alive [PhyClockGuard]s.
    ///
    /// # Panics
    /// This function panics if the PHY clock is inactive. If the ref count is
    /// lower than the number of alive [PhyClockGuard]s, dropping a guard can
    /// now panic.
    fn decrease_phy_clock_ref_count(&self) {
        decrease_phy_clock_ref_count_internal();
    }
}

#[cfg(soc_has_wifi)]
#[instability::unstable]
impl<'d> ModemClockController<'d> for WIFI<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_wifi(enable);
    }
}

#[cfg(soc_has_bt)]
#[instability::unstable]
impl<'d> ModemClockController<'d> for BT<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_bt(enable);
    }
}

#[cfg(all(feature = "unstable", soc_has_ieee802154))]
#[instability::unstable]
impl<'d> ModemClockController<'d> for IEEE802154<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_ieee802154(enable);
    }
}