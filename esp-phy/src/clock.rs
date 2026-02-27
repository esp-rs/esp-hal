// esp-phy/src/clock.rs
use core::{cell::Cell, marker::PhantomData};
use esp_sync::RawMutex;

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
    PHY_CLOCK_REF_COUNTER.lock(|c| {
        let n = c.get();
        if n == 0 {
            clocks_ll::enable_phy(true);
        }
        let new_n = crate::unwrap!(n.checked_add(1), "PHY clock ref count overflowed.");
        c.set(new_n);
    })
}

fn decrease_phy_clock_ref_count_internal() {
    PHY_CLOCK_REF_COUNTER.lock(|c| {
        let new_n = crate::unwrap!(
            c.get().checked_sub(1),
            "PHY clock ref count underflowed."
        );
        if new_n == 0 {
            clocks_ll::enable_phy(false);
        }
        c.set(new_n);
    })
}

#[derive(Debug)]
pub struct PhyClockGuard<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl PhyClockGuard<'_> {
    #[inline]
    pub fn release(self) {}
}

impl Drop for PhyClockGuard<'_> {
    fn drop(&mut self) {
        decrease_phy_clock_ref_count_internal();
    }
}
