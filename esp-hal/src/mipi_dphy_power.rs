//! The `VDD_MIPI_DPHY` rail, shared by the MIPI-CSI and MIPI-DSI D-PHYs.
//!
//! On ESP32-P4 the rail is driven by PMU internal LDO channel 3 (VO3), the
//! `ext_ldo_p0_0p2a` register pair. Both PHYs need it, so it stays on while
//! either driver lives, and is switched off when the last one is dropped.
//! Clocks and resets of each PHY are not shared and stay with their driver.

use esp_sync::NonReentrantMutex;

use crate::{peripherals::PMU, rom};

/// Live [`DphyPower`] guards.
static USERS: NonReentrantMutex<u32> = NonReentrantMutex::new(0);

/// Keeps `VDD_MIPI_DPHY` powered while alive.
pub(crate) struct DphyPower(());

impl DphyPower {
    /// Powers the rail if no other guard does.
    ///
    /// The first guard waits for the LDO output to settle with the lock held,
    /// so a driver on the other core never sees the rail before it is stable.
    pub(crate) fn acquire() -> Self {
        USERS.with(|users| {
            if *users == 0 {
                ldo_on();
            }
            *users += 1;
        });
        Self(())
    }
}

impl Drop for DphyPower {
    fn drop(&mut self) {
        USERS.with(|users| {
            *users -= 1;
            if *users == 0 {
                ldo_off();
            }
        });
    }
}

/// Without eFuse calibration the nominal output formula gives:
///   Vref = 1.0 V  (dref = 9)
///   Vout = Vref × (1 + 0.25 × mul) = 2.5 V  (mul = 6)
fn ldo_on() {
    // force_tieh_sel[7]=1 → SW-owned, xpd[8]=1 → enabled.
    // target0/target1 kept at reset values (0x80 / 0x40).
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0180) });
    // dref[31:28]=9, mul[25:23]=6 → 2.5 V.
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits((9u32 << 28) | (6u32 << 23)) });
    // Allow output to settle before the PHY analog circuits start.
    rom::ets_delay_us(500);
}

fn ldo_off() {
    // Reset state: xpd=0, SW ownership released.
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
}
