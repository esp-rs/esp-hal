use core::sync::atomic::{AtomicU32, Ordering};

use esp_hal::ram;

/// **************************************************************************
/// Name: phy_enter_critical
///
/// Description:
///   Enter critical state
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   CPU PS value
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_phy_enter_critical() -> u32 {
    trace!("phy_enter_critical");

    unsafe { crate::ESP_PHY_LOCK.acquire().inner() }
}

/// **************************************************************************
/// Name: phy_exit_critical
///
/// Description:
///   Exit from critical state
///
/// Input Parameters:
///   level - CPU PS value
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_phy_exit_critical(level: u32) {
    trace!("phy_exit_critical {}", level);

    unsafe {
        let token = esp_sync::RestoreState::new(level);
        crate::ESP_PHY_LOCK.release(token);
    }
}

/// Nesting depth of `phy_i2c_enter_critical`, only accessed with the analog I2C lock held.
static I2C_CRITICAL_DEPTH: AtomicU32 = AtomicU32::new(0);
/// Token of the outermost `phy_i2c_enter_critical`, only accessed with the analog I2C lock held.
static I2C_CRITICAL_TOKEN: AtomicU32 = AtomicU32::new(0);

/// Locks the analog I2C master, which esp-hal also uses.
///
/// The PHY library's weak default does not lock anything, so this definition replaces it. The
/// library keeps no token between this call and `phy_i2c_exit_critical`, and it nests the calls.
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e3/components/esp_phy/src/phy_override.c#L50-L59>.
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_i2c_enter_critical() {
    let token = unsafe { esp_hal::__regi2c::LOCK.acquire() };
    let depth = I2C_CRITICAL_DEPTH.load(Ordering::Relaxed);
    if depth == 0 {
        I2C_CRITICAL_TOKEN.store(token.inner(), Ordering::Relaxed);
    }
    I2C_CRITICAL_DEPTH.store(depth + 1, Ordering::Relaxed);
}

/// Unlocks the analog I2C master locked by `phy_i2c_enter_critical`.
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_i2c_exit_critical() {
    let depth = unwrap!(
        I2C_CRITICAL_DEPTH.load(Ordering::Relaxed).checked_sub(1),
        "phy_i2c_exit_critical without phy_i2c_enter_critical"
    );
    I2C_CRITICAL_DEPTH.store(depth, Ordering::Relaxed);
    if depth == 0 {
        // Nested tokens are reentries, which release nothing, so only the outermost one is kept.
        let token = I2C_CRITICAL_TOKEN.load(Ordering::Relaxed);
        unsafe { esp_hal::__regi2c::LOCK.release(esp_sync::RestoreState::new(token)) };
    }
}

/// **************************************************************************
/// Name: esp_dport_access_reg_read
///
/// Description:
///   Read register value safely in SMP
///
/// Input Parameters:
///   reg - Register address
///
/// Returned Value:
///   Register value
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_phy_esp_dport_access_reg_read(reg: u32) -> u32 {
    unsafe {
        // trace!("esp_dport_access_reg_read {:x} => {:x}", reg, res);
        (reg as *mut u32).read_volatile()
    }
}

#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_phy_rtc_get_xtal() -> u32 {
    esp_hal::clock::xtal_clock().as_mhz()
}
