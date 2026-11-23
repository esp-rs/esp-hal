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

    let token = unsafe { crate::ESP_PHY_LOCK.acquire() };
    // The ESP32 PHY library does not call `phy_i2c_enter_critical`. It wraps its register accesses
    // to the analog I2C master in this critical section instead, so the section also locks the
    // master.
    #[cfg(esp32)]
    unsafe {
        regi2c_enter_critical()
    };
    token.inner()
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

    #[cfg(esp32)]
    unsafe {
        regi2c_exit_critical()
    };
    unsafe {
        let token = esp_sync::RestoreState::new(level);
        crate::ESP_PHY_LOCK.release(token);
    }
}

unsafe extern "C" {
    /// Locks the analog I2C master, defined by esp-hal. Calls nest.
    fn regi2c_enter_critical();
    /// Unlocks the analog I2C master locked by `regi2c_enter_critical`.
    fn regi2c_exit_critical();
}

/// Runs `f` with the analog I2C master locked, which also masks interrupts.
///
/// On the ESP32, this enters the PHY critical section, which takes [crate::ESP_PHY_LOCK] before the
/// analog I2C lock. Taking the analog I2C lock alone would invert that order once the PHY library
/// enters the critical section within `f`.
#[cfg(any(esp32, esp32s2))]
pub(crate) fn with_regi2c_lock<R>(f: impl FnOnce() -> R) -> R {
    cfg_select! {
        esp32 => {
            let level = unsafe { __esp_phy_enter_critical() };
            let result = f();
            unsafe { __esp_phy_exit_critical(level) };
            result
        }
        _ => {
            unsafe { regi2c_enter_critical() };
            let result = f();
            unsafe { regi2c_exit_critical() };
            result
        }
    }
}

/// Locks the analog I2C master, which esp-hal also uses.
///
/// The PHY library's weak default does not lock anything, so this definition replaces it.
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e3/components/esp_phy/src/phy_override.c#L50-L59>.
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_i2c_enter_critical() {
    unsafe { regi2c_enter_critical() };
}

/// Unlocks the analog I2C master locked by `phy_i2c_enter_critical`.
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_i2c_exit_critical() {
    unsafe { regi2c_exit_critical() };
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
