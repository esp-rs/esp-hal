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

/// **************************************************************************
/// Name: esp_dport_access_reg_read
///
/// Description:
///   Read regitser value safely in SMP
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
    esp_hal::clock::Clocks::get().xtal_clock.as_mhz()
}
