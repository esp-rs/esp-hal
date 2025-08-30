#![no_std]
use core::cell::RefCell;

use esp_hal::clock::{ModemClockController, PhyClockGuard};
#[cfg(esp32)]
use esp_hal::time::{Duration, Instant};
use esp_wifi_sys::include::*;

mod fmt;

mod ffi;
mod phy_init_data;
mod common_adapter;

pub(crate) mod private {
    pub trait Sealed {}
}

/// Length of the PHY calibration data.
pub const PHY_CALIBRATION_DATA_LENGTH: usize =
    core::mem::size_of::<esp_wifi_sys::include::esp_phy_calibration_data_t>();
/// Type alias for opaque calibration data.
pub type PhyCalibrationData = [u8; PHY_CALIBRATION_DATA_LENGTH];

#[cfg(phy_backed_up_digital_register_count_is_set)]
type PhyDigRegsBackup = [u32; esp_metadata_generated::property!("phy.backed_up_digital_register_count")];

#[cfg(esp32)]
/// Callback to update the MAC time.
///
/// The duration is the delta, that has been accumulated between the PHY clock and the normal
/// system timers, since the last time this callback was called. This accounts for the PHY being
/// enabled and disabled, before this callback was set.
pub type MacTimeUpdateCb = fn(Duration);

/// PHY initialization state
struct PhyState {
    /// Number of references to the PHY.
    ref_count: usize,
    /// The calibration data used for initialization.
    ///
    /// If this is [None], when `PhyController::enable_phy` is called, it will be initialized to
    /// zero and a full calibration is performed.
    calibration_data: Option<PhyCalibrationData>,
    /// Has the PHY been calibrated since the chip was powered up.
    calibrated: bool,

    #[cfg(phy_backed_up_digital_register_count_is_set)]
    /// Backup of the digital PHY registers.
    phy_digital_register_backup: Option<PhyDigRegsBackup>,

    // Chip specific.
    #[cfg(esp32)]
    /// Timestamp at which the modem clock domain state transitioned.
    phy_clock_state_transition_timestamp: Instant,
    #[cfg(esp32)]
    /// The accumulated delta since the last time the callback was called.
    mac_clock_delta_since_last_call: Duration,
    #[cfg(esp32)]
    /// Callback to update the MAC time.
    mac_time_update_cb: Option<MacTimeUpdateCb>,
}
impl PhyState {
    pub const fn new() -> Self {
        Self {
            ref_count: 0,
            calibration_data: None,
            calibrated: true,

            #[cfg(phy_backed_up_digital_register_count_is_set)]
            phy_digital_register_backup: None,

            #[cfg(esp32)]
            phy_clock_state_transition_timestamp: Instant::EPOCH,
            #[cfg(esp32)]
            mac_clock_delta_since_last_call: Duration::ZERO,
            #[cfg(esp32)]
            mac_time_update_cb: None,
        }
    }
    /// Get a reference to the calibration data.
    ///
    /// If no calibration data is available, it will be initialized to zero.
    pub fn calibration_data(&mut self) -> &mut PhyCalibrationData {
        self.calibration_data
            .get_or_insert([0u8; PHY_CALIBRATION_DATA_LENGTH])
    }
    fn calibrate(&mut self) {
        #[cfg(esp32s2)]
        unsafe {
            use esp_hal::efuse::Efuse;
            phy_eco_version_sel(Efuse::major_chip_version());
        }
        #[cfg(phy_combo_module)]
        unsafe {
            phy_init_param_set(1);
        }

        #[cfg(all(phy_enable_usb, any(soc_has_usb0, soc_has_usb_device), not(esp32s2)))]
        unsafe {
            unsafe extern "C" {
                fn phy_bbpll_en_usb(param: bool);
            }
            phy_bbpll_en_usb(true);
        }

        let calibration_data_available = self.calibration_data.is_some();
        let calibration_mode = if calibration_data_available {
            // If the SOC just woke up from deep sleep and
            // `phy_skip_calibration_after_deep_sleep` is enabled.
            cfg_if::cfg_if! {
                if #[cfg(phy_skip_calibration_after_deep_sleep)] {
                    use esp_hal::{rtc_cntl::SocResetReason, system::reset_reason};
                    if reset_reason() == Some(SocResetReason::CoreDeepSleep) {
                        esp_phy_calibration_mode_t_PHY_RF_CAL_NONE
                    } else {
                        esp_phy_calibration_mode_t_PHY_RF_CAL_PARTIAL
                    }
                } else {
                    esp_phy_calibration_mode_t_PHY_RF_CAL_PARTIAL
                }
            }
        } else {
            esp_phy_calibration_mode_t_PHY_RF_CAL_FULL
        };
        let init_data = &phy_init_data::PHY_INIT_DATA_DEFAULT;
        unsafe {
            register_chipv7_phy(
                init_data,
                self.calibration_data() as *mut PhyCalibrationData as *mut _,
                calibration_mode,
            );
        }
        self.calibrated = true;
    }
    #[cfg(phy_backed_up_digital_register_count_is_set)]
    /// Backup the digital PHY register into memory.
    fn backup_digital_regs(&mut self) {
        unsafe {
            phy_dig_reg_backup(
                true,
                self.phy_digital_register_backup.get_or_insert_default() as *mut u32,
            );
        }
    }
    #[cfg(phy_backed_up_digital_register_count_is_set)]
    /// Restore the digital PHY registers from memory.
    ///
    /// This panics if the registers weren't previously backed up.
    fn restore_digital_regs(&mut self) {
        unsafe {
            phy_dig_reg_backup(
                false,
                self.phy_digital_register_backup
                    .as_mut()
                    .expect("Can't restore digital PHY registers from backup, without a backup.")
                    as *mut u32,
            );
            self.phy_digital_register_backup = None;
        }
    }
    /// Increase the number of references to the PHY.
    ///
    /// If the ref count was zero, the PHY will be initialized.
    pub fn increase_ref_count(&mut self) {
        self.ref_count += 1;
        if self.ref_count == 0 {
            #[cfg(esp32)]
            {
                let now = Instant::now();
                let delta = now - self.phy_clock_state_transition_timestamp;
                self.phy_clock_state_transition_timestamp = now;
                self.mac_clock_delta_since_last_call += delta;
            }
            if self.calibrated {
                unsafe {
                    phy_wakeup_init();
                }
                #[cfg(phy_backed_up_digital_register_count_is_set)]
                self.restore_digital_regs();
            } else {
                self.calibrate();
                self.calibrated = true;
            }
        }
        #[cfg(esp32)]
        if let Some(cb) = self.mac_time_update_cb {
            (cb)(self.mac_clock_delta_since_last_call);
            self.mac_clock_delta_since_last_call = Duration::ZERO;
        }
    }
    pub fn decrease_ref_count(&mut self) {
        self.ref_count = self
            .ref_count
            .checked_sub(1)
            .expect("PHY init ref count dropped below zero.");
        if self.ref_count == 0 {
            #[cfg(phy_backed_up_digital_register_count_is_set)]
            self.backup_digital_regs();
            unsafe {
                // Disable PHY and RF.
                phy_close_rf();

                // Power down PHY temperature sensor.
                #[cfg(not(esp32))]
                phy_xpd_tsens();
            }
            #[cfg(esp32)]
            {
                self.phy_clock_state_transition_timestamp = Instant::now();
            }
            // The PHY clock guard will get released in the drop code of the PHYInitGuard. Note
            // that this accepts a slight skewing of the delta, since the clock will be disabled
            // after we record the value. This shouldn't be too bad though.
        }
    }
}

/// Global PHY initialization state
static PHY_STATE: critical_section::Mutex<RefCell<PhyState>> =
    critical_section::Mutex::new(RefCell::new(PhyState::new()));
pub struct PhyInitGuard<'d> {
    _phy_clock_guard: PhyClockGuard<'d>,
}
impl Drop for PhyInitGuard<'_> {
    fn drop(&mut self) {
        critical_section::with(|cs| PHY_STATE.borrow_ref_mut(cs).decrease_ref_count());
    }
}

pub trait PhyController<'d>: private::Sealed + ModemClockController<'d> {
    fn enable_phy(&self) -> PhyInitGuard<'d> {
        // In esp-idf, this is done after calculating the MAC time delta, but it shouldn't make
        // much of a difference.
        let _phy_clock_guard = self.enable_phy_clock();

        critical_section::with(|cs| PHY_STATE.borrow_ref_mut(cs).increase_ref_count());

        PhyInitGuard { _phy_clock_guard }
    }
    /// Decreases the PHY init reference count for this modem ignoring
    /// currently alive [PhyInitGuard]s.
    ///
    /// This will also decrease the PHY clock ref count.
    /// # Panics
    /// This function panics if the PHY is inactive. If the ref count is
    /// lower than the number of alive [PhyInitGuard]s, dropping a guard can
    /// now panic.
    fn decrease_phy_init_ref_count(&self) {
        critical_section::with(|cs| PHY_STATE.borrow_ref_mut(cs).decrease_ref_count());
        self.decrease_phy_clock_ref_count();
    }
}
macro_rules! impl_phy_controller {
    ($feature_gate:ident, $peripheral:tt) => {
        #[cfg($feature_gate)]
        impl private::Sealed for esp_hal::peripherals::$peripheral<'_> {}
        #[cfg($feature_gate)]
        impl<'d> PhyController<'d> for esp_hal::peripherals::$peripheral<'d> {}
    }
}
impl_phy_controller!(wifi, WIFI);
impl_phy_controller!(bt, BT);
impl_phy_controller!(ieee802154, IEEE802154);

#[cfg(esp32)]
/// Trait providing MAC time functionality for the Wi-Fi peripheral.
pub trait MacTimeExt {
    /// Set the MAC time update callback.
    ///
    /// See [MacTimeUpdateCb] for details.
    fn set_mac_time_update_cb(&self, mac_time_update_cb: MacTimeUpdateCb) {
        critical_section::with(|cs| {
            PHY_STATE.borrow_ref_mut(cs).mac_time_update_cb = Some(mac_time_update_cb);
        })
    }
}
#[cfg(esp32)]
impl MacTimeExt for esp_hal::peripherals::WIFI<'_> {}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Calibration data was already set.
pub struct CalibrationDataAlreadySetError;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// No calibration data is available.
pub struct NoCalibrationDataError;

/// Load previously backed up PHY calibration data.
///
/// If `no_calibration` is `false`, a partial calibration will be performed. Otherwise no extra
/// calibration will be performed. See the ESP-IDF documentation on [RF Calibration](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/RF_calibration.html)
/// for further details.
pub fn set_phy_calibration_data(
    calibration_data: &PhyCalibrationData,
) -> Result<(), CalibrationDataAlreadySetError> {
    critical_section::with(|cs| {
        let mut phy_state = PHY_STATE.borrow_ref_mut(cs);
        if phy_state.calibration_data.is_some() {
            Err(CalibrationDataAlreadySetError)
        } else {
            phy_state.calibration_data = Some(*calibration_data);
            Ok(())
        }
    })
}
/// Backup the PHY calibration data to the provided slice.
pub fn backup_phy_calibration_data(
    buffer: &mut PhyCalibrationData,
) -> Result<(), NoCalibrationDataError> {
    critical_section::with(|cs| {
        PHY_STATE
            .borrow_ref_mut(cs)
            .calibration_data
            .as_mut()
            .ok_or(NoCalibrationDataError)
            .map(|calibration_data| buffer.copy_from_slice(calibration_data.as_slice()))
    })
}
