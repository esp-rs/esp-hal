#![no_std]
use core::{cell::RefCell, sync::atomic::Ordering};

use esp_hal::clock::{ModemClockController, PhyClockGuard};
#[cfg(esp32)]
use esp_hal::time::{Duration, Instant};
use portable_atomic::AtomicUsize;

mod phy_init_data;

pub(crate) mod private {
    pub trait Sealed {}
}

/// Length of the PHY calibration data.
pub const PHY_CALIBRATION_DATA_LENGTH: usize =
    core::mem::size_of::<esp_wifi_sys::include::esp_phy_calibration_data_t>();
/// Type alias for opaque calibration data.
pub type PhyCalibrationData = [u8; PHY_CALIBRATION_DATA_LENGTH];

enum PhyCalibrationMode {
    None    = 0,
    Partial = 1,
    Full    = 2,
}

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
    /// Calibration mode used for PHY initialization.
    calibration_mode: PhyCalibrationMode,
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
            calibration_mode: PhyCalibrationMode::Full,
            #[cfg(esp32)]
            phy_clock_state_transition_timestamp: Instant::EPOCH,
            #[cfg(esp32)]
            mac_clock_delta_since_last_call: Duration::ZERO,
            #[cfg(esp32)]
            mac_time_update_cb: None
        }
    }
    pub fn calibration_data(&mut self) -> &PhyCalibrationData {
        self.calibration_data
            .get_or_insert_with(|| [0u8; PHY_CALIBRATION_DATA_LENGTH])
    }
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
        }
        #[cfg(esp32)]
        if let Some(cb) = self.mac_time_update_cb {
            (cb)(self.mac_clock_delta_since_last_call);
            self.mac_clock_delta_since_last_call = Duration::ZERO;
        }
    }
    pub fn decrease_ref_count(&mut self) {
        self.ref_count = self.ref_count.checked_sub(1).expect("PHY init ref count dropped below zero.");
        if self.ref_count == 0 {
            #[cfg(esp32)]
            self.phy_clock_state_transition_timestamp = Instant::now();            
        }
    }
}

/// Global PHY initialization state
static PHY_STATE: critical_section::Mutex<RefCell<PhyState>> =
    critical_section::Mutex::new(RefCell::new(PhyState::new()));
pub struct PhyInitGuard<'d> {
    _phy_clock_guard: PhyClockGuard<'d>,
}

pub trait PhyController<'d>: private::Sealed + ModemClockController<'d> {
    fn enable_phy(&self) -> PhyInitGuard<'d> {
        let _phy_clock_guard = self.enable_phy_clock();

        critical_section::with(|cs| PHY_STATE.borrow_ref_mut(cs).increase_ref_count());

        PhyInitGuard {
            _phy_clock_guard
        }
    }
}
#[cfg(wifi)]
impl esp_hal::peripherals::WIFI {
    pub const fn set_mac_time_update_cb(&self, mac_time_update_cb: MacTimeUpdateCb) {
        critical_section::with(|cs| {
            
        })
    }
}

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
    no_calibration: bool,
) -> Result<(), CalibrationDataAlreadySetError> {
    critical_section::with(|cs| {
        let mut phy_state = PHY_STATE.borrow_ref_mut(cs);
        if phy_state.calibration_data.is_some() {
            Err(CalibrationDataAlreadySetError)
        } else {
            phy_state.calibration_mode = if no_calibration {
                PhyCalibrationMode::None
            } else {
                PhyCalibrationMode::Partial
            };
            phy_state.calibration_data.insert(*calibration_data);
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
            .inspect(|calibration_data| buffer.copy_from_slice(calibration_data.as_slice()))
    })
}

