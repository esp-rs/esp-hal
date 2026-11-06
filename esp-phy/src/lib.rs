//! PHY initialization handling for chips with a radio.
//!
//! This should be considered an implementation detail of `esp-radio` and similar 3rd party crates.
//!
//! # Usage
//! ## Enabling and Disabling the PHY
//! Use [enable_phy] to enable the PHY. Drop the returned [PhyInitGuard] to disable the PHY.
//! Enabling / disabling the PHY is ref-counted so these actions need to be balanced.
//!
//! ## Backing Up and Restoring PHY Calibration Data
//! If the PHY has already been calibrated, you can use [backup_phy_calibration_data] to persist
//! calibration data elsewhere (e.g. in flash). Using [set_phy_calibration_data] you can restore
//! previously persisted calibration data.
//! ## Config Options
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_phy_config_table.md"))]
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![no_std]
#![deny(missing_docs)]

// MUST be the first module
mod fmt;
pub(crate) mod reg_access;

use core::{
    cell::{Cell, UnsafeCell},
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicPtr, Ordering},
};

#[cfg(esp32)]
use esp_hal::time::{Duration, Instant};
use esp_hal::{interrupt::RunLevel, system::Cpu};
use esp_sync::{NonReentrantMutex, RawMutex, RestoreState};

/// Tracks the number of references to the PHY clock.
static PHY_CLOCK_REF_COUNTER: NonReentrantMutex<u8> = NonReentrantMutex::new(0);

fn increase_phy_clock_ref_count_internal() {
    PHY_CLOCK_REF_COUNTER.with(|refcount| {
        let count = *refcount;

        if count == 0 {
            phy_clocks::enable_phy(true);
        }

        *refcount = unwrap!(count.checked_add(1), "PHY clock ref count overflowed.");
    })
}

fn decrease_phy_clock_ref_count_internal() {
    PHY_CLOCK_REF_COUNTER.with(|refcount| {
        let count = unwrap!(
            refcount.checked_sub(1),
            "PHY clock ref count underflowed. Either you forgot a PhyClockGuard, or used PhyController::decrease_phy_clock_ref_count incorrectly."
        );

        if count == 0 {
            phy_clocks::enable_phy(false);
        }

        *refcount = count;
    })
}

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
    #[inline]
    /// Release the clock guard.
    ///
    /// The PHY clock will be disabled, if this is the last clock guard.
    pub fn release(self) {
        // Runs the Drop implementation
    }
}

impl Drop for PhyClockGuard<'_> {
    fn drop(&mut self) {
        decrease_phy_clock_ref_count_internal();
    }
}
pub(crate) mod sys {
    #[cfg(esp32)]
    pub use esp_wifi_sys_esp32::*;
    #[cfg(esp32c2)]
    pub use esp_wifi_sys_esp32c2::*;
    #[cfg(esp32c3)]
    pub use esp_wifi_sys_esp32c3::*;
    #[cfg(esp32c5)]
    pub use esp_wifi_sys_esp32c5::*;
    #[cfg(esp32c6)]
    pub use esp_wifi_sys_esp32c6::*;
    #[cfg(esp32c61)]
    pub use esp_wifi_sys_esp32c61::*;
    #[cfg(esp32h2)]
    pub use esp_wifi_sys_esp32h2::*;
    #[cfg(esp32h4)]
    pub use esp_wifi_sys_esp32h4::*;
    #[cfg(esp32s2)]
    pub use esp_wifi_sys_esp32s2::*;
    #[cfg(esp32s3)]
    pub use esp_wifi_sys_esp32s3::*;
    #[cfg(esp32s31)]
    pub use esp_wifi_sys_esp32s31::*;
}

mod common_adapter;
mod phy_clocks;
mod phy_init_data;

/// Length of the PHY calibration data.
pub const PHY_CALIBRATION_DATA_LENGTH: usize =
    core::mem::size_of::<sys::include::esp_phy_calibration_data_t>();

/// Type alias for opaque calibration data.
pub type PhyCalibrationData = [u8; PHY_CALIBRATION_DATA_LENGTH];

#[cfg(phy_backed_up_digital_register_count_is_set)]
type PhyDigRegsBackup =
    [u32; esp_metadata_generated::property!("phy.backed_up_digital_register_count")];

#[cfg(esp32)]
/// Callback to update the MAC time.
///
/// The duration is the delta, that has been accumulated between the PHY clock and the normal
/// system timers, since the last time this callback was called. This accounts for the PHY being
/// enabled and disabled, before this callback was set.
pub type MacTimeUpdateCb = fn(Duration);

static ESP_PHY_LOCK: RawMutex = RawMutex::new();

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
    /// Last calibration result code.
    calibration_result: i32,

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
    /// Initialize the PHY state.
    pub const fn new() -> Self {
        Self {
            ref_count: 0,
            calibration_data: None,
            calibrated: false,
            calibration_result: 0,

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

    /// Calibrate the PHY.
    fn calibrate(&mut self) {
        #[cfg(esp32s2)]
        unsafe {
            sys::include::phy_eco_version_sel(esp_hal::efuse::chip_revision().major);
        }
        // For a combo module, PHY enable will not put the
        // radio into the Wi-Fi RX state by default; the Wi-Fi driver is then responsible for
        // turning Wi-Fi RX on/off via `set_wifi_rx_enabled` when it enables/disables the PHY.
        // This mirrors `esp_phy_load_cal_and_init` in ESP-IDF.
        cfg_select! {
            phy_combo_module => unsafe {
                sys::include::phy_init_param_set(1);
            },
            _ => {}
        }

        #[cfg(phy_enable_usb)]
        unsafe {
            // FIXME: we should be using from esp-wifi-sys, but the function is missing for C6
            // (CONFIG_ESP_PHY_ENABLE_USB is not defined)
            unsafe extern "C" {
                fn phy_bbpll_en_usb(param: bool);
            }
            phy_bbpll_en_usb(true);
        }

        let calibration_data_available = self.calibration_data.is_some();
        let calibration_mode = if calibration_data_available {
            // If the SOC just woke up from deep sleep and
            // `phy_skip_calibration_after_deep_sleep` is enabled, no calibration will be
            // performed.
            if cfg!(phy_skip_calibration_after_deep_sleep) && is_reset_from_deepsleep() {
                sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_NONE
            } else if cfg!(phy_full_calibration) {
                sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_FULL
            } else {
                sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_PARTIAL
            }
        } else {
            sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_FULL
        };
        let init_data = &phy_init_data::PHY_INIT_DATA_DEFAULT;
        unsafe {
            self.calibration_result = sys::include::register_chipv7_phy(
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
            sys::include::phy_dig_reg_backup(
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
            sys::include::phy_dig_reg_backup(
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
    /// If the ref count was zero, the PHY will be initialized. The first initialization calibrates
    /// the PHY with `lock` unlocked.
    pub fn increase_ref_count(&mut self, lock: &mut PhyStateLock) {
        if self.ref_count == 0 {
            #[cfg(esp32)]
            {
                let now = Instant::now();
                let delta = now - self.phy_clock_state_transition_timestamp;
                self.phy_clock_state_transition_timestamp = now;
                self.mac_clock_delta_since_last_call += delta;
            }
            if !self.calibrated {
                lock.unlock_for_calibration();
            }
            phy_clocks::with_calibration_clocks(|| {
                if self.calibrated {
                    unsafe {
                        sys::include::phy_wakeup_init();
                    }
                    #[cfg(phy_backed_up_digital_register_count_is_set)]
                    self.restore_digital_regs();
                } else {
                    self.calibrate();
                    self.calibrated = true;
                }
            });
        }
        #[cfg(esp32)]
        if let Some(cb) = self.mac_time_update_cb {
            (cb)(self.mac_clock_delta_since_last_call);
            self.mac_clock_delta_since_last_call = Duration::ZERO;
        }

        self.ref_count += 1;
    }

    /// Decrease the number of reference to the PHY.
    ///
    /// If the ref count hits zero, the PHY will be deinitialized.
    ///
    /// # Panics
    /// This panics, if the PHY ref count is already at zero.
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
                sys::include::phy_close_rf();

                // Power down PHY temperature sensor.
                #[cfg(not(esp32))]
                sys::include::phy_xpd_tsens();
            }
            #[cfg(esp32)]
            {
                self.phy_clock_state_transition_timestamp = Instant::now();
            }
            // The PHY clock guard will get released in the drop code of the PhyInitGuard. Note
            // that this accepts a slight skewing of the delta, since the clock will be disabled
            // after we record the value. This shouldn't be too bad though.
        }
    }
}

fn is_reset_from_deepsleep() -> bool {
    // feature gated to avoid forgetting to double check the correct value for future chips
    #[cfg(any(
        esp32, esp32c2, esp32c3, esp32c5, esp32c6, esp32c61, esp32h2, esp32h4, esp32s2, esp32s3,
        esp32s31
    ))]
    const CORE_DEEP_SLEEP: u32 = 5;

    unsafe extern "C" {
        fn rtc_get_reset_reason(cpu_num: u32) -> u32;
    }

    let reason = unsafe { rtc_get_reset_reason(Cpu::current() as u32) };

    reason == CORE_DEEP_SLEEP
}

/// Scheduler mutex that threads wait on while another thread uses the PHY.
#[doc(hidden)]
// Public for esp-radio, which owns the scheduler that implements the mutex.
#[derive(Debug)]
pub struct PhyMutex {
    /// Blocks the calling thread until it owns the mutex. The owner may lock it again.
    pub lock: fn(),
    /// Releases [PhyMutex::lock].
    pub unlock: fn(),
}

static PHY_MUTEX: AtomicPtr<PhyMutex> = AtomicPtr::new(core::ptr::null_mut());

/// Set while the PHY calibrates with interrupts enabled and [PHY_STATE] unlocked.
static CALIBRATING: AtomicBool = AtomicBool::new(false);

/// Install the mutex that threads wait on for the PHY.
///
/// Until then, nothing waits. Another context that uses the PHY while it calibrates panics instead.
#[doc(hidden)]
// Public for esp-radio, which installs the mutex once the scheduler runs.
pub fn set_phy_mutex(mutex: &'static PhyMutex) {
    PHY_MUTEX.store(core::ptr::from_ref(mutex).cast_mut(), Ordering::Release);
}

/// Whether this context can wait for [PHY_MUTEX]: a thread with interrupts enabled.
fn can_wait() -> bool {
    // Xtensa critical sections raise the run level, RISC-V ones clear `mstatus.MIE` (bit 3).
    RunLevel::current().is_thread()
        && cfg_select! {
            riscv => {{
                let mstatus: u32;
                unsafe { core::arch::asm!("csrr {0}, mstatus", out(reg) mstatus) };
                mstatus & (1 << 3) != 0
            }}
            _ => true,
        }
}

/// Holds [PHY_MUTEX] until dropped, if this context can wait for it.
struct PhyMutexGuard(Option<&'static PhyMutex>);

impl PhyMutexGuard {
    fn new() -> Self {
        // SAFETY: `set_phy_mutex` only stores `'static` references.
        let mutex = unsafe { PHY_MUTEX.load(Ordering::Acquire).as_ref() }.filter(|_| can_wait());
        if let Some(mutex) = mutex {
            (mutex.lock)();
        }
        Self(mutex)
    }
}

impl Drop for PhyMutexGuard {
    fn drop(&mut self) {
        if let Some(mutex) = self.0 {
            (mutex.unlock)();
        }
    }
}

fn assert_not_calibrating() {
    if CALIBRATING.load(Ordering::Relaxed) {
        panic!("The PHY was used while it calibrates, by a context that cannot wait for it");
    }
}

/// Run `f` with [PHY_MUTEX] and [ESP_PHY_LOCK] held, and interrupts masked.
///
/// # Panics
///
/// See [enable_phy].
#[doc(hidden)]
// Public for esp-radio. Calibration borrows modem clocks and restores them afterwards, so esp-radio
// changes modem clocks under this lock.
pub fn with_phy_lock<R>(f: impl FnOnce() -> R) -> R {
    let _mutex = PhyMutexGuard::new();
    ESP_PHY_LOCK.lock(|| {
        assert_not_calibrating();
        f()
    })
}

/// Global PHY initialization state, used by one context at a time.
///
/// `in_use` catches a context that locks it again.
struct PhyStateCell {
    lock: RawMutex,
    in_use: Cell<bool>,
    state: UnsafeCell<PhyState>,
}

// SAFETY: `PhyStateLock` hands out one reference to `state` at a time.
unsafe impl Sync for PhyStateCell {}

static PHY_STATE: PhyStateCell = PhyStateCell {
    lock: RawMutex::new(),
    in_use: Cell::new(false),
    state: UnsafeCell::new(PhyState::new()),
};

/// Keeps [PHY_STATE] in use until dropped, with interrupts masked except for the calibration.
struct PhyStateLock {
    /// [None] while the PHY calibrates.
    token: Option<RestoreState>,
}

impl PhyStateLock {
    fn new() -> Self {
        let token = unsafe { PHY_STATE.lock.acquire() };
        assert_not_calibrating();
        if PHY_STATE.in_use.replace(true) {
            panic!("The PHY state is already in use");
        }
        Self { token: Some(token) }
    }

    /// Unmasks interrupts for the calibration, until dropped.
    fn unlock_for_calibration(&mut self) {
        if let Some(token) = self.token.take() {
            CALIBRATING.store(true, Ordering::Relaxed);
            unsafe { PHY_STATE.lock.release(token) };
        }
    }
}

impl Drop for PhyStateLock {
    fn drop(&mut self) {
        let token = self.token.take().unwrap_or_else(|| {
            let token = unsafe { PHY_STATE.lock.acquire() };
            CALIBRATING.store(false, Ordering::Relaxed);
            token
        });
        PHY_STATE.in_use.set(false);
        unsafe { PHY_STATE.lock.release(token) };
    }
}

/// Run `f` with the PHY state locked.
fn with_phy_state<R>(f: impl FnOnce(&mut PhyState, &mut PhyStateLock) -> R) -> R {
    let _mutex = PhyMutexGuard::new();
    let mut state_lock = PhyStateLock::new();
    // SAFETY: `state_lock` set `in_use`, so this is the only reference to the state.
    f(unsafe { &mut *PHY_STATE.state.get() }, &mut state_lock)
}

/// Prevents the PHY from being deinitialized.
///
/// As long as at least one [PhyInitGuard] exists, the PHY will remain initialized. To release this
/// guard, you can either let it go out of scope, or use [PhyInitGuard::release] to explicitly
/// release it.
#[derive(Debug)]
pub struct PhyInitGuard<'d> {
    _phy_clock_guard: PhyClockGuard<'d>,
}

impl PhyInitGuard<'_> {
    #[inline]
    /// Release the init guard.
    ///
    /// The PHY will be disabled, if this is the last init guard.
    pub fn release(self) {
        // Runs the Drop implementation
    }
}

impl Drop for PhyInitGuard<'_> {
    fn drop(&mut self) {
        with_phy_state(|phy_state, _| phy_state.decrease_ref_count());
    }
}

/// Enable the PHY.
///
/// If no other [PhyInitGuard] is currently alive, this will also initialize the PHY, which
/// will involve a full RF calibration, unless you loaded previously backed up calibration
/// data with [set_phy_calibration_data].
///
/// The first initialization since power-up calibrates the PHY with interrupts enabled. Other
/// threads that use the PHY meanwhile wait for it, once `esp-radio` has initialized. Wake-up and
/// shutdown mask interrupts.
///
/// # Panics
///
/// Panics if called from an interrupt handler or with interrupts masked while the PHY calibrates.
pub fn enable_phy<'d>() -> PhyInitGuard<'d> {
    // In esp-idf, this is done after calculating the MAC time delta, but it shouldn't make
    // much of a difference.
    let _phy_clock_guard = enable_phy_clock();

    with_phy_state(PhyState::increase_ref_count);

    PhyInitGuard { _phy_clock_guard }
}

/// Manually disable the PHY.
///
/// This is only useful if you [core::mem::forget] the [PhyInitGuard].
///
/// # Panics
///
/// See [enable_phy].
pub fn disable_phy() {
    with_phy_state(|phy_state, _| phy_state.decrease_ref_count());
    // Balance the PhyClockGuard that was mem::forget'd with PhyInitGuard.
    // Without this, PHY_CLOCK_REF_COUNTER (u8) leaks on every phy_enable/phy_disable
    // cycle from the WiFi blob C-callback interface, overflowing after ~9 TCP connects.
    decrease_phy_clock_ref_count_internal();
}

/// Enable the PHY for Wi-Fi: enables the PHY ([enable_phy]) and turns Wi-Fi RX on.
///
/// Wi-Fi must use this instead of [enable_phy] so that combo modules (which do not power up in the
/// Wi-Fi RX state) actually receive.
pub fn enable_phy_with_wifi_rx() {
    core::mem::forget(enable_phy());
    set_wifi_rx_enabled(true);
}

/// Disable the PHY for Wi-Fi: turns Wi-Fi RX off and disables the PHY ([disable_phy]).
///
/// Counterpart to [enable_phy_with_wifi_rx]. Mirrors ESP-IDF's `esp_phy_disable_wrapper`.
pub fn disable_phy_with_wifi_rx() {
    set_wifi_rx_enabled(false);
    disable_phy();
}

/// Set the Wi-Fi RX state of the radio.
///
/// On combo modules the PHY does not power up in the Wi-Fi RX state, so Wi-Fi RX has to be enabled
/// explicitly whenever the PHY is brought up, and disabled again when it is torn down. This
/// mirrors the `phy_wifi_enable_set` calls ESP-IDF performs in its PHY enable/disable
/// wrappers.
///
/// On non-combo modules this is a no-op.
fn set_wifi_rx_enabled(enabled: bool) {
    cfg_select! {
        phy_combo_module => unsafe {
            sys::include::phy_wifi_enable_set(enabled as u8);
        },
        _ => {
            let _ = enabled;
        }
    }
}

/// Enable the PHY clock and acquire a [PhyClockGuard].
///
/// The PHY clock will only be disabled once all [PhyClockGuard]s are dropped.
pub fn enable_phy_clock<'d>() -> PhyClockGuard<'d> {
    increase_phy_clock_ref_count_internal();
    PhyClockGuard {
        _phantom: PhantomData,
    }
}

/// Set the MAC time update callback.
///
/// See [MacTimeUpdateCb] for details.
///
/// # Panics
///
/// See [enable_phy].
#[cfg(esp32)]
pub fn set_mac_time_update_cb(mac_time_update_cb: MacTimeUpdateCb) {
    with_phy_state(|phy_state, _| phy_state.mac_time_update_cb = Some(mac_time_update_cb));
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Calibration data was already set.
pub struct CalibrationDataAlreadySetError;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// No calibration data is available.
pub struct NoCalibrationDataError;

/// Result of the PHY calibration.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CalibrationResult {
    /// The calibration data was valid and was used for calibration.
    Ok,

    /// The calibration data checksum check failed, or the calibration data was outdated.
    DataCheckFailed,
}

/// Load previously backed up PHY calibration data.
///
/// # Panics
///
/// See [enable_phy].
pub fn set_phy_calibration_data(
    calibration_data: &PhyCalibrationData,
) -> Result<(), CalibrationDataAlreadySetError> {
    with_phy_state(|phy_state, _| {
        if phy_state.calibration_data.is_some() {
            Err(CalibrationDataAlreadySetError)
        } else {
            phy_state.calibration_data = Some(*calibration_data);
            Ok(())
        }
    })
}

/// Backup the PHY calibration data to the provided slice.
///
/// # Panics
///
/// See [enable_phy].
pub fn backup_phy_calibration_data(
    buffer: &mut PhyCalibrationData,
) -> Result<(), NoCalibrationDataError> {
    with_phy_state(|phy_state, _| {
        phy_state
            .calibration_data
            .as_mut()
            .ok_or(NoCalibrationDataError)
            .map(|calibration_data| buffer.copy_from_slice(calibration_data.as_slice()))
    })
}

/// Get the last calibration result.
///
/// This can be used to know if any previously persisted calibration data is outdated/invalid and
/// needs to get updated.
///
/// # Panics
///
/// See [enable_phy].
pub fn last_calibration_result() -> Option<CalibrationResult> {
    with_phy_state(|phy_state, _| {
        if phy_state.calibrated {
            Some(
                if phy_state.calibration_result == sys::include::ESP_CAL_DATA_CHECK_FAIL as i32 {
                    CalibrationResult::DataCheckFailed
                } else {
                    CalibrationResult::Ok
                },
            )
        } else {
            None
        }
    })
}
