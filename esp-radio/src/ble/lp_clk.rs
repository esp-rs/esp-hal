//! BLE low-power clock, read from the clock tree.
//!
//! The tree reports the divided output frequency. An RC source has no calibrated rate, so selecting
//! one panics here.

use esp_hal::clock::ll::{self, BleLpClkConfig};
#[cfg(bt_controller = "btdm")]
use {
    enumset::EnumSet,
    esp_hal::{
        clock::ll::ClockSource,
        rtc_cntl::{WakeupSource, sleep::WrappedSleepConfig},
    },
    portable_atomic::{AtomicU32, Ordering},
};

/// BTDM blob parameters for the selected low-power clock.
#[cfg(bt_controller = "btdm")]
pub(crate) struct BtdmLpClk {
    /// Value for `btdm_lpclk_select_src`. Crystal is 0. RTC slow is 2.
    pub select: u32,
    /// Value for `btdm_lpclk_set_div`.
    pub divider: u32,
    /// Microseconds per low-power cycle, in Q19 format.
    pub lpcycle_us: u32,
}

fn selected() -> BleLpClkConfig {
    ll::ClockTree::with(|clocks| clocks.ble_lp_clk()).expect("BLE_LP_CLK is not configured")
}

/// Turns the BLE low-power clock on for as long as the controller runs.
///
/// The controller wakes from sleep on this clock, so it must keep running even while the
/// controller is asleep.
pub(crate) fn request() {
    ll::ClockTree::with(ll::request_ble_lp_clk);
}

/// Releases the request taken by [`request`].
pub(crate) fn release() {
    ll::ClockTree::with(ll::release_ble_lp_clk);
}

/// Returns whether the chip can light-sleep while the controller sleeps.
///
/// Only the BTDM controllers support light sleep. ESP32 needs a 32 kHz crystal for it, because it
/// cannot light-sleep with the main crystal as the controller clock.
pub(crate) fn light_sleep_supported() -> bool {
    cfg_select! {
        esp32 => ll::ClockTree::with(ll::ble_lp_clk_root_source) != Some(ClockSource::XtalClk),
        bt_controller = "btdm" => true,
        _ => false,
    }
}

/// The source that `BLE_LP_CLK` runs on, as the bits of an [`EnumSet`].
#[cfg(bt_controller = "btdm")]
static WAKE_CLOCK: AtomicU32 = AtomicU32::new(0);

/// Claims the `Bt` wakeup source, so that the chip can sleep while the controller sleeps.
///
/// Call this only with modem sleep on, after the controller is enabled, and only when
/// [`light_sleep_supported`] returns `true`. Otherwise the radio wake lock stays.
#[cfg(bt_controller = "btdm")]
pub(crate) fn claim_wake_source() {
    let root = ll::ClockTree::with(ll::ble_lp_clk_root_source);
    WAKE_CLOCK.store(
        root.map_or(0, |source| EnumSet::only(source).as_u32()),
        Ordering::Relaxed,
    );
    WakeupSource::Bt.enable_with_hooks(Some(sleep_entry), None);
}

/// Releases the claim taken by [`claim_wake_source`].
///
/// Call this before the controller is disabled.
#[cfg(bt_controller = "btdm")]
pub(crate) fn release_wake_source() {
    WakeupSource::Bt.disable();
}

#[cfg(bt_controller = "btdm")]
fn sleep_entry(config: &mut WrappedSleepConfig<'_>) {
    // The controller does not survive a deep sleep, so it needs no clock through one.
    if config.is_deep_sleep() {
        return;
    }

    // The controller holds the PHY while it is awake, and it then runs on clocks that light sleep
    // stops.
    let awake = !super::MODEM_PHY_OFF.load(Ordering::Relaxed);
    if awake || super::porting::hci_packet_in_flight() {
        config.reject_sleep();
        return;
    }

    // The controller wakes on its own timer, but the chip must be awake before it does.
    let remaining = super::porting::time_until_controller_wakes();
    if remaining.as_micros() == 0 {
        config.reject_sleep();
        return;
    }
    config.limit_sleep(remaining);

    for source in EnumSet::<ClockSource>::from_u32_truncated(WAKE_CLOCK.load(Ordering::Relaxed)) {
        config.keep_clock_running(source);
    }
}

fn reject_rc(config: BleLpClkConfig) {
    let rc = cfg_select! {
        bt_controller = "btdm" => match config {
            BleLpClkConfig::Xtal => false,
            BleLpClkConfig::RtcSlow => matches!(
                ll::ClockTree::with(|clocks| clocks.rtc_slow_clk()),
                Some(ll::RtcSlowClkConfig::RcSlow) | Some(ll::RtcSlowClkConfig::RcFast)
            ),
        },
        esp32c2 => config.sclk() == ll::BleLpClkSclk::RcSlow,
        _ => config == BleLpClkConfig::RcSlow,
    };
    if rc {
        panic!("RC clocks cannot be selected as the BLE low-power clock");
    }
}

/// Returns the BLE low-power clock frequency, in hertz.
///
/// # Panics
///
/// Panics when `BLE_LP_CLK` is not configured, or when its source is an RC oscillator.
#[cfg(not(bt_controller = "btdm"))]
pub(crate) fn frequency_hz() -> u32 {
    reject_rc(selected());
    ll::ble_lp_clk_frequency()
}

/// Returns the BTDM blob clock-select parameters for the configured source.
#[cfg(bt_controller = "btdm")]
pub(crate) fn btdm() -> BtdmLpClk {
    let config = selected();
    reject_rc(config);

    let hz = ll::ble_lp_clk_frequency().max(1);
    let lpcycle_us = ((1_000_000u64 << 19) / u64::from(hz)) as u32;

    let (select, divider) = match config {
        BleLpClkConfig::Xtal => {
            let ratio = ll::xtal_clk_frequency() / ll::ble_lp_xtal_clk_frequency();
            // The ESP32 blob takes the divider field, which is one less than the ratio.
            let divider = cfg_select! {
                esp32 => ratio - 1,
                _ => ratio,
            };
            (0, divider)
        }
        BleLpClkConfig::RtcSlow => (2, 0),
    };

    BtdmLpClk {
        select,
        divider,
        lpcycle_us,
    }
}
