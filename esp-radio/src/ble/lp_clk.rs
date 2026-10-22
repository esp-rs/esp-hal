//! BLE low-power clock, read from the clock tree.
//!
//! The tree reports the divided output frequency.

#[cfg(bt_controller = "btdm")]
use esp_hal::clock::ll::BleLpClkConfig;
use esp_hal::clock::ll::{self, ClockSource};
#[cfg(sleep_driver_supported)]
use {
    enumset::EnumSet,
    esp_hal::rtc_cntl::{WakeupSource, sleep::WrappedSleepConfig},
    portable_atomic::{AtomicU32, Ordering},
};

/// Sleep clock accuracy (in ppm) reported to the BLE controller when `BLE_LP_CLK` runs from an RC
/// oscillator (see `ble_rtc_clk_init()` in `bt.c`).
#[cfg(bt_controller = "npl")]
pub(crate) const RC_SCA_PPM: u16 = 3000;

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

#[cfg(bt_controller = "btdm")]
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
/// ESP32 needs a 32 kHz crystal for it, because it cannot light-sleep with the main crystal as the
/// controller clock.
pub(crate) fn light_sleep_supported() -> bool {
    cfg_select! {
        not(sleep_driver_supported) => false,
        esp32 => ll::ClockTree::with(ll::ble_lp_clk_root_source) != Some(ClockSource::XtalClk),
        _ => true,
    }
}

/// The source that `BLE_LP_CLK` runs on, as the bits of an [`EnumSet`].
#[cfg(sleep_driver_supported)]
static WAKE_CLOCK: AtomicU32 = AtomicU32::new(0);

/// Claims the `Bt` wakeup source, so that the chip can sleep while the controller sleeps.
///
/// Call this only with modem sleep on, after the controller is enabled, and only when
/// [`light_sleep_supported`] returns `true`. Otherwise the radio wake lock stays.
pub(crate) fn claim_wake_source() {
    #[cfg(sleep_driver_supported)]
    {
        let root = ll::ClockTree::with(ll::ble_lp_clk_root_source);
        WAKE_CLOCK.store(
            root.map_or(0, |source| EnumSet::only(source).as_u32()),
            Ordering::Relaxed,
        );
        WakeupSource::Bt.enable_with_hooks(Some(sleep_entry), None);
    }
}

/// Releases the claim taken by [`claim_wake_source`].
///
/// Call this before the controller is disabled.
pub(crate) fn release_wake_source() {
    #[cfg(sleep_driver_supported)]
    WakeupSource::Bt.disable();
}

#[cfg(sleep_driver_supported)]
fn sleep_entry(config: &mut WrappedSleepConfig<'_>) {
    // The controller does not survive a deep sleep, so it needs no clock through one.
    if config.is_deep_sleep() {
        return;
    }

    // The controller holds the PHY while it is awake, and it then runs on clocks that light sleep
    // stops.
    let awake = !super::MODEM_PHY_OFF.load(Ordering::Relaxed);
    #[cfg(bt_controller = "btdm")]
    let awake = awake || super::porting::hci_packet_in_flight();
    if awake {
        config.reject_sleep();
        return;
    }

    cfg_select! {
        bt_controller = "btdm" => {
            // The controller wakes on its own timer, but the chip must be awake before it does.
            let remaining = super::porting::time_until_controller_wakes();
            if remaining.as_micros() == 0 {
                config.reject_sleep();
                return;
            }
            config.limit_sleep(remaining);
        }
        // The BLE timer wakes the chip through the `Bt` source. The controller refuses the sleep
        // when its next event is too close.
        not(esp32c2) => {
            if super::porting::controller_skips_light_sleep() {
                config.reject_sleep();
                return;
            }
        }
        // The BLE timer wakes the chip through the `Bt` source.
        _ => {}
    }

    for source in EnumSet::<ClockSource>::from_u32_truncated(WAKE_CLOCK.load(Ordering::Relaxed)) {
        config.keep_clock_running(source);
    }
}

/// Returns whether the clock node's root source is an internal RC oscillator.
#[cfg(not(bt_controller = "btdm2"))]
pub(crate) fn source_is_rc(f: fn(&mut ll::ClockTree) -> Option<ClockSource>) -> bool {
    matches!(
        ll::ClockTree::with(f),
        Some(ClockSource::RcSlowClk | ClockSource::RcFastClk)
    )
}

/// Returns the BLE low-power clock frequency, in hertz.
#[cfg(not(bt_controller = "btdm"))]
pub(crate) fn frequency_hz() -> u32 {
    ll::ble_lp_clk_frequency()
}

/// Returns the microseconds per low-power cycle.
#[cfg(bt_controller = "btdm")]
pub(crate) fn btdm_lpcycle_us() -> u32 {
    let hz = ll::ble_lp_clk_frequency().max(1);
    ((1_000_000u64 << 19) / u64::from(hz)) as u32
}

/// Returns the BTDM blob clock-select parameters for the configured source.
#[cfg(bt_controller = "btdm")]
pub(crate) fn btdm() -> BtdmLpClk {
    // ESP32 only supports the main crystal or an external 32 kHz crystal/oscillator as the BLE
    // sleep clock (see `btdm_low_power_mode_init()` in `esp32/bt.c`).
    #[cfg(esp32)]
    if source_is_rc(ll::ble_lp_clk_root_source) {
        panic!("RC clocks cannot be selected as the BLE low-power clock");
    }

    let config = selected();
    let lpcycle_us = btdm_lpcycle_us();

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
