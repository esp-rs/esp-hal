#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.esp-rs.org/esp-hal/'>browse the <code>esp-wifi</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C3, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs might change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! This documentation is built for the
#![cfg_attr(esp32, doc = "**ESP32**")]
#![cfg_attr(esp32s2, doc = "**ESP32-S2**")]
#![cfg_attr(esp32s3, doc = "**ESP32-S3**")]
#![cfg_attr(esp32c2, doc = "**ESP32-C2**")]
#![cfg_attr(esp32c3, doc = "**ESP32-C3**")]
#![cfg_attr(esp32c6, doc = "**ESP32-C6**")]
#![cfg_attr(esp32h2, doc = "**ESP32-H2**")]
//! . Please ensure you are reading the correct documentation for your target
//! device.
//!
//! ## Usage
//!
//! ### Importing
//!
//! Note that this crate currently requires you to enable the `unstable` feature
//! on `esp-hal`.
//!
//! Ensure that the right features are enabled for your chip. See [Examples](https://github.com/esp-rs/esp-hal/tree/main/examples#examples) for more examples.
//!
//! ```toml
//! [dependencies.esp-wifi]
//! # A supported chip needs to be specified, as well as specific use-case features
#![doc = concat!(r#"features = [""#, esp_hal::chip!(), r#"", "wifi", "esp-now"]"#)]
//! ```
//! 
//! ### Optimization Level
//!
//! It is necessary to build with optimization level 2 or 3 since otherwise, it
//! might not even be able to connect or advertise.
//!
//! To make it work also for your debug builds add this to your `Cargo.toml`
//! ```toml
//! [profile.dev.package.esp-wifi]
//! opt-level = 3
//! ```
//! ## Globally disable logging
//!
//! `esp-wifi` contains a lot of trace-level logging statements.
//! For maximum performance you might want to disable logging via
//! a feature flag of the `log` crate. See [documentation](https://docs.rs/log/0.4.19/log/#compile-time-filters).
//! You should set it to `release_max_level_off`.
//!
//! ### Xtensa considerations
//!
//! Within this crate, `CCOMPARE0` CPU timer is used for timing, ensure that in
//! your application you are not using this CPU timer.
//!
//! # Features flags
//!
//! Note that not all features are available on every MCU. For example, `ble`
//! (and thus, `coex`) is not available on ESP32-S2.
//!
//! When using the `dump_packets` config you can use the extcap in
//! `extras/esp-wifishark` to analyze the frames in Wireshark.
//! For more information see
//! [extras/esp-wifishark/README.md](../extras/esp-wifishark/README.md)
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Below is a table of tunable parameters
//! for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_wifi_config_table.md"))]
#![doc = ""]
//! It's important to note that due to a [bug in cargo](https://github.com/rust-lang/cargo/issues/10358),
//! any modifications to the environment, local or otherwise will only get
//! picked up on a full clean build of the project.

#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(feature = "sys-logs", feature(c_variadic))]
#![allow(rustdoc::bare_urls)]
// allow until num-derive doesn't generate this warning anymore (unknown_lints because Xtensa
// toolchain doesn't know about that lint, yet)
#![allow(unknown_lints)]
#![allow(non_local_definitions)]

extern crate alloc;

// MUST be the first module
mod fmt;

use core::marker::PhantomData;

use common_adapter::chip_specific::phy_mem_init;
use esp_config::*;
use esp_hal as hal;
use esp_hal::peripheral::Peripheral;
#[cfg(not(feature = "esp32"))]
use esp_hal::timer::systimer::Alarm;
use fugit::MegahertzU32;
use hal::{
    clock::Clocks,
    rng::{Rng, Trng},
    system::RadioClockController,
    timer::{timg::Timer as TimgTimer, AnyTimer, PeriodicTimer},
    Blocking,
};
use portable_atomic::Ordering;

#[cfg(feature = "wifi")]
use crate::wifi::WifiError;
use crate::{
    tasks::init_tasks,
    timer::{setup_timer_isr, shutdown_timer_isr},
};

mod binary {
    pub use esp_wifi_sys::*;
}
mod compat;
mod preempt;

mod timer;

#[cfg(feature = "wifi")]
pub mod wifi;

#[cfg(feature = "ble")]
pub mod ble;

#[cfg(feature = "esp-now")]
pub mod esp_now;

pub mod config;

pub(crate) mod common_adapter;

#[doc(hidden)]
pub mod tasks;

pub(crate) mod memory_fence;

// this is just to verify that we use the correct defaults in `build.rs`
#[allow(clippy::assertions_on_constants)] // TODO: try assert_eq once it's usable in const context
const _: () = {
    cfg_if::cfg_if! {
        if #[cfg(not(esp32h2))] {
            core::assert!(binary::include::CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM == 10);
            core::assert!(binary::include::CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM == 32);
            core::assert!(binary::include::WIFI_STATIC_TX_BUFFER_NUM == 0);
            core::assert!(binary::include::CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM == 32);
            core::assert!(binary::include::CONFIG_ESP_WIFI_AMPDU_RX_ENABLED == 1);
            core::assert!(binary::include::CONFIG_ESP_WIFI_AMPDU_TX_ENABLED == 1);
            core::assert!(binary::include::WIFI_AMSDU_TX_ENABLED == 0);
            core::assert!(binary::include::CONFIG_ESP32_WIFI_RX_BA_WIN == 6);
        }
    };
};

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Tunable parameters for the WiFi driver
#[allow(unused)] // currently there are no ble tunables
struct Config {
    rx_queue_size: usize,
    tx_queue_size: usize,
    static_rx_buf_num: usize,
    dynamic_rx_buf_num: usize,
    static_tx_buf_num: usize,
    dynamic_tx_buf_num: usize,
    csi_enable: bool,
    ampdu_rx_enable: bool,
    ampdu_tx_enable: bool,
    amsdu_tx_enable: bool,
    rx_ba_win: usize,
    max_burst_size: usize,
    country_code: &'static str,
    country_code_operating_class: u8,
    mtu: usize,
    tick_rate_hz: u32,
    listen_interval: u16,
    beacon_timeout: u16,
    ap_beacon_timeout: u16,
    failure_retry_cnt: u8,
    scan_method: u32,
}

pub(crate) const CONFIG: config::EspWifiConfig = config::EspWifiConfig {
    rx_queue_size: esp_config_int!(usize, "ESP_WIFI__RX_QUEUE_SIZE"),
    tx_queue_size: esp_config_int!(usize, "ESP_WIFI__TX_QUEUE_SIZE"),
    static_rx_buf_num: esp_config_int!(usize, "ESP_WIFI__STATIC_RX_BUF_NUM"),
    dynamic_rx_buf_num: esp_config_int!(usize, "ESP_WIFI__DYNAMIC_RX_BUF_NUM"),
    static_tx_buf_num: esp_config_int!(usize, "ESP_WIFI__STATIC_TX_BUF_NUM"),
    dynamic_tx_buf_num: esp_config_int!(usize, "ESP_WIFI__DYNAMIC_TX_BUF_NUM"),
    csi_enable: esp_config_bool!("ESP_WIFI__CSI_ENABLE"),
    ampdu_rx_enable: esp_config_bool!("ESP_WIFI__AMPDU_RX_ENABLE"),
    ampdu_tx_enable: esp_config_bool!("ESP_WIFI__AMPDU_TX_ENABLE"),
    amsdu_tx_enable: esp_config_bool!("ESP_WIFI__AMSDU_TX_ENABLE"),
    rx_ba_win: esp_config_int!(usize, "ESP_WIFI__RX_BA_WIN"),
    max_burst_size: esp_config_int!(usize, "ESP_WIFI__MAX_BURST_SIZE"),
    country_code: esp_config_str!("ESP_WIFI__COUNTRY_CODE"),
    country_code_operating_class: esp_config_int!(u8, "ESP_WIFI__COUNTRY_CODE_OPERATING_CLASS"),
    mtu: esp_config_int!(usize, "ESP_WIFI__MTU"),
    tick_rate_hz: esp_config_int!(u32, "ESP_WIFI__TICK_RATE_HZ"),
    listen_interval: esp_config_int!(u16, "ESP_WIFI__LISTEN_INTERVAL"),
    beacon_timeout: esp_config_int!(u16, "ESP_WIFI__BEACON_TIMEOUT"),
    ap_beacon_timeout: esp_config_int!(u16, "ESP_WIFI__AP_BEACON_TIMEOUT"),
    failure_retry_cnt: esp_config_int!(u8, "ESP_WIFI__FAILURE_RETRY_CNT"),
    scan_method: esp_config_int!(u32, "ESP_WIFI__SCAN_METHOD"),
};

// Validate the configuration at compile time
#[allow(clippy::assertions_on_constants)]
const _: () = {
    // We explicitely use `core` assert here because this evaluation happens at
    // compile time and won't bloat the binary
    core::assert!(
        CONFIG.rx_ba_win < CONFIG.dynamic_rx_buf_num,
        "WiFi configuration check: rx_ba_win should not be larger than dynamic_rx_buf_num!"
    );
    core::assert!(CONFIG.rx_ba_win < (CONFIG.static_rx_buf_num * 2), "WiFi configuration check: rx_ba_win should not be larger than double of the static_rx_buf_num!");
};

type TimeBase = PeriodicTimer<'static, Blocking, AnyTimer>;

pub(crate) mod flags {
    use portable_atomic::{AtomicBool, AtomicUsize};

    pub(crate) static ESP_WIFI_INITIALIZED: AtomicBool = AtomicBool::new(false);
    pub(crate) static WIFI: AtomicUsize = AtomicUsize::new(0);
    pub(crate) static BLE: AtomicBool = AtomicBool::new(false);
}

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EspWifiController<'d> {
    _inner: PhantomData<&'d ()>,
}

impl EspWifiController<'_> {
    /// Is the WiFi part of the radio running
    pub fn wifi(&self) -> bool {
        crate::flags::WIFI.load(Ordering::Acquire) > 0
    }

    /// Is the BLE part of the radio running
    pub fn ble(&self) -> bool {
        crate::flags::BLE.load(Ordering::Acquire)
    }

    /// De-initialize the radio
    pub fn deinit(self) -> Result<(), InitializationError> {
        if crate::flags::ESP_WIFI_INITIALIZED.load(Ordering::Acquire) {
            // safety: no other driver can be using this if this is callable
            unsafe { deinit_unchecked() }
        } else {
            Ok(())
        }
    }

    pub(crate) unsafe fn conjure() -> Self {
        Self {
            _inner: PhantomData,
        }
    }
}

impl Drop for EspWifiController<'_> {
    fn drop(&mut self) {
        if crate::flags::ESP_WIFI_INITIALIZED.load(Ordering::Acquire) {
            // safety: no other driver can be using this if this is callable
            unsafe { deinit_unchecked().ok() };
        }
    }
}

/// A trait to allow better UX for initializing esp-wifi.
///
/// This trait is meant to be used only for the `init` function.
/// Calling `timers()` multiple times may panic.
pub trait EspWifiTimerSource: private::Sealed {
    /// Returns the timer source.
    fn timer(self) -> TimeBase;
}

/// Helper trait to reduce boilerplate.
///
/// We can't blanket-implement for `Into<AnyTimer>` because of possible
/// conflicting implementations.
trait IntoAnyTimer: Into<AnyTimer> {}

impl IntoAnyTimer for TimgTimer where Self: Into<AnyTimer> {}

#[cfg(not(feature = "esp32"))]
impl IntoAnyTimer for Alarm where Self: Into<AnyTimer> {}

impl private::Sealed for AnyTimer {}
impl IntoAnyTimer for AnyTimer {}

impl<T> EspWifiTimerSource for T
where
    T: IntoAnyTimer + private::Sealed,
{
    fn timer(self) -> TimeBase {
        TimeBase::new(self.into()).timer()
    }
}

impl EspWifiTimerSource for TimeBase {
    fn timer(self) -> TimeBase {
        self
    }
}

impl private::Sealed for TimeBase {}
impl private::Sealed for TimgTimer where Self: Into<AnyTimer> {}
#[cfg(not(feature = "esp32"))]
impl private::Sealed for Alarm where Self: Into<AnyTimer> {}

/// A marker trait for suitable Rng sources for esp-wifi
pub trait EspWifiRngSource: rand_core::RngCore + private::Sealed {}

impl EspWifiRngSource for Rng {}
impl private::Sealed for Rng {}
impl EspWifiRngSource for Trng<'_> {}
impl private::Sealed for Trng<'_> {}

/// Initialize for using WiFi and or BLE.
///
/// # The `timer` argument
///
/// The `timer` argument is a timer source that is used by the WiFi driver to
/// schedule internal tasks. The timer source can be any of the following:
///
/// - A timg `Timer` instance
/// - A systimer `Alarm` instance
/// - An `AnyTimer` instance
/// - A `OneShotTimer` instance
///
/// # Examples
///
/// ```rust, no_run
#[doc = esp_hal::before_snippet!()]
/// use esp_hal::{rng::Rng, timer::timg::TimerGroup};
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0);
/// let init = esp_wifi::init(
///     timg0.timer0,
///     Rng::new(peripherals.RNG),
///     peripherals.RADIO_CLK,
/// )
/// .unwrap();
/// # }
/// ```
pub fn init<'d, T: EspWifiTimerSource>(
    timer: impl Peripheral<P = T> + 'd,
    _rng: impl EspWifiRngSource,
    _radio_clocks: impl Peripheral<P = hal::peripherals::RADIO_CLK> + 'd,
) -> Result<EspWifiController<'d>, InitializationError> {
    // A minimum clock of 80MHz is required to operate WiFi module.
    const MIN_CLOCK: u32 = 80;
    let clocks = Clocks::get();
    if clocks.cpu_clock < MegahertzU32::MHz(MIN_CLOCK) {
        return Err(InitializationError::WrongClockConfig);
    }

    info!("esp-wifi configuration {:?}", crate::CONFIG);
    crate::common_adapter::chip_specific::enable_wifi_power_domain();
    phy_mem_init();
    init_tasks();
    setup_timer_isr(unsafe { timer.clone_unchecked() }.timer());

    wifi_set_log_verbose();
    init_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => return Err(InitializationError::General(error)),
    }

    crate::flags::ESP_WIFI_INITIALIZED.store(true, Ordering::Release);

    Ok(EspWifiController {
        _inner: PhantomData,
    })
}

/// Deinitializes the entire radio stack
///
/// This can be useful to shutdown the stack before going to sleep for example.
///
/// # Safety
///
/// The user must ensure that any use of the radio via the WIFI/BLE/ESP-NOW
/// drivers are complete, else undefined behaviour may occur within those
/// drivers.
pub unsafe fn deinit_unchecked() -> Result<(), InitializationError> {
    // Disable coexistence
    #[cfg(coex)]
    {
        unsafe { crate::wifi::os_adapter::coex_disable() };
        unsafe { crate::wifi::os_adapter::coex_deinit() };
    }

    let controller = unsafe { EspWifiController::conjure() };

    // Peripheral drivers should already take care of shutting these down
    // we have to check this in the case where a user calls `deinit_unchecked`
    // directly.
    if controller.wifi() {
        #[cfg(feature = "wifi")]
        crate::wifi::wifi_deinit()?;
        crate::flags::WIFI.store(0, Ordering::Release);
    }

    if controller.ble() {
        #[cfg(feature = "ble")]
        crate::ble::ble_deinit();
        crate::flags::BLE.store(false, Ordering::Release);
    }

    shutdown_timer_isr();
    crate::preempt::delete_all_tasks();

    crate::timer::TIMER.with(|timer| timer.take());

    crate::flags::ESP_WIFI_INITIALIZED.store(false, Ordering::Release);

    Ok(())
}

pub(crate) mod private {
    pub trait Sealed {}
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error which can be returned during [`init`].
pub enum InitializationError {
    General(i32),
    #[cfg(feature = "wifi")]
    WifiError(WifiError),
    WrongClockConfig,
}

#[cfg(feature = "wifi")]
impl From<WifiError> for InitializationError {
    fn from(value: WifiError) -> Self {
        InitializationError::WifiError(value)
    }
}

/// Enable verbose logging within the WiFi driver
/// Does nothing unless the `sys-logs` feature is enabled.
pub fn wifi_set_log_verbose() {
    #[cfg(all(feature = "sys-logs", not(esp32h2)))]
    unsafe {
        use crate::binary::include::{
            esp_wifi_internal_set_log_level,
            wifi_log_level_t_WIFI_LOG_VERBOSE,
        };

        esp_wifi_internal_set_log_level(wifi_log_level_t_WIFI_LOG_VERBOSE);
    }
}

fn init_clocks() {
    let mut radio_clocks = unsafe { esp_hal::peripherals::RADIO_CLK::steal() };
    radio_clocks.init_clocks();
}
