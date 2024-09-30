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
//! Ensure that the right features are enabled for your chip. See [Examples](https://github.com/esp-rs/esp-hal/tree/main/examples#examples) for more examples.
//!
//! ```toml
//! [dependencies.esp-wifi]
//! # A supported chip needs to be specified, as well as specific use-case features
//! features = ["esp32s3", "wifi", "esp-now"]
//! ```
//!
//! ### Link configuration
//!
//! Make sure to include the rom functions for your target:
//!
//! ```toml
//! # .cargo/config.toml
//! rustflags = [
//!     "-C", "link-arg=-Tlinkall.x",
//!     "-C", "link-arg=-Trom_functions.x",
//! ]
//! ```
//!
//! At the time of writing, you will already have the `linkall` flag if you used
//! `cargo generate`. Generating from a template does not include the
//! `rom_functions` flag.
//!
//! ### Optimization Level
//!
//! It is necessary to build with optimization level 2 or 3 since otherwise, it
//! might not even be able to connect or advertise.
//!
//! To make it work also for your debug builds add this to your `Cargo.toml`
//!
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
//! ## USB-SERIAL-JTAG
//!
//! When using USB-SERIAL-JTAG (for example by selecting `jtag-serial` in [`esp-println`](https://crates.io/crates/esp-println)) you have to activate the feature `phy-enable-usb`.
//!
//! Don't use this feature if you are _not_ using USB-SERIAL-JTAG as it might
//! reduce WiFi performance.
//!
//! # Features flags
//!
//! Note that not all features are available on every MCU. For example, `ble`
//! (and thus, `coex`) is not available on ESP32-S2.
//!
//! When using the `dump-packets` feature you can use the extcap in
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
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![cfg_attr(feature = "sys-logs", feature(c_variadic))]
#![allow(rustdoc::bare_urls)]
// allow until num-derive doesn't generate this warning anymore (unknown_lints because Xtensa
// toolchain doesn't know about that lint, yet)
#![allow(unknown_lints)]
#![allow(non_local_definitions)]

extern crate alloc;

// MUST be the first module
mod fmt;

use common_adapter::{chip_specific::phy_mem_init, init_radio_clock_control, RADIO_CLOCKS};
use esp_config::*;
use esp_hal as hal;
#[cfg(not(feature = "esp32"))]
use esp_hal::timer::systimer::Alarm;
use fugit::MegahertzU32;
use hal::{
    clock::Clocks,
    system::RadioClockController,
    timer::{timg::Timer as TimgTimer, AnyTimer, PeriodicTimer},
};
#[cfg(feature = "wifi")]
use wifi::WifiError;

use crate::{common_adapter::init_rng, tasks::init_tasks, timer::setup_timer_isr};

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

pub(crate) mod common_adapter;

#[doc(hidden)]
pub mod tasks;

pub(crate) mod memory_fence;

use timer::{get_systimer_count, ticks_to_millis};

#[cfg(all(feature = "wifi", any(feature = "tcp", feature = "udp")))]
pub mod wifi_interface;

/// Return the current systimer time in milliseconds
pub fn current_millis() -> u64 {
    ticks_to_millis(get_systimer_count())
}

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

pub(crate) const CONFIG: Config = Config {
    rx_queue_size: esp_config_int!(usize, "ESP_WIFI_RX_QUEUE_SIZE"),
    tx_queue_size: esp_config_int!(usize, "ESP_WIFI_TX_QUEUE_SIZE"),
    static_rx_buf_num: esp_config_int!(usize, "ESP_WIFI_STATIC_RX_BUF_NUM"),
    dynamic_rx_buf_num: esp_config_int!(usize, "ESP_WIFI_DYNAMIC_RX_BUF_NUM"),
    static_tx_buf_num: esp_config_int!(usize, "ESP_WIFI_STATIC_TX_BUF_NUM"),
    dynamic_tx_buf_num: esp_config_int!(usize, "ESP_WIFI_DYNAMIC_TX_BUF_NUM"),
    ampdu_rx_enable: esp_config_bool!("ESP_WIFI_AMPDU_RX_ENABLE"),
    ampdu_tx_enable: esp_config_bool!("ESP_WIFI_AMPDU_TX_ENABLE"),
    amsdu_tx_enable: esp_config_bool!("ESP_WIFI_AMSDU_TX_ENABLE"),
    rx_ba_win: esp_config_int!(usize, "ESP_WIFI_RX_BA_WIN"),
    max_burst_size: esp_config_int!(usize, "ESP_WIFI_MAX_BURST_SIZE"),
    country_code: esp_config_str!("ESP_WIFI_COUNTRY_CODE"),
    country_code_operating_class: esp_config_int!(u8, "ESP_WIFI_COUNTRY_CODE_OPERATING_CLASS"),
    mtu: esp_config_int!(usize, "ESP_WIFI_MTU"),
    tick_rate_hz: esp_config_int!(u32, "ESP_WIFI_TICK_RATE_HZ"),
    listen_interval: esp_config_int!(u16, "ESP_WIFI_LISTEN_INTERVAL"),
    beacon_timeout: esp_config_int!(u16, "ESP_WIFI_BEACON_TIMEOUT"),
    ap_beacon_timeout: esp_config_int!(u16, "ESP_WIFI_AP_BEACON_TIMEOUT"),
    failure_retry_cnt: esp_config_int!(u8, "ESP_WIFI_FAILURE_RETRY_CNT"),
    scan_method: esp_config_int!(u32, "ESP_WIFI_SCAN_METHOD"),
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

type TimeBase = PeriodicTimer<'static, AnyTimer>;

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// An internal struct designed to make [`EspWifiInitialization`] uncreatable
/// outside of this crate.
pub struct EspWifiInitializationInternal;

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Initialized the driver for WiFi, Bluetooth or both.
pub enum EspWifiInitialization {
    #[cfg(feature = "wifi")]
    Wifi(EspWifiInitializationInternal),
    #[cfg(feature = "ble")]
    Ble(EspWifiInitializationInternal),
    #[cfg(coex)]
    WifiBle(EspWifiInitializationInternal),
}

impl EspWifiInitialization {
    #[allow(unused)]
    fn is_wifi(&self) -> bool {
        match self {
            #[cfg(feature = "ble")]
            EspWifiInitialization::Ble(_) => false,
            _ => true,
        }
    }

    #[allow(unused)]
    fn is_ble(&self) -> bool {
        match self {
            #[cfg(feature = "wifi")]
            EspWifiInitialization::Wifi(_) => false,
            _ => true,
        }
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Initialize the driver for WiFi, Bluetooth or both.
pub enum EspWifiInitFor {
    #[cfg(feature = "wifi")]
    Wifi,
    #[cfg(feature = "ble")]
    Ble,
    #[cfg(coex)]
    WifiBle,
}

impl EspWifiInitFor {
    #[allow(unused)]
    fn is_wifi(&self) -> bool {
        match self {
            #[cfg(feature = "ble")]
            EspWifiInitFor::Ble => false,
            _ => true,
        }
    }

    #[allow(unused)]
    fn is_ble(&self) -> bool {
        match self {
            #[cfg(feature = "wifi")]
            EspWifiInitFor::Wifi => false,
            _ => true,
        }
    }
}

/// A trait to allow better UX for initializing esp-wifi.
///
/// This trait is meant to be used only for the `init` function.
/// Calling `timers()` multiple times may panic.
pub trait EspWifiTimerSource {
    /// Returns the timer source.
    fn timer(self) -> TimeBase;
}

/// Helper trait to reduce boilerplate.
///
/// We can't blanket-implement for `Into<AnyTimer>` because of possible
/// conflicting implementations.
trait IntoAnyTimer: Into<AnyTimer> {}

impl<T, DM> IntoAnyTimer for TimgTimer<T, DM>
where
    DM: esp_hal::Mode,
    Self: Into<AnyTimer>,
{
}

#[cfg(not(feature = "esp32"))]
impl<T, DM, COMP, UNIT> IntoAnyTimer for Alarm<'_, T, DM, COMP, UNIT>
where
    DM: esp_hal::Mode,
    Self: Into<AnyTimer>,
{
}

impl IntoAnyTimer for AnyTimer {}

impl<T> EspWifiTimerSource for T
where
    T: IntoAnyTimer,
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
/// use esp_hal::{rng::Rng, timg::TimerGroup};
/// use esp_wifi::EspWifiInitFor;
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0);
/// let init = esp_wifi::initialize(
///     EspWifiInitFor::Wifi,
///     timg0.timer0,
///     Rng::new(peripherals.RNG),
///     peripherals.RADIO_CLK,
/// )
/// .unwrap();
/// # }
/// ```
pub fn initialize(
    init_for: EspWifiInitFor,
    timer: impl EspWifiTimerSource,
    rng: hal::rng::Rng,
    radio_clocks: hal::peripherals::RADIO_CLK,
) -> Result<EspWifiInitialization, InitializationError> {
    // A minimum clock of 80MHz is required to operate WiFi module.
    const MIN_CLOCK: u32 = 80;
    let clocks = Clocks::get();
    if clocks.cpu_clock < MegahertzU32::MHz(MIN_CLOCK) {
        return Err(InitializationError::WrongClockConfig);
    }

    info!("esp-wifi configuration {:?}", crate::CONFIG);

    crate::common_adapter::chip_specific::enable_wifi_power_domain();

    phy_mem_init();
    init_radio_clock_control(radio_clocks);
    init_rng(rng);
    init_tasks();
    setup_timer_isr(timer.timer())?;
    wifi_set_log_verbose();
    init_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => return Err(InitializationError::General(error)),
    }

    #[cfg(feature = "wifi")]
    if init_for.is_wifi() {
        debug!("wifi init");
        // wifi init
        crate::wifi::wifi_init()?;
    }

    #[cfg(feature = "ble")]
    if init_for.is_ble() {
        // ble init
        // for some reason things don't work when initializing things the other way
        // around while the original implementation in NuttX does it like that
        debug!("ble init");
        crate::ble::ble_init();
    }

    match init_for {
        #[cfg(feature = "wifi")]
        EspWifiInitFor::Wifi => Ok(EspWifiInitialization::Wifi(EspWifiInitializationInternal)),
        #[cfg(feature = "ble")]
        EspWifiInitFor::Ble => Ok(EspWifiInitialization::Ble(EspWifiInitializationInternal)),
        #[cfg(coex)]
        EspWifiInitFor::WifiBle => Ok(EspWifiInitialization::WifiBle(
            EspWifiInitializationInternal,
        )),
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error which can be returned during [`initialize`].
pub enum InitializationError {
    General(i32),
    #[cfg(feature = "wifi")]
    WifiError(WifiError),
    WrongClockConfig,
    Timer(hal::timer::Error),
}

impl From<hal::timer::Error> for InitializationError {
    fn from(value: hal::timer::Error) -> Self {
        InitializationError::Timer(value)
    }
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
    unsafe {
        unwrap!(RADIO_CLOCKS.as_mut()).init_clocks();
    }
}
