#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-wifi</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C3, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs might change depending on the chip.</p></div>\n\n<br/>\n\n"
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
#![doc = concat!(r#"features = [""#, chip!(), r#"", "wifi", "esp-now"]"#)]
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
//! ### WiFi performance considerations
//!
//! The default configuration is quite conservative to reduce power and memory consumption.
//!
//! There are a number of settings which influence the general performance. Optimal settings are chip and applications specific.
//! You can get inspiration from the [ESP-IDF examples](https://github.com/espressif/esp-idf/tree/release/v5.3/examples/wifi/iperf)
//!
//! Please note that the configuration keys are usually named slightly different and not all configuration keys apply.
//!
//! By default the power-saving mode is [PowerSaveMode::None](crate::config::PowerSaveMode::None) and `ESP_WIFI_PHY_ENABLE_USB` is enabled by default.
//!
//! In addition pay attention to these configuration keys:
//! - `ESP_WIFI_RX_QUEUE_SIZE`
//! - `ESP_WIFI_TX_QUEUE_SIZE`
//! - `ESP_WIFI_MAX_BURST_SIZE`
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
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(feature = "sys-logs", feature(c_variadic))]
#![deny(rust_2018_idioms, rustdoc::all)]
#![allow(rustdoc::bare_urls)]
// allow until num-derive doesn't generate this warning anymore (unknown_lints because Xtensa
// toolchain doesn't know about that lint, yet)
#![allow(unknown_lints)]
#![allow(non_local_definitions)]
#![cfg_attr(
    not(any(feature = "wifi", feature = "ble")),
    allow(
        unused,
        reason = "There are a number of places where code is needed for either wifi or ble,
        and cfg-ing them out would make the code less readable just to avoid warnings in the
        less common case. Truly unused code will be flagged by the check that enables either
        ble or wifi."
    )
)]

#[macro_use]
extern crate esp_metadata_generated;

extern crate alloc;

// MUST be the first module
mod fmt;

use core::marker::PhantomData;

use common_adapter::chip_specific::phy_mem_init;
use esp_config::*;
use esp_hal::{self as hal};
use esp_radio_preempt_driver as preempt;
use hal::{
    clock::{Clocks, init_radio_clocks},
    time::Rate,
};

#[cfg(feature = "wifi")]
use crate::wifi::WifiError;
use crate::{
    preempt::yield_task,
    radio::{setup_radio_isr, shutdown_radio_isr},
    tasks::init_tasks,
};

mod binary {
    pub use esp_wifi_sys::*;
}
mod compat;

mod radio;
mod time;

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

pub(crate) const CONFIG: config::EspWifiConfig = config::EspWifiConfig {
    rx_queue_size: esp_config_int!(usize, "ESP_WIFI_CONFIG_RX_QUEUE_SIZE"),
    tx_queue_size: esp_config_int!(usize, "ESP_WIFI_CONFIG_TX_QUEUE_SIZE"),
    static_rx_buf_num: esp_config_int!(usize, "ESP_WIFI_CONFIG_STATIC_RX_BUF_NUM"),
    dynamic_rx_buf_num: esp_config_int!(usize, "ESP_WIFI_CONFIG_DYNAMIC_RX_BUF_NUM"),
    static_tx_buf_num: esp_config_int!(usize, "ESP_WIFI_CONFIG_STATIC_TX_BUF_NUM"),
    dynamic_tx_buf_num: esp_config_int!(usize, "ESP_WIFI_CONFIG_DYNAMIC_TX_BUF_NUM"),
    ampdu_rx_enable: esp_config_bool!("ESP_WIFI_CONFIG_AMPDU_RX_ENABLE"),
    ampdu_tx_enable: esp_config_bool!("ESP_WIFI_CONFIG_AMPDU_TX_ENABLE"),
    amsdu_tx_enable: esp_config_bool!("ESP_WIFI_CONFIG_AMSDU_TX_ENABLE"),
    rx_ba_win: esp_config_int!(usize, "ESP_WIFI_CONFIG_RX_BA_WIN"),
    max_burst_size: esp_config_int!(usize, "ESP_WIFI_CONFIG_MAX_BURST_SIZE"),
    country_code: esp_config_str!("ESP_WIFI_CONFIG_COUNTRY_CODE"),
    country_code_operating_class: esp_config_int!(
        u8,
        "ESP_WIFI_CONFIG_COUNTRY_CODE_OPERATING_CLASS"
    ),
    mtu: esp_config_int!(usize, "ESP_WIFI_CONFIG_MTU"),
    listen_interval: esp_config_int!(u16, "ESP_WIFI_CONFIG_LISTEN_INTERVAL"),
    beacon_timeout: esp_config_int!(u16, "ESP_WIFI_CONFIG_BEACON_TIMEOUT"),
    ap_beacon_timeout: esp_config_int!(u16, "ESP_WIFI_CONFIG_AP_BEACON_TIMEOUT"),
    failure_retry_cnt: esp_config_int!(u8, "ESP_WIFI_CONFIG_FAILURE_RETRY_CNT"),
    scan_method: esp_config_int!(u32, "ESP_WIFI_CONFIG_SCAN_METHOD"),
};

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EspWifiController<'d> {
    _inner: PhantomData<&'d ()>,
}

impl Drop for EspWifiController<'_> {
    fn drop(&mut self) {
        // Disable coexistence
        #[cfg(coex)]
        {
            unsafe { crate::wifi::os_adapter::coex_disable() };
            unsafe { crate::wifi::os_adapter::coex_deinit() };
        }

        shutdown_radio_isr();

        // This shuts down the task switcher and timer tick interrupt.
        preempt::disable();
    }
}

/// Initialize for using WiFi and or BLE.
///
/// Make sure to **not** call this function while interrupts are disabled.
pub fn init<'d>() -> Result<EspWifiController<'d>, InitializationError> {
    if crate::is_interrupts_disabled() {
        return Err(InitializationError::InterruptsDisabled);
    }

    if !preempt::initialized() {
        return Err(InitializationError::SchedulerNotInitialized);
    }

    // A minimum clock of 80MHz is required to operate WiFi module.
    const MIN_CLOCK: Rate = Rate::from_mhz(80);
    let clocks = Clocks::get();
    if clocks.cpu_clock < MIN_CLOCK {
        return Err(InitializationError::WrongClockConfig);
    }

    info!("esp-wifi configuration {:?}", crate::CONFIG);
    crate::common_adapter::chip_specific::enable_wifi_power_domain();
    phy_mem_init();

    setup_radio_isr();

    // This initializes the task switcher
    preempt::enable();

    init_tasks();
    yield_task();

    wifi_set_log_verbose();
    init_radio_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => return Err(InitializationError::General(error)),
    }

    Ok(EspWifiController {
        _inner: PhantomData,
    })
}

/// Returns true if at least some interrupt levels are disabled.
fn is_interrupts_disabled() -> bool {
    #[cfg(target_arch = "xtensa")]
    return hal::xtensa_lx::interrupt::get_level() != 0
        || hal::xtensa_lx::interrupt::get_mask() == 0;

    #[cfg(target_arch = "riscv32")]
    return !hal::riscv::register::mstatus::read().mie()
        || hal::interrupt::current_runlevel() >= hal::interrupt::Priority::Priority1;
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error which can be returned during [`init`].
#[non_exhaustive]
pub enum InitializationError {
    /// A general error occurred.
    /// The internal error code is reported.
    General(i32),
    /// An error from the WiFi driver.
    #[cfg(feature = "wifi")]
    WifiError(WifiError),
    /// The current CPU clock frequency is too low.
    WrongClockConfig,
    /// Tried to initialize while interrupts are disabled.
    /// This is not supported.
    InterruptsDisabled,
    /// The scheduler is not initialized.
    SchedulerNotInitialized,
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
