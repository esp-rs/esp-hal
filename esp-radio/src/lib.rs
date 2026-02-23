#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-radio</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C3, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs might change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! # Wireless support for Espressif ESP32 devices.
//!
//! This documentation is built for the
#![cfg_attr(esp32, doc = "**ESP32**")]
#![cfg_attr(esp32s2, doc = "**ESP32-S2**")]
#![cfg_attr(esp32s3, doc = "**ESP32-S3**")]
#![cfg_attr(esp32c2, doc = "**ESP32-C2**")]
#![cfg_attr(esp32c3, doc = "**ESP32-C3**")]
#![cfg_attr(esp32c5, doc = "**ESP32-C5**")]
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
//! You will also need a dynamic memory allocator, and a preemptive task scheduler in your
//! application. For the dynamic allocator, we recommend using `esp-alloc`. For the task scheduler,
//! the simplest option that is supported by us is `esp-rtos`, but you may use Ariel
//! OS or other operating systems as well.
#![cfg_attr(
    feature = "ieee802154",
    doc = "<div class=\"warning\"><b>Hint:</b> The scheduler is not required for the 802.15.4.</div>"
)]
#![doc = ""]
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! use esp_hal::interrupt::software::SoftwareInterruptControl;
//! use esp_hal::ram;
//! use esp_hal::timer::timg::TimerGroup;
//!
//! esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
//! esp_alloc::heap_allocator!(size: 36 * 1024);
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
//!
//! // THIS IS IMPORTANT FOR WIFI AND BLE: You MUST start the scheduler
//! // before initializing the radio!
//! esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
#![cfg_attr(
    wifi_driver_supported,
    doc = r#"

if let Ok((controller, interfaces)) = esp_radio::wifi::new(
    peripherals.WIFI,
    Default::default(),
) {}
"#
)]
#![cfg_attr(
    all(bt_driver_supported, not(wifi_driver_supported)),
    doc = r#"

# use esp_radio::ble::controller::BleConnector;
if let Ok(controller) = BleConnector::new(peripherals.BT, Default::default()) {}
"#
)]
#![doc = esp_hal::after_snippet!()]
//! ```
//! ```toml
//! [dependencies.esp-radio]
//! # A supported chip needs to be specified, as well as specific use-case features
#![doc = concat!(r#"features = [""#, chip!(), r#"", "wifi", "esp-now", "esp-alloc"]"#)]
//! [dependencies.esp-rtos]
#![doc = concat!(r#"features = [""#, chip!(), r#"", "esp-radio", "esp-alloc"]"#)]
//! [dependencies.esp-alloc]
#![doc = concat!(r#"features = [""#, chip!(), r#""]"#)]
//! ```
//! 
//! ### Optimization Level
//!
//! It is necessary to build with optimization level 2 or 3 since otherwise, it
//! might not even be able to connect or advertise.
//!
//! To make it work also for your debug builds add this to your `Cargo.toml`
//! ```toml
//! [profile.dev.package.esp-radio]
//! opt-level = 3
//! ```
//! ## Globally disable logging
//!
//! `esp-radio` contains a lot of trace-level logging statements.
//! For maximum performance you might want to disable logging via
//! a feature flag of the `log` crate. See [documentation](https://docs.rs/log/0.4.19/log/#compile-time-filters).
//! You should set it to `release_max_level_off`.
//!
//! ### Wi-Fi performance considerations
//!
//! The default configuration is quite conservative to reduce power and memory consumption.
//!
//! There are a number of settings which influence the general performance. Optimal settings are chip and applications specific.
//! You can get inspiration from the [ESP-IDF examples](https://github.com/espressif/esp-idf/tree/release/v5.3/examples/wifi/iperf)
//!
//! Please note that the configuration keys are usually named slightly different and not all configuration keys apply.
#![cfg_attr(
    feature = "wifi",
    doc = "By default the power-saving mode is [`PowerSaveMode::None`](crate::wifi::PowerSaveMode::None) and `ESP_PHY_CONFIG_PHY_ENABLE_USB` is enabled by default."
)]
//! In addition pay attention to these configuration keys:
//! - `ESP_RADIO_CONFIG_RX_QUEUE_SIZE`
//! - `ESP_RADIO_CONFIG_TX_QUEUE_SIZE`
//! - `ESP_RADIO_CONFIG_MAX_BURST_SIZE`
#![cfg_attr(
    multi_core,
    doc = concat!(
        "### Running on the Second Core",
        "\n\n",
        "BLE and Wi-Fi can also be run on the second core.",
        "\n\n",
        "`esp_radio::init` is recommended to be called on the first core. The tasks ",
        "created by `esp-radio` are pinned to the first core.",
        "\n\n",
        "It's also important to allocate adequate stack for the second core; in many ",
        "cases 8kB is not enough, and 16kB or more may be required depending on your ",
        "use case. Failing to allocate adequate stack may result in strange behaviour, ",
        "such as your application silently failing at some point during execution."
    )
)]
//! ## Feature flags
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
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_radio_config_table.md"))]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(feature = "print-logs-from-driver", feature(c_variadic))]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]
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
#![cfg_attr(docsrs, feature(doc_cfg, custom_inner_attributes, proc_macro_hygiene))]

#[macro_use]
extern crate esp_metadata_generated;

extern crate alloc;

// MUST be the first module
mod fmt;

use esp_hal as hal;
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
pub use esp_phy::CalibrationResult;
#[cfg(not(feature = "unstable"))]
use esp_phy::CalibrationResult;
use esp_radio_rtos_driver as preempt;
#[cfg(esp32)]
use hal::analog::adc::{release_adc2, try_claim_adc2};
#[cfg(feature = "wifi")]
use hal::{after_snippet, before_snippet};
use hal::{
    clock::{Clocks, init_radio_clocks},
    time::Rate,
};
use sys::include::esp_phy_calibration_data_t;

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
    #[cfg(esp32h2)]
    pub use esp_wifi_sys_esp32h2::*;
    #[cfg(esp32s2)]
    pub use esp_wifi_sys_esp32s2::*;
    #[cfg(esp32s3)]
    pub use esp_wifi_sys_esp32s3::*;
}

use crate::radio::{setup_radio_isr, shutdown_radio_isr};
#[cfg(feature = "wifi")]
use crate::wifi::WifiError;

// can't use instability on inline module definitions, see https://github.com/rust-lang/rust/issues/54727
#[doc(hidden)]
macro_rules! unstable_module {
    ($(
        $(#[$meta:meta])*
        pub mod $module:ident;
    )*) => {
        $(
            $(#[$meta])*
            #[cfg(feature = "unstable")]
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            pub mod $module;

            $(#[$meta])*
            #[cfg(not(feature = "unstable"))]
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            #[allow(unused)]
            pub(crate) mod $module;
        )*
    };
}

mod compat;

mod radio;
mod time;

#[cfg(feature = "wifi")]
pub mod wifi;

unstable_module! {
    #[cfg(feature = "esp-now")]
    #[cfg_attr(docsrs, doc(cfg(feature = "esp-now")))]
    pub mod esp_now;
    #[cfg(feature = "ble")]
    #[cfg_attr(docsrs, doc(cfg(feature = "ble")))]
    pub mod ble;
    #[cfg(feature = "ieee802154")]
    #[cfg_attr(docsrs, doc(cfg(feature = "ieee802154")))]
    pub mod ieee802154;
}

pub(crate) mod common_adapter;

#[cfg(all(feature = "ble", bt_controller = "npl"))]
pub(crate) static ESP_RADIO_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

static RADIO_REFCOUNT: esp_sync::NonReentrantMutex<u32> = esp_sync::NonReentrantMutex::new(0);

// this is just to verify that we use the correct defaults in `build.rs`
#[allow(clippy::assertions_on_constants)] // TODO: try assert_eq once it's usable in const context
const _: () = {
    cfg_if::cfg_if! {
        if #[cfg(wifi_driver_supported)] {
            core::assert!(sys::include::CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM == 10);
            core::assert!(sys::include::CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM == 32);
            core::assert!(sys::include::WIFI_STATIC_TX_BUFFER_NUM == 0);
            core::assert!(sys::include::CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM == 32);
            core::assert!(sys::include::CONFIG_ESP_WIFI_AMPDU_RX_ENABLED == 1);
            core::assert!(sys::include::CONFIG_ESP_WIFI_AMPDU_TX_ENABLED == 1);
            core::assert!(sys::include::WIFI_AMSDU_TX_ENABLED == 0);
            core::assert!(sys::include::CONFIG_ESP32_WIFI_RX_BA_WIN == 6);
        }
    };
};

#[procmacros::doc_replace]
/// Initialize for using Wi-Fi and or BLE.
///
/// Wi-Fi and BLE require a preemptive scheduler to be present. Without one, the underlying firmware
/// can't operate. The scheduler must implement the interfaces in the `esp-radio-rtos-driver`
/// crate. If you are using an embedded RTOS like Ariel OS, it needs to provide an appropriate
/// implementation.
///
/// If you are not using an embedded RTOS, use the `esp-rtos` crate which provides the
/// necessary functionality.
///
/// Make sure to **not** call this function while interrupts are disabled.
///
/// ## Errors
///
/// - The function may return an error if the scheduler is not initialized.
#[cfg_attr(
    esp32,
    doc = " - The function may return an error if ADC2 is already in use."
)]
/// - The function may return an error if interrupts are disabled.
/// - The function may return an error if initializing the underlying driver fails.
pub(crate) fn init() {
    #[cfg(esp32)]
    if try_claim_adc2(unsafe { hal::Internal::conjure() }).is_err() {
        panic!(
            "ADC2 is currently in use by esp-hal, but esp-radio requires it for Wi-Fi operation."
        );
    }

    if !preempt::initialized() {
        panic!("The scheduler must be initialized before initializing the radio.");
    }

    // A minimum clock of 80MHz is required to operate Wi-Fi module.
    const MIN_CLOCK: Rate = Rate::from_mhz(80);
    let clocks = Clocks::get();
    if clocks.cpu_clock < MIN_CLOCK {
        panic!(
            "CPU clock {} MHz is too slow for Wi-Fi operation, minimum required is {} MHz",
            clocks.cpu_clock.as_mhz(),
            MIN_CLOCK.as_mhz()
        );
    }

    crate::common_adapter::enable_wifi_power_domain();

    setup_radio_isr();

    wifi_set_log_verbose();
    init_radio_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => panic!("Failed to initialize coexistence, error code: {}", error),
    }

    debug!("Radio initialized");
}

pub(crate) fn deinit() {
    // Disable coexistence
    #[cfg(coex)]
    {
        unsafe { crate::wifi::os_adapter::coex_disable() };
        unsafe { crate::wifi::os_adapter::coex_deinit() };
    }

    shutdown_radio_isr();

    #[cfg(esp32)]
    // Allow using `ADC2` again
    release_adc2(unsafe { esp_hal::Internal::conjure() });

    debug!("Radio deinitialized");
}

/// Management of the global reference count
/// and conditional hardware initialization/deinitialization.
#[derive(Debug)]
pub(crate) struct RadioRefGuard;

impl RadioRefGuard {
    /// Increments the refcount. If the old count was 0, it performs hardware init.
    /// If hardware init fails, it rolls back the refcount only once.
    fn new() -> Self {
        RADIO_REFCOUNT.with(|rc| {
            debug!("Creating RadioRefGuard");

            if *rc == 0 {
                init();
            }

            *rc += 1;
            RadioRefGuard
        })
    }
}

impl Drop for RadioRefGuard {
    /// Decrements the refcount. If the count drops to 0, it performs hardware de-init.
    fn drop(&mut self) {
        RADIO_REFCOUNT.with(|rc| {
            debug!("Dropping RadioRefGuard");

            *rc -= 1;
            if *rc == 0 {
                deinit();
            }
        })
    }
}

/// Returns true if at least some interrupt levels are disabled.
#[cfg(any(feature = "wifi", all(feature = "ble", bt_controller = "btdm")))]
fn is_interrupts_disabled() -> bool {
    #[cfg(target_arch = "xtensa")]
    return hal::xtensa_lx::interrupt::get_level() != 0
        || hal::xtensa_lx::interrupt::get_mask() == 0;

    #[cfg(target_arch = "riscv32")]
    return !hal::riscv::register::mstatus::read().mie()
        || !hal::interrupt::RunLevel::current().is_thread();
}

/// Enable verbose logging within the Wi-Fi driver
/// Does nothing unless the `print-logs-from-driver` feature is enabled.
#[instability::unstable]
pub fn wifi_set_log_verbose() {
    #[cfg(all(feature = "print-logs-from-driver", not(esp32h2)))]
    unsafe {
        use crate::sys::include::{
            esp_wifi_internal_set_log_level,
            wifi_log_level_t_WIFI_LOG_VERBOSE,
        };

        esp_wifi_internal_set_log_level(wifi_log_level_t_WIFI_LOG_VERBOSE);
    }
}

/// Get calibration data.
///
/// Returns the last calibration result.
///
/// If [last_calibration_result] returns [CalibrationResult::DataCheckFailed], consider persisting
/// the new data.
#[instability::unstable]
pub fn phy_calibration_data(data: &mut [u8; esp_phy::PHY_CALIBRATION_DATA_LENGTH]) {
    let _ = esp_phy::backup_phy_calibration_data(data);
}

/// Set calibration data.
///
/// This will be used next time the phy gets initialized.
#[instability::unstable]
pub fn set_phy_calibration_data(data: &[u8; core::mem::size_of::<esp_phy_calibration_data_t>()]) {
    // Although we're ignoring the result here, this doesn't change the behavior, as this just
    // doesn't do anything in case an error is returned.
    let _ = esp_phy::set_phy_calibration_data(data);
}

/// Get the last calibration result.
///
/// This can be used to know if any previously persisted calibration data is outdated/invalid and
/// needs to get updated.
#[instability::unstable]
pub fn last_calibration_result() -> Option<CalibrationResult> {
    esp_phy::last_calibration_result()
}
