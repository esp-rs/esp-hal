#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-radio</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C3, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs might change depending on the chip.</p></div>\n\n<br/>\n\n"
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
//! [dependencies.esp-radio]
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
    doc = "By default the power-saving mode is [PowerSaveMode::None](crate::wifi::PowerSaveMode::None) and `ESP_PHY_PHY_ENABLE_USB` is enabled by default."
)]
//! In addition pay attention to these configuration keys:
//! - `ESP_RADIO_RX_QUEUE_SIZE`
//! - `ESP_RADIO_TX_QUEUE_SIZE`
//! - `ESP_RADIO_MAX_BURST_SIZE`
#![cfg_attr(
    multi_core,
    doc = concat!(
        "### Running on the Second Core",
        "\n\n",
        "BLE and Wi-Fi can also be run on the second core.",
        "\n\n",
        "`esp_rtos::init` and `esp_radio::init` _must_ be called on the core on",
        "which you intend to run the wireless code. This will correctly initialize",
        "the radio peripheral to run on that core, and ensure that interrupts are",
        "serviced by the correct core.",
        "\n\n",
        "It's also important to allocate adequate stack for the second core; in many",
        "cases 8kB is not enough, and 16kB or more may be required depending on your",
        "use case. Failing to allocate adequate stack may result in strange behaviour,",
        "such as your application silently failing at some point during execution."
    )
)]
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
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_radio_config_table.md"))]
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
#![cfg_attr(docsrs, feature(doc_cfg, custom_inner_attributes, proc_macro_hygiene))]

#[macro_use]
extern crate esp_metadata_generated;

extern crate alloc;

// MUST be the first module
mod fmt;

use core::marker::PhantomData;

pub use common_adapter::{phy_calibration_data, set_phy_calibration_data};
use esp_hal::{self as hal};
use esp_radio_rtos_driver as preempt;
use esp_sync::RawMutex;
#[cfg(esp32)]
use hal::analog::adc::{release_adc2, try_claim_adc2};
use hal::{
    clock::{Clocks, init_radio_clocks},
    time::Rate,
};

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

mod binary {
    pub use esp_wifi_sys::*;
}
mod compat;

mod radio;
mod time;

#[cfg(feature = "wifi")]
pub mod wifi;

unstable_module! {
    #[cfg(feature = "esp-now")]
    pub mod esp_now;
    #[cfg(feature = "ble")]
    pub mod ble;
    #[cfg(feature = "ieee802154")]
    pub mod ieee802154;
}

pub(crate) mod common_adapter;
pub(crate) mod memory_fence;

pub(crate) static ESP_RADIO_LOCK: RawMutex = RawMutex::new();

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

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Controller for the ESP Radio driver.
pub struct Controller<'d> {
    _inner: PhantomData<&'d ()>,
}

impl Drop for Controller<'_> {
    fn drop(&mut self) {
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
    }
}

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
///
/// ## Example
///
/// For examples of the necessary setup, see your RTOS's documentation. If you are
/// using the `esp-rtos` crate, you will need to initialize the scheduler before calling this
/// function:
///
/// ```rust, no_run
#[doc = esp_hal::before_snippet!()]
/// use esp_hal::timer::timg::TimerGroup;
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0);
#[cfg_attr(
    riscv,
    doc = " let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);"
)]
#[cfg_attr(riscv, doc = " esp_rtos::start(timg0.timer0, software_interrupt);")]
#[cfg_attr(xtensa, doc = " esp_rtos::start(timg0.timer0);")]
/// // You can now start esp-radio:
/// let esp_radio_controller = esp_radio::init().unwrap();
/// # }
/// ```
pub fn init<'d>() -> Result<Controller<'d>, InitializationError> {
    #[cfg(esp32)]
    if try_claim_adc2(unsafe { hal::Internal::conjure() }).is_err() {
        return Err(InitializationError::Adc2IsUsed);
    }

    if crate::is_interrupts_disabled() {
        return Err(InitializationError::InterruptsDisabled);
    }

    if !preempt::initialized() {
        return Err(InitializationError::SchedulerNotInitialized);
    }

    // A minimum clock of 80MHz is required to operate Wi-Fi module.
    const MIN_CLOCK: Rate = Rate::from_mhz(80);
    let clocks = Clocks::get();
    if clocks.cpu_clock < MIN_CLOCK {
        return Err(InitializationError::WrongClockConfig);
    }

    crate::common_adapter::enable_wifi_power_domain();

    setup_radio_isr();

    wifi_set_log_verbose();
    init_radio_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => return Err(InitializationError::General(error)),
    }

    Ok(Controller {
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
    /// An error from the Wi-Fi driver.
    #[cfg(feature = "wifi")]
    WifiError(WifiError),
    /// The current CPU clock frequency is too low.
    WrongClockConfig,
    /// Tried to initialize while interrupts are disabled.
    /// This is not supported.
    InterruptsDisabled,
    /// The scheduler is not initialized.
    SchedulerNotInitialized,
    #[cfg(esp32)]
    /// ADC2 is required by esp-radio, but it is in use by esp-hal.
    Adc2IsUsed,
}

impl core::error::Error for InitializationError {}

impl core::fmt::Display for InitializationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InitializationError::General(e) => write!(f, "A general error {e} occurred"),
            #[cfg(feature = "wifi")]
            InitializationError::WifiError(e) => {
                write!(f, "Wi-Fi driver related error occured: {e}")
            }
            InitializationError::WrongClockConfig => {
                write!(f, "The current CPU clock frequency is too low")
            }
            InitializationError::InterruptsDisabled => write!(
                f,
                "Attempted to initialize while interrupts are disabled (Unsupported)"
            ),
            InitializationError::SchedulerNotInitialized => {
                write!(f, "The scheduler is not initialized")
            }
            #[cfg(esp32)]
            InitializationError::Adc2IsUsed => write!(
                f,
                "ADC2 cannot be used with `radio` functionality on `esp32`"
            ),
        }
    }
}

#[cfg(feature = "wifi")]
impl From<WifiError> for InitializationError {
    fn from(value: WifiError) -> Self {
        InitializationError::WifiError(value)
    }
}

/// Enable verbose logging within the Wi-Fi driver
/// Does nothing unless the `sys-logs` feature is enabled.
#[instability::unstable]
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
