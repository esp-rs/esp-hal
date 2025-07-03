#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! # Bare-metal (`no_std`) HAL for all Espressif ESP32 devices.
//!
//! This documentation is built for the
#![cfg_attr(esp32, doc = "**ESP32**")]
#![cfg_attr(esp32s2, doc = "**ESP32-S2**")]
#![cfg_attr(esp32s3, doc = "**ESP32-S3**")]
#![cfg_attr(esp32c2, doc = "**ESP32-C2**")]
#![cfg_attr(esp32c3, doc = "**ESP32-C3**")]
#![cfg_attr(esp32c6, doc = "**ESP32-C6**")]
#![cfg_attr(esp32h2, doc = "**ESP32-H2**")]
//! . Please ensure you are reading the correct [documentation] for your target
//! device.
//!
//! ## Choosing a Device
//!
//! Depending on your target device, you need to enable the chip feature
//! for that device. You may also need to do this on ancillary esp-hal crates.
//!
//! ## Overview
//!
//! ### Peripheral drivers
//!
//! The HAL implements both blocking _and_ async APIs for many peripherals.
//! Where applicable, driver implement the [embedded-hal] and
//! [embedded-hal-async] traits.
//!
//! ### Peripheral singletons
//!
//! Each peripheral driver needs a peripheral singleton that tells the driver
//! which hardware block to use. The peripheral singletons are created by the
//! HAL initialization, and are returned from [`init`] as fields of the
//! [`Peripherals`] struct.
//!
//! These singletons, by default, represent peripherals for the entire lifetime
//! of the program. To allow for reusing peripherals, the HAL provides a
//! `reborrow` method on each peripheral singleton. This method creates a new
//! handle to the peripheral with a shorter lifetime. This allows you to pass
//! the handle to a driver, while still keeping the original handle alive. Once
//! you drop the driver, you will be able to reborrow the peripheral again.
//!
//! For example, if you want to use the [`I2c`](i2c::master::I2c) driver and you
//! don't intend to drop the driver, you can pass the peripheral singleton to
//! the driver by value:
//!
//! ```rust, ignore
//! // Peripheral singletons are returned from the `init` function.
//! let peripherals = esp_hal::init(esp_hal::Config::default());
//!
//! let mut i2c = I2C::new(peripherals.I2C0, /* ... */);
//! ```
//!
//! If you want to use the peripheral in multiple places (for example, you want
//! to drop the driver for some period of time to minimize power consumption),
//! you can reborrow the peripheral singleton and pass it to the driver by
//! reference:
//!
//! ```rust, ignore
//! // Note that in this case, `peripherals` needs to be mutable.
//! let mut peripherals = esp_hal::init(esp_hal::Config::default());
//!
//! let i2c = I2C::new(peripherals.I2C0.reborrow(), /* ... */);
//!
//! // Do something with the I2C driver...
//!
//! core::mem::drop(i2c); // Drop the driver to minimize power consumption.
//!
//! // Do something else...
//!
//! // You can then take or reborrow the peripheral singleton again.
//! let i2c = I2C::new(peripherals.I2C0.reborrow(), /* ... */);
//! ```
//!
//! ## Examples
//!
//! We have a plethora of [examples] in the esp-hal repository. We use
//! an [xtask] to automate the building, running, and testing of code and
//! examples within esp-hal.
//!
//! Invoke the following command in the root of the esp-hal repository to get
//! started:
//!
//! ```bash
//! cargo xtask help
//! ```
//!
//! ## Creating a Project
//!
//! We have a [book] that explains the full esp-rs ecosystem
//! and how to get started, it's advisable to give that a read
//! before proceeding. We also have a [training] that covers some common
//! scenarios with examples.
//!
//! We have developed a project generation tool, [esp-generate], which we
//! recommend when starting new projects. It can be installed and run, e.g.
//! for the ESP32-C6, as follows:
//!
//! ```bash
//! cargo install esp-generate
//! esp-generate --chip=esp32c6 your-project
//! ```
//!
//! ## Blinky
//!
//! Some minimal code to blink an LED looks like this:
//!
//! ```rust, no_run
//! #![no_std]
//! #![no_main]
//!
//! use esp_hal::{
//!     clock::CpuClock,
//!     gpio::{Io, Level, Output, OutputConfig},
//!     main,
//!     time::{Duration, Instant},
//! };
//!
//! // You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
//! // something similar, but you can also bring your own like this:
//! #[panic_handler]
//! fn panic(_: &core::panic::PanicInfo) -> ! {
//!     esp_hal::system::software_reset()
//! }
//!
//! #[main]
//! fn main() -> ! {
//!     let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//!     let peripherals = esp_hal::init(config);
//!
//!     // Set GPIO0 as an output, and set its state high initially.
//!     let mut led = Output::new(peripherals.GPIO0, Level::High, OutputConfig::default());
//!
//!     loop {
//!         led.toggle();
//!         // Wait for half a second
//!         let delay_start = Instant::now();
//!         while delay_start.elapsed() < Duration::from_millis(500) {}
//!     }
//! }
//! ```
//!
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Note that unstable options can only be
//! enabled when the `unstable` feature is enabled for the crate. Below is a
//! table of tunable parameters for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_hal_config_table.md"))]
#![doc = ""]
//! ## Don't use `core::mem::forget`
//!
//! You should never use `core::mem::forget` on any type defined in the HAL.
//! Some types heavily rely on their `Drop` implementation to not leave the
//! hardware in undefined state and causing UB.
//!
//! You might want to consider using [`#[deny(clippy::mem_forget)`](https://rust-lang.github.io/rust-clippy/v0.0.212/index.html#mem_forget) in your project.
//!
//! [documentation]: https://docs.espressif.com/projects/rust/esp-hal/latest/
//! [examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [xtask]: https://github.com/matklad/cargo-xtask
//! [esp-generate]: https://github.com/esp-rs/esp-generate
//! [book]: https://docs.espressif.com/projects/rust/book/
//! [training]: https://docs.espressif.com/projects/rust/no_std-training/
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register, async_fn_in_trait, stable_features)]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]
#![allow(rustdoc::private_doc_tests)] // compile tests are done via rustdoc
#![cfg_attr(docsrs, feature(doc_cfg))]
#![no_std]

// MUST be the first module
mod fmt;

#[macro_use]
/// This module contains code generated by `esp-metadata`.
///
/// The main API is the [`property!`] macro, which takes a string key and
/// returns a value. This macro is available anywhere inside esp-hal, without
/// additional imports. Note that this macro can't be used in public macros, as
/// they will fail to compile.
mod _generated {
    // This must go before other code, so that they can see macros.
    // Generated by esp-metadata.
    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

use core::marker::PhantomData;

use esp_rom_sys as _;

metadata!("build_info", CHIP_NAME, chip!());

#[cfg(riscv)]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[cfg_attr(not(feature = "unstable"), doc(hidden))]
pub use esp_riscv_rt::{self, riscv};
#[cfg(xtensa)]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[cfg_attr(not(feature = "unstable"), doc(hidden))]
pub use xtensa_lx_rt::{self, xtensa_lx};

#[cfg(soc_has_efuse)]
#[instability::unstable]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub use self::soc::efuse;
#[cfg(lp_core)]
#[instability::unstable]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub use self::soc::lp_core;
pub use self::soc::peripherals;
pub(crate) use self::soc::peripherals::pac;
#[instability::unstable]
#[cfg(feature = "psram")]
pub use self::soc::psram;
#[cfg(ulp_riscv_core)]
#[instability::unstable]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub use self::soc::ulp_core;

#[cfg(any(soc_has_dport, soc_has_hp_sys, soc_has_pcr, soc_has_system))]
pub mod clock;
#[cfg(soc_has_gpio)]
pub mod gpio;
#[cfg(any(soc_has_i2c0, soc_has_i2c1))]
pub mod i2c;
#[cfg(all(feature = "unstable", any(soc_has_hmac, soc_has_sha)))]
mod reg_access;
#[cfg(any(soc_has_spi0, soc_has_spi1, soc_has_spi2, soc_has_spi3))]
pub mod spi;
pub mod system;
pub mod time;
#[cfg(any(soc_has_uart0, soc_has_uart1, soc_has_uart2))]
pub mod uart;

mod macros;
mod peripheral;

pub use procmacros::blocking_main as main;
#[cfg(any(lp_core, ulp_riscv_core))]
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
pub use procmacros::load_lp_code;
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[instability::unstable]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub use procmacros::{handler, ram};

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

// can't use instability on inline module definitions, see https://github.com/rust-lang/rust/issues/54727
// we don't want unstable drivers to be compiled even, unless enabled
#[doc(hidden)]
macro_rules! unstable_driver {
    ($(
        $(#[$meta:meta])*
        pub mod $module:ident;
    )*) => {
        $(
            $(#[$meta])*
            #[cfg(feature = "unstable")]
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            pub mod $module;
        )*
    };
}

pub(crate) use unstable_driver;
pub(crate) use unstable_module;

unstable_module! {
    pub mod asynch;
    pub mod config;
    pub mod debugger;
    #[cfg(any(soc_has_dport, soc_has_interrupt_core0, soc_has_interrupt_core1))]
    pub mod interrupt;
    pub mod rom;
    #[doc(hidden)]
    pub mod sync;
    // Drivers needed for initialization or they are tightly coupled to something else.
    #[cfg(any(adc, dac))]
    pub mod analog;
    #[cfg(any(systimer, timergroup))]
    pub mod timer;
    #[cfg(soc_has_lpwr)]
    pub mod rtc_cntl;
    #[cfg(any(gdma, pdma))]
    pub mod dma;
    #[cfg(soc_has_etm)]
    pub mod etm;
    #[cfg(soc_has_usb0)]
    pub mod otg_fs;
}

unstable_driver! {
    #[cfg(soc_has_aes)]
    pub mod aes;
    #[cfg(soc_has_assist_debug)]
    pub mod assist_debug;
    pub mod delay;
    #[cfg(soc_has_ecc)]
    pub mod ecc;
    #[cfg(soc_has_hmac)]
    pub mod hmac;
    #[cfg(any(soc_has_i2s0, soc_has_i2s1))]
    pub mod i2s;
    #[cfg(soc_has_lcd_cam)]
    pub mod lcd_cam;
    #[cfg(soc_has_ledc)]
    pub mod ledc;
    #[cfg(any(soc_has_mcpwm0, soc_has_mcpwm1))]
    pub mod mcpwm;
    #[cfg(soc_has_parl_io)]
    pub mod parl_io;
    #[cfg(soc_has_pcnt)]
    pub mod pcnt;
    #[cfg(soc_has_rmt)]
    pub mod rmt;
    #[cfg(soc_has_rng)]
    pub mod rng;
    #[cfg(soc_has_rsa)]
    pub mod rsa;
    #[cfg(soc_has_sha)]
    pub mod sha;
    #[cfg(touch)]
    pub mod touch;
    #[cfg(soc_has_trace0)]
    pub mod trace;
    #[cfg(soc_has_tsens)]
    pub mod tsens;
    #[cfg(any(soc_has_twai0, soc_has_twai1))]
    pub mod twai;
    #[cfg(soc_has_usb_device)]
    pub mod usb_serial_jtag;
}

/// State of the CPU saved when entering exception or interrupt
#[instability::unstable]
#[allow(unused_imports)]
pub mod trapframe {
    #[cfg(riscv)]
    pub use esp_riscv_rt::TrapFrame;
    #[cfg(xtensa)]
    pub use xtensa_lx_rt::exception::Context as TrapFrame;
}

// The `soc` module contains chip-specific implementation details and should not
// be directly exposed.
mod soc;

#[cfg(is_debug_build)]
procmacros::warning! {"
WARNING: use --release
  We *strongly* recommend using release profile when building esp-hal.
  The dev profile can potentially be one or more orders of magnitude
  slower than release, and may cause issues with timing-senstive
  peripherals and/or devices.
"}

/// A marker trait for driver modes.
///
/// Different driver modes offer different features and different API. Using
/// this trait as a generic parameter ensures that the driver is initialized in
/// the correct mode.
pub trait DriverMode: crate::private::Sealed {}

/// Marker type signalling that a driver is initialized in blocking mode.
///
/// Drivers are constructed in blocking mode by default. To learn about the
/// differences between blocking and async drivers, see the [`Async`] mode
/// documentation.
///
/// [`Async`] drivers can be converted to a [`Blocking`] driver using the
/// `into_blocking` method, for example:
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::uart::{Config, Uart};
/// let uart = Uart::new(
///     peripherals.UART0,
///     Config::default())?
/// .with_rx(peripherals.GPIO1)
/// .with_tx(peripherals.GPIO2)
/// .into_async();
///
/// let blocking_uart = uart.into_blocking();
///
/// # Ok(())
/// # }
/// ```
#[derive(Debug)]
#[non_exhaustive]
pub struct Blocking;

/// Marker type signalling that a driver is initialized in async mode.
///
/// Drivers are constructed in blocking mode by default. To set up an async
/// driver, a [`Blocking`] driver must be converted to an `Async` driver using
/// the `into_async` method, for example:
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::uart::{Config, Uart};
/// let uart = Uart::new(
///     peripherals.UART0,
///     Config::default())?
/// .with_rx(peripherals.GPIO1)
/// .with_tx(peripherals.GPIO2)
/// .into_async();
///
/// # Ok(())
/// # }
/// ```
/// 
/// Drivers can be converted back to blocking mode using the `into_blocking`
/// method, see [`Blocking`] documentation for more details.
///
/// Async mode drivers offer most of the same features as blocking drivers, but
/// with the addition of async APIs. Interrupt-related functions are not
/// available in async mode, as they are handled by the driver's interrupt
/// handlers.
///
/// Note that async functions usually take up more space than their blocking
/// counterparts, and they are generally slower. This is because async functions
/// are implemented using a state machine that is driven by interrupts and is
/// polled by a runtime. For short operations, the overhead of the state machine
/// can be significant. Consider using the blocking functions on the async
/// driver for small transfers.
///
/// When initializing an async driver, the driver disables user-specified
/// interrupt handlers, and sets up internal interrupt handlers that drive the
/// driver's async API. The driver's interrupt handlers run on the same core as
/// the driver was initialized on. This means that the driver can not be sent
/// across threads, to prevent incorrect concurrent access to the peripheral.
///
/// Switching back to blocking mode will disable the interrupt handlers and
/// return the driver to a state where it can be sent across threads.
#[derive(Debug)]
#[non_exhaustive]
pub struct Async(PhantomData<*const ()>);

unsafe impl Sync for Async {}

impl crate::DriverMode for Blocking {}
impl crate::DriverMode for Async {}
impl crate::private::Sealed for Blocking {}
impl crate::private::Sealed for Async {}

pub(crate) mod private {
    use core::mem::ManuallyDrop;

    pub trait Sealed {}

    #[non_exhaustive]
    #[doc(hidden)]
    /// Magical incantation to gain access to internal APIs.
    pub struct Internal;

    impl Internal {
        /// Obtain magical powers to access internal APIs.
        ///
        /// # Safety
        ///
        /// By calling this function, you accept that you are using an internal
        /// API that is not guaranteed to be documented, stable, working
        /// and may change at any time.
        ///
        /// You declare that you have tried to look for other solutions, that
        /// you have opened a feature request or an issue to discuss the
        /// need for this function.
        pub unsafe fn conjure() -> Self {
            Self
        }
    }

    pub(crate) struct OnDrop<F: FnOnce()>(ManuallyDrop<F>);
    impl<F: FnOnce()> OnDrop<F> {
        pub fn new(cb: F) -> Self {
            Self(ManuallyDrop::new(cb))
        }

        pub fn defuse(self) {
            core::mem::forget(self);
        }
    }

    impl<F: FnOnce()> Drop for OnDrop<F> {
        fn drop(&mut self) {
            unsafe { (ManuallyDrop::take(&mut self.0))() }
        }
    }
}

#[cfg(feature = "unstable")]
#[doc(hidden)]
pub use private::Internal;

/// Marker trait for types that can be safely used in `#[ram(persistent)]`.
///
/// # Safety
///
/// - The type must be inhabited
/// - The type must be valid for any bit pattern of its backing memory in case a reset occurs during
///   a write or a reset interrupts the zero initialization on first boot.
/// - Structs must contain only `Persistable` fields and padding
#[instability::unstable]
pub unsafe trait Persistable: Sized {}

macro_rules! impl_persistable {
    ($($t:ty),+) => {$(
        unsafe impl Persistable for $t {}
    )+};
    (atomic $($t:ident),+) => {$(
        unsafe impl Persistable for portable_atomic::$t {}
    )+};
}

impl_persistable!(
    u8, i8, u16, i16, u32, i32, u64, i64, u128, i128, usize, isize, f32, f64
);
impl_persistable!(atomic AtomicU8, AtomicI8, AtomicU16, AtomicI16, AtomicU32, AtomicI32, AtomicUsize, AtomicIsize);

unsafe impl<T: Persistable, const N: usize> Persistable for [T; N] {}

#[doc(hidden)]
pub mod __macro_implementation {
    //! Private implementation details of esp-hal-procmacros.

    #[instability::unstable]
    pub const fn assert_is_zeroable<T: bytemuck::Zeroable>() {}

    #[instability::unstable]
    pub const fn assert_is_persistable<T: super::Persistable>() {}

    #[cfg(riscv)]
    pub use esp_riscv_rt::entry as __entry;
    #[cfg(xtensa)]
    pub use xtensa_lx_rt::entry as __entry;
}

#[cfg(feature = "unstable")]
use crate::config::{WatchdogConfig, WatchdogStatus};
use crate::{
    clock::{Clocks, CpuClock},
    peripherals::Peripherals,
};

/// System configuration.
///
/// This `struct` is marked with `#[non_exhaustive]` and can't be instantiated
/// directly. This is done to prevent breaking changes when new fields are added
/// to the `struct`. Instead, use the [`Config::default()`] method to create a
/// new instance.
///
/// For usage examples, see the [config module documentation](crate::config).
#[non_exhaustive]
#[derive(Default, Clone, Copy, procmacros::BuilderLite)]
pub struct Config {
    /// The CPU clock configuration.
    cpu_clock: CpuClock,

    /// Enable watchdog timer(s).
    #[cfg(feature = "unstable")]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    #[builder_lite(unstable)]
    watchdog: WatchdogConfig,

    /// PSRAM configuration.
    #[cfg(feature = "unstable")]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    #[cfg(feature = "psram")]
    #[builder_lite(unstable)]
    psram: psram::PsramConfig,
}

/// Initialize the system.
///
/// This function sets up the CPU clock and watchdog, then, returns the
/// peripherals and clocks.
pub fn init(config: Config) -> Peripherals {
    crate::soc::pre_init();

    system::disable_peripherals();

    let mut peripherals = Peripherals::take();

    // RTC domain must be enabled before we try to disable
    let mut rtc = crate::rtc_cntl::Rtc::new(peripherals.LPWR.reborrow());

    // Handle watchdog configuration with defaults
    cfg_if::cfg_if! {
        if #[cfg(feature = "unstable")]
        {
            #[cfg(not(any(esp32, esp32s2)))]
            if config.watchdog.swd() {
                rtc.swd.enable();
            } else {
                rtc.swd.disable();
            }

            match config.watchdog.rwdt() {
                WatchdogStatus::Enabled(duration) => {
                    rtc.rwdt.enable();
                    rtc.rwdt
                        .set_timeout(crate::rtc_cntl::RwdtStage::Stage0, duration);
                }
                WatchdogStatus::Disabled => {
                    rtc.rwdt.disable();
                }
            }

            #[cfg(timergroup_timg0)]
            match config.watchdog.timg0() {
                WatchdogStatus::Enabled(duration) => {
                    let mut timg0_wd = crate::timer::timg::Wdt::<crate::peripherals::TIMG0<'static>>::new();
                    timg0_wd.enable();
                    timg0_wd.set_timeout(crate::timer::timg::MwdtStage::Stage0, duration);
                }
                WatchdogStatus::Disabled => {
                    crate::timer::timg::Wdt::<crate::peripherals::TIMG0<'static>>::new().disable();
                }
            }

            #[cfg(timergroup_timg1)]
            match config.watchdog.timg1() {
                WatchdogStatus::Enabled(duration) => {
                    let mut timg1_wd = crate::timer::timg::Wdt::<crate::peripherals::TIMG1<'static>>::new();
                    timg1_wd.enable();
                    timg1_wd.set_timeout(crate::timer::timg::MwdtStage::Stage0, duration);
                }
                WatchdogStatus::Disabled => {
                    crate::timer::timg::Wdt::<crate::peripherals::TIMG1<'static>>::new().disable();
                }
            }
        }
        else
        {
            #[cfg(not(any(esp32, esp32s2)))]
            rtc.swd.disable();

            rtc.rwdt.disable();

            #[cfg(timergroup_timg0)]
            crate::timer::timg::Wdt::<crate::peripherals::TIMG0<'static>>::new().disable();

            #[cfg(timergroup_timg1)]
            crate::timer::timg::Wdt::<crate::peripherals::TIMG1<'static>>::new().disable();
        }
    }

    Clocks::init(config.cpu_clock);

    #[cfg(esp32)]
    crate::time::time_init();

    crate::gpio::interrupt::bind_default_interrupt_handler();

    #[cfg(feature = "psram")]
    crate::psram::init_psram(config.psram);

    peripherals
}
