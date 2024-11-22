#![cfg_attr(
    docsrs,
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.esp-rs.org/esp-hal/'>browse the <code>esp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! # Bare-metal (`no_std`) HAL for all Espressif ESP32 devices.
//!
//! ## Overview
//! The HAL implements both blocking _and_ async
//! APIs for many peripherals. Where applicable, driver implement
//! the [embedded-hal] and [embedded-hal-async] traits.
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
//! // You'll need a panic handler e.g. `use esp_backtrace as _;`
//! # #[panic_handler]
//! # fn panic(_ : &core::panic::PanicInfo) -> ! {
//! #     loop {}
//! # }
//! use esp_hal::{
//!     delay::Delay,
//!     gpio::{Io, Level, Output},
//!     prelude::*,
//! };
//!
//! #[entry]
//! fn main() -> ! {
//!     let peripherals = esp_hal::init({
//!         let mut config = esp_hal::Config::default();
//!         // Configure the CPU to run at the maximum frequency.
//!         config.cpu_clock = CpuClock::max();
//!         config
//!     });
//!
//!     // Set GPIO0 as an output, and set its state high initially.
//!     let mut led = Output::new(peripherals.GPIO0, Level::High);
//!
//!     let delay = Delay::new();
//!
//!     loop {
//!         led.toggle();
//!         delay.delay_millis(1000);
//!     }
//! }
//! ```
//!
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Below is a table of tunable parameters
//! for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_hal_config_table.md"))]
#![doc = ""]
//! It's important to note that due to a [bug in cargo](https://github.com/rust-lang/cargo/issues/10358),
//! any modifications to the environment, local or otherwise will only get
//! picked up on a full clean build of the project.
//!
//! ## `Peripheral` Pattern
//!
//! Drivers take pins and peripherals as [peripheral::Peripheral] in most
//! circumstances. This means you can pass the pin/peripheral or a mutable
//! reference to the pin/peripheral.
//!
//! The latter can be used to regain access to the pin when the driver gets
//! dropped. Then it's possible to reuse the pin/peripheral for a different
//! purpose.
//!
//! ## Don't use `core::mem::forget`
//!
//! You should never use `core::mem::forget` on any type defined in the HAL.
//! Some types heavily rely on their `Drop` implementation to not leave the
//! hardware in undefined state and causing UB.
//!
//! You might want to consider using [`#[deny(clippy::mem_forget)`](https://rust-lang.github.io/rust-clippy/v0.0.212/index.html#mem_forget) in your project.
//!
//! [documentation]: https://docs.esp-rs.org/esp-hal
//! [examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [xtask]: https://github.com/matklad/cargo-xtask
//! [esp-generate]: https://github.com/esp-rs/esp-generate
//! [book]: https://docs.esp-rs.org/book/
//! [training]: https://docs.esp-rs.org/no_std-training/
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register, async_fn_in_trait, stable_features)]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![deny(missing_docs, rust_2018_idioms)]
#![no_std]

// MUST be the first module
mod fmt;

#[cfg(riscv)]
pub use esp_riscv_rt::{self, entry, riscv};
#[cfg(xtensa)]
pub use xtensa_lx;
#[cfg(xtensa)]
pub use xtensa_lx_rt::{self, entry};

#[cfg(any(esp32, esp32s3))]
pub use self::soc::cpu_control;
#[cfg(efuse)]
pub use self::soc::efuse;
#[cfg(lp_core)]
pub use self::soc::lp_core;
pub use self::soc::peripherals;
#[cfg(any(feature = "quad-psram", feature = "octal-psram"))]
pub use self::soc::psram;
#[cfg(ulp_riscv_core)]
pub use self::soc::ulp_core;

#[cfg(aes)]
pub mod aes;
#[cfg(any(adc, dac))]
pub mod analog;
#[cfg(assist_debug)]
pub mod assist_debug;
#[cfg(any(dport, hp_sys, pcr, system))]
pub mod clock;

pub mod config;

#[cfg(any(xtensa, all(riscv, systimer)))]
pub mod delay;
#[cfg(any(gdma, pdma))]
pub mod dma;
#[cfg(ecc)]
pub mod ecc;
#[cfg(soc_etm)]
pub mod etm;
#[cfg(gpio)]
pub mod gpio;
#[cfg(hmac)]
pub mod hmac;
#[cfg(any(i2c0, i2c1))]
pub mod i2c;
#[cfg(any(i2s0, i2s1))]
pub mod i2s;
#[cfg(any(dport, interrupt_core0, interrupt_core1))]
pub mod interrupt;
#[cfg(lcd_cam)]
pub mod lcd_cam;
#[cfg(ledc)]
pub mod ledc;
#[cfg(any(mcpwm0, mcpwm1))]
pub mod mcpwm;
#[cfg(usb0)]
pub mod otg_fs;
#[cfg(parl_io)]
pub mod parl_io;
#[cfg(pcnt)]
pub mod pcnt;
pub mod peripheral;
pub mod prelude;
#[cfg(any(hmac, sha))]
mod reg_access;
#[cfg(any(lp_clkrst, rtc_cntl))]
pub mod reset;
#[cfg(rmt)]
pub mod rmt;
#[cfg(rng)]
pub mod rng;
pub mod rom;
#[cfg(rsa)]
pub mod rsa;
#[cfg(any(lp_clkrst, rtc_cntl))]
pub mod rtc_cntl;
#[cfg(sha)]
pub mod sha;
#[cfg(any(spi0, spi1, spi2, spi3))]
pub mod spi;
#[cfg(any(dport, hp_sys, pcr, system))]
pub mod system;
pub mod time;
#[cfg(any(systimer, timg0, timg1))]
pub mod timer;
#[cfg(touch)]
pub mod touch;
#[cfg(trace0)]
pub mod trace;
#[cfg(any(twai0, twai1))]
pub mod twai;
#[cfg(any(uart0, uart1, uart2))]
pub mod uart;
#[cfg(usb_device)]
pub mod usb_serial_jtag;

pub mod debugger;

#[doc(hidden)]
pub mod sync;

pub mod macros;

/// State of the CPU saved when entering exception or interrupt
pub mod trapframe {
    #[cfg(riscv)]
    pub use esp_riscv_rt::TrapFrame;
    #[cfg(xtensa)]
    pub use xtensa_lx_rt::exception::Context as TrapFrame;
}

// The `soc` module contains chip-specific implementation details and should not
// be directly exposed.
mod soc;

#[cfg(xtensa)]
#[no_mangle]
extern "C" fn EspDefaultHandler(_level: u32, _interrupt: peripherals::Interrupt) {
    panic!("Unhandled level {} interrupt: {:?}", _level, _interrupt);
}

#[cfg(riscv)]
#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: peripherals::Interrupt) {
    panic!("Unhandled interrupt: {:?}", _interrupt);
}

/// A marker trait for initializing drivers in a specific mode.
pub trait Mode: crate::private::Sealed {}

/// Driver initialized in blocking mode.
#[derive(Debug)]
pub struct Blocking;

/// Driver initialized in async mode.
#[derive(Debug)]
pub struct Async;

impl crate::Mode for Blocking {}
impl crate::Mode for Async {}
impl crate::private::Sealed for Blocking {}
impl crate::private::Sealed for Async {}

pub(crate) mod private {
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
}

#[doc(hidden)]
pub use private::Internal;

/// Marker trait for types that can be safely used in `#[ram(persistent)]`.
///
/// # Safety
///
/// - The type must be inhabited
/// - The type must be valid for any bit pattern of its backing memory in case a
///   reset occurs during a write or a reset interrupts the zero initialization
///   on first boot.
/// - Structs must contain only `Persistable` fields and padding
pub unsafe trait Persistable: Sized {}

macro_rules! impl_persistable {
    ($($t:ty),+) => {$(
        unsafe impl Persistable for $t {}
    )+};
    (atomic $($t:ident),+) => {$(
        unsafe impl Persistable for portable_atomic::$t {}
    )+};
}

impl_persistable!(u8, i8, u16, i16, u32, i32, u64, i64, u128, i128, usize, isize, f32, f64);
impl_persistable!(atomic AtomicU8, AtomicI8, AtomicU16, AtomicI16, AtomicU32, AtomicI32, AtomicUsize, AtomicIsize);

unsafe impl<T: Persistable, const N: usize> Persistable for [T; N] {}

#[doc(hidden)]
pub mod __macro_implementation {
    //! Unstable private implementation details of esp-hal-procmacros.

    pub const fn assert_is_zeroable<T: bytemuck::Zeroable>() {}
    pub const fn assert_is_persistable<T: super::Persistable>() {}
}

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, Copy, Clone, PartialEq, Eq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu = 1,
}

impl Cpu {
    /// The number of available cores.
    pub const COUNT: usize = 1 + cfg!(multi_core) as usize;

    /// Returns the core the application is currently executing on
    #[inline(always)]
    pub fn current() -> Self {
        // This works for both RISCV and Xtensa because both
        // get_raw_core functions return zero, _or_ something
        // greater than zero; 1 in the case of RISCV and 0x2000
        // in the case of Xtensa.
        match raw_core() {
            0 => Cpu::ProCpu,
            #[cfg(all(multi_core, riscv))]
            1 => Cpu::AppCpu,
            #[cfg(all(multi_core, xtensa))]
            0x2000 => Cpu::AppCpu,
            _ => unreachable!(),
        }
    }

    /// Returns an iterator over the "other" cores.
    #[inline(always)]
    pub(crate) fn other() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                match Self::current() {
                    Cpu::ProCpu => [Cpu::AppCpu].into_iter(),
                    Cpu::AppCpu => [Cpu::ProCpu].into_iter(),
                }
            } else {
                [].into_iter()
            }
        }
    }

    /// Returns an iterator over all cores.
    #[inline(always)]
    pub(crate) fn all() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                [Cpu::ProCpu, Cpu::AppCpu].into_iter()
            } else {
                [Cpu::ProCpu].into_iter()
            }
        }
    }
}

/// Returns the raw value of the mhartid register.
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(riscv)]
#[inline(always)]
fn raw_core() -> usize {
    #[cfg(multi_core)]
    {
        riscv::register::mhartid::read()
    }

    #[cfg(not(multi_core))]
    0
}

/// Returns the result of reading the PRID register logically ANDed with 0x2000,
/// the 13th bit in the register. Espressif Xtensa chips use this bit to
/// determine the core id.
///
/// Returns either 0 or 0x2000
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(xtensa)]
#[inline(always)]
fn raw_core() -> usize {
    (xtensa_lx::get_processor_id() & 0x2000) as usize
}

/// Default (unhandled) interrupt handler
pub const DEFAULT_INTERRUPT_HANDLER: interrupt::InterruptHandler = interrupt::InterruptHandler::new(
    unsafe { core::mem::transmute::<*const (), extern "C" fn()>(EspDefaultHandler as *const ()) },
    crate::interrupt::Priority::min(),
);

/// Trait implemented by drivers which allow the user to set an
/// [interrupt::InterruptHandler]
pub trait InterruptConfigurable: private::Sealed {
    /// Set the interrupt handler
    ///
    /// Note that this will replace any previously registered interrupt handler.
    /// Some peripherals offer a shared interrupt handler for multiple purposes.
    /// It's the users duty to honor this.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [DEFAULT_INTERRUPT_HANDLER]
    fn set_interrupt_handler(&mut self, handler: interrupt::InterruptHandler);
}

#[cfg(riscv)]
#[export_name = "hal_main"]
fn hal_main(a0: usize, a1: usize, a2: usize) -> ! {
    extern "Rust" {
        // This symbol will be provided by the user via `#[entry]`
        fn main(a0: usize, a1: usize, a2: usize) -> !;
    }

    extern "C" {
        static mut __stack_chk_guard: u32;
    }

    unsafe {
        let stack_chk_guard = core::ptr::addr_of_mut!(__stack_chk_guard);
        // we _should_ use a random value but we don't have a good source for random
        // numbers here
        stack_chk_guard.write_volatile(0xdeadbabe);

        main(a0, a1, a2);
    }
}

#[export_name = "__stack_chk_fail"]
unsafe extern "C" fn stack_chk_fail() {
    panic!("Stack corruption detected");
}

use crate::{
    clock::{Clocks, CpuClock},
    config::{WatchdogConfig, WatchdogStatus},
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
#[derive(Default)]
pub struct Config {
    /// The CPU clock configuration.
    pub cpu_clock: CpuClock,
    /// Enable watchdog timer(s).
    pub watchdog: WatchdogConfig,
}

/// Initialize the system.
///
/// This function sets up the CPU clock and watchdog, then, returns the
/// peripherals and clocks.
pub fn init(config: Config) -> Peripherals {
    system::disable_peripherals();

    let mut peripherals = Peripherals::take();

    // RTC domain must be enabled before we try to disable
    let mut rtc = crate::rtc_cntl::Rtc::new(&mut peripherals.LPWR);

    #[cfg(not(any(esp32, esp32s2)))]
    if config.watchdog.swd {
        rtc.swd.enable();
    } else {
        rtc.swd.disable();
    }

    match config.watchdog.rwdt {
        WatchdogStatus::Enabled(duration) => {
            rtc.rwdt.enable();
            rtc.rwdt
                .set_timeout(crate::rtc_cntl::RwdtStage::Stage0, duration);
        }
        WatchdogStatus::Disabled => {
            rtc.rwdt.disable();
        }
    }

    match config.watchdog.timg0 {
        WatchdogStatus::Enabled(duration) => {
            let mut timg0_wd = crate::timer::timg::Wdt::<self::peripherals::TIMG0>::new();
            timg0_wd.enable();
            timg0_wd.set_timeout(crate::timer::timg::MwdtStage::Stage0, duration);
        }
        WatchdogStatus::Disabled => {
            crate::timer::timg::Wdt::<self::peripherals::TIMG0>::new().disable();
        }
    }

    #[cfg(timg1)]
    match config.watchdog.timg1 {
        WatchdogStatus::Enabled(duration) => {
            let mut timg1_wd = crate::timer::timg::Wdt::<self::peripherals::TIMG1>::new();
            timg1_wd.enable();
            timg1_wd.set_timeout(crate::timer::timg::MwdtStage::Stage0, duration);
        }
        WatchdogStatus::Disabled => {
            crate::timer::timg::Wdt::<self::peripherals::TIMG1>::new().disable();
        }
    }

    Clocks::init(config.cpu_clock);

    #[cfg(esp32)]
    crate::time::time_init();

    crate::gpio::bind_default_interrupt_handler();

    peripherals
}

/// Asynchronous utilities.
pub mod asynch {
    use core::{cell::Cell, task::Waker};

    use embassy_sync::blocking_mutex::Mutex;

    use crate::sync::RawMutex;

    /// TODO: this just exists to test RawMutex, otherwise it should be replaced
    /// by embassy_sync::waitqueue::atomic_waker::GenericAtomicWaker.
    pub struct AtomicWaker {
        waker: Mutex<RawMutex, Cell<Option<Waker>>>,
    }

    impl AtomicWaker {
        /// Create a new `AtomicWaker`.
        #[allow(clippy::new_without_default)]
        pub const fn new() -> Self {
            Self {
                waker: Mutex::const_new(RawMutex::new(), Cell::new(None)),
            }
        }

        /// Register a waker. Overwrites the previous waker, if any.
        pub fn register(&self, w: &Waker) {
            self.waker.lock(|cell| {
                cell.set(match cell.replace(None) {
                    Some(w2) if (w2.will_wake(w)) => Some(w2),
                    _ => Some(w.clone()),
                })
            })
        }

        /// Wake the registered waker, if any.
        pub fn wake(&self) {
            self.waker.lock(|cell| {
                if let Some(w) = cell.replace(None) {
                    w.wake_by_ref();
                    cell.set(Some(w));
                }
            })
        }
    }
}
