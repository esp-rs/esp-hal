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
//! We have a template for quick starting bare-metal projects, [esp-template].
//! The template uses [cargo-generate], so ensure that it is installed and run:
//!
//! ```bash
//! cargo generate -a esp-rs/esp-template
//! ```
//!
//! ## Commonly Used Setup
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
//!     let peripherals = esp_hal::init(esp_hal::Config::default());
//!
//!     // Set GPIO0 as an output, and set its state high initially.
//!     let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!     let mut led = Output::new(io.pins.gpio0, Level::High);
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
//! The steps here are:
//! - Call [`init`] with the desired [`CpuClock`] configuration
//! - Create [`gpio::Io`] which provides access to the GPIO pins
//! - Create an [`gpio::Output`] pin driver which lets us control the logical
//!   level of an output pin
//! - Create a [`delay::Delay`] driver
//! - In a loop, toggle the output pin's logical level with a delay of 1000 ms
//!
//! ## `PeripheralRef` Pattern
//!
//! Generally drivers take pins and peripherals as [peripheral::PeripheralRef].
//! This means you can pass the pin/peripheral or a mutable reference to the
//! pin/peripheral.
//!
//! The latter can be used to regain access to the pin when the driver gets
//! dropped. Then it's possible to reuse the pin/peripheral for a different
//! purpose.
//!
//! ## Don't use [core::mem::forget]
//!
//! In general drivers are _NOT_ safe to use with [core::mem::forget]
//!
//! You should never use [core::mem::forget] on any type defined in the HAL.
//!
//! Some types heavily rely on their [Drop] implementation to not leave the
//! hardware in undefined state and causing UB.
//!
//! You might want to consider using [`#[deny(clippy::mem_forget)`](https://rust-lang.github.io/rust-clippy/v0.0.212/index.html#mem_forget) in your project.
//!
//! [documentation]: https://docs.esp-rs.org/esp-hal
//! [examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [xtask]: https://github.com/matklad/cargo-xtask
//! [esp-template]: https://github.com/esp-rs/esp-template
//! [cargo-generate]: https://github.com/cargo-generate/cargo-generate
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
pub use procmacros as macros;
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
#[cfg(psram)]
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
    #[cfg(not(feature = "defmt"))]
    panic!("Unhandled level {} interrupt: {:?}", _level, _interrupt);

    #[cfg(feature = "defmt")]
    panic!(
        "Unhandled level {} interrupt: {:?}",
        _level,
        defmt::Debug2Format(&_interrupt)
    );
}

#[cfg(riscv)]
#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: peripherals::Interrupt) {
    #[cfg(not(feature = "defmt"))]
    panic!("Unhandled interrupt: {:?}", _interrupt);

    #[cfg(feature = "defmt")]
    panic!(
        "Unhandled interrupt: {:?}",
        defmt::Debug2Format(&_interrupt)
    );
}

/// A marker trait for intializing drivers in a specific mode.
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

    pub struct Internal;
}

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

/// Which core the application is currently executing on
#[inline(always)]
pub fn get_core() -> Cpu {
    // This works for both RISCV and Xtensa because both
    // get_raw_core functions return zero, _or_ something
    // greater than zero; 1 in the case of RISCV and 0x2000
    // in the case of Xtensa.
    match get_raw_core() {
        0 => Cpu::ProCpu,
        #[cfg(all(multi_core, riscv))]
        1 => Cpu::AppCpu,
        #[cfg(all(multi_core, xtensa))]
        0x2000 => Cpu::AppCpu,
        _ => unreachable!(),
    }
}

/// Returns the raw value of the mhartid register.
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(riscv)]
#[inline(always)]
fn get_raw_core() -> usize {
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
fn get_raw_core() -> usize {
    (xtensa_lx::get_processor_id() & 0x2000) as usize
}

mod critical_section_impl {
    struct CriticalSection;

    critical_section::set_impl!(CriticalSection);

    #[cfg(xtensa)]
    mod xtensa {
        // PS has 15 useful bits. Bits 12..16 and 19..32 are unused, so we can use bit
        // #31 as our reentry flag.
        #[cfg(multi_core)]
        const REENTRY_FLAG: u32 = 1 << 31;

        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let mut tkn: critical_section::RawRestoreState;
                core::arch::asm!("rsil {0}, 5", out(reg) tkn);
                #[cfg(multi_core)]
                {
                    use super::multicore::{LockKind, MULTICORE_LOCK};

                    match MULTICORE_LOCK.lock() {
                        LockKind::Lock => {
                            // We can assume the reserved bit is 0 otherwise
                            // rsil - wsr pairings would be undefined behavior
                        }
                        LockKind::Reentry => tkn |= REENTRY_FLAG,
                    }
                }
                tkn
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                #[cfg(multi_core)]
                {
                    use super::multicore::MULTICORE_LOCK;

                    debug_assert!(MULTICORE_LOCK.is_owned_by_current_thread());

                    if token & REENTRY_FLAG != 0 {
                        return;
                    }

                    MULTICORE_LOCK.unlock();
                }

                const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;
                debug_assert!(token & RESERVED_MASK == 0);

                core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync", in(reg) token)
            }
        }
    }

    #[cfg(riscv)]
    mod riscv {
        // The restore state is a u8 that is casted from a bool, so it has a value of
        // 0x00 or 0x01 before we add the reentry flag to it.
        #[cfg(multi_core)]
        const REENTRY_FLAG: u8 = 1 << 7;

        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let mut mstatus = 0u32;
                core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);

                #[cfg_attr(single_core, allow(unused_mut))]
                let mut tkn = ((mstatus & 0b1000) != 0) as critical_section::RawRestoreState;

                #[cfg(multi_core)]
                {
                    use super::multicore::{LockKind, MULTICORE_LOCK};

                    match MULTICORE_LOCK.lock() {
                        LockKind::Lock => {}
                        LockKind::Reentry => tkn |= REENTRY_FLAG,
                    }
                }

                tkn
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                #[cfg(multi_core)]
                {
                    use super::multicore::MULTICORE_LOCK;

                    debug_assert!(MULTICORE_LOCK.is_owned_by_current_thread());

                    if token & REENTRY_FLAG != 0 {
                        return;
                    }

                    MULTICORE_LOCK.unlock();
                }

                if token != 0 {
                    riscv::interrupt::enable();
                }
            }
        }
    }

    #[cfg(multi_core)]
    mod multicore {
        use portable_atomic::{AtomicUsize, Ordering};

        // We're using a value that we know get_raw_core() will never return. This
        // avoids an unnecessary increment of the core ID.
        //
        // Safety: Ensure that when adding new chips get_raw_core doesn't return this
        // value. TODO when we have HIL tests ensure this is the case!
        const UNUSED_THREAD_ID_VALUE: usize = 0x100;

        fn thread_id() -> usize {
            crate::get_raw_core()
        }

        pub(super) static MULTICORE_LOCK: ReentrantMutex = ReentrantMutex::new();

        pub(super) enum LockKind {
            Lock = 0,
            Reentry,
        }

        pub(super) struct ReentrantMutex {
            owner: AtomicUsize,
        }

        impl ReentrantMutex {
            const fn new() -> Self {
                Self {
                    owner: AtomicUsize::new(UNUSED_THREAD_ID_VALUE),
                }
            }

            pub fn is_owned_by_current_thread(&self) -> bool {
                self.owner.load(Ordering::Relaxed) == thread_id()
            }

            pub(super) fn lock(&self) -> LockKind {
                let current_thread_id = thread_id();

                if self.try_lock(current_thread_id) {
                    return LockKind::Lock;
                }

                let current_owner = self.owner.load(Ordering::Relaxed);
                if current_owner == current_thread_id {
                    return LockKind::Reentry;
                }

                while !self.try_lock(current_thread_id) {}

                LockKind::Lock
            }

            fn try_lock(&self, new_owner: usize) -> bool {
                self.owner
                    .compare_exchange(
                        UNUSED_THREAD_ID_VALUE,
                        new_owner,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    )
                    .is_ok()
            }

            pub(super) fn unlock(&self) {
                self.owner.store(UNUSED_THREAD_ID_VALUE, Ordering::Release);
            }
        }
    }
}

// The state of a re-entrant lock
pub(crate) struct LockState {
    #[cfg(multi_core)]
    core: portable_atomic::AtomicUsize,
}

impl LockState {
    #[cfg(multi_core)]
    const UNLOCKED: usize = usize::MAX;

    pub const fn new() -> Self {
        Self {
            #[cfg(multi_core)]
            core: portable_atomic::AtomicUsize::new(Self::UNLOCKED),
        }
    }
}

// This is preferred over critical-section as this allows you to have multiple
// locks active at the same time rather than using the global mutex that is
// critical-section.
#[allow(unused_variables)]
pub(crate) fn lock<T>(state: &LockState, f: impl FnOnce() -> T) -> T {
    // In regards to disabling interrupts, we only need to disable
    // the interrupts that may be calling this function.

    #[cfg(not(multi_core))]
    {
        // Disabling interrupts is enough on single core chips to ensure mutual
        // exclusion.

        #[cfg(riscv)]
        return riscv::interrupt::free(f);
        #[cfg(xtensa)]
        return xtensa_lx::interrupt::free(|_| f());
    }

    #[cfg(multi_core)]
    {
        use portable_atomic::Ordering;

        let current_core = get_core() as usize;

        let mut f = f;

        loop {
            let func = || {
                // Use Acquire ordering in success to ensure `f()` "happens after" the lock is
                // taken. Use Relaxed ordering in failure as there's no
                // synchronisation happening.
                if let Err(locked_core) = state.core.compare_exchange(
                    LockState::UNLOCKED,
                    current_core,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                ) {
                    assert_ne!(
                        locked_core, current_core,
                        "esp_hal::lock is not re-entrant!"
                    );

                    Err(f)
                } else {
                    let result = f();

                    // Use Release ordering here to ensure `f()` "happens before" this lock is
                    // released.
                    state.core.store(LockState::UNLOCKED, Ordering::Release);

                    Ok(result)
                }
            };

            #[cfg(riscv)]
            let result = riscv::interrupt::free(func);
            #[cfg(xtensa)]
            let result = xtensa_lx::interrupt::free(|_| func());

            match result {
                Ok(result) => break result,
                Err(the_function) => f = the_function,
            }

            // Consider using core::hint::spin_loop(); Might need SW_INT.
        }
    }
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

#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use esp_hal::prelude::*;
# use procmacros::handler;
# use esp_hal::interrupt;
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#     let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}

use crate::{
    clock::{Clocks, CpuClock},
    peripherals::Peripherals,
};

/// System configuration.
#[non_exhaustive]
#[derive(Default)]
pub struct Config {
    /// The CPU clock configuration.
    pub cpu_clock: CpuClock,
}

/// Initialize the system.
///
/// This function sets up the CPU clock and returns the peripherals and clocks.
pub fn init(config: Config) -> Peripherals {
    let mut peripherals = Peripherals::take();

    Clocks::init(config.cpu_clock);

    #[cfg(xtensa)]
    crate::interrupt::setup_interrupts();
    #[cfg(esp32)]
    crate::time::time_init();

    // RTC domain must be enabled before we try to disable
    let mut rtc = crate::rtc_cntl::Rtc::new(&mut peripherals.LPWR);
    #[cfg(not(any(esp32, esp32s2)))]
    rtc.swd.disable();
    rtc.rwdt.disable();

    unsafe {
        crate::timer::timg::Wdt::<self::peripherals::TIMG0, Blocking>::set_wdt_enabled(false);
        #[cfg(timg1)]
        crate::timer::timg::Wdt::<self::peripherals::TIMG1, Blocking>::set_wdt_enabled(false);
    }

    peripherals
}
