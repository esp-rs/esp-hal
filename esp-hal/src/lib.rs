#![cfg_attr(
    docsrs,
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.esp-rs.org/esp-hal/'>browse the <code>esp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! Bare-metal (`no_std`) HAL for all Espressif ESP32 devices.
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
//! please ensure you are reading the correct [documentation] for your target
//! device.
//!
//! ## Choosing a device
//!
//! Depending on your target device, you need to enable the chip feature
//! for that device. You may also need to do this on [ancillary esp crates].
//!
//! ## Examples
//!
//! We have a plethora of [examples] in the esp-hal repository. We use
//! an [xtask] to automate the building, running, and testing of code and
//! examples within esp-hal.
//!
//! Invoke the following command in the root of the esp-hal repository to get
//! started.
//!
//! ```no_run
//! cargo xtask help
//! ```
//!
//! ## Creating a project
//!
//! We have a [book] that explains the full esp-rs ecosystem
//! and how to get started, it's advisable to give that a read
//! before proceeding.
//!
//! We have a template for quick starting bare-metal projects, [esp-template].
//! The template uses [cargo-generate], so ensure that it is installed and run
//!
//! ```no_run
//! cargo generate -a esp-rs/esp-template
//! ```
//!
//! [documentation]: https://docs.esp-rs.org/esp-hal
//! [examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [xtask]: https://github.com/matklad/cargo-xtask
//! [esp-template]: https://github.com/esp-rs/esp-template
//! [cargo-generate]: https://github.com/cargo-generate/cargo-generate
//! [ancillary esp crates]: https://github.com/esp-rs/esp-hal?tab=readme-ov-file#ancillary-crates
//! [book]: https://docs.esp-rs.org/book/
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register)]
#![cfg_attr(feature = "async", allow(stable_features, async_fn_in_trait))]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
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
#[cfg(feature = "embassy")]
pub mod embassy;
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
#[cfg(systimer)]
pub mod systimer;
#[cfg(any(timg0, timg1))]
pub mod timer;
#[cfg(trace0)]
pub mod trace;
#[cfg(any(twai0, twai1))]
pub mod twai;
#[cfg(any(uart0, uart1, uart2))]
pub mod uart;
#[cfg(usb_device)]
pub mod usb_serial_jtag;

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
fn get_raw_core() -> usize {
    riscv::register::mhartid::read()
}

/// Returns the result of reading the PRID register logically ANDed with 0x2000,
/// the 13th bit in the register. Espressif Xtensa chips use this bit to
/// determine the core id.
///
/// Returns either 0 or 0x2000
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(xtensa)]
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

/// FlashSafeDma
///
/// The embedded-hal traits make no guarantees about
/// where the buffers are placed. The DMA implementation in Espressif chips has
/// a limitation in that it can only access the RAM address space, meaning data
/// to be transmitted from the flash address space must be copied into RAM
/// first.
///
/// This wrapper struct should be used when a peripheral using the DMA engine
/// needs to transmit data from flash (ROM) via the embedded-hal traits. This is
/// often a `const` variable.
///
/// Example usage using [`spi::master::dma::SpiDma`]
/// ```no_run
/// const ARRAY_IN_FLASH = [0xAA; 128]
///
/// let spi = SpiDma::new(/* */);
///
/// spi.write(&ARRAY_IN_FLASH[..]).unwrap(); // error when transmission starts
///
/// let spi = FlashSafeDma::new(spi);
///
/// spi.write(&ARRAY_IN_FLASH[..]).unwrap(); // success
/// ```
pub struct FlashSafeDma<T, const SIZE: usize> {
    inner: T,
    #[allow(unused)]
    buffer: [u8; SIZE],
}

impl<T, const SIZE: usize> FlashSafeDma<T, SIZE> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            buffer: [0u8; SIZE],
        }
    }

    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.inner
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }

    pub fn free(self) -> T {
        self.inner
    }
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
