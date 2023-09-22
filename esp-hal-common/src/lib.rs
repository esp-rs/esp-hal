//! `no_std` HAL implementations for the peripherals which are common among
//! Espressif devices. Implements a number of the traits defined by
//! [embedded-hal].
//!
//! This crate should not be used directly; you should use one of the
//! device-specific HAL crates instead:
//!
//! - [esp32-hal]
//! - [esp32c2-hal]
//! - [esp32c3-hal]
//! - [esp32c6-hal]
//! - [esp32h2-hal]
//! - [esp32s2-hal]
//! - [esp32s3-hal]
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
//! [esp32c2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c2-hal
//! [esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
//! [esp32c6-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-hal
//! [esp32h2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32h2-hal
//! [esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
//! [esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal

#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(
    feature = "async",
    allow(incomplete_features),
    feature(async_fn_in_trait),
    feature(impl_trait_projections)
)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

// MUST be the first module
mod fmt;

#[cfg(riscv)]
pub use esp_riscv_rt::{self, entry, riscv};
pub use procmacros as macros;
#[cfg(xtensa)]
pub use xtensa_lx;
#[cfg(xtensa)]
pub use xtensa_lx_rt::{self, entry};

#[cfg(adc)]
pub use self::analog::adc;
#[cfg(dac)]
pub use self::analog::dac;
#[cfg(any(xtensa, all(riscv, systimer)))]
pub use self::delay::Delay;
#[cfg(gdma)]
pub use self::dma::gdma;
#[cfg(pdma)]
pub use self::dma::pdma;
#[cfg(gpio)]
pub use self::gpio::IO;
#[cfg(rmt)]
pub use self::rmt::Rmt;
#[cfg(rng)]
pub use self::rng::Rng;
#[cfg(any(lp_clkrst, rtc_cntl))]
pub use self::rtc_cntl::{Rtc, Rwdt};
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
#[cfg(any(spi0, spi1, spi2, spi3))]
pub use self::spi::Spi;
#[cfg(any(timg0, timg1))]
pub use self::timer::Timer;
#[cfg(any(uart0, uart1, uart2))]
pub use self::uart::{Uart, UartRx, UartTx};
#[cfg(usb_device)]
pub use self::usb_serial_jtag::UsbSerialJtag;

#[cfg(aes)]
pub mod aes;
#[cfg(any(adc, dac))]
pub mod analog;
#[cfg(assist_debug)]
pub mod assist_debug;
pub mod clock;
#[cfg(any(xtensa, all(riscv, systimer)))]
pub mod delay;
#[cfg(any(gdma, pdma))]
pub mod dma;
#[cfg(ecc)]
pub mod ecc;
#[cfg(feature = "embassy")]
pub mod embassy;
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
#[cfg(radio)]
pub mod radio;
#[cfg(any(hmac, sha))]
mod reg_access;
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
#[cfg(any(dport, pcr, system))]
pub mod system;
#[cfg(systimer)]
pub mod systimer;
#[cfg(any(timg0, timg1))]
pub mod timer;
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

#[no_mangle]
extern "C" fn EspDefaultHandler(_level: u32, _interrupt: peripherals::Interrupt) {
    #[cfg(feature = "log")]
    warn!("Unhandled level {} interrupt: {:?}", _level, _interrupt);

    #[cfg(feature = "defmt")]
    warn!(
        "Unhandled level {} interrupt: {:?}",
        _level,
        defmt::Debug2Format(&_interrupt)
    );
}

#[cfg(xtensa)]
#[no_mangle]
extern "C" fn DefaultHandler() {}

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu,
}

#[cfg(all(xtensa, multi_core))]
fn get_raw_core() -> u32 {
    xtensa_lx::get_processor_id() & 0x2000
}

/// Which core the application is currently executing on
#[cfg(all(xtensa, multi_core))]
pub fn get_core() -> Cpu {
    match get_raw_core() {
        0 => Cpu::ProCpu,
        _ => Cpu::AppCpu,
    }
}

/// Which core the application is currently executing on
#[cfg(not(all(xtensa, multi_core)))]
pub fn get_core() -> Cpu {
    Cpu::ProCpu
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
        use esp_riscv_rt::riscv;

        #[cfg(multi_core)]
        // The restore state is a u8 that is casted from a bool, so it has a value of
        // 0x00 or 0x01 before we add the reentry flag to it.
        const REENTRY_FLAG: u8 = 1 << 7;

        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let mut mstatus = 0u32;
                core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);
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
        use core::sync::atomic::{AtomicUsize, Ordering};

        // We're using a value that we know get_raw_core() will never return. This
        // avoids an unnecessary increment of the core ID.
        #[cfg(xtensa)] // TODO: first multi-core RISC-V target will show if this value is OK
                       // globally or only for Xtensa
        const UNUSED_THREAD_ID_VALUE: usize = 0x0001;

        fn thread_id() -> usize {
            crate::get_raw_core() as usize
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
/// Example usage using [`spi::dma::SpiDma`]
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
