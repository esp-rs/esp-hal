//! `no_std` HAL implementations for the peripherals which are common among
//! Espressif devices. Implements a number of the traits defined by
//! [embedded-hal].
//!
//! This crate should not be used directly; you should use one of the
//! device-specific HAL crates instead:
//!
//! - [esp32-hal]
//! - [esp32c3-hal]
//! - [esp32s2-hal]
//! - [esp32s3-hal]
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
//! [esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
//! [esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
//! [esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal

#![no_std]
#![cfg_attr(not(esp32c3), feature(asm_experimental_arch))]

#[cfg(esp32)]
pub use esp32 as pac;
#[cfg(esp32c3)]
pub use esp32c3 as pac;
#[cfg(esp32s2)]
pub use esp32s2 as pac;
#[cfg(esp32s3)]
pub use esp32s3 as pac;

pub mod delay;

#[cfg_attr(esp32, path = "efuse/esp32.rs")]
#[cfg_attr(esp32c3, path = "efuse/esp32c3.rs")]
#[cfg_attr(esp32s2, path = "efuse/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "efuse/esp32s3.rs")]
pub mod efuse;

pub mod gpio;
pub mod i2c;
#[cfg_attr(esp32c3, path = "interrupt/riscv.rs")]
#[cfg_attr(not(esp32c3), path = "interrupt/xtensa.rs")]
pub mod interrupt;
pub mod ledc;
pub mod prelude;
pub mod pulse_control;
pub mod rng;
pub mod rom;
pub mod rtc_cntl;
pub mod serial;
pub mod spi;
pub mod timer;
#[cfg(any(esp32c3, esp32s3))]
pub mod usb_serial_jtag;
pub mod utils;

pub use delay::Delay;
pub use gpio::*;
pub use interrupt::*;
pub use procmacros as macros;
pub use pulse_control::PulseControl;
pub use rng::Rng;
pub use rtc_cntl::{Rtc, Rwdt};
pub use serial::Serial;
pub use spi::Spi;
pub use timer::Timer;
#[cfg(any(esp32c3, esp32s3))]
pub use usb_serial_jtag::UsbSerialJtag;
#[cfg(any(esp32c3, esp32s3, esp32s2))]
pub mod systimer;

pub mod clock;
pub mod system;

pub mod analog;

#[cfg_attr(esp32, path = "cpu_control/esp32.rs")]
#[cfg_attr(esp32c3, path = "cpu_control/none.rs")]
#[cfg_attr(esp32s2, path = "cpu_control/none.rs")]
#[cfg_attr(esp32s3, path = "cpu_control/esp32s3.rs")]
pub mod cpu_control;

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

pub fn get_core() -> Cpu {
    #[cfg(all(not(esp32c3), feature = "multi_core"))]
    match ((xtensa_lx::get_processor_id() >> 13) & 1) != 0 {
        false => Cpu::ProCpu,
        true => Cpu::AppCpu,
    }

    // #[cfg(all(esp32c3, feature = "multi_core"))]
    // TODO get hart_id

    // single core always has ProCpu only
    #[cfg(feature = "single_core")]
    Cpu::ProCpu
}

mod critical_section_impl {
    struct CriticalSection;

    critical_section::set_impl!(CriticalSection);

    #[cfg(not(esp32c3))]
    mod xtensa {

        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let tkn: critical_section::RawRestoreState;
                core::arch::asm!("rsil {0}, 15", out(reg) tkn);
                #[cfg(feature = "multicore")]
                {
                    let guard = multicore::MULTICORE_LOCK.lock();
                    core::mem::forget(guard); // forget it so drop doesn't run
                }
                tkn
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                if token != 0 {
                    #[cfg(feature = "multicore")]
                    {
                        debug_assert!(multicore::MULTICORE_LOCK.is_owned_by_current_thread());
                        // safety: we logically own the mutex from acquire()
                        multicore::MULTICORE_LOCK.force_unlock();
                    }
                    core::arch::asm!(
                        "wsr.ps {0}",
                        "rsync", in(reg) token)
                }
            }
        }
    }

    #[cfg(esp32c3)]
    mod riscv {
        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let interrupts_active = riscv::register::mstatus::read().mie();
                riscv::interrupt::disable();

                #[cfg(feature = "multicore")]
                {
                    let guard = multicore::MULTICORE_LOCK.lock();
                    core::mem::forget(guard); // forget it so drop doesn't run
                }

                interrupts_active as _
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                if token != 0 {
                    #[cfg(feature = "multicore")]
                    {
                        debug_assert!(multicore::MULTICORE_LOCK.is_owned_by_current_thread());
                        // safety: we logically own the mutex from acquire()
                        multicore::MULTICORE_LOCK.force_unlock();
                    }
                    riscv::interrupt::enable();
                }
            }
        }
    }

    #[cfg(feature = "multicore")]
    mod multicore {
        use core::sync::atomic::{AtomicBool, Ordering};

        use lock_api::{GetThreadId, GuardSend, RawMutex};

        use crate::get_core;

        /// Reentrant Mutex
        ///
        /// Currently implemented using an atomic spin lock.
        /// In the future we can optimize this raw mutex to use some hardware
        /// features.
        pub(crate) static MULTICORE_LOCK: lock_api::ReentrantMutex<RawSpinlock, RawThreadId, ()> =
            lock_api::ReentrantMutex::const_new(RawSpinlock::INIT, RawThreadId::INIT, ());

        pub(crate) struct RawThreadId;

        unsafe impl lock_api::GetThreadId for RawThreadId {
            const INIT: Self = RawThreadId;

            fn nonzero_thread_id(&self) -> core::num::NonZeroUsize {
                core::num::NonZeroUsize::new((get_core() as usize) + 1).unwrap()
            }
        }

        pub(crate) struct RawSpinlock(AtomicBool);

        unsafe impl lock_api::RawMutex for RawSpinlock {
            const INIT: RawSpinlock = RawSpinlock(AtomicBool::new(false));

            // A spinlock guard can be sent to another thread and unlocked there
            type GuardMarker = GuardSend;

            fn lock(&self) {
                while !self.try_lock() {}
            }

            fn try_lock(&self) -> bool {
                self.0
                    .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
                    .is_ok()
            }

            unsafe fn unlock(&self) {
                self.0.store(false, Ordering::Release);
            }
        }
    }
}
