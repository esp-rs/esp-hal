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
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

#[cfg(feature = "esp32")]
pub use esp32_pac as pac;
#[cfg(feature = "esp32c3")]
pub use esp32c3_pac as pac;
#[cfg(feature = "esp32s2")]
pub use esp32s2_pac as pac;
#[cfg(feature = "esp32s3")]
pub use esp32s3_pac as pac;

pub mod delay;

#[cfg_attr(feature = "esp32", path = "efuse/esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "efuse/esp32c3.rs")]
#[cfg_attr(feature = "esp32s2", path = "efuse/esp32s2.rs")]
#[cfg_attr(feature = "esp32s3", path = "efuse/esp32s3.rs")]
pub mod efuse;

pub mod gpio;
pub mod i2c;
#[cfg_attr(feature = "risc_v", path = "interrupt/riscv.rs")]
#[cfg_attr(feature = "xtensa", path = "interrupt/xtensa.rs")]
pub mod interrupt;
pub mod prelude;
pub mod pulse_control;
pub mod rng;
#[cfg(not(feature = "esp32c3"))]
pub mod rtc_cntl;
pub mod serial;
pub mod spi;
pub mod timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub mod usb_serial_jtag;
pub mod utils;

pub use delay::Delay;
pub use gpio::*;
pub use interrupt::*;
pub use procmacros::ram;
pub use pulse_control::PulseControl;
pub use rng::Rng;
#[cfg(not(feature = "esp32c3"))]
pub use rtc_cntl::RtcCntl;
pub use serial::Serial;
pub use spi::Spi;
pub use timer::Timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub use usb_serial_jtag::UsbSerialJtag;
#[cfg(any(feature = "esp32c3", feature = "esp32s3", feature = "esp32s2"))]
pub mod systimer;

pub mod clock;
pub mod system;

pub mod analog;

#[cfg_attr(feature = "esp32", path = "cpu_control/esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "cpu_control/none.rs")]
#[cfg_attr(feature = "esp32s2", path = "cpu_control/none.rs")]
#[cfg_attr(feature = "esp32s3", path = "cpu_control/esp32s3.rs")]
pub mod cpu_control;

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
#[derive(Copy, Clone, Debug)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

pub fn get_core() -> Cpu {
    #[cfg(target_arch = "xtensa")]
    match ((xtensa_lx::get_processor_id() >> 13) & 1) != 0 {
        false => Cpu::ProCpu,
        true => Cpu::AppCpu,
    }
    #[cfg(target_arch = "riscv32")] // TODO get hart_id
    Cpu::ProCpu
}

// TODO for next release of cs, we need to impl for RISCV too
#[cfg(target_arch = "xtensa")]
mod critical_section_impl {
    struct CriticalSection;

    critical_section::custom_impl!(CriticalSection);

    // Virtual representation of the PS (processor state) of an Xtensa chip
    static mut VPS: u32 = 0; // TODO remove when 32bit tokens are supported in CS crate

    unsafe impl critical_section::Impl for CriticalSection {
        unsafe fn acquire() -> u8 {
            core::arch::asm!("rsil {0}, 15", out(reg) VPS);
            #[cfg(feature = "multicore")]
            {
                let guard = multicore::MULTICORE_LOCK.lock();
                core::mem::forget(guard); // forget it so drop doesn't run
            }
            0
        }

        unsafe fn release(_token: u8) {
            #[cfg(feature = "multicore")]
            {
                debug_assert!(multicore::MULTICORE_LOCK.is_owned_by_current_thread());
                // safety: we logically own the mutex from acquire()
                multicore::MULTICORE_LOCK.force_unlock();
            }
            core::arch::asm!("wsr.ps {0}", in(reg) VPS)
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
