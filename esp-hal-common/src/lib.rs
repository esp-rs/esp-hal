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
//! - [esp32s2-hal]
//! - [esp32s3-hal]
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
//! [esp32c2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c2-hal
//! [esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
//! [esp32c6-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-hal
//! [esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
//! [esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal

#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(feature = "async", allow(incomplete_features))]
#![cfg_attr(feature = "async", feature(async_fn_in_trait))]
#![cfg_attr(feature = "async", feature(impl_trait_projections))]

#[cfg(riscv)]
pub use esp_riscv_rt;
#[cfg(riscv)]
pub use esp_riscv_rt::entry;
#[cfg(riscv)]
pub use esp_riscv_rt::riscv;
pub use procmacros as macros;
#[cfg(xtensa)]
pub use xtensa_lx;
#[cfg(xtensa)]
pub use xtensa_lx_rt;
#[cfg(xtensa)]
pub use xtensa_lx_rt::entry;

/// State of the CPU saved when entering exception or interrupt
pub mod trapframe {
    #[cfg(riscv)]
    pub use esp_riscv_rt::TrapFrame;
    #[cfg(xtensa)]
    pub use xtensa_lx_rt::exception::Context as TrapFrame;
}

#[cfg(rmt)]
pub use self::pulse_control::PulseControl;
#[cfg(any(esp32, esp32s3))]
pub use self::soc::cpu_control;
#[cfg(usb_serial_jtag)]
pub use self::usb_serial_jtag::UsbSerialJtag;
pub use self::{
    delay::Delay,
    interrupt::*,
    rng::Rng,
    rtc_cntl::{Rtc, Rwdt},
    soc::{efuse, peripherals},
    spi::Spi,
    timer::Timer,
    uart::Uart,
};

#[cfg(aes)]
pub mod aes;
pub mod analog;
pub mod clock;
pub mod delay;
pub mod dma;
#[cfg(feature = "embassy")]
pub mod embassy;
pub mod gpio;
pub mod i2c;
#[cfg(i2s)]
pub mod i2s;
pub mod ledc;
#[cfg(mcpwm)]
pub mod mcpwm;
#[cfg(usb_otg)]
pub mod otg_fs;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c6))]
pub mod pcnt;
pub mod peripheral;
pub mod prelude;
#[cfg(rmt)]
pub mod pulse_control;
#[cfg(radio)]
pub mod radio;
pub mod rng;
pub mod rom;
pub mod rtc_cntl;
pub mod sha;
pub mod soc;
pub mod spi;
pub mod system;
#[cfg(systimer)]
pub mod systimer;
pub mod timer;
#[cfg(any(esp32c3, esp32c6, esp32s3))]
pub mod twai;
pub mod uart;
#[cfg(usb_serial_jtag)]
pub mod usb_serial_jtag;
#[cfg(rmt)]
pub mod utils;

#[cfg_attr(riscv, path = "interrupt/riscv.rs")]
#[cfg_attr(xtensa, path = "interrupt/xtensa.rs")]
pub mod interrupt;

#[cfg(esp32c6)]
pub fn disable_apm_filter() {
    unsafe {
        (&*esp32c6::LP_APM::PTR).func_ctrl.write(|w| w.bits(0));
        (&*esp32c6::LP_APM0::PTR).func_ctrl.write(|w| w.bits(0));
        (&*esp32c6::HP_APM::PTR).func_ctrl.write(|w| w.bits(0));
    }
}

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

pub fn get_core() -> Cpu {
    #[cfg(all(xtensa, multi_core))]
    match ((xtensa_lx::get_processor_id() >> 13) & 1) != 0 {
        false => Cpu::ProCpu,
        true => Cpu::AppCpu,
    }

    // #[cfg(all(riscv, multi_core))]
    // TODO get hart_id

    // single core always has ProCpu only
    #[cfg(single_core)]
    Cpu::ProCpu
}

mod critical_section_impl {
    struct CriticalSection;

    critical_section::set_impl!(CriticalSection);

    #[cfg(xtensa)]
    mod xtensa {
        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let tkn: critical_section::RawRestoreState;
                core::arch::asm!("rsil {0}, 5", out(reg) tkn);
                #[cfg(multi_core)]
                {
                    let guard = super::multicore::MULTICORE_LOCK.lock();
                    core::mem::forget(guard); // forget it so drop doesn't run
                }
                tkn
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                if token != 0 {
                    #[cfg(multi_core)]
                    {
                        debug_assert!(super::multicore::MULTICORE_LOCK.is_owned_by_current_thread());
                        // safety: we logically own the mutex from acquire()
                        super::multicore::MULTICORE_LOCK.force_unlock();
                    }
                    core::arch::asm!(
                        "wsr.ps {0}",
                        "rsync", in(reg) token)
                }
            }
        }
    }

    #[cfg(riscv)]
    mod riscv {
        use esp_riscv_rt::riscv;

        unsafe impl critical_section::Impl for super::CriticalSection {
            unsafe fn acquire() -> critical_section::RawRestoreState {
                let mut mstatus = 0u32;
                core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);
                let interrupts_active = (mstatus & 0b1000) != 0;
                #[cfg(multi_core)]
                {
                    let guard = multicore::MULTICORE_LOCK.lock();
                    core::mem::forget(guard); // forget it so drop doesn't run
                }

                interrupts_active as _
            }

            unsafe fn release(token: critical_section::RawRestoreState) {
                if token != 0 {
                    #[cfg(multi_core)]
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

    #[cfg(multi_core)]
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
