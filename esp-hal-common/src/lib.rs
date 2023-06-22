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

#[cfg(riscv)]
pub use esp_riscv_rt::{self, entry, riscv};
pub use procmacros as macros;
#[cfg(xtensa)]
pub use xtensa_lx;
#[cfg(xtensa)]
pub use xtensa_lx_rt::{self, entry};

#[cfg(adc)]
pub use self::analog::adc::implementation as adc;
#[cfg(dac)]
pub use self::analog::dac::implementation as dac;
#[cfg(any(xtensa, all(riscv, systimer)))]
pub use self::delay::Delay;
#[cfg(gdma)]
pub use self::dma::gdma;
#[cfg(pdma)]
pub use self::dma::pdma;
#[cfg(gpio)]
pub use self::gpio::IO;
#[cfg(rmt)]
pub use self::pulse_control::PulseControl;
#[cfg(rng)]
pub use self::rng::Rng;
#[cfg(any(lp_clkrst, rtc_cntl))]
pub use self::rtc_cntl::{Rtc, Rwdt};
#[cfg(any(esp32, esp32s3))]
pub use self::soc::cpu_control;
#[cfg(efuse)]
pub use self::soc::efuse;
pub use self::soc::peripherals;
#[cfg(psram)]
pub use self::soc::psram;
#[cfg(any(spi0, spi1, spi2, spi3))]
pub use self::spi::Spi;
#[cfg(any(timg0, timg1))]
pub use self::timer::Timer;
#[cfg(any(uart0, uart1, uart2))]
pub use self::uart::Uart;
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
#[cfg(feature = "embassy")]
pub mod embassy;
#[cfg(gpio)]
pub mod gpio;
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
#[cfg(pcnt)]
pub mod pcnt;
pub mod peripheral;
pub mod prelude;
#[cfg(rmt)]
pub mod pulse_control;
#[cfg(radio)]
pub mod radio;
pub mod reset;
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
extern "C" fn EspDefaultHandler(_level: u32, _interrupt: peripherals::Interrupt) {}

#[cfg(xtensa)]
#[no_mangle]
extern "C" fn DefaultHandler() {}

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, PartialEq, Eq)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu,
}

/// Which core the application is currently executing on
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
