//! `no_std` HAL for the ESP32-S2 from Espressif.
//!
//! Implements a number of the traits defined by the various packages in the
//! [embedded-hal] repository.
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal

#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

use esp_hal_common::xtensa_lx_rt::exception::ExceptionCause;
pub use esp_hal_common::*;
// Always enable atomic emulation on ESP32-S2
use xtensa_atomic_emulation_trap as _;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SensExt};
}

/// Function initializes ESP32 specific memories (RTC slow and fast) and
/// then calls original Reset function
///
/// ENTRY point is defined in memory.x
/// *Note: the pre_init function is called in the original reset handler
/// after the initializations done in this function*
#[cfg(feature = "rt")]
#[doc(hidden)]
#[no_mangle]
pub unsafe extern "C" fn ESP32Reset() -> ! {
    // These symbols come from `memory.x`
    extern "C" {
        static mut _rtc_fast_bss_start: u32;
        static mut _rtc_fast_bss_end: u32;

        static mut _rtc_slow_bss_start: u32;
        static mut _rtc_slow_bss_end: u32;

        static mut _stack_end_cpu0: u32;
    }

    // set stack pointer to end of memory: no need to retain stack up to this point
    esp_hal_common::xtensa_lx::set_stack_pointer(&mut _stack_end_cpu0);

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_slow_bss_start, &mut _rtc_slow_bss_end);

    // continue with default reset handler
    esp_hal_common::xtensa_lx_rt::Reset();
}

/// The ESP32 has a first stage bootloader that handles loading program data
/// into the right place therefore we skip loading it again.
#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub extern "Rust" fn __init_data() -> bool {
    false
}

/// Atomic Emulation is always enabled on ESP32-S2
#[doc(hidden)]
#[no_mangle]
#[export_name = "__exception"] // this overrides the exception handler in xtensa_lx_rt
#[link_section = ".rwtext"]
unsafe fn exception(cause: ExceptionCause, save_frame: &mut trapframe::TrapFrame) {
    match cause {
        ExceptionCause::Illegal => {
            let mut regs = [
                save_frame.A0,
                save_frame.A1,
                save_frame.A2,
                save_frame.A3,
                save_frame.A4,
                save_frame.A5,
                save_frame.A6,
                save_frame.A7,
                save_frame.A8,
                save_frame.A9,
                save_frame.A10,
                save_frame.A11,
                save_frame.A12,
                save_frame.A13,
                save_frame.A14,
                save_frame.A15,
            ];

            if xtensa_atomic_emulation_trap::atomic_emulation(save_frame.PC, &mut regs) {
                save_frame.PC += 3; // 24bit instruction

                save_frame.A0 = regs[0];
                save_frame.A1 = regs[1];
                save_frame.A2 = regs[2];
                save_frame.A3 = regs[3];
                save_frame.A4 = regs[4];
                save_frame.A5 = regs[5];
                save_frame.A6 = regs[6];
                save_frame.A7 = regs[7];
                save_frame.A8 = regs[8];
                save_frame.A9 = regs[9];
                save_frame.A10 = regs[10];
                save_frame.A11 = regs[11];
                save_frame.A12 = regs[12];
                save_frame.A13 = regs[13];
                save_frame.A14 = regs[14];
                save_frame.A15 = regs[15];

                return;
            }
        }
        _ => (),
    }

    extern "C" {
        fn __user_exception(cause: ExceptionCause, save_frame: &mut trapframe::TrapFrame);
    }

    __user_exception(cause, save_frame);
}
