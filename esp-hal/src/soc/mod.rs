#![cfg_attr(not(feature = "rt"), expect(unused))]

use core::ops::Range;

pub use self::implementation::*;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
mod implementation;

#[allow(unused)]
pub(crate) fn is_valid_ram_address(address: usize) -> bool {
    addr_in_range(address, memory_range!("DRAM"))
}

#[allow(unused)]
pub(crate) fn is_slice_in_dram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, memory_range!("DRAM"))
}

#[allow(unused)]
#[cfg(psram)]
pub(crate) fn is_valid_psram_address(address: usize) -> bool {
    addr_in_range(address, crate::psram::psram_range())
}

#[allow(unused)]
#[cfg(psram)]
pub(crate) fn is_slice_in_psram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, crate::psram::psram_range())
}

#[allow(unused)]
pub(crate) fn is_valid_memory_address(address: usize) -> bool {
    if is_valid_ram_address(address) {
        return true;
    }
    #[cfg(psram)]
    if is_valid_psram_address(address) {
        return true;
    }

    false
}

fn slice_in_range<T>(slice: &[T], range: Range<usize>) -> bool {
    let slice = slice.as_ptr_range();
    let start = slice.start as usize;
    let end = slice.end as usize;
    // `end` is >= `start`, so we don't need to check that `end > range.start`
    // `end` is also one past the last element, so it can be equal to the range's
    // end which is also one past the memory region's last valid address.
    addr_in_range(start, range.clone()) && end <= range.end
}

pub(crate) fn addr_in_range(addr: usize, range: Range<usize>) -> bool {
    range.contains(&addr)
}

#[cfg(feature = "rt")]
#[cfg(riscv)]
#[unsafe(export_name = "hal_main")]
fn hal_main(a0: usize, a1: usize, a2: usize) -> ! {
    unsafe extern "Rust" {
        // This symbol will be provided by the user via `#[entry]`
        fn main(a0: usize, a1: usize, a2: usize) -> !;
    }

    setup_stack_guard();

    unsafe {
        main(a0, a1, a2);
    }
}

#[cfg(all(xtensa, feature = "rt"))]
mod xtensa {
    use core::arch::{global_asm, naked_asm};

    /// The ESP32 has a first stage bootloader that handles loading program data
    /// into the right place therefore we skip loading it again. This function
    /// is called by xtensa-lx-rt in Reset.
    #[unsafe(export_name = "__init_data")]
    extern "C" fn __init_data() -> bool {
        false
    }

    extern "C" fn __init_persistent() -> bool {
        matches!(
            crate::system::reset_reason(),
            None | Some(crate::rtc_cntl::SocResetReason::ChipPowerOn)
        )
    }

    unsafe extern "C" {
        static _rtc_fast_bss_start: u32;
        static _rtc_fast_bss_end: u32;
        static _rtc_fast_persistent_end: u32;
        static _rtc_fast_persistent_start: u32;

        static _rtc_slow_bss_start: u32;
        static _rtc_slow_bss_end: u32;
        static _rtc_slow_persistent_end: u32;
        static _rtc_slow_persistent_start: u32;

        fn _xtensa_lx_rt_zero_fill(s: *mut u32, e: *mut u32);

        static mut __stack_chk_guard: u32;
    }

    global_asm!(
        "
        .literal sym_init_persistent, {__init_persistent}
        .literal sym_xtensa_lx_rt_zero_fill, {_xtensa_lx_rt_zero_fill}

        .literal sym_rtc_fast_bss_start, {_rtc_fast_bss_start}
        .literal sym_rtc_fast_bss_end, {_rtc_fast_bss_end}
        .literal sym_rtc_fast_persistent_end, {_rtc_fast_persistent_end}
        .literal sym_rtc_fast_persistent_start, {_rtc_fast_persistent_start}

        .literal sym_rtc_slow_bss_start, {_rtc_slow_bss_start}
        .literal sym_rtc_slow_bss_end, {_rtc_slow_bss_end}
        .literal sym_rtc_slow_persistent_end, {_rtc_slow_persistent_end}
        .literal sym_rtc_slow_persistent_start, {_rtc_slow_persistent_start}
        ",
        __init_persistent = sym __init_persistent,
        _xtensa_lx_rt_zero_fill = sym _xtensa_lx_rt_zero_fill,

        _rtc_fast_bss_end = sym _rtc_fast_bss_end,
        _rtc_fast_bss_start = sym _rtc_fast_bss_start,
        _rtc_fast_persistent_end = sym _rtc_fast_persistent_end,
        _rtc_fast_persistent_start = sym _rtc_fast_persistent_start,

        _rtc_slow_bss_end = sym _rtc_slow_bss_end,
        _rtc_slow_bss_start = sym _rtc_slow_bss_start,
        _rtc_slow_persistent_end = sym _rtc_slow_persistent_end,
        _rtc_slow_persistent_start = sym _rtc_slow_persistent_start,
    );

    #[unsafe(export_name = "__post_init")]
    #[unsafe(naked)]
    #[allow(named_asm_labels)]
    extern "C" fn post_init() {
        naked_asm!(
            "
            entry a1, 0

            l32r   a6, sym_xtensa_lx_rt_zero_fill      // Pre-load address of zero-fill function

            l32r   a10, sym_rtc_fast_bss_start         // Set input range to .rtc_fast.bss
            l32r   a11, sym_rtc_fast_bss_end           //
            callx8 a6                                  // Zero-fill

            l32r   a10, sym_rtc_slow_bss_start         // Set input range to .rtc_slow.bss
            l32r   a11, sym_rtc_slow_bss_end           //
            callx8 a6                                  // Zero-fill

            l32r   a5,  sym_init_persistent            // Do we need to initialize persistent data?
            callx8 a5
            beqz   a10, .Lpost_init_return             // If not, skip initialization

            l32r   a10, sym_rtc_fast_persistent_start  // Set input range to .rtc_fast.persistent
            l32r   a11, sym_rtc_fast_persistent_end    //
            callx8 a6                                  // Zero-fill

            l32r   a10, sym_rtc_slow_persistent_start  // Set input range to .rtc_slow.persistent
            l32r   a11, sym_rtc_slow_persistent_end    //
            callx8 a6                                  // Zero-fill

        .Lpost_init_return:
            retw.n
        ",
        )
    }

    #[cfg(esp32s3)]
    global_asm!(".section .rwtext,\"ax\",@progbits");
    global_asm!(
        "
        .literal sym_stack_chk_guard, {__stack_chk_guard}
        .literal stack_guard_value, {stack_guard_value}
        ",
        __stack_chk_guard = sym __stack_chk_guard,
        stack_guard_value = const esp_config::esp_config_int!(
            u32,
            "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
        )
    );

    #[cfg_attr(esp32s3, unsafe(link_section = ".rwtext"))]
    #[unsafe(export_name = "__pre_init")]
    #[unsafe(naked)]
    unsafe extern "C" fn esp32_reset() {
        // Set up stack protector value before jumping to a rust function
        naked_asm! {
            "
            entry a1, 0x20

            // Set up the stack protector value
            l32r   a2, sym_stack_chk_guard
            l32r   a3, stack_guard_value
            s32i.n a3, a2, 0

            call8 {esp32_init}

            retw.n
            ",
            esp32_init = sym esp32_init
        }
    }

    #[cfg_attr(esp32s3, unsafe(link_section = ".rwtext"))]
    fn esp32_init() {
        unsafe {
            super::configure_cpu_caches();
        }

        crate::interrupt::setup_interrupts();
    }
}

#[cfg(feature = "rt")]
#[unsafe(export_name = "__stack_chk_fail")]
unsafe extern "C" fn stack_chk_fail() {
    panic!("Stack corruption detected");
}

#[cfg(all(feature = "rt", riscv))]
fn setup_stack_guard() {
    unsafe extern "C" {
        static mut __stack_chk_guard: u32;
    }

    unsafe {
        let stack_chk_guard = core::ptr::addr_of_mut!(__stack_chk_guard);
        // we _should_ use a random value but we don't have a good source for random
        // numbers here
        stack_chk_guard.write_volatile(esp_config::esp_config_int!(
            u32,
            "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
        ));
    }
}

#[cfg(all(feature = "rt", stack_guard_monitoring))]
pub(crate) fn enable_main_stack_guard_monitoring() {
    unsafe {
        unsafe extern "C" {
            static mut __stack_chk_guard: u32;
        }

        let guard_addr = core::ptr::addr_of_mut!(__stack_chk_guard) as *mut _ as u32;
        crate::debugger::set_stack_watchpoint(guard_addr as usize);
    }
}
