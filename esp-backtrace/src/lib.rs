#![allow(rustdoc::bare_urls, unused_macros)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![doc = include_str!("../README.md")]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]

#[cfg(feature = "defmt")]
use defmt as _;
#[cfg(feature = "println")]
use esp_println as _;

const MAX_BACKTRACE_ADDRESSES: usize =
    esp_config::esp_config_int!(usize, "ESP_BACKTRACE_CONFIG_BACKTRACE_FRAMES");

const RESET: &str = "\u{001B}[0m";
const RED: &str = "\u{001B}[31m";

#[cfg(feature = "defmt")]
macro_rules! println {
    ($($arg:tt)*) => {
        defmt::error!($($arg)*);
    };
}

#[cfg(all(feature = "println", not(feature = "defmt")))]
macro_rules! println {
    ($($arg:tt)*) => {
        esp_println::println!($($arg)*);
    };
}

#[allow(unused, unused_variables)]
fn set_color_code(code: &str) {
    #[cfg(all(feature = "colors", feature = "println"))]
    {
        println!("{}", code);
    }
}

#[cfg_attr(target_arch = "riscv32", path = "riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
pub mod arch;

#[cfg(feature = "panic-handler")]
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    pre_backtrace();

    println!("");
    println!("====================== PANIC ======================");

    println!("{}", info);

    println!("");
    println!("Backtrace:");
    println!("");

    let backtrace = crate::arch::backtrace();
    #[cfg(target_arch = "riscv32")]
    if backtrace.iter().filter(|e| e.is_some()).count() == 0 {
        println!("No backtrace available - make sure to force frame-pointers. (see https://crates.io/crates/esp-backtrace)");
    }
    for addr in backtrace.into_iter().flatten() {
        println!("0x{:x}", addr - crate::arch::RA_OFFSET);
    }

    abort();
}

#[cfg(all(feature = "exception-handler", target_arch = "xtensa"))]
#[no_mangle]
#[link_section = ".rwtext"]
unsafe fn __user_exception(cause: arch::ExceptionCause, context: arch::Context) {
    pre_backtrace();

    println!("\n\nException occurred '{}'", cause);
    println!("{:?}", context);

    let backtrace = crate::arch::backtrace_internal(context.A1, 0);
    for e in backtrace {
        if let Some(addr) = e {
            println!("0x{:x}", addr);
        }
    }

    abort();
}

#[cfg(all(feature = "exception-handler", target_arch = "riscv32"))]
#[export_name = "ExceptionHandler"]
fn exception_handler(context: &arch::TrapFrame) -> ! {
    pre_backtrace();

    let mepc = context.pc;
    let code = context.mcause & 0xff;
    let mtval = context.mtval;

    if code == 14 {
        println!("");
        println!(
            "Stack overflow detected at 0x{:x} called by 0x{:x}",
            mepc, context.ra
        );
        println!("");
    } else {
        let code = match code {
            0 => "Instruction address misaligned",
            1 => "Instruction access fault",
            2 => "Illegal instruction",
            3 => "Breakpoint",
            4 => "Load address misaligned",
            5 => "Load access fault",
            6 => "Store/AMO address misaligned",
            7 => "Store/AMO access fault",
            8 => "Environment call from U-mode",
            9 => "Environment call from S-mode",
            10 => "Reserved",
            11 => "Environment call from M-mode",
            12 => "Instruction page fault",
            13 => "Load page fault",
            14 => "Reserved",
            15 => "Store/AMO page fault",
            _ => "UNKNOWN",
        };

        println!(
            "Exception '{}' mepc=0x{:08x}, mtval=0x{:08x}",
            code, mepc, mtval
        );

        println!("{:?}", context);

        let backtrace = crate::arch::backtrace_internal(context.s0 as u32, 0);
        if backtrace.iter().filter(|e| e.is_some()).count() == 0 {
            println!("No backtrace available - make sure to force frame-pointers. (see https://crates.io/crates/esp-backtrace)");
        }
        for addr in backtrace.into_iter().flatten() {
            println!("0x{:x}", addr - crate::arch::RA_OFFSET);
        }
    }

    abort();
}

// Ensure that the address is in DRAM and that it is 16-byte aligned.
//
// Based loosely on the `esp_stack_ptr_in_dram` function from
// `components/esp_hw_support/include/esp_memory_utils.h` in ESP-IDF.
//
// Address ranges can be found in `components/soc/$CHIP/include/soc/soc.h` as
// `SOC_DRAM_LOW` and `SOC_DRAM_HIGH`.
fn is_valid_ram_address(address: u32) -> bool {
    if (address & 0xF) != 0 {
        return false;
    }

    #[cfg(feature = "esp32")]
    if !(0x3FFA_E000..=0x4000_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32c2")]
    if !(0x3FCA_0000..=0x3FCE_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32c3")]
    if !(0x3FC8_0000..=0x3FCE_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32c6")]
    if !(0x4080_0000..=0x4088_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32h2")]
    if !(0x4080_0000..=0x4085_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32p4")]
    if !(0x4FF0_0000..=0x4FFC_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32s2")]
    if !(0x3FFB_0000..=0x4000_0000).contains(&address) {
        return false;
    }

    #[cfg(feature = "esp32s3")]
    if !(0x3FC8_8000..=0x3FD0_0000).contains(&address) {
        return false;
    }

    true
}

#[allow(unused)]
fn halt() -> ! {
    cfg_if::cfg_if! {
        if #[cfg(feature = "custom-halt")] {
            // call custom code
            extern "Rust" {
                fn custom_halt() -> !;
            }
            unsafe { custom_halt() }
        } else if #[cfg(any(feature = "esp32", /*feature = "esp32p4",*/ feature = "esp32s3"))] {
            // multi-core
            #[cfg(feature = "esp32")]
            mod registers {
                pub(crate) const OPTIONS0: u32 = 0x3ff48000;
                pub(crate) const SW_CPU_STALL: u32 = 0x3ff480ac;
            }

            #[cfg(feature = "esp32p4")]
            mod registers {
                pub(crate) const SW_CPU_STALL: u32 = 0x50115200;
            }

            #[cfg(feature = "esp32s3")]
            mod registers {
                pub(crate) const OPTIONS0: u32 = 0x60008000;
                pub(crate) const SW_CPU_STALL: u32 = 0x600080bc;
            }

            let sw_cpu_stall = registers::SW_CPU_STALL as *mut u32;

            unsafe {
                // We need to write the value "0x86" to stall a particular core. The write
                // location is split into two separate bit fields named "c0" and "c1", and the
                // two fields are located in different registers. Each core has its own pair of
                // "c0" and "c1" bit fields.

                let options0 = registers::OPTIONS0 as *mut u32;

                options0.write_volatile(options0.read_volatile() & !(0b1111) | 0b1010);

                sw_cpu_stall.write_volatile(
                    sw_cpu_stall.read_volatile() & !(0b111111 << 20) & !(0b111111 << 26)
                        | (0x21 << 20)
                        | (0x21 << 26),
                );
            }
        }
    }

    loop {}
}

#[allow(unused)]
fn pre_backtrace() {
    #[cfg(feature = "custom-pre-backtrace")]
    {
        extern "Rust" {
            fn custom_pre_backtrace();
        }
        unsafe { custom_pre_backtrace() }
    }

    set_color_code(RED);
}

#[allow(unused)]
fn abort() -> ! {
    println!("");
    println!("");
    println!("");

    set_color_code(RESET);

    cfg_if::cfg_if! {
        if #[cfg(feature = "semihosting")] {
            semihosting::process::abort();
        } else {
            halt();
        }
    }
}
