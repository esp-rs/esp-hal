#![allow(rustdoc::bare_urls, unused_macros)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![doc = include_str!("../README.md")]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]

#[cfg(feature = "defmt")]
use defmt as _;
#[cfg(feature = "println")]
use esp_println as _;

const MAX_BACKTRACE_ADDRESSES: usize = 10;

#[cfg(feature = "esp32")]
const RAM: (u32, u32) = (0x3FFA_E000, 0x4000_0000);

#[cfg(feature = "esp32c2")]
const RAM: (u32, u32) = (0x3FCA_0000, 0x3FCE_0000);

#[cfg(feature = "esp32c3")]
const RAM: (u32, u32) = (0x3FC8_0000, 0x3FCE_0000);

#[cfg(feature = "esp32c6")]
const RAM: (u32, u32) = (0x4080_0000, 0x4088_0000);

#[cfg(feature = "esp32h2")]
const RAM: (u32, u32) = (0x4080_0000, 0x4085_0000);

#[cfg(feature = "esp32p4")]
const RAM: (u32, u32) = (0x4FF0_0000, 0x4FFC_0000);

#[cfg(feature = "esp32s2")]
const RAM: (u32, u32) = (0x3FFB_0000, 0x4000_0000);

#[cfg(feature = "esp32s3")]
const RAM: (u32, u32) = (0x3FC8_8000, 0x3FD0_0000);

#[cfg(feature = "colors")]
const RESET: &str = "\u{001B}[0m";
#[cfg(feature = "colors")]
const RED: &str = "\u{001B}[31m";

#[cfg(feature = "defmt")]
macro_rules! println {
    ("") => {
        // Do nothing if the string is just a space
    };
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
    #[cfg(feature = "println")]
    {
        println!("{}", code);
    }
}

#[cfg(any(feature = "coredump", feature = "coredump-all"))]
pub(crate) mod coredump;

#[cfg_attr(target_arch = "riscv32", path = "riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
pub mod arch;

extern "C" {
    static _stack_start: u32;
    static _stack_end: u32;
}

#[cfg(all(
    feature = "panic-handler",
    not(any(feature = "coredump", feature = "coredump-all"))
))]
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    pre_backtrace();

    #[cfg(feature = "colors")]
    set_color_code(RED);

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
        #[cfg(all(feature = "colors", feature = "println"))]
        println!("{}0x{:x}", RED, addr - crate::arch::RA_OFFSET);

        #[cfg(not(all(feature = "colors", feature = "println")))]
        println!("0x{:x}", addr - crate::arch::RA_OFFSET);
    }

    #[cfg(feature = "colors")]
    set_color_code(RESET);

    #[cfg(feature = "semihosting")]
    semihosting::process::abort();

    #[cfg(not(feature = "semihosting"))]
    halt();
}

#[cfg(all(
    feature = "panic-handler",
    any(feature = "coredump", feature = "coredump-all")
))]
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "colors")]
    set_color_code(RED);

    println!("");
    println!("====================== PANIC ======================");
    println!("{}", info);

    #[cfg(feature = "colors")]
    set_color_code(RESET);

    unsafe {
        #[cfg(target_arch = "riscv32")]
        core::arch::asm!("ecall");

        #[cfg(target_arch = "xtensa")]
        core::arch::asm!("syscall");
    }

    halt();
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

    (RAM.0..=RAM.1).contains(&address)
}

#[cfg(all(
    any(
        not(any(feature = "esp32", feature = "esp32p4", feature = "esp32s3")),
        not(feature = "halt-cores")
    ),
    not(feature = "custom-halt")
))]
#[allow(unused)]
fn halt() -> ! {
    loop {
        continue;
    }
}

#[cfg(feature = "custom-halt")]
fn halt() -> ! {
    extern "Rust" {
        fn custom_halt() -> !;
    }
    unsafe { custom_halt() }
}

// TODO: Enable `halt` function for `esp32p4` feature once implemented
#[cfg(all(any(feature = "esp32", feature = "esp32s3"), feature = "halt-cores"))]
#[allow(unused)]
fn halt() -> ! {
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

    #[cfg(feature = "esp32p4")]
    unsafe {}

    #[cfg(not(feature = "esp32p4"))]
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

    loop {}
}

#[cfg(not(feature = "custom-pre-backtrace"))]
#[allow(unused)]
fn pre_backtrace() {}

#[cfg(feature = "custom-pre-backtrace")]
fn pre_backtrace() {
    extern "Rust" {
        fn custom_pre_backtrace();
    }
    unsafe { custom_pre_backtrace() }
}
