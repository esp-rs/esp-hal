//! This is a lightweight crate for obtaining backtraces on Espressif devices.
//!
//! It provides an optional panic handler and supports a range of output options, all configurable
//! through feature flags.
#![cfg_attr(target_arch = "riscv32", doc = "\n")]
#![cfg_attr(
    target_arch = "riscv32",
    doc = "\nPlease note that you **need** to force frame pointers (i.e. `\"-C\", \"force-frame-pointers\",` in your `.cargo/config.toml`).\n"
)]
#![cfg_attr(
    target_arch = "riscv32",
    doc = "Otherwise the panic handler will emit a stack dump which needs tooling to decode it.\n\n"
)]
#![cfg_attr(target_arch = "riscv32", doc = "\n")]
//! You can get an array of backtrace addresses (limited to 10 entries by default) via
//! `arch::backtrace()` if you want to create a backtrace yourself (i.e. not using the panic
//! handler).
//!
//! ## Features
#![doc = document_features::document_features!()]
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Below is a table of tunable parameters
//! for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_backtrace_config_table.md"))]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![no_std]

#[macro_use]
extern crate esp_metadata_generated;

#[cfg(feature = "defmt")]
use defmt as _;
#[cfg(feature = "println")]
use esp_println as _;

const MAX_BACKTRACE_ADDRESSES: usize =
    esp_config::esp_config_int!(usize, "ESP_BACKTRACE_CONFIG_BACKTRACE_FRAMES");

pub struct Backtrace(pub(crate) heapless::Vec<BacktraceFrame, MAX_BACKTRACE_ADDRESSES>);

impl Backtrace {
    /// Captures a stack backtrace.
    #[inline]
    pub fn capture() -> Self {
        arch::backtrace()
    }

    /// Returns the backtrace frames as a slice.
    #[inline]
    pub fn frames(&self) -> &[BacktraceFrame] {
        &self.0
    }
}

pub struct BacktraceFrame {
    pub(crate) pc: usize,
}

impl BacktraceFrame {
    pub fn program_counter(&self) -> usize {
        self.pc - crate::arch::RA_OFFSET
    }
}

#[cfg(feature = "panic-handler")]
const RESET: &str = "\u{001B}[0m";
#[cfg(feature = "panic-handler")]
const RED: &str = "\u{001B}[31m";

#[cfg(all(feature = "panic-handler", feature = "defmt"))]
macro_rules! println {
    ($($arg:tt)*) => {
        defmt::error!($($arg)*);
    };
}

#[cfg(all(feature = "panic-handler", feature = "defmt", stack_dump))]
pub(crate) use println;

#[cfg(all(feature = "panic-handler", feature = "println"))]
macro_rules! println {
    ($($arg:tt)*) => {
        esp_println::println!($($arg)*);
    };
}

#[cfg(feature = "panic-handler")]
fn set_color_code(_code: &str) {
    #[cfg(all(feature = "colors", feature = "println"))]
    {
        println!("{}", _code);
    }
}

#[cfg_attr(target_arch = "riscv32", path = "riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
pub(crate) mod arch;

#[cfg(feature = "panic-handler")]
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    pre_backtrace();

    set_color_code(RED);
    println!("");
    println!("====================== PANIC ======================");

    println!("{}", info);
    set_color_code(RESET);

    cfg_if::cfg_if! {
        if #[cfg(not(stack_dump))]
        {
            println!("");
            println!("Backtrace:");
            println!("");

            let backtrace = Backtrace::capture();
            #[cfg(target_arch = "riscv32")]
            if backtrace.frames().is_empty() {
                println!(
                    "No backtrace available - make sure to force frame-pointers. (see https://crates.io/crates/esp-backtrace)"
                );
            }
            for frame in backtrace.frames() {
                println!("0x{:x}", frame.program_counter());
            }
        } else {
            arch::dump_stack();
        }
    }

    abort()
}

// Ensure that the address is in DRAM.
//
// Address ranges can be found in `esp-metadata/devices/$CHIP.toml` in the `device` table.
fn is_valid_ram_address(address: u32) -> bool {
    memory_range!("DRAM").contains(&address)
}

#[cfg(feature = "halt-cores")]
fn halt() {
    #[cfg(any(feature = "esp32", feature = "esp32s3"))]
    {
        #[cfg(feature = "esp32")]
        mod registers {
            pub(crate) const OPTIONS0: u32 = 0x3ff48000;
            pub(crate) const SW_CPU_STALL: u32 = 0x3ff480ac;
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

#[cfg(feature = "panic-handler")]
fn pre_backtrace() {
    #[cfg(feature = "custom-pre-backtrace")]
    {
        unsafe extern "Rust" {
            fn custom_pre_backtrace();
        }
        unsafe { custom_pre_backtrace() }
    }
}

#[cfg(feature = "panic-handler")]
fn abort() -> ! {
    println!("");
    println!("");
    println!("");

    cfg_if::cfg_if! {
        if #[cfg(feature = "semihosting")] {
            arch::interrupt_free(|| {
                semihosting::process::abort();
            });
        } else if #[cfg(feature = "halt-cores")] {
            halt();
        } else if #[cfg(feature = "custom-halt")] {
            // call custom code
            unsafe extern "Rust" {
                fn custom_halt() -> !;
            }
            unsafe { custom_halt() }
        }
    }

    #[allow(unreachable_code)]
    arch::interrupt_free(|| {
        #[allow(clippy::empty_loop)]
        loop {}
    })
}
