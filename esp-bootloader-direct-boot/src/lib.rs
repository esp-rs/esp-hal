//! # Direct boot for esp-hal applications
//!
//! With this crate the ROM starts the application directly, without the ESP-IDF 2nd-stage
//! bootloader, a partition table or an app image header.
//!
//! If the ROM finds the magic word `0xaedb041d` twice in a row at a fixed flash offset, it maps
//! the start of flash into the instruction and data caches and jumps to the instruction right
//! after the magic. Nothing is loaded into RAM. This crate provides the linker scripts that lay
//! the application out for that, and an entry stub that copies the RAM sections from flash and
//! then starts the regular runtime.
//!
//! Direct boot is supported on all chips except the ESP32 and the ESP32-S2. The magic sits at
//! flash offset `0x0`, except on the ESP32-S3, where it is at `0x400`. On the ESP32-C3 direct boot
//! needs chip revision v0.3 or later. The ESP32-P4 needs chip revision v3.0 or later: the entry
//! stub runs before `esp_hal::init` checks the revision, so an older chip hangs without an error
//! message.
//!
//! ## Usage
//!
//! The application enables `esp-hal/unstable`, links with `-Tlinkall.x` as usual and invokes
//! [`direct_boot!`] once:
//!
//! ```rust, ignore
//! esp_bootloader_direct_boot::direct_boot!();
//! ```
//!
//! Use this crate instead of `esp-bootloader-esp-idf`, not next to it.
//!
//! ## Flashing
//!
//! The flash image is the ELF file's loadable sections at their load addresses, which are flash
//! offsets:
//!
//! ```text
//! rust-objcopy -O binary --gap-fill 0xff app app.bin
//! espflash write-bin 0x0 app.bin
//! ```
//!
//! `objcopy` starts the image at the lowest load address, so on the ESP32-S3 write it to `0x400`.
//! `espflash flash` and `espflash save-image` create an ESP-IDF image and cannot be used.
//!
//! ## Limitations
//!
//! - There is no partition table, OTA, app descriptor or secure boot.
//! - `#[ram(reclaimed)]` is not available. The memory the 2nd-stage bootloader would occupy is
//!   regular RAM instead.
//! - Direct boot can be disabled permanently with the `DIS_DIRECT_BOOT` eFuse.
//! - When switching from an ESP-IDF image, erase the flash once, so a stale bootloader or partition
//!   table cannot get in the way.
//! - The image must fit into the flash the linker scripts map: 4 MB on the ESP32-C2 and ESP32-S3
//!   (the ESP32-S3 ROM maps no more for direct boot), 8 MB on the ESP32-C3, 16 MB on the ESP32-C5,
//!   -C6, -C61, -H2 and -H4, and 64 MB on the ESP32-P4 and ESP32-S31.
//! - On the ESP32-P4, `ESP_HAL_CONFIG_L2_CACHE_SIZE` must not be `0KB`.
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![no_std]

mod init;
#[cfg(riscv)]
mod riscv;
#[cfg(xtensa)]
mod xtensa;

#[unsafe(link_section = ".espressif.metadata")]
#[used]
#[unsafe(export_name = "bootloader.NAME")]
static BOOTLOADER_NAME: [u8; 11] = *b"DIRECT-BOOT";

/// The entry stub copies `.data`, so xtensa-lx-rt must not copy it in `Reset`.
#[cfg(xtensa)]
#[unsafe(export_name = "__init_data")]
extern "C" fn __init_data() -> bool {
    false
}

/// Copies the RTC memory images, unless the chip woke up from deep sleep: RTC memory keeps its
/// contents then, and initialized RTC statics keep their values, as with the ESP-IDF bootloader.
///
/// Runs before `.bss` is zeroed, so this must not touch any statics.
///
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/bootloader_support/src/esp_image_format.c#L931-L970>.
#[inline(always)]
fn load_rtc_memory() {
    const RESET_REASON_CORE_DEEP_SLEEP: u32 = 5;

    unsafe extern "C" {
        static _direct_boot_rtc_copy_table_start: [u32; 3];
        static _direct_boot_rtc_copy_table_end: [u32; 3];
    }

    if esp_rom_sys::rom::rtc_get_reset_reason(0) == RESET_REASON_CORE_DEEP_SLEEP {
        return;
    }

    let mut entry = &raw const _direct_boot_rtc_copy_table_start;
    while entry < &raw const _direct_boot_rtc_copy_table_end {
        unsafe {
            let [mut src, mut dst, end] = entry.read();
            while dst < end {
                (dst as *mut u32).write_volatile((src as *const u32).read_volatile());
                src += 4;
                dst += 4;
            }
            entry = entry.add(1);
        }
    }
}

#[doc(hidden)]
pub mod __private {
    pub static LINK: u8 = 0;
}

/// Makes sure this crate is linked into the application.
///
/// rustc only links crates the application references. Without this, the linker would not find
/// the entry stub.
#[macro_export]
macro_rules! direct_boot {
    () => {
        #[used]
        static __ESP_BOOTLOADER_DIRECT_BOOT: &u8 = &$crate::__private::LINK;
    };
}
