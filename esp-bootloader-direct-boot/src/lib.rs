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
//! flash offset `0x0`, except on the ESP32-S3, where it is at `0x400`.
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
//! - `#[ram(reclaimed)]` is not available.
//! - Direct boot can be disabled permanently with the `DIS_DIRECT_BOOT` eFuse.
//! - When switching from an ESP-IDF image, erase the flash once, so a stale bootloader or partition
//!   table cannot get in the way.
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
