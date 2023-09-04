//! `no_std` HAL for the ESP32-C3/ESP8685 from Espressif.
//!
//! Implements a number of the traits defined by the various packages in the
//! [embedded-hal] repository.
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal
//!
//! ### Cargo Features
//!
//! - `async` - Enable support for asynchronous operation, with interfaces
//!   provided by [embedded-hal-async] and [embedded-io-async]
//! - `debug` - Enable debug features in the HAL (used for development)
//! - `defmt` - Enable [`defmt::Format`] on certain types
//! - `direct-boot` - Use the direct boot image format
//! - `direct-vectoring` - Enable direct vector table hooking support
//! - `eh1` - Implement the traits defined in the `1.0.0-xxx` pre-releases of
//!   [embedded-hal], [embedded-hal-nb], and [embedded-io]
//! - `embassy` - Enable support for [embassy], a modern asynchronous embedded
//!   framework
//! - `embassy-time-systick` - Enable the [embassy] time driver using the
//!   `SYSTIMER` peripheral
//! - `embassy-time-timg0` - Enable the [embassy] time driver using the `TIMG0`
//!   peripheral
//! - `interrupt-preemption` - Enable priority-based interrupt preemption
//! - `log` - enable log output using the `log` crate
//! - `mcu-boot` - Use the MCUboot image format
//! - `rt` - Runtime support
//! - `ufmt` - Implement the [`ufmt_write::uWrite`] trait for the UART and USB
//!   Serial JTAG drivers
//! - `vectored` - Enable interrupt vectoring
//!
//! #### Default Features
//!
//! The `rt` and `vectored` features are enabled by default.
//!
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [embedded-io-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io-async
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-nb]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-nb
//! [embedded-io]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io
//! [embassy]: https://github.com/embassy-rs/embassy
//! [`ufmt_write::uWrite`]: https://docs.rs/ufmt-write/latest/ufmt_write/trait.uWrite.html
//! [`defmt::Format`]: https://docs.rs/defmt/0.3.5/defmt/trait.Format.html
//!
//! ### Supported Image Formats
//!
//! This HAL supports building multiple different application image formats. You
//! can read about each below.
//!
//! The ESP-IDF Bootloader format is used unless some other format is specified
//! via its feature.
//!
//! #### ESP-IDF Bootloader
//!
//! Use the second-stage bootloader from [ESP-IDF] and its associated
//! application image format. See the [App Image Format] documentation for more
//! information about this format.
//!
//! [ESP-IDF]: https://github.com/espressif/esp-idf
//! [App Image Format]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/app_image_format.html
//!
//! #### Direct Boot
//!
//! This device additionally supports direct-boot, which allows an application
//! to be executed directly from flash, without using the second-stage
//! bootloader. For more information please see the
//! [esp32c3-direct-boot-example] in the Espressif organization on GitHub.
//!
//! [esp32c3-direct-boot-example]: https://github.com/espressif/esp32c3-direct-boot-example
//!
//! #### MCUboot
//!
//! Use the MCUBoot bootloader and its associated image format. See the [MCUBoot
//! design document] for more information about this format.
//!
//! [MCUBoot design document]: https://docs.mcuboot.com/design.html

#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

pub use esp_hal_common::*;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

extern "C" {
    cfg_if::cfg_if! {
        if #[cfg(feature = "mcu-boot")] {
            // Required for retrieving the entry point address
            fn _start();

            // Functions from internal ROM
            fn cache_suspend_icache() -> u32;
            fn cache_resume_icache(val: u32);
            fn cache_invalidate_icache_all();
            fn cache_dbus_mmu_set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;
            fn cache_ibus_mmu_set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;

            /* IROM metadata:
             * - Destination address (VMA) for IROM region
             * - Flash offset (LMA) for start of IROM region
             * - Size of IROM region
             */
            static mut _image_irom_vma: u32;
            static mut _image_irom_lma: u32;
            static mut _image_irom_size: u32;

            /* DROM metadata:
             * - Destination address (VMA) for DROM region
             * - Flash offset (LMA) for start of DROM region
             * - Size of DROM region
             */
            static mut _image_drom_vma: u32;
            static mut _image_drom_lma: u32;
            static mut _image_drom_size: u32;
        }
    }
}

#[cfg(feature = "mcu-boot")]
#[link_section = ".entry_addr"]
#[no_mangle]
#[used]
// Entry point address for the MCUboot image header
static ENTRY_POINT: unsafe extern "C" fn() = _start;

#[cfg(feature = "mcu-boot")]
#[link_section = ".rwtext"]
unsafe fn configure_mmu() {
    const PARTITION_OFFSET: u32 = 0x10000;
    let app_irom_lma = PARTITION_OFFSET + ((&_image_irom_lma as *const u32) as u32);
    let app_irom_size = (&_image_irom_size as *const u32) as u32;
    let app_irom_vma = (&_image_irom_vma as *const u32) as u32;
    let app_drom_lma = PARTITION_OFFSET + ((&_image_drom_lma as *const u32) as u32);
    let app_drom_size = (&_image_drom_size as *const u32) as u32;
    let app_drom_vma = (&_image_drom_vma as *const u32) as u32;

    let autoload = cache_suspend_icache();
    cache_invalidate_icache_all();

    // Clear the MMU entries that are already set up, so the new app only has
    // the mappings it creates.

    const FLASH_MMU_TABLE: *mut u32 = 0x600c_5000 as *mut u32;
    const ICACHE_MMU_SIZE: usize = 0x200;
    const FLASH_MMU_TABLE_SIZE: usize = ICACHE_MMU_SIZE / core::mem::size_of::<u32>();
    const MMU_TABLE_INVALID_VAL: u32 = 0x100;

    for i in 0..FLASH_MMU_TABLE_SIZE {
        FLASH_MMU_TABLE.add(i).write_volatile(MMU_TABLE_INVALID_VAL);
    }

    const MMU_BLOCK_SIZE: u32 = 0x0001_0000;
    const MMU_FLASH_MASK: u32 = !(MMU_BLOCK_SIZE - 1);

    let calc_mmu_pages = |size, vaddr| {
        (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_BLOCK_SIZE - 1) / MMU_BLOCK_SIZE
    };

    let drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
    let drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
    let drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
    cache_dbus_mmu_set(
        0,
        drom_vma_aligned,
        drom_lma_aligned,
        64,
        drom_page_count,
        0,
    );

    let irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
    let irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
    let irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
    cache_ibus_mmu_set(
        0,
        irom_vma_aligned,
        irom_lma_aligned,
        64,
        irom_page_count,
        0,
    );

    let extmem = unsafe { &*peripherals::EXTMEM::ptr() };
    extmem.icache_ctrl1.modify(|_, w| {
        w.icache_shut_ibus()
            .clear_bit()
            .icache_shut_dbus()
            .clear_bit()
    });

    cache_resume_icache(autoload);
}

#[export_name = "__post_init"]
#[cfg_attr(feature = "mcu-boot", link_section = ".rwtext")]
unsafe fn post_init() {
    #[cfg(feature = "mcu-boot")]
    unsafe {
        configure_mmu();
    }

    use esp_hal_common::{
        peripherals::{RTC_CNTL, TIMG0, TIMG1},
        timer::Wdt,
    };

    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(RTC_CNTL::steal());
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0>::set_wdt_enabled(false);
    Wdt::<TIMG1>::set_wdt_enabled(false);
}
