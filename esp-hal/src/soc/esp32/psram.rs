//! # PSRAM "virtual peripheral" driver (ESP32)
//!
//! ## Overview
//!
//! The `PSRAM` module provides support for accessing and controlling
//! the `Pseudo Static Random Access Memory (PSRAM)` on the `ESP32`.
//!
//! The `PSRAM` module enables users to interface with the `PSRAM` memory
//! present on the `ESP32` chip. `PSRAM` provides additional external memory to
//! supplement the internal memory of the `ESP32`, allowing for increased
//! storage capacity and improved performance in certain applications.
//!
//! ## Examples
//!
//! ### Quad PSRAM
//! This example shows how to use PSRAM as heap-memory via esp-alloc.
//! You need an ESP32 with at least 2 MB of PSRAM memory.
//! Notice that PSRAM example **must** be built in release mode!
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # extern crate alloc;
//! # use alloc::{string::String, vec::Vec};
//! # use esp_alloc as _;
//! # use esp_hal::{psram, prelude::*};
//!
//! // Initialize PSRAM and add it as a heap memory region
//! fn init_psram_heap(start: *mut u8, size: usize) {
//!     unsafe {
//!         esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
//!             start,
//!             size,
//!             esp_alloc::MemoryCapability::External.into(),
//!         ));
//!     }
//! }
//!
//! // Initialize PSRAM and add it to the heap
//! let (start, size) = psram::init_psram(peripherals.PSRAM,
//! psram::PsramConfig::default()); init_psram_heap(start, size);
//!
//! let mut large_vec: Vec<u32> = Vec::with_capacity(500 * 1024 / 4);
//!
//! for i in 0..(500 * 1024 / 4) {
//!     large_vec.push((i & 0xff) as u32);
//! }
//!
//! // esp_println::println!("vec size = {} bytes", large_vec.len() * 4);
//! // esp_println::println!("vec address = {:p}", large_vec.as_ptr());
//! // esp_println::println!("vec[..100] = {:?}", &large_vec[..100]);
//!
//! let string = String::from("A string allocated in PSRAM");
//!
//! // esp_println::println!("'{}' allocated at {:p}", &string,
//! // string.as_ptr());
//! # }
//! ```

pub use crate::soc::psram_common::*;

const EXTMEM_ORIGIN: usize = 0x3F800000;

/// Cache Speed
#[derive(PartialEq, Eq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum PsramCacheSpeed {
    #[default]
    PsramCacheF80mS40m = 0,
    PsramCacheF40mS40m,
    PsramCacheF80mS80m,
}

/// PSRAM configuration
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramConfig {
    /// PSRAM size
    pub size: PsramSize,
    /// Cache speed
    pub cache_speed: PsramCacheSpeed,
}

/// Initializes the PSRAM memory on supported devices.
///
/// Returns the start of the mapped memory and the size
pub(crate) fn init_psram(config: PsramConfig) {
    let mut config = config;

    utils::psram_init(&config);

    if config.size.is_auto() {
        // Reading the device-id turned out to not work as expected (some bits flipped
        // for unknown reason)
        //
        // As a workaround we just map 4m (maximum we can do) and
        // probe if we can access top of PSRAM - if not we assume it's 2m
        //
        // This currently doesn't work as expected because of https://github.com/esp-rs/esp-hal/issues/2182
        utils::s_mapping(EXTMEM_ORIGIN as u32, 4 * 1024 * 1024);

        let guessed_size = unsafe {
            let ptr = (EXTMEM_ORIGIN + 4 * 1024 * 1024 - 36 * 1024) as *mut u8;
            for i in 0..(36 * 1024) {
                ptr.add(i).write_volatile(0x7f);
            }

            let ptr = EXTMEM_ORIGIN as *mut u8;
            for i in 0..(36 * 1024) {
                ptr.add(i).write_volatile(0x7f);
            }

            let mut success = true;
            let ptr = (EXTMEM_ORIGIN + 4 * 1024 * 1024 - 36 * 1024) as *mut u8;
            for i in 0..(36 * 1024) {
                if ptr.add(i).read_volatile() != 0x7f {
                    success = false;
                    break;
                }
            }

            if success {
                4 * 1024 * 1024
            } else {
                2 * 1024 * 1024
            }
        };

        info!("Assuming {} bytes of PSRAM", guessed_size);
        config.size = PsramSize::Size(guessed_size);
    } else {
        utils::s_mapping(EXTMEM_ORIGIN as u32, config.size.get() as u32);
    }

    unsafe {
        crate::soc::MAPPED_PSRAM.memory_range = EXTMEM_ORIGIN..EXTMEM_ORIGIN + config.size.get();
    }
}

pub(crate) mod utils {
    use core::ptr::addr_of_mut;

    use procmacros::ram;

    use super::*;

    #[ram]
    pub(crate) fn s_mapping(v_start: u32, size: u32) {
        // Enable external RAM in MMU
        cache_sram_mmu_set(0, 0, v_start, 0, 32, size / 1024 / 32);
        // Flush and enable icache for APP CPU
        unsafe {
            let dport = &*esp32::DPORT::PTR;
            dport
                .app_cache_ctrl1()
                .modify(|_, w| w.app_cache_mask_dram1().clear_bit());
        }

        cache_sram_mmu_set(1, 0, v_start, 0, 32, size / 1024 / 32);
    }

    // we can use the ROM version of this: it works well enough and keeps the size
    // of the binary down.
    #[ram]
    fn cache_sram_mmu_set(
        cpu_no: u32,
        pid: u32,
        vaddr: u32,
        paddr: u32,
        psize: u32,
        num: u32,
    ) -> i32 {
        unsafe { cache_sram_mmu_set_rom(cpu_no, pid, vaddr, paddr, psize, num) }
    }

    // PSRAM clock and cs IO should be configured based on hardware design.
    // For ESP32-WROVER or ESP32-WROVER-B module, the clock IO is IO17, the cs IO is
    // IO16, they are the default value for these two configs.
    const D0WD_PSRAM_CLK_IO: u8 = 17;
    const D0WD_PSRAM_CS_IO: u8 = 16;

    const D2WD_PSRAM_CLK_IO: u8 = 9; // Default value is 9
    const D2WD_PSRAM_CS_IO: u8 = 10; // Default value is 10

    // For ESP32-PICO chip, the psram share clock with flash. The flash clock pin is
    // fixed, which is IO6.
    const PICO_PSRAM_CLK_IO: u8 = 6;
    const PICO_PSRAM_CS_IO: u8 = 10; // Default value is 10

    const PICO_V3_02_PSRAM_CLK_IO: u8 = 10;
    const PICO_V3_02_PSRAM_CS_IO: u8 = 9;
    const PICO_V3_02_PSRAM_SPIWP_SD3_IO: u8 = 18;

    const ESP_ROM_EFUSE_FLASH_DEFAULT_SPI: u32 = 0;
    const ESP_ROM_EFUSE_FLASH_DEFAULT_HSPI: u32 = 1;

    const SPI_IOMUX_PIN_NUM_CLK: u8 = 6;
    const SPI_IOMUX_PIN_NUM_CS: u8 = 11;

    // IO-pins for PSRAM.
    // WARNING: PSRAM shares all but the CS and CLK pins with the flash, so these
    // defines hardcode the flash pins as well, making this code incompatible
    // with either a setup that has the flash on non-standard pins or ESP32s
    // with built-in flash.
    const PSRAM_SPIQ_SD0_IO: u8 = 7;
    const PSRAM_SPID_SD1_IO: u8 = 8;
    const PSRAM_SPIWP_SD3_IO: u8 = 10;
    const PSRAM_SPIHD_SD2_IO: u8 = 9;

    const FLASH_HSPI_CLK_IO: u8 = 14;
    const FLASH_HSPI_CS_IO: u8 = 15;

    const PSRAM_HSPI_SPIQ_SD0_IO: u8 = 12;
    const PSRAM_HSPI_SPID_SD1_IO: u8 = 13;
    const PSRAM_HSPI_SPIWP_SD3_IO: u8 = 2;
    const PSRAM_HSPI_SPIHD_SD2_IO: u8 = 4;

    const DR_REG_SPI1_BASE: u32 = 0x3ff42000;
    const SPI1_USER_REG: u32 = DR_REG_SPI1_BASE + 0x1C;
    const SPI1_W0_REG: u32 = DR_REG_SPI1_BASE + 0x80;

    const fn psram_cs_hold_time_from_psram_speed(speed: PsramCacheSpeed) -> u32 {
        match speed {
            PsramCacheSpeed::PsramCacheF80mS40m => 0,
            PsramCacheSpeed::PsramCacheF40mS40m => 0,
            PsramCacheSpeed::PsramCacheF80mS80m => 1,
        }
    }

    const SPI_USR: u32 = 1 << 18;

    const PSRAM_INTERNAL_IO_28: u32 = 28;
    const PSRAM_INTERNAL_IO_29: u32 = 29;
    const SIG_GPIO_OUT_IDX: u32 = 256;
    const SPICLK_OUT_IDX: u32 = 0;
    const SIG_IN_FUNC224_IDX: u32 = 224;
    const SIG_IN_FUNC225_IDX: u32 = 225;
    const SPICS0_OUT_IDX: u32 = 5;
    const SPICS1_OUT_IDX: u32 = 6;
    const SPIQ_OUT_IDX: u32 = 1;
    const SPIQ_IN_IDX: u32 = 1;
    const SPID_OUT_IDX: u32 = 2;
    const SPID_IN_IDX: u32 = 2;
    const SPIWP_OUT_IDX: u32 = 4;
    const SPIWP_IN_IDX: u32 = 4;
    const SPIHD_OUT_IDX: u32 = 3;
    const SPIHD_IN_IDX: u32 = 3;
    const FUNC_SD_CLK_SPICLK: u32 = 1;
    const PIN_FUNC_GPIO: u32 = 2;

    const PSRAM_QUAD_WRITE: u32 = 0x38;
    const PSRAM_FAST_READ_QUAD_DUMMY: u32 = 0x5;
    const PSRAM_FAST_READ_QUAD: u32 = 0xEB;

    const SPI_FWRITE_DUAL_S: u32 = 12;
    const SPI_FWRITE_DUAL_M: u32 = 1 << 12;

    const SPI_FREAD_QIO_M: u32 = 1 << 24;
    const SPI0_R_QIO_DUMMY_CYCLELEN: u32 = 3;
    const SPI_FREAD_DIO_M: u32 = 1 << 23;
    const SPI0_R_DIO_DUMMY_CYCLELEN: u32 = 1;
    const SPI0_R_DIO_ADDR_BITSLEN: u32 = 27;
    const SPI_FREAD_QUAD_M: u32 = 1 << 20;
    const SPI_FREAD_DUAL_M: u32 = 1 << 14;
    const SPI0_R_FAST_DUMMY_CYCLELEN: u32 = 7;
    const PSRAM_IO_MATRIX_DUMMY_40M: u8 = 1;
    const PSRAM_IO_MATRIX_DUMMY_80M: u8 = 2;

    const _SPI_CACHE_PORT: u8 = 0;
    const _SPI_FLASH_PORT: u8 = 1;
    const _SPI_80M_CLK_DIV: u8 = 1;
    const _SPI_40M_CLK_DIV: u8 = 2;

    const FLASH_ID_GD25LQ32C: u32 = 0xC86016;

    const EFUSE_SPICONFIG_RET_SPICLK_MASK: u32 = 0x3f;
    const EFUSE_SPICONFIG_RET_SPICLK_SHIFT: u8 = 0;
    const EFUSE_SPICONFIG_RET_SPIQ_MASK: u32 = 0x3f;
    const EFUSE_SPICONFIG_RET_SPIQ_SHIFT: u8 = 6;
    const EFUSE_SPICONFIG_RET_SPID_MASK: u32 = 0x3f;
    const EFUSE_SPICONFIG_RET_SPID_SHIFT: u8 = 12;
    const EFUSE_SPICONFIG_RET_SPICS0_MASK: u32 = 0x3f;
    const EFUSE_SPICONFIG_RET_SPICS0_SHIFT: u8 = 18;
    const EFUSE_SPICONFIG_RET_SPIHD_MASK: u32 = 0x3f;
    const EFUSE_SPICONFIG_RET_SPIHD_SHIFT: u8 = 24;

    fn efuse_spiconfig_ret(spi_config: u32, mask: u32, shift: u8) -> u8 {
        (((spi_config) >> shift) & mask) as u8
    }

    #[derive(PartialEq, Eq, Debug, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    struct PsramIo {
        flash_clk_io: u8,
        flash_cs_io: u8,
        psram_clk_io: u8,
        psram_cs_io: u8,
        psram_spiq_sd0_io: u8,
        psram_spid_sd1_io: u8,
        psram_spiwp_sd3_io: u8,
        psram_spihd_sd2_io: u8,
    }

    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    enum PsramClkMode {
        PsramClkModeNorm = 0, // Normal SPI mode
        PsramClkModeDclk = 1, // Two extra clock cycles after CS is set high level
    }

    #[repr(C)]
    pub(super) struct EspRomSpiflashChip {
        pub device_id: u32,
        pub chip_size: u32, // chip size in bytes
        pub block_size: u32,
        pub sector_size: u32,
        pub page_size: u32,
        pub status_mask: u32,
    }

    extern "C" {
        fn esp_rom_efuse_get_flash_gpio_info() -> u32;

        fn esp_rom_gpio_connect_out_signal(
            gpio_num: u32,
            signal_idx: u32,
            out_inv: bool,
            oen_inv: bool,
        );

        fn esp_rom_gpio_connect_in_signal(gpio_num: u32, signal_idx: u32, inv: bool);

        fn esp_rom_spiflash_config_clk(freqdiv: u8, spi: u8) -> i32;

        static mut g_rom_spiflash_dummy_len_plus: u8;

        pub(super) static g_rom_flashchip: EspRomSpiflashChip;

        fn cache_sram_mmu_set_rom(
            cpu_no: u32,
            pid: u32,
            vaddr: u32,
            paddr: u32,
            psize: u32,
            num: u32,
        ) -> i32;
    }

    #[ram]
    pub(crate) fn psram_init(config: &PsramConfig) {
        let chip = crate::efuse::Efuse::chip_type();

        let mode = config.cache_speed;
        let mut psram_io = PsramIo::default();
        let clk_mode;

        match chip {
            crate::efuse::ChipType::Esp32D0wdq6 | crate::efuse::ChipType::Esp32D0wdq5 => {
                clk_mode = PsramClkMode::PsramClkModeNorm;
                psram_io.psram_clk_io = D0WD_PSRAM_CLK_IO;
                psram_io.psram_cs_io = D0WD_PSRAM_CS_IO;
            }
            crate::efuse::ChipType::Esp32D2wdq5 => {
                clk_mode = PsramClkMode::PsramClkModeDclk;
                psram_io.psram_clk_io = D2WD_PSRAM_CLK_IO;
                psram_io.psram_cs_io = D2WD_PSRAM_CS_IO;
            }
            crate::efuse::ChipType::Esp32Picod2 => {
                clk_mode = PsramClkMode::PsramClkModeNorm;
                psram_io.psram_clk_io = PICO_PSRAM_CLK_IO;
                psram_io.psram_cs_io = PICO_PSRAM_CS_IO;
            }
            crate::efuse::ChipType::Esp32Picod4 => {
                panic!("PSRAM is unsupported on this chip");
            }
            crate::efuse::ChipType::Esp32Picov302 => {
                clk_mode = PsramClkMode::PsramClkModeNorm;
                psram_io.psram_clk_io = PICO_V3_02_PSRAM_CLK_IO;
                psram_io.psram_cs_io = PICO_V3_02_PSRAM_CS_IO;
            }
            crate::efuse::ChipType::Unknown => {
                panic!("Unknown chip type. PSRAM is not supported");
            }
        }

        let spiconfig = unsafe { esp_rom_efuse_get_flash_gpio_info() };
        if spiconfig == ESP_ROM_EFUSE_FLASH_DEFAULT_SPI {
            psram_io.flash_clk_io = SPI_IOMUX_PIN_NUM_CLK;
            psram_io.flash_cs_io = SPI_IOMUX_PIN_NUM_CS;
            psram_io.psram_spiq_sd0_io = PSRAM_SPIQ_SD0_IO;
            psram_io.psram_spid_sd1_io = PSRAM_SPID_SD1_IO;
            psram_io.psram_spiwp_sd3_io = PSRAM_SPIWP_SD3_IO;
            psram_io.psram_spihd_sd2_io = PSRAM_SPIHD_SD2_IO;
        } else if spiconfig == ESP_ROM_EFUSE_FLASH_DEFAULT_HSPI {
            psram_io.flash_clk_io = FLASH_HSPI_CLK_IO;
            psram_io.flash_cs_io = FLASH_HSPI_CS_IO;
            psram_io.psram_spiq_sd0_io = PSRAM_HSPI_SPIQ_SD0_IO;
            psram_io.psram_spid_sd1_io = PSRAM_HSPI_SPID_SD1_IO;
            psram_io.psram_spiwp_sd3_io = PSRAM_HSPI_SPIWP_SD3_IO;
            psram_io.psram_spihd_sd2_io = PSRAM_HSPI_SPIHD_SD2_IO;
        } else if chip == crate::efuse::ChipType::Esp32Picov302 {
            psram_io.flash_clk_io = efuse_spiconfig_ret(
                spiconfig,
                EFUSE_SPICONFIG_RET_SPICLK_MASK,
                EFUSE_SPICONFIG_RET_SPICLK_SHIFT,
            );
            psram_io.flash_cs_io = efuse_spiconfig_ret(
                spiconfig,
                EFUSE_SPICONFIG_RET_SPICS0_MASK,
                EFUSE_SPICONFIG_RET_SPICS0_SHIFT,
            );
            psram_io.psram_spiq_sd0_io = efuse_spiconfig_ret(
                spiconfig,
                EFUSE_SPICONFIG_RET_SPIQ_MASK,
                EFUSE_SPICONFIG_RET_SPIQ_SHIFT,
            );
            psram_io.psram_spid_sd1_io = efuse_spiconfig_ret(
                spiconfig,
                EFUSE_SPICONFIG_RET_SPID_MASK,
                EFUSE_SPICONFIG_RET_SPID_SHIFT,
            );
            psram_io.psram_spihd_sd2_io = efuse_spiconfig_ret(
                spiconfig,
                EFUSE_SPICONFIG_RET_SPIHD_MASK,
                EFUSE_SPICONFIG_RET_SPIHD_SHIFT,
            );
            psram_io.psram_spiwp_sd3_io = PICO_V3_02_PSRAM_SPIWP_SD3_IO;
        } else {
            panic!("Getting Flash/PSRAM pins from efuse is not supported");
            // psram_io.flash_clk_io = EFUSE_SPICONFIG_RET_SPICLK(spiconfig);
            // psram_io.flash_cs_io = EFUSE_SPICONFIG_RET_SPICS0(spiconfig);
            // psram_io.psram_spiq_sd0_io = EFUSE_SPICONFIG_RET_SPIQ(spiconfig);
            // psram_io.psram_spid_sd1_io = EFUSE_SPICONFIG_RET_SPID(spiconfig);
            // psram_io.psram_spihd_sd2_io =
            // EFUSE_SPICONFIG_RET_SPIHD(spiconfig);
            // psram_io.psram_spiwp_sd3_io = bootloader_flash_get_wp_pin();
        }
        info!("PS-RAM pins {:?}", &psram_io);

        unsafe {
            let spi0 = &*crate::peripherals::SPI0::PTR;
            let spi1 = &*crate::peripherals::SPI1::PTR;

            spi0.ext3().modify(|_, w| w.bits(0x1));
            spi1.user().modify(|_, w| w.usr_prep_hold().clear_bit());
        }

        psram_spi_init(mode, clk_mode);

        match mode {
            PsramCacheSpeed::PsramCacheF80mS80m => unsafe {
                esp_rom_gpio_connect_out_signal(
                    psram_io.psram_clk_io as u32,
                    SPICLK_OUT_IDX,
                    false,
                    false,
                );
            },
            _ => unsafe {
                if clk_mode == PsramClkMode::PsramClkModeDclk {
                    // We need to delay CLK to the PSRAM with respect to the clock signal as output
                    // by the SPI peripheral. We do this by routing it signal to
                    // signal 224/225, which are used as a loopback; the extra run through
                    // the GPIO matrix causes the delay. We use GPIO20 (which is not in any package
                    // but has pad logic in silicon) as a temporary pad for
                    // this. So the signal path is: SPI CLK --> GPIO28 -->
                    // signal224(in then out) --> internal GPIO29 --> signal225(in then out) -->
                    // GPIO17(PSRAM CLK)
                    esp_rom_gpio_connect_out_signal(
                        PSRAM_INTERNAL_IO_28,
                        SPICLK_OUT_IDX,
                        false,
                        false,
                    );
                    esp_rom_gpio_connect_in_signal(PSRAM_INTERNAL_IO_28, SIG_IN_FUNC224_IDX, false);
                    esp_rom_gpio_connect_out_signal(
                        PSRAM_INTERNAL_IO_29,
                        SIG_IN_FUNC224_IDX,
                        false,
                        false,
                    );
                    esp_rom_gpio_connect_in_signal(PSRAM_INTERNAL_IO_29, SIG_IN_FUNC225_IDX, false);
                    esp_rom_gpio_connect_out_signal(
                        psram_io.psram_clk_io as u32,
                        SIG_IN_FUNC225_IDX,
                        false,
                        false,
                    );
                } else {
                    esp_rom_gpio_connect_out_signal(
                        psram_io.psram_clk_io as u32,
                        SPICLK_OUT_IDX,
                        false,
                        false,
                    );
                }
            },
        }

        let extra_dummy = psram_gpio_config(&psram_io, mode);
        info!("extra dummy = {}", extra_dummy);

        // psram_is_32mbit_ver0 would need special handling here

        unsafe {
            esp_rom_gpio_connect_out_signal(PSRAM_INTERNAL_IO_28, SIG_GPIO_OUT_IDX, false, false);
            esp_rom_gpio_connect_out_signal(PSRAM_INTERNAL_IO_29, SIG_GPIO_OUT_IDX, false, false);
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_clk_io as u32,
                SPICLK_OUT_IDX,
                false,
                false,
            );
        }

        // Update cs timing according to psram driving method.
        psram_set_cs_timing_spi1(mode, clk_mode);
        psram_set_cs_timing_spi0(mode, clk_mode); // SPI_CACHE_PORT
        psram_enable_qio_mode_spi1(clk_mode, mode);

        info!("PS-RAM vaddrmode = {:?}", PsramVaddrMode::Lowhigh);

        psram_cache_init(mode, PsramVaddrMode::Lowhigh, clk_mode, extra_dummy);
    }

    #[allow(unused)]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    enum PsramVaddrMode {
        /// App and pro CPU use their own flash cache for external RAM access
        Normal = 0,
        /// App and pro CPU share external RAM caches: pro CPU has low * 2M, app
        /// CPU has high 2M
        Lowhigh,
        ///  App and pro CPU share external RAM caches: pro CPU does even 32yte
        /// ranges, app does odd ones.
        Evenodd,
    }

    // register initialization for sram cache params and r/w commands
    fn psram_cache_init(
        psram_cache_mode: PsramCacheSpeed,
        vaddrmode: PsramVaddrMode,
        clk_mode: PsramClkMode,
        extra_dummy: u32,
    ) {
        unsafe {
            let spi = &*crate::peripherals::SPI0::PTR;
            info!(
                "PS-RAM cache_init, psram_cache_mode={:?}, extra_dummy={}, clk_mode={:?}",
                psram_cache_mode, extra_dummy, clk_mode
            );
            match psram_cache_mode {
                PsramCacheSpeed::PsramCacheF80mS80m => {
                    // flash 1 div clk,80+40;
                    // There's no register on bit 31. No information about it in IDF, nor TRM,
                    // so just doing it in this way.
                    spi.date().modify(|r, w| {
                        let current_bits = r.bits();
                        let new_bits = current_bits & !((1 << 31) | (1 << 30));
                        w.bits(new_bits)
                    });
                    // pre clk div , ONLY IF SPI/ SRAM@ DIFFERENT SPEED,JUST
                    // FOR SPI0. FLASH DIV
                    // 2+SRAM DIV4
                }
                PsramCacheSpeed::PsramCacheF80mS40m => {
                    spi.clock().modify(|_, w| w.clk_equ_sysclk().clear_bit());
                    spi.clock().modify(|_, w| w.clkdiv_pre().bits(0));
                    spi.clock().modify(|_, w| w.clkcnt_n().bits(1));
                    spi.clock().modify(|_, w| w.clkcnt_h().bits(0));
                    spi.clock().modify(|_, w| w.clkcnt_l().bits(1));

                    spi.date().modify(|r, w| {
                        let current_bits = r.bits();
                        let new_bits = (current_bits | (1 << 31)) & !(1 << 30);
                        w.bits(new_bits)
                    });
                }
                _ => {
                    spi.date().modify(|r, w| {
                        let current_bits = r.bits();
                        let new_bits = current_bits & !((1 << 31) | (1 << 30));
                        w.bits(new_bits)
                    });
                }
            }

            spi.cache_sctrl()
                .modify(|_, w| w.usr_sram_dio().clear_bit()); // disable dio mode for cache command
            spi.cache_sctrl().modify(|_, w| w.usr_sram_qio().set_bit()); // enable qio mode for cache command
            spi.cache_sctrl()
                .modify(|_, w| w.cache_sram_usr_rcmd().set_bit()); // enable cache read command
            spi.cache_sctrl()
                .modify(|_, w| w.cache_sram_usr_wcmd().set_bit()); // enable cache write command
            spi.cache_sctrl()
                .modify(|_, w| w.sram_addr_bitlen().bits(23)); // write address for cache command.
            spi.cache_sctrl()
                .modify(|_, w| w.usr_rd_sram_dummy().set_bit()); // enable cache read dummy

            // config sram cache r/w command
            spi.sram_drd_cmd()
                .modify(|_, w| w.cache_sram_usr_rd_cmd_bitlen().bits(7));
            spi.sram_drd_cmd().modify(|_, w| {
                w.cache_sram_usr_rd_cmd_value()
                    .bits(PSRAM_FAST_READ_QUAD as u16)
            });
            spi.sram_dwr_cmd()
                .modify(|_, w| w.cache_sram_usr_wr_cmd_bitlen().bits(7));
            spi.sram_dwr_cmd().modify(|_, w| {
                w.cache_sram_usr_wr_cmd_value()
                    .bits(PSRAM_QUAD_WRITE as u16)
            });

            // dummy, psram cache : 40m--+1dummy; 80m--+2dummy
            spi.cache_sctrl().modify(|_, w| {
                w.sram_dummy_cyclelen()
                    .bits((PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy) as u8)
            });

            match psram_cache_mode {
                PsramCacheSpeed::PsramCacheF80mS80m => (), // in this mode , no delay is needed
                _ => {
                    if clk_mode == PsramClkMode::PsramClkModeDclk {
                        spi.sram_drd_cmd()
                            .modify(|_, w| w.cache_sram_usr_rd_cmd_bitlen().bits(15)); // read command length, 2 bytes(1byte for delay),sending in qio mode in
                                                                                       // cache
                        spi.sram_drd_cmd().modify(|_, w| {
                            w.cache_sram_usr_rd_cmd_value()
                                .bits((PSRAM_FAST_READ_QUAD << 8) as u16)
                        }); // read command value,(0x00 for delay,0xeb for cmd)

                        spi.sram_dwr_cmd()
                            .modify(|_, w| w.cache_sram_usr_wr_cmd_bitlen().bits(15)); // write command length,2 bytes(1byte for delay,send in qio mode in cache)
                        spi.sram_dwr_cmd().modify(|_, w| {
                            w.cache_sram_usr_wr_cmd_value()
                                .bits((PSRAM_QUAD_WRITE << 8) as u16)
                        }); // write command value,(0x00 for delay)
                        spi.cache_sctrl().modify(|_, w| {
                            w.sram_dummy_cyclelen()
                                .bits((PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy) as u8)
                        }); // dummy, psram cache : 40m--+1dummy; 80m--+2dummy
                    }
                }
            }

            let dport = &*esp32::DPORT::PTR;

            dport
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_dram_hl().clear_bit().pro_dram_split().clear_bit());
            dport
                .app_cache_ctrl()
                .modify(|_, w| w.app_dram_hl().clear_bit().app_dram_split().clear_bit());
            if vaddrmode == PsramVaddrMode::Lowhigh {
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_dram_hl().set_bit());
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_dram_hl().set_bit());
            } else if vaddrmode == PsramVaddrMode::Evenodd {
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_dram_split().set_bit());
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_dram_split().set_bit());
            }

            // use Dram1 to visit ext sram. cache page mode : 1 -->16k  4 -->2k
            // 0-->32k,(accord with the settings in cache_sram_mmu_set)
            dport.pro_cache_ctrl1().modify(|_, w| {
                w.pro_cache_mask_dram1()
                    .clear_bit()
                    .pro_cache_mask_opsdram()
                    .clear_bit()
            });
            dport
                .pro_cache_ctrl1()
                .modify(|_, w| w.pro_cmmu_sram_page_mode().bits(0));

            // use Dram1 to visit ext sram. cache page mode : 1 -->16k  4 -->2k
            // 0-->32k,(accord with the settings in cache_sram_mmu_set)
            dport.app_cache_ctrl1().modify(|_, w| {
                w.app_cache_mask_dram1()
                    .clear_bit()
                    .app_cache_mask_opsdram()
                    .clear_bit()
            });
            dport
                .app_cache_ctrl1()
                .modify(|_, w| w.app_cmmu_sram_page_mode().bits(0));

            // ENABLE SPI0 CS1 TO PSRAM(CS0--FLASH; CS1--SRAM)
            spi.pin().modify(|_, w| w.cs1_dis().clear_bit());
        }
    }

    // spi param init for psram
    #[ram]
    fn psram_spi_init(
        // psram_spi_num_t spi_num = PSRAM_SPI_1,
        mode: PsramCacheSpeed,
        clk_mode: PsramClkMode,
    ) {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;
            // We need to clear last bit of INT_EN field here.
            // clear_peri_reg_mask(SPI1_SLAVE_REG, SPI_TRANS_DONE << 5);
            spi.slave().modify(|r, w| {
                let current_bits = r.int_en().bits();
                let new_bits = current_bits & !(1 << 4);
                w.int_en().bits(new_bits)
            });
            // SPI_CPOL & SPI_CPHA
            spi.pin().modify(|_, w| w.ck_idle_edge().clear_bit());
            spi.user().modify(|_, w| w.ck_out_edge().clear_bit());
            // SPI bit order
            spi.ctrl().modify(|_, w| w.wr_bit_order().clear_bit());
            spi.ctrl().modify(|_, w| w.rd_bit_order().clear_bit());
            // SPI bit order
            spi.user().modify(|_, w| w.doutdin().clear_bit());
            // May be not must to do.
            spi.user1().modify(|_, w| w.bits(0));
            // SPI mode type
            spi.slave().modify(|_, w| w.mode().clear_bit());

            let ptr = SPI1_W0_REG as *mut u32;
            for i in 0..16 {
                ptr.offset(i).write_volatile(0);
            }

            psram_set_cs_timing_spi1(mode, clk_mode);
        }
    }

    fn psram_set_cs_timing_spi1(psram_cache_mode: PsramCacheSpeed, clk_mode: PsramClkMode) {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;
            if clk_mode == PsramClkMode::PsramClkModeNorm {
                spi.user().modify(|_, w| w.cs_hold().set_bit());
                spi.user().modify(|_, w| w.cs_setup().set_bit());

                spi.ctrl2().modify(|_, w| {
                    w.hold_time()
                        .bits(psram_cs_hold_time_from_psram_speed(psram_cache_mode) as u8)
                });

                // Set cs time.
                spi.ctrl2().modify(|_, w| w.setup_time().bits(0));
            } else {
                spi.user().modify(|_, w| w.cs_hold().clear_bit());
                spi.user().modify(|_, w| w.cs_setup().clear_bit());
            }
        }
    }

    fn psram_set_cs_timing_spi0(psram_cache_mode: PsramCacheSpeed, clk_mode: PsramClkMode) {
        unsafe {
            let spi = &*crate::peripherals::SPI0::PTR;
            if clk_mode == PsramClkMode::PsramClkModeNorm {
                spi.user().modify(|_, w| w.cs_hold().set_bit());
                spi.user().modify(|_, w| w.cs_setup().set_bit());

                spi.ctrl2().modify(|_, w| {
                    w.hold_time()
                        .bits(psram_cs_hold_time_from_psram_speed(psram_cache_mode) as u8)
                });

                // Set cs time.
                spi.ctrl2().modify(|_, w| w.setup_time().bits(0));
            } else {
                spi.user().modify(|_, w| w.cs_hold().clear_bit());
                spi.user().modify(|_, w| w.cs_setup().clear_bit());
            }
        }
    }

    #[derive(Debug, Copy, Clone, PartialEq)]
    struct PsramCmd {
        cmd: u16,             // Command value
        cmd_bit_len: u16,     // Command byte length
        addr: *const u32,     // Point to address value
        addr_bit_len: u16,    // Address byte length
        tx_data: *const u32,  // Point to send data buffer
        tx_data_bit_len: u16, // Send data byte length.
        rx_data: *mut u32,    // Point to recevie data buffer
        rx_data_bit_len: u16, // Recevie Data byte length.
        dummy_bit_len: u32,
    }

    impl Default for PsramCmd {
        fn default() -> Self {
            Self {
                cmd: Default::default(),
                cmd_bit_len: Default::default(),
                addr: core::ptr::null(),
                addr_bit_len: Default::default(),
                tx_data: core::ptr::null(),
                tx_data_bit_len: Default::default(),
                rx_data: core::ptr::null_mut(),
                rx_data_bit_len: Default::default(),
                dummy_bit_len: Default::default(),
            }
        }
    }

    const PSRAM_ENTER_QMODE: u32 = 0x35;

    // enter QPI mode
    #[ram]
    fn psram_enable_qio_mode_spi1(clk_mode: PsramClkMode, psram_mode: PsramCacheSpeed) {
        let mut ps_cmd: PsramCmd = PsramCmd::default();
        let addr: u32 = PSRAM_ENTER_QMODE << 24;

        ps_cmd.cmd_bit_len = 0;
        if clk_mode == PsramClkMode::PsramClkModeDclk {
            match psram_mode {
                PsramCacheSpeed::PsramCacheF80mS80m => (),
                _ => {
                    ps_cmd.cmd_bit_len = 2;
                }
            }
        }
        ps_cmd.cmd = 0;
        ps_cmd.addr = &addr;
        ps_cmd.addr_bit_len = 8;
        ps_cmd.tx_data = core::ptr::null();
        ps_cmd.tx_data_bit_len = 0;
        ps_cmd.rx_data = core::ptr::null_mut();
        ps_cmd.rx_data_bit_len = 0;
        ps_cmd.dummy_bit_len = 0;
        let (backup_usr, backup_usr1, backup_usr2) = psram_cmd_config_spi1(&ps_cmd);
        psram_cmd_recv_start_spi1(core::ptr::null_mut(), 0, PsramCmdMode::PsramCmdSpi);
        psram_cmd_end_spi1(backup_usr, backup_usr1, backup_usr2);
    }

    #[ram]
    fn psram_cmd_end_spi1(backup_usr: u32, backup_usr1: u32, backup_usr2: u32) {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;
            loop {
                if spi.cmd().read().bits() & SPI_USR == 0 {
                    break;
                }
            }

            spi.user().modify(|_, w| w.bits(backup_usr));
            spi.user1().modify(|_, w| w.bits(backup_usr1));
            spi.user2().modify(|_, w| w.bits(backup_usr2));
        }
    }

    // setup spi command/addr/data/dummy in user mode
    #[ram]
    fn psram_cmd_config_spi1(p_in_data: &PsramCmd) -> (u32, u32, u32) {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;
            loop {
                if spi.cmd().read().bits() & SPI_USR == 0 {
                    break;
                }
            }

            let backup_usr = spi.user().read().bits();
            let backup_usr1 = spi.user1().read().bits();
            let backup_usr2 = spi.user2().read().bits();

            // Set command by user.
            if p_in_data.cmd_bit_len != 0 {
                // Max command length 16 bits.
                spi.user2().modify(|_, w| {
                    w.usr_command_bitlen()
                        .bits((p_in_data.cmd_bit_len - 1) as u8)
                });
                // Enable command
                spi.user().modify(|_, w| w.usr_command().set_bit());
                // Load command,bit15-0 is cmd value.
                spi.user2()
                    .modify(|_, w| w.usr_command_value().bits(p_in_data.cmd));
            } else {
                spi.user().modify(|_, w| w.usr_command().clear_bit());
                spi.user2().modify(|_, w| w.usr_command_bitlen().bits(0));
            }
            // Set Address by user.
            if p_in_data.addr_bit_len != 0 {
                spi.user1()
                    .modify(|_, w| w.usr_addr_bitlen().bits((p_in_data.addr_bit_len - 1) as u8));
                // Enable address
                spi.user().modify(|_, w| w.usr_addr().set_bit());
                // Set address
                spi.addr()
                    .modify(|_, w| w.bits(p_in_data.addr.read_volatile()));
            } else {
                spi.user().modify(|_, w| w.usr_addr().clear_bit());
                spi.user1().modify(|_, w| w.usr_addr_bitlen().bits(0));
            }
            // Set data by user.
            let p_tx_val = p_in_data.tx_data;
            if p_in_data.tx_data_bit_len != 0 {
                // Enable MOSI
                spi.user().modify(|_, w| w.usr_mosi().clear_bit());
                // Load send buffer
                let len = (p_in_data.tx_data_bit_len + 31) / 32;
                if !p_tx_val.is_null() {
                    for i in 0..len {
                        spi.w(0)
                            .modify(|_, w| w.bits(p_tx_val.offset(i as isize).read_volatile()));
                    }
                }
                // Set data send buffer length.Max data length 64 bytes.
                spi.mosi_dlen().modify(|_, w| {
                    w.usr_mosi_dbitlen()
                        .bits((p_in_data.tx_data_bit_len - 1) as u32)
                });
            } else {
                spi.user().modify(|_, w| w.usr_mosi().clear_bit());
                spi.mosi_dlen().modify(|_, w| w.usr_mosi_dbitlen().bits(0));
            }
            // Set rx data by user.
            if p_in_data.rx_data_bit_len != 0 {
                // Enable MISO
                spi.user().modify(|_, w| w.usr_miso().set_bit());
                // Set data send buffer length.Max data length 64 bytes.
                spi.miso_dlen().modify(|_, w| {
                    w.usr_miso_dbitlen()
                        .bits((p_in_data.rx_data_bit_len - 1) as u32)
                });
            } else {
                spi.user().modify(|_, w| w.usr_miso().clear_bit());
                spi.miso_dlen().modify(|_, w| w.usr_miso_dbitlen().bits(0));
            }
            if p_in_data.dummy_bit_len != 0 {
                spi.user().modify(|_, w| w.usr_dummy().set_bit()); // dummy en
                spi.user1().modify(|_, w| {
                    w.usr_dummy_cyclelen()
                        .bits((p_in_data.dummy_bit_len - 1) as u8)
                }); // DUMMY
            } else {
                spi.user().modify(|_, w| w.usr_dummy().clear_bit()); // dummy dis
                spi.user1().modify(|_, w| w.usr_dummy_cyclelen().bits(0)); // DUMMY
            }

            (backup_usr, backup_usr1, backup_usr2)
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    enum PsramCmdMode {
        PsramCmdQpi,
        PsramCmdSpi,
    }

    // start sending cmd/addr and optionally, receiving data
    #[ram]
    fn psram_cmd_recv_start_spi1(
        p_rx_data: *mut u32,
        rx_data_len_words: usize,
        cmd_mode: PsramCmdMode,
    ) {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;
            // get cs1
            spi.pin().modify(|_, w| w.cs1_dis().clear_bit());
            spi.pin().modify(|_, w| w.cs0_dis().clear_bit());

            let mode_backup: u32 = (spi.user().read().bits() >> SPI_FWRITE_DUAL_S) & 0xf;
            let rd_mode_backup: u32 = spi.ctrl().read().bits()
                & (SPI_FREAD_DIO_M | SPI_FREAD_DUAL_M | SPI_FREAD_QUAD_M | SPI_FREAD_QIO_M);

            if cmd_mode == PsramCmdMode::PsramCmdSpi {
                psram_set_basic_write_mode_spi1();
                psram_set_basic_read_mode_spi1();
            } else if cmd_mode == PsramCmdMode::PsramCmdQpi {
                psram_set_qio_write_mode_spi1();
                psram_set_qio_read_mode_spi1();
            }

            // Wait for SPI0 to idle
            loop {
                if spi.ext2().read().bits() == 0 {
                    break;
                }
            }

            // DPORT_SET_PERI_REG_MASK(DPORT_HOST_INF_SEL_REG, 1 << 14);
            let dport = &*esp32::DPORT::PTR;
            dport
                .host_inf_sel()
                .modify(|r, w| w.bits(r.bits() | 1 << 14));

            // Start send data
            spi.cmd().modify(|_, w| w.usr().set_bit());
            loop {
                if spi.cmd().read().bits() & SPI_USR == 0 {
                    break;
                }
            }

            // DPORT_CLEAR_PERI_REG_MASK(DPORT_HOST_INF_SEL_REG, 1 << 14);
            let dport = &*esp32::DPORT::PTR;
            dport
                .host_inf_sel()
                .modify(|r, w| w.bits(r.bits() & !(1 << 14)));

            // recover spi mode
            // TODO: get back to this, why writing on `0xf` address?
            set_peri_reg_bits(
                SPI1_USER_REG,
                if !p_rx_data.is_null() {
                    SPI_FWRITE_DUAL_M
                } else {
                    0xf
                },
                mode_backup,
                SPI_FWRITE_DUAL_S,
            );

            spi.ctrl().modify(|_, w| w.fread_dio().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_dual().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_quad().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_qio().clear_bit());

            spi.ctrl().modify(|_, w| w.bits(rd_mode_backup));

            // return cs to cs0
            spi.pin().modify(|_, w| w.cs1_dis().set_bit());
            spi.pin().modify(|_, w| w.cs0_dis().clear_bit());

            if !p_rx_data.is_null() {
                // Read data out
                for i in 0..rx_data_len_words {
                    p_rx_data.add(i).write_volatile(spi.w(i).read().bits());
                }
            }
        }
    }

    // set basic SPI write mode
    fn psram_set_basic_write_mode_spi1() {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;

            spi.user().modify(|_, w| w.fwrite_qio().clear_bit());
            spi.user().modify(|_, w| w.fwrite_dio().clear_bit());
            spi.user().modify(|_, w| w.fwrite_quad().clear_bit());
            spi.user().modify(|_, w| w.fwrite_dual().clear_bit());
        }
    }
    // set QPI write mode
    fn psram_set_qio_write_mode_spi1() {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;

            spi.user().modify(|_, w| w.fwrite_qio().set_bit());
            spi.user().modify(|_, w| w.fwrite_dio().clear_bit());
            spi.user().modify(|_, w| w.fwrite_quad().clear_bit());
            spi.user().modify(|_, w| w.fwrite_dual().clear_bit());
        }
    }
    // set QPI read mode
    fn psram_set_qio_read_mode_spi1() {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;

            spi.ctrl().modify(|_, w| w.fread_qio().set_bit());
            spi.ctrl().modify(|_, w| w.fread_quad().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_dual().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_dio().clear_bit());
        }
    }
    // set SPI read mode
    fn psram_set_basic_read_mode_spi1() {
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;

            spi.ctrl().modify(|_, w| w.fread_qio().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_quad().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_dual().clear_bit());
            spi.ctrl().modify(|_, w| w.fread_dio().clear_bit());
        }
    }

    // psram gpio init , different working frequency we have different solutions
    fn psram_gpio_config(psram_io: &PsramIo, mode: PsramCacheSpeed) -> u32 {
        unsafe {
            let spi = &*crate::peripherals::SPI0::PTR;
            let g_rom_spiflash_dummy_len_plus_ptr = addr_of_mut!(g_rom_spiflash_dummy_len_plus);

            #[derive(Debug, Clone, Copy)]
            enum Field {
                McuSel,
                FunDrv,
            }

            macro_rules! apply_to_field {
                ($w:ident, $field:expr, $bits:expr) => {
                    match $field {
                        Field::McuSel => $w.mcu_sel().bits($bits),
                        Field::FunDrv => $w.fun_drv().bits($bits),
                    }
                };
            }

            fn configure_gpio(gpio: u8, field: Field, bits: u8) {
                unsafe {
                    let ptr = crate::gpio::io_mux_reg(gpio);
                    ptr.modify(|_, w| apply_to_field!(w, field, bits));
                }
            }

            let spi_cache_dummy;
            let rd_mode_reg = spi.ctrl().read().bits();
            if (rd_mode_reg & SPI_FREAD_QIO_M) != 0 {
                spi_cache_dummy = SPI0_R_QIO_DUMMY_CYCLELEN;
            } else if (rd_mode_reg & SPI_FREAD_DIO_M) != 0 {
                spi_cache_dummy = SPI0_R_DIO_DUMMY_CYCLELEN;
                spi.user1()
                    .modify(|_, w| w.usr_addr_bitlen().bits(SPI0_R_DIO_ADDR_BITSLEN as u8));
            } else {
                spi_cache_dummy = SPI0_R_FAST_DUMMY_CYCLELEN;
            }

            let extra_dummy;

            match mode {
                PsramCacheSpeed::PsramCacheF80mS40m => {
                    extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;

                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);

                    spi.user1().modify(|_, w| {
                        w.usr_dummy_cyclelen()
                            .bits(spi_cache_dummy as u8 + PSRAM_IO_MATRIX_DUMMY_80M)
                    }); // DUMMY

                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);

                    // set drive ability for clock

                    configure_gpio(psram_io.flash_clk_io, Field::FunDrv, 3);
                    configure_gpio(psram_io.psram_clk_io, Field::FunDrv, 2);
                }
                PsramCacheSpeed::PsramCacheF80mS80m => {
                    extra_dummy = PSRAM_IO_MATRIX_DUMMY_80M;
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);

                    spi.user1().modify(|_, w| {
                        w.usr_dummy_cyclelen()
                            .bits(spi_cache_dummy as u8 + PSRAM_IO_MATRIX_DUMMY_80M)
                    }); // DUMMY

                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_FLASH_PORT);

                    // set drive ability for clock
                    configure_gpio(psram_io.flash_clk_io, Field::FunDrv, 3);
                    configure_gpio(psram_io.psram_clk_io, Field::FunDrv, 3);
                }
                PsramCacheSpeed::PsramCacheF40mS40m => {
                    extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;

                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);

                    spi.user1().modify(|_, w| {
                        w.usr_dummy_cyclelen()
                            .bits(spi_cache_dummy as u8 + PSRAM_IO_MATRIX_DUMMY_40M)
                    }); // DUMMY

                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);

                    // set drive ability for clock
                    configure_gpio(psram_io.flash_clk_io, Field::FunDrv, 2);
                    configure_gpio(psram_io.psram_clk_io, Field::FunDrv, 2);
                }
            }

            spi.user().modify(|_, w| w.usr_dummy().set_bit()); // dummy enable

            // In bootloader, all the signals are already configured,
            // We keep the following code in case the bootloader is some older version.

            esp_rom_gpio_connect_out_signal(
                psram_io.flash_cs_io as u32,
                SPICS0_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_cs_io as u32,
                SPICS1_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_spiq_sd0_io as u32,
                SPIQ_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_in_signal(psram_io.psram_spiq_sd0_io as u32, SPIQ_IN_IDX, false);
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_spid_sd1_io as u32,
                SPID_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_in_signal(psram_io.psram_spid_sd1_io as u32, SPID_IN_IDX, false);
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_spiwp_sd3_io as u32,
                SPIWP_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_in_signal(psram_io.psram_spiwp_sd3_io as u32, SPIWP_IN_IDX, false);
            esp_rom_gpio_connect_out_signal(
                psram_io.psram_spihd_sd2_io as u32,
                SPIHD_OUT_IDX,
                false,
                false,
            );
            esp_rom_gpio_connect_in_signal(psram_io.psram_spihd_sd2_io as u32, SPIHD_IN_IDX, false);

            // select pin function gpio
            if (psram_io.flash_clk_io == SPI_IOMUX_PIN_NUM_CLK)
                && (psram_io.flash_clk_io != psram_io.psram_clk_io)
            {
                // flash clock signal should come from IO MUX.
                configure_gpio(
                    psram_io.flash_clk_io,
                    Field::McuSel,
                    FUNC_SD_CLK_SPICLK as u8,
                );
            } else {
                // flash clock signal should come from GPIO matrix.
                configure_gpio(psram_io.flash_clk_io, Field::McuSel, PIN_FUNC_GPIO as u8);
            }
            configure_gpio(psram_io.flash_cs_io, Field::McuSel, PIN_FUNC_GPIO as u8);
            configure_gpio(psram_io.psram_cs_io, Field::McuSel, PIN_FUNC_GPIO as u8);
            configure_gpio(psram_io.psram_clk_io, Field::McuSel, PIN_FUNC_GPIO as u8);
            configure_gpio(
                psram_io.psram_spiq_sd0_io,
                Field::McuSel,
                PIN_FUNC_GPIO as u8,
            );
            configure_gpio(
                psram_io.psram_spid_sd1_io,
                Field::McuSel,
                PIN_FUNC_GPIO as u8,
            );
            configure_gpio(
                psram_io.psram_spihd_sd2_io,
                Field::McuSel,
                PIN_FUNC_GPIO as u8,
            );
            configure_gpio(
                psram_io.psram_spiwp_sd3_io,
                Field::McuSel,
                PIN_FUNC_GPIO as u8,
            );

            let flash_id: u32 = g_rom_flashchip.device_id;
            info!("Flash-ID = {}", flash_id);
            info!("Flash size = {}", g_rom_flashchip.chip_size);

            if flash_id == FLASH_ID_GD25LQ32C {
                // Set drive ability for 1.8v flash in 80Mhz.
                configure_gpio(psram_io.flash_cs_io, Field::FunDrv, 3);
                configure_gpio(psram_io.flash_clk_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_cs_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_clk_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_spiq_sd0_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_spid_sd1_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_spihd_sd2_io, Field::FunDrv, 3);
                configure_gpio(psram_io.psram_spiwp_sd3_io, Field::FunDrv, 3);
            }

            extra_dummy as u32
        }
    }

    fn set_peri_reg_bits(reg: u32, bitmap: u32, value: u32, shift: u32) {
        unsafe {
            (reg as *mut u32).write_volatile(
                ((reg as *mut u32).read_volatile() & !(bitmap << shift))
                    | ((value & bitmap) << shift),
            );
        }
    }
}
