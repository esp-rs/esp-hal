use core::ops::Range;

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::peripherals::EXTMEM;

/// Frequency of PSRAM memory
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiRamFreq {
    /// PSRAM frequency 20 MHz
    Freq20m = 20,
    /// PSRAM frequency 26 MHz
    Freq26m = 26,
    /// PSRAM frequency 40 MHz
    Freq40m = 40,
    /// PSRAM frequency 80 MHz. Default for Espressif modules.
    #[default]
    Freq80m = 80,
}

impl SpiRamFreq {
    fn divider(self) -> u32 {
        match self {
            Self::Freq80m => 1,
            Self::Freq40m => 2,
            Self::Freq26m => 3,
            Self::Freq20m => 4,
        }
    }
}

/// PSRAM configuration
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramConfig {
    /// PSRAM size
    pub size: PsramSize,
    /// Frequency of PSRAM memory
    pub ram_frequency: SpiRamFreq,
}

/// Initializes PSRAM to be used for data.
#[procmacros::ram]
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    let success = quad_spi_impl::psram_init(config);
    if !success {
        warn!(
            "Failed to configure PSRAM. This may indicate a missing/inoperable PSRAM chip, or an incorrect PSRAM configuration. Check if the PSRAM chip is present and the configuration is correct."
        );
    }
    success
}

#[procmacros::ram]
pub(crate) fn map_psram(config: PsramConfig) -> Range<usize> {
    const MMU_ACCESS_SPIRAM: u32 = 1 << 16;

    unsafe extern "C" {
        /// Sets DCache mmu mapping.
        ///
        /// [`ext_ram`]: u32 DPORT_MMU_ACCESS_FLASH for flash, DPORT_MMU_ACCESS_SPIRAM for spiram, DPORT_MMU_INVALID for invalid.
        /// [`vaddr`]: u32 Virtual address in CPU address space.
        /// [`paddr`]: u32 Physical address in external memory. Should be aligned by psize.
        /// [`psize`]: u32 Page size of DCache, in kilobytes. Should be 64 here.
        /// [`num`]: u32 Pages to be set.
        /// [`fixes`]: u32 0 for physical pages grow with virtual pages, other for virtual pages map to same physical page.
        fn cache_dbus_mmu_set(
            ext_ram: u32,
            vaddr: u32,
            paddr: u32,
            psize: u32,
            num: u32,
            fixed: u32,
        ) -> i32;
    }

    const START_PAGE: u32 = 0;

    let cache_dbus_mmu_set_res = unsafe {
        cache_dbus_mmu_set(
            MMU_ACCESS_SPIRAM,
            EXTMEM_ORIGIN as u32,
            START_PAGE << 16,
            64,
            config.size.get() as u32 / 1024 / 64,
            0,
        )
    };

    if cache_dbus_mmu_set_res != 0 {
        panic!("cache_dbus_mmu_set failed");
    }

    EXTMEM::regs().pro_dcache_ctrl1().modify(|_, w| {
        w.pro_dcache_mask_bus0().clear_bit();
        w.pro_dcache_mask_bus1().clear_bit();
        w.pro_dcache_mask_bus2().clear_bit()
    });

    EXTMEM_ORIGIN..EXTMEM_ORIGIN + config.size.get()
}

pub(crate) mod quad_spi_impl {
    use procmacros::ram;

    use super::*;
    use crate::psram::quad_xtensa;

    #[ram]
    pub(crate) fn psram_init(config: &mut PsramConfig) -> bool {
        psram_gpio_config();

        if config.size.is_auto() {
            let Some(size) = quad_xtensa::detect_quad_size() else {
                return false;
            };
            config.size = PsramSize::Size(size);
        }

        quad_xtensa::psram_reset_mode_spi1();
        quad_xtensa::psram_enable_qio_mode_spi1();
        quad_xtensa::config_psram_spi_phases();
        mspi_timing_enter_high_speed_mode(config);

        info!("PSRAM initialized successfully in Quad SPI mode");
        true
    }

    #[ram]
    fn mspi_timing_enter_high_speed_mode(config: &PsramConfig) {
        let psram_div = config.ram_frequency.divider();
        info!(
            "PSRAM {} MHz, psram_div = {}",
            config.ram_frequency as u32, psram_div
        );
        quad_xtensa::spi0_timing_config_set_psram_clock(psram_div);
    }

    #[repr(C)]
    struct PsRamIo {
        flash_clk_io: u8,
        flash_cs_io: u8,
        psram_clk_io: u8,
        psram_cs_io: u8,
        psram_spiq_sd0_io: u8,
        psram_spid_sd1_io: u8,
        psram_spiwp_sd3_io: u8,
        psram_spihd_sd2_io: u8,
    }

    #[ram]
    fn psram_gpio_config() {
        unsafe extern "C" {
            fn esp_rom_efuse_get_flash_gpio_info() -> u32;

            /// Enables Quad I/O pin functions.
            ///
            /// Sets the HD & WP pin functions for Quad I/O modes, based on the
            /// efuse SPI pin configuration.
            ///
            /// [`wp_gpio_num`]: u8 Number of the WP pin to reconfigure for quad I/O
            /// [`spiconfig`]: u32 Pin configuration, as returned from ets_efuse_get_spiconfig().
            /// - If this parameter is 0, default SPI pins are used and wp_gpio_num parameter is
            ///   ignored.
            /// - If this parameter is 1, default HSPI pins are used and wp_gpio_num parameter is
            ///   ignored.
            /// - For other values, this parameter encodes the HD pin number and also the CLK pin
            ///   number. CLK pin selection is used to determine if HSPI or SPI peripheral will be
            ///   used (use HSPI if CLK pin is the HSPI clock pin, otherwise use SPI). Both HD & WP
            ///   pins are configured via GPIO matrix to map to the selected peripheral.
            fn esp_rom_spiflash_select_qio_pins(wp_gpio_num: u8, spiconfig: u32);
        }

        let psram_io = PsRamIo {
            flash_clk_io: 30, // SPI_CLK_GPIO_NUM
            flash_cs_io: 29,  // SPI_CS0_GPIO_NUM
            psram_clk_io: 30,
            psram_cs_io: 26,        // SPI_CS1_GPIO_NUM
            psram_spiq_sd0_io: 31,  // SPI_Q_GPIO_NUM
            psram_spid_sd1_io: 32,  // SPI_D_GPIO_NUM
            psram_spiwp_sd3_io: 28, // SPI_WP_GPIO_NUM
            psram_spihd_sd2_io: 27, // SPI_HD_GPIO_NUM
        };

        const ESP_ROM_EFUSE_FLASH_DEFAULT_SPI: u32 = 0;

        unsafe {
            let spiconfig = esp_rom_efuse_get_flash_gpio_info();
            if spiconfig == ESP_ROM_EFUSE_FLASH_DEFAULT_SPI {
                // FLASH pins(except wp / hd) are all configured via IO_MUX in
                // rom.
            } else {
                panic!(
                    "Unsupported for now! The case 'FLASH pins are all configured via GPIO matrix in ROM.' is not yet supported."
                );
            }
            esp_rom_spiflash_select_qio_pins(psram_io.psram_spiwp_sd3_io, spiconfig);
        }
    }
}
