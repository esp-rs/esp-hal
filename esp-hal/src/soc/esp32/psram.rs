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
//! The `PSRAM` module is accessed through a virtual address, defined as
//! `PSRAM_VADDR`. The starting virtual address for the PSRAM module is
//! 0x3F800000. The `PSRAM` module size depends on the configuration specified
//! during the compilation process. The available `PSRAM` sizes are `2MB`,
//! `4MB`, and `8MB`.
//!
//! NOTE: If you want to use `PSRAM` on `ESP32` or `ESP32-S3`, it'll work only
//! in `release` mode.

const PSRAM_VADDR: u32 = 0x3F800000;

pub fn psram_vaddr_start() -> usize {
    PSRAM_VADDR_START
}

cfg_if::cfg_if! {
    if #[cfg(feature = "psram-2m")] {
        const PSRAM_SIZE: u32 = 2;
    } else if #[cfg(feature = "psram-4m")] {
        const PSRAM_SIZE: u32 = 4;
    } else if #[cfg(feature = "psram-8m")] {
        const PSRAM_SIZE: u32 = 8;
    } else {
        const PSRAM_SIZE: u32 = 0;
    }
}

pub const PSRAM_BYTES: usize = PSRAM_SIZE as usize * 1024 * 1024;

pub const PSRAM_VADDR_START: usize = PSRAM_VADDR as usize;

#[cfg(any(feature = "psram-2m", feature = "psram-4m", feature = "psram-8m"))]
pub fn init_psram(_peripheral: impl crate::peripheral::Peripheral<P = crate::peripherals::PSRAM>) {
    utils::psram_init();
    utils::s_mapping(PSRAM_VADDR, PSRAM_BYTES as u32);
}

#[cfg(any(feature = "psram-2m", feature = "psram-4m", feature = "psram-8m"))]
pub(crate) mod utils {
    use core::ptr::addr_of_mut;

    use procmacros::ram;

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
    const SPI1_SLAVE_REG: u32 = DR_REG_SPI1_BASE + 0x038;
    const SPI1_PIN_REG: u32 = DR_REG_SPI1_BASE + 0x34;
    const SPI1_CTRL_REG: u32 = DR_REG_SPI1_BASE + 0x8;
    const SPI1_USER1_REG: u32 = DR_REG_SPI1_BASE + 0x20;
    const SPI1_W0_REG: u32 = DR_REG_SPI1_BASE + 0x80;
    const SPI1_CTRL2_REG: u32 = DR_REG_SPI1_BASE + 0x14;
    const SPI1_CMD_REG: u32 = DR_REG_SPI1_BASE;
    const SPI1_USER2_REG: u32 = DR_REG_SPI1_BASE + 0x24;
    const SPI1_MOSI_DLEN_REG: u32 = DR_REG_SPI1_BASE + 0x28;
    const SPI1_MISO_DLEN_REG: u32 = DR_REG_SPI1_BASE + 0x2C;
    const SPI1_ADDR_REG: u32 = DR_REG_SPI1_BASE + 0x4;

    const DR_REG_SPI0_BASE: u32 = 0x3ff43000;
    const SPI0_EXT2_REG: u32 = DR_REG_SPI0_BASE + 0xf8;
    const SPI0_EXT3_REG: u32 = DR_REG_SPI0_BASE + 0xfc;
    const SPI0_USER_REG: u32 = DR_REG_SPI0_BASE + 0x1C;
    const SPI0_PIN_REG: u32 = DR_REG_SPI0_BASE + 0x34;
    const SPI0_CTRL_REG: u32 = DR_REG_SPI0_BASE + 0x8;
    const SPI0_USER1_REG: u32 = DR_REG_SPI0_BASE + 0x20;
    const SPI0_CTRL2_REG: u32 = DR_REG_SPI0_BASE + 0x14;
    const SPI0_DATE_REG: u32 = DR_REG_SPI0_BASE + 0x3FC;
    const SPI0_CLOCK_REG: u32 = DR_REG_SPI0_BASE + 0x18;
    const SPI0_CACHE_SCTRL_REG: u32 = DR_REG_SPI0_BASE + 0x54;
    const SPI0_SRAM_DRD_CMD_REG: u32 = DR_REG_SPI0_BASE + 0x5C;
    const SPI0_SRAM_DWR_CMD_REG: u32 = DR_REG_SPI0_BASE + 0x60;

    const SPI_USR_PREP_HOLD_M: u32 = 1 << 23;
    const SPI_TRANS_DONE: u32 = 1 << 4;
    const SPI_CK_IDLE_EDGE: u32 = 1 << 29;
    const SPI_CK_OUT_EDGE: u32 = 1 << 7;
    const SPI_WR_BIT_ORDER: u32 = 1 << 26;
    const SPI_RD_BIT_ORDER: u32 = 1 << 25;
    const SPI_DOUTDIN: u32 = 1 << 0;
    const SPI_SLAVE_MODE: u32 = 1 << 30;
    const SPI_CS_HOLD_M: u32 = 1 << 4;
    const SPI_CS_SETUP_M: u32 = 1 << 5;
    const SPI_HOLD_TIME_V: u32 = 0xF;

    const fn psram_cs_hold_time_from_psram_speed(speed: PsramCacheSpeed) -> u32 {
        match speed {
            PsramCacheSpeed::PsramCacheF80mS40m => 0,
            PsramCacheSpeed::PsramCacheF40mS40m => 0,
            PsramCacheSpeed::PsramCacheF80mS80m => 1,
        }
    }

    const SPI_HOLD_TIME_S: u32 = 4;
    const SPI_SETUP_TIME_V: u32 = 0xF;
    const SPI_SETUP_TIME_S: u32 = 0;
    const SPI_USR_DUMMY: u32 = 1 << 29;

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

    const FUN_DRV_V: u32 = 0x3;
    const FUN_DRV_S: u32 = 10;
    const FUN_DRV: u32 = 0x3;

    const SPI_CLK_EQU_SYSCLK_M: u32 = 1 << 31;
    const SPI_CLKDIV_PRE_V: u32 = 0x1FFF;
    const SPI_CLKDIV_PRE_S: u32 = 18;
    const SPI_CLKCNT_N: u32 = 0x0000003F;
    const SPI_CLKCNT_N_S: u32 = 12;
    const SPI_CLKCNT_H: u32 = 0x0000003F;
    const SPI_CLKCNT_H_S: u32 = 6;
    const SPI_CLKCNT_L: u32 = 0x0000003F;
    const SPI_CLKCNT_L_S: u32 = 0;
    const SPI_USR_SRAM_DIO_M: u32 = 1 << 1;
    const SPI_USR_SRAM_QIO_M: u32 = 1 << 2;
    const SPI_CACHE_SRAM_USR_RCMD_M: u32 = 1 << 5;
    const SPI_CACHE_SRAM_USR_WCMD_M: u32 = 1 << 28;
    const SPI_SRAM_ADDR_BITLEN_V: u32 = 0x3F;
    const SPI_USR_RD_SRAM_DUMMY_M: u32 = 1 << 4;
    const SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_V: u32 = 0xF;
    const SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_S: u32 = 28;
    const SPI_CACHE_SRAM_USR_RD_CMD_VALUE_V: u32 = 0xFFFF;
    const SPI_CACHE_SRAM_USR_RD_CMD_VALUE_S: u32 = 0;
    const SPI_CACHE_SRAM_USR_WR_CMD_BITLEN: u32 = 0x0000000F;
    const SPI_CACHE_SRAM_USR_WR_CMD_BITLEN_S: u32 = 28;
    const SPI_CACHE_SRAM_USR_WR_CMD_VALUE: u32 = 0x0000FFFF;
    const PSRAM_QUAD_WRITE: u32 = 0x38;
    const SPI_CACHE_SRAM_USR_WR_CMD_VALUE_S: u32 = 0;
    const SPI_SRAM_DUMMY_CYCLELEN_V: u32 = 0xFF;
    const PSRAM_FAST_READ_QUAD_DUMMY: u32 = 0x5;
    const SPI_SRAM_DUMMY_CYCLELEN_S: u32 = 14;
    const SPI_SRAM_ADDR_BITLEN_S: u32 = 22;
    const PSRAM_FAST_READ_QUAD: u32 = 0xEB;

    const SPI_USR: u32 = 1 << 18;
    const SPI_USR_COMMAND_BITLEN: u32 = 0x0000000F;
    const SPI_USR_COMMAND_BITLEN_S: u32 = 28;
    const SPI_USR_COMMAND: u32 = 1 << 31;
    const SPI_USR_COMMAND_VALUE: u32 = 0x0000FFFF;
    const SPI_USR_COMMAND_VALUE_S: u32 = 0;
    const SPI_USR_ADDR_BITLEN: u32 = 0x0000003F;
    const SPI_USR_ADDR: u32 = 1 << 30;
    const SPI_USR_MOSI: u32 = 1 << 27;
    const SPI_USR_MISO_DBITLEN: u32 = 0x00FFFFFF;
    const SPI_USR_MOSI_DBITLEN: u32 = 0x00FFFFFF;
    const SPI_USR_MOSI_DBITLEN_S: u32 = 0;
    const SPI_USR_MISO_DBITLEN_S: u32 = 0;
    const SPI_USR_MISO: u32 = 1 << 28;
    const SPI_FWRITE_DUAL_S: u32 = 12;
    const SPI_FWRITE_DUAL_M: u32 = 1 << 12;

    const SPI_CS1_DIS_M: u32 = 1 << 1;
    const SPI_CS0_DIS_M: u32 = 1 << 0;
    const SPI_FWRITE_QIO: u32 = 1 << 15;
    const SPI_FWRITE_DIO: u32 = 1 << 14;
    const SPI_FWRITE_QUAD: u32 = 1 << 13;
    const SPI_FWRITE_DUAL: u32 = 1 << 12;
    const SPI_FREAD_QIO: u32 = 1 << 24;
    const SPI_FREAD_QUAD: u32 = 1 << 20;
    const SPI_FREAD_DUAL: u32 = 1 << 14;
    const SPI_FREAD_DIO: u32 = 1 << 23;

    const SPI_FREAD_QIO_M: u32 = 1 << 24;
    const SPI0_R_QIO_DUMMY_CYCLELEN: u32 = 3;
    const SPI_FREAD_DIO_M: u32 = 1 << 23;
    const SPI0_R_DIO_DUMMY_CYCLELEN: u32 = 1;
    const SPI_USR_ADDR_BITLEN_V: u32 = 0x3F;
    const SPI0_R_DIO_ADDR_BITSLEN: u32 = 27;
    const SPI_USR_ADDR_BITLEN_S: u32 = 26;
    const SPI_FREAD_QUAD_M: u32 = 1 << 20;
    const SPI_FREAD_DUAL_M: u32 = 1 << 14;
    const SPI0_R_FAST_DUMMY_CYCLELEN: u32 = 7;
    const PSRAM_IO_MATRIX_DUMMY_40M: u8 = 1;
    const PSRAM_IO_MATRIX_DUMMY_80M: u8 = 2;

    const _SPI_CACHE_PORT: u8 = 0;
    const _SPI_FLASH_PORT: u8 = 1;
    const SPI_USR_DUMMY_CYCLELEN_V: u32 = 0xFF;
    const SPI_USR_DUMMY_CYCLELEN_S: u32 = 0;
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

    #[derive(PartialEq, Eq, Clone, Copy, Debug)]
    #[allow(unused)]
    enum PsramCacheSpeed {
        PsramCacheF80mS40m = 0,
        PsramCacheF40mS40m,
        PsramCacheF80mS80m,
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
    enum PsramClkMode {
        PsramClkModeNorm = 0, // Normal SPI mode
        PsramClkModeDclk = 1, // Two extra clock cycles after CS is set high level
    }

    #[repr(C)]
    struct EspRomSpiflashChip {
        device_id: u32,
        chip_size: u32, // chip size in bytes
        block_size: u32,
        sector_size: u32,
        page_size: u32,
        status_mask: u32,
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

        static g_rom_flashchip: EspRomSpiflashChip;

        fn cache_sram_mmu_set_rom(
            cpu_no: u32,
            pid: u32,
            vaddr: u32,
            paddr: u32,
            psize: u32,
            num: u32,
        ) -> i32;
    }

    pub(crate) fn psram_init() {
        let chip = crate::efuse::Efuse::get_chip_type();

        let mode = PsramCacheSpeed::PsramCacheF40mS40m; // How to make this configurable
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

        write_peri_reg(SPI0_EXT3_REG, 0x1);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_PREP_HOLD_M);

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
        info!(
            "PS-RAM cache_init, psram_cache_mode={:?}, extra_dummy={}, clk_mode={:?}",
            psram_cache_mode, extra_dummy, clk_mode
        );
        match psram_cache_mode {
            PsramCacheSpeed::PsramCacheF80mS80m => {
                // flash 1 div clk,80+40;
                clear_peri_reg_mask(SPI0_DATE_REG, 1 << 31);

                // pre clk div , ONLY IF SPI/ SRAM@ DIFFERENT SPEED,JUST FOR SPI0. FLASH DIV
                // 2+SRAM DIV4
                clear_peri_reg_mask(SPI0_DATE_REG, 1 << 30);
            }
            PsramCacheSpeed::PsramCacheF80mS40m => {
                clear_peri_reg_mask(SPI0_CLOCK_REG, SPI_CLK_EQU_SYSCLK_M);
                set_peri_reg_bits(SPI0_CLOCK_REG, SPI_CLKDIV_PRE_V, 0, SPI_CLKDIV_PRE_S);
                set_peri_reg_bits(SPI0_CLOCK_REG, SPI_CLKCNT_N, 1, SPI_CLKCNT_N_S);
                set_peri_reg_bits(SPI0_CLOCK_REG, SPI_CLKCNT_H, 0, SPI_CLKCNT_H_S);
                set_peri_reg_bits(SPI0_CLOCK_REG, SPI_CLKCNT_L, 1, SPI_CLKCNT_L_S);

                // flash 1 div clk
                set_peri_reg_mask(SPI0_DATE_REG, 1 << 31);

                // pre clk div ONLY IF SPI/SRAM@ DIFFERENT SPEED,JUST FOR SPI0.
                clear_peri_reg_mask(SPI0_DATE_REG, 1 << 30);
            }
            _ => {
                clear_peri_reg_mask(SPI0_DATE_REG, 1 << 31); // flash 1 div clk
                clear_peri_reg_mask(SPI0_DATE_REG, 1 << 30); // pre clk div
            }
        }

        clear_peri_reg_mask(SPI0_CACHE_SCTRL_REG, SPI_USR_SRAM_DIO_M); // disable dio mode for cache command
        set_peri_reg_mask(SPI0_CACHE_SCTRL_REG, SPI_USR_SRAM_QIO_M); // enable qio mode for cache command
        set_peri_reg_mask(SPI0_CACHE_SCTRL_REG, SPI_CACHE_SRAM_USR_RCMD_M); // enable cache read command
        set_peri_reg_mask(SPI0_CACHE_SCTRL_REG, SPI_CACHE_SRAM_USR_WCMD_M); // enable cache write command
        set_peri_reg_bits(
            SPI0_CACHE_SCTRL_REG,
            SPI_SRAM_ADDR_BITLEN_V,
            23,
            SPI_SRAM_ADDR_BITLEN_S,
        ); // write address for cache command.
        set_peri_reg_mask(SPI0_CACHE_SCTRL_REG, SPI_USR_RD_SRAM_DUMMY_M); // enable cache read dummy

        // config sram cache r/w command
        set_peri_reg_bits(
            SPI0_SRAM_DRD_CMD_REG,
            SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_V,
            7,
            SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_S,
        );
        set_peri_reg_bits(
            SPI0_SRAM_DRD_CMD_REG,
            SPI_CACHE_SRAM_USR_RD_CMD_VALUE_V,
            PSRAM_FAST_READ_QUAD,
            SPI_CACHE_SRAM_USR_RD_CMD_VALUE_S,
        ); // 0xEB
        set_peri_reg_bits(
            SPI0_SRAM_DWR_CMD_REG,
            SPI_CACHE_SRAM_USR_WR_CMD_BITLEN,
            7,
            SPI_CACHE_SRAM_USR_WR_CMD_BITLEN_S,
        );
        set_peri_reg_bits(
            SPI0_SRAM_DWR_CMD_REG,
            SPI_CACHE_SRAM_USR_WR_CMD_VALUE,
            PSRAM_QUAD_WRITE,
            SPI_CACHE_SRAM_USR_WR_CMD_VALUE_S,
        ); // 0x38
        set_peri_reg_bits(
            SPI0_CACHE_SCTRL_REG,
            SPI_SRAM_DUMMY_CYCLELEN_V,
            PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy,
            SPI_SRAM_DUMMY_CYCLELEN_S,
        ); // dummy, psram cache : 40m--+1dummy; 80m--+2dummy

        match psram_cache_mode {
            PsramCacheSpeed::PsramCacheF80mS80m => (), // in this mode , no delay is needed
            _ => {
                if clk_mode == PsramClkMode::PsramClkModeDclk {
                    set_peri_reg_bits(
                        SPI0_SRAM_DRD_CMD_REG,
                        SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_V,
                        15,
                        SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_S,
                    ); // read command length, 2 bytes(1byte for delay),sending in qio mode in cache
                    set_peri_reg_bits(
                        SPI0_SRAM_DRD_CMD_REG,
                        SPI_CACHE_SRAM_USR_RD_CMD_VALUE_V,
                        (PSRAM_FAST_READ_QUAD) << 8,
                        SPI_CACHE_SRAM_USR_RD_CMD_VALUE_S,
                    ); // 0xEB, read command value,(0x00 for delay,0xeb for cmd)
                    set_peri_reg_bits(
                        SPI0_SRAM_DWR_CMD_REG,
                        SPI_CACHE_SRAM_USR_WR_CMD_BITLEN,
                        15,
                        SPI_CACHE_SRAM_USR_WR_CMD_BITLEN_S,
                    ); // write command length,2 bytes(1byte for delay,send in qio mode in cache)
                    set_peri_reg_bits(
                        SPI0_SRAM_DWR_CMD_REG,
                        SPI_CACHE_SRAM_USR_WR_CMD_VALUE,
                        (PSRAM_QUAD_WRITE) << 8,
                        SPI_CACHE_SRAM_USR_WR_CMD_VALUE_S,
                    ); // 0x38, write command value,(0x00 for delay)
                    set_peri_reg_bits(
                        SPI0_CACHE_SCTRL_REG,
                        SPI_SRAM_DUMMY_CYCLELEN_V,
                        PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy,
                        SPI_SRAM_DUMMY_CYCLELEN_S,
                    ); // dummy, psram cache : 40m--+1dummy; 80m--+2dummy
                }
            }
        }

        unsafe {
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
        }

        // ENABLE SPI0 CS1 TO PSRAM(CS0--FLASH; CS1--SRAM)
        clear_peri_reg_mask(SPI0_PIN_REG, SPI_CS1_DIS_M);
    }

    // spi param init for psram
    fn psram_spi_init(
        // psram_spi_num_t spi_num = PSRAM_SPI_1,
        mode: PsramCacheSpeed,
        clk_mode: PsramClkMode,
    ) {
        clear_peri_reg_mask(SPI1_SLAVE_REG, SPI_TRANS_DONE << 5);
        // SPI_CPOL & SPI_CPHA
        clear_peri_reg_mask(SPI1_PIN_REG, SPI_CK_IDLE_EDGE);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_CK_OUT_EDGE);
        // SPI bit order
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_WR_BIT_ORDER);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_RD_BIT_ORDER);
        // SPI bit order
        clear_peri_reg_mask(SPI1_USER_REG, SPI_DOUTDIN);
        // May be not must to do.
        write_peri_reg(SPI1_USER1_REG, 0);
        // SPI mode type
        clear_peri_reg_mask(SPI1_SLAVE_REG, SPI_SLAVE_MODE);
        unsafe {
            let ptr = SPI1_W0_REG as *mut u32;
            for i in 0..16 {
                ptr.offset(i).write_volatile(0);
            }
        }
        psram_set_cs_timing_spi1(mode, clk_mode);
    }

    fn psram_set_cs_timing_spi1(psram_cache_mode: PsramCacheSpeed, clk_mode: PsramClkMode) {
        if clk_mode == PsramClkMode::PsramClkModeNorm {
            set_peri_reg_mask(SPI1_USER_REG, SPI_CS_HOLD_M | SPI_CS_SETUP_M);
            // Set cs time.
            set_peri_reg_bits(
                SPI1_CTRL2_REG,
                SPI_HOLD_TIME_V,
                psram_cs_hold_time_from_psram_speed(psram_cache_mode),
                SPI_HOLD_TIME_S,
            );
            set_peri_reg_bits(SPI1_CTRL2_REG, SPI_SETUP_TIME_V, 0, SPI_SETUP_TIME_S);
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_CS_HOLD_M | SPI_CS_SETUP_M);
        }
    }

    fn psram_set_cs_timing_spi0(psram_cache_mode: PsramCacheSpeed, clk_mode: PsramClkMode) {
        if clk_mode == PsramClkMode::PsramClkModeNorm {
            set_peri_reg_mask(SPI0_USER_REG, SPI_CS_HOLD_M | SPI_CS_SETUP_M);
            // Set cs time.
            set_peri_reg_bits(
                SPI0_CTRL2_REG,
                SPI_HOLD_TIME_V,
                psram_cs_hold_time_from_psram_speed(psram_cache_mode),
                SPI_HOLD_TIME_S,
            );
            set_peri_reg_bits(SPI0_CTRL2_REG, SPI_SETUP_TIME_V, 0, SPI_SETUP_TIME_S);
        } else {
            clear_peri_reg_mask(SPI0_USER_REG, SPI_CS_HOLD_M | SPI_CS_SETUP_M);
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
        loop {
            if read_peri_reg(SPI1_CMD_REG) & SPI_USR == 0 {
                break;
            }
        }

        write_peri_reg(SPI1_USER_REG, backup_usr);
        write_peri_reg(SPI1_USER1_REG, backup_usr1);
        write_peri_reg(SPI1_USER2_REG, backup_usr2);
    }

    // setup spi command/addr/data/dummy in user mode
    #[ram]
    fn psram_cmd_config_spi1(p_in_data: &PsramCmd) -> (u32, u32, u32) {
        loop {
            if read_peri_reg(SPI1_CMD_REG) & SPI_USR == 0 {
                break;
            }
        }

        let backup_usr = read_peri_reg(SPI1_USER_REG);
        let backup_usr1 = read_peri_reg(SPI1_USER1_REG);
        let backup_usr2 = read_peri_reg(SPI1_USER2_REG);

        // Set command by user.
        if p_in_data.cmd_bit_len != 0 {
            // Max command length 16 bits.
            set_peri_reg_bits(
                SPI1_USER2_REG,
                SPI_USR_COMMAND_BITLEN,
                (p_in_data.cmd_bit_len - 1) as u32,
                SPI_USR_COMMAND_BITLEN_S,
            );
            // Enable command
            set_peri_reg_mask(SPI1_USER_REG, SPI_USR_COMMAND);
            // Load command,bit15-0 is cmd value.
            set_peri_reg_bits(
                SPI1_USER2_REG,
                SPI_USR_COMMAND_VALUE,
                p_in_data.cmd as u32,
                SPI_USR_COMMAND_VALUE_S,
            );
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_COMMAND);
            set_peri_reg_bits(
                SPI1_USER2_REG,
                SPI_USR_COMMAND_BITLEN,
                0,
                SPI_USR_COMMAND_BITLEN_S,
            );
        }
        // Set Address by user.
        if p_in_data.addr_bit_len != 0 {
            set_peri_reg_bits(
                SPI1_USER1_REG,
                SPI_USR_ADDR_BITLEN,
                (p_in_data.addr_bit_len - 1) as u32,
                SPI_USR_ADDR_BITLEN_S,
            );
            // Enable address
            set_peri_reg_mask(SPI1_USER_REG, SPI_USR_ADDR);
            // Set address
            write_peri_reg(SPI1_ADDR_REG, unsafe { p_in_data.addr.read_volatile() });
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_ADDR);
            set_peri_reg_bits(
                SPI1_USER1_REG,
                SPI_USR_ADDR_BITLEN,
                0,
                SPI_USR_ADDR_BITLEN_S,
            );
        }
        // Set data by user.
        let p_tx_val = p_in_data.tx_data;
        if p_in_data.tx_data_bit_len != 0 {
            // Enable MOSI
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_MOSI);
            // Load send buffer
            let len = (p_in_data.tx_data_bit_len + 31) / 32;
            if !p_tx_val.is_null() {
                for i in 0..len {
                    write_peri_reg(SPI1_W0_REG, unsafe {
                        p_tx_val.offset(i as isize).read_volatile()
                    });
                }
            }
            // Set data send buffer length.Max data length 64 bytes.
            set_peri_reg_bits(
                SPI1_MOSI_DLEN_REG,
                SPI_USR_MOSI_DBITLEN,
                (p_in_data.tx_data_bit_len - 1) as u32,
                SPI_USR_MOSI_DBITLEN_S,
            );
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_MOSI);
            set_peri_reg_bits(
                SPI1_MOSI_DLEN_REG,
                SPI_USR_MOSI_DBITLEN,
                0,
                SPI_USR_MOSI_DBITLEN_S,
            );
        }
        // Set rx data by user.
        if p_in_data.rx_data_bit_len != 0 {
            // Enable MOSI
            set_peri_reg_mask(SPI1_USER_REG, SPI_USR_MISO);
            // Set data send buffer length.Max data length 64 bytes.
            set_peri_reg_bits(
                SPI1_MISO_DLEN_REG,
                SPI_USR_MISO_DBITLEN,
                (p_in_data.rx_data_bit_len - 1) as u32,
                SPI_USR_MISO_DBITLEN_S,
            );
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_MISO);
            set_peri_reg_bits(
                SPI1_MISO_DLEN_REG,
                SPI_USR_MISO_DBITLEN,
                0,
                SPI_USR_MISO_DBITLEN_S,
            );
        }
        if p_in_data.dummy_bit_len != 0 {
            set_peri_reg_mask(SPI1_USER_REG, SPI_USR_DUMMY); // dummy en
            set_peri_reg_bits(
                SPI1_USER1_REG,
                SPI_USR_DUMMY_CYCLELEN_V,
                p_in_data.dummy_bit_len - 1,
                SPI_USR_DUMMY_CYCLELEN_S,
            ); // DUMMY
        } else {
            clear_peri_reg_mask(SPI1_USER_REG, SPI_USR_DUMMY); // dummy en
            set_peri_reg_bits(
                SPI1_USER1_REG,
                SPI_USR_DUMMY_CYCLELEN_V,
                0,
                SPI_USR_DUMMY_CYCLELEN_S,
            ); // DUMMY
        }

        (backup_usr, backup_usr1, backup_usr2)
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    enum PsramCmdMode {
        PsramCmdQpi,
        PsramCmdSpi,
    }

    // start sending cmd/addr and optionally, receiving data
    #[ram]
    fn psram_cmd_recv_start_spi1(p_rx_data: *mut u32, rx_byte_len: u16, cmd_mode: PsramCmdMode) {
        // get cs1
        clear_peri_reg_mask(SPI1_PIN_REG, SPI_CS1_DIS_M);
        set_peri_reg_mask(SPI1_PIN_REG, SPI_CS0_DIS_M);

        let mode_backup: u32 = (read_peri_reg(SPI1_USER_REG) >> SPI_FWRITE_DUAL_S) & 0xf;
        let rd_mode_backup: u32 = read_peri_reg(SPI1_CTRL_REG)
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
            if read_peri_reg(SPI0_EXT2_REG) == 0 {
                break;
            }
        }
        unsafe {
            // DPORT_SET_PERI_REG_MASK(DPORT_HOST_INF_SEL_REG, 1 << 14);
            let dport = &*esp32::DPORT::PTR;
            dport
                .host_inf_sel()
                .modify(|r, w| w.bits(r.bits() | 1 << 14));
        }

        // Start send data
        set_peri_reg_mask(SPI1_CMD_REG, SPI_USR);
        loop {
            if read_peri_reg(SPI1_CMD_REG) & SPI_USR == 0 {
                break;
            }
        }
        unsafe {
            // DPORT_CLEAR_PERI_REG_MASK(DPORT_HOST_INF_SEL_REG, 1 << 14);
            let dport = &*esp32::DPORT::PTR;
            dport
                .host_inf_sel()
                .modify(|r, w| w.bits(r.bits() & !(1 << 14)));
        }

        // recover spi mode
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
        clear_peri_reg_mask(
            SPI1_CTRL_REG,
            SPI_FREAD_DIO_M | SPI_FREAD_DUAL_M | SPI_FREAD_QUAD_M | SPI_FREAD_QIO_M,
        );
        set_peri_reg_mask(SPI1_CTRL_REG, rd_mode_backup);

        // return cs to cs0
        set_peri_reg_mask(SPI1_PIN_REG, SPI_CS1_DIS_M);
        clear_peri_reg_mask(SPI1_PIN_REG, SPI_CS0_DIS_M);

        if !p_rx_data.is_null() {
            let mut idx = 0;
            // Read data out
            loop {
                unsafe {
                    p_rx_data
                        .offset(idx)
                        .write_volatile(read_peri_reg(SPI1_W0_REG + ((idx as u32) << 2)));
                }

                idx += 1;
                if idx > ((rx_byte_len / 4) + if (rx_byte_len % 4) != 0 { 1 } else { 0 }) as isize {
                    break;
                }
            }
        }
    }

    // set basic SPI write mode
    fn psram_set_basic_write_mode_spi1() {
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_QIO);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_DIO);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_QUAD);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_DUAL);
    }
    // set QPI write mode
    fn psram_set_qio_write_mode_spi1() {
        set_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_QIO);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_DIO);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_QUAD);
        clear_peri_reg_mask(SPI1_USER_REG, SPI_FWRITE_DUAL);
    }
    // set QPI read mode
    fn psram_set_qio_read_mode_spi1() {
        set_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_QIO);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_QUAD);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_DUAL);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_DIO);
    }
    // set SPI read mode
    fn psram_set_basic_read_mode_spi1() {
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_QIO);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_QUAD);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_DUAL);
        clear_peri_reg_mask(SPI1_CTRL_REG, SPI_FREAD_DIO);
    }

    // psram gpio init , different working frequency we have different solutions
    fn psram_gpio_config(psram_io: &PsramIo, mode: PsramCacheSpeed) -> u32 {
        let g_rom_spiflash_dummy_len_plus_ptr =
            unsafe { addr_of_mut!(g_rom_spiflash_dummy_len_plus) };

        fn gpio_pin_mux_reg(gpio: u8) -> u32 {
            crate::gpio::get_io_mux_reg(gpio).as_ptr() as u32
        }

        fn gpio_hal_iomux_func_sel(reg: u32, function: u32) {
            unsafe {
                let ptr = reg as *mut u32;
                let old = ptr.read_volatile();
                ptr.write_volatile((old & !(0b111 << 12)) | (function << 12));
            }
        }

        let spi_cache_dummy;
        let rd_mode_reg = read_peri_reg(SPI0_CTRL_REG);
        if (rd_mode_reg & SPI_FREAD_QIO_M) != 0 {
            spi_cache_dummy = SPI0_R_QIO_DUMMY_CYCLELEN;
        } else if (rd_mode_reg & SPI_FREAD_DIO_M) != 0 {
            spi_cache_dummy = SPI0_R_DIO_DUMMY_CYCLELEN;
            set_peri_reg_bits(
                SPI0_USER1_REG,
                SPI_USR_ADDR_BITLEN_V,
                SPI0_R_DIO_ADDR_BITSLEN,
                SPI_USR_ADDR_BITLEN_S,
            );
        } else {
            spi_cache_dummy = SPI0_R_FAST_DUMMY_CYCLELEN;
        }

        let extra_dummy;

        match mode {
            PsramCacheSpeed::PsramCacheF80mS40m => {
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;
                unsafe {
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);
                }
                set_peri_reg_bits(
                    SPI0_USER1_REG,
                    SPI_USR_DUMMY_CYCLELEN_V,
                    spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_80M as u32,
                    SPI_USR_DUMMY_CYCLELEN_S,
                ); // DUMMY
                unsafe {
                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);
                }
                // set drive ability for clock
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.flash_clk_io),
                    FUN_DRV,
                    3,
                    FUN_DRV_S,
                );
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.psram_clk_io),
                    FUN_DRV,
                    2,
                    FUN_DRV_S,
                );
            }
            PsramCacheSpeed::PsramCacheF80mS80m => {
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_80M;
                unsafe {
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_80M);
                }
                set_peri_reg_bits(
                    SPI0_USER1_REG,
                    SPI_USR_DUMMY_CYCLELEN_V,
                    spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_80M as u32,
                    SPI_USR_DUMMY_CYCLELEN_S,
                ); // DUMMY
                unsafe {
                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_FLASH_PORT);
                }
                // set drive ability for clock
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.flash_clk_io),
                    FUN_DRV,
                    3,
                    FUN_DRV_S,
                );
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.psram_clk_io),
                    FUN_DRV,
                    3,
                    FUN_DRV_S,
                );
            }
            PsramCacheSpeed::PsramCacheF40mS40m => {
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;
                unsafe {
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_CACHE_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);
                    g_rom_spiflash_dummy_len_plus_ptr
                        .offset(_SPI_FLASH_PORT as isize)
                        .write_volatile(PSRAM_IO_MATRIX_DUMMY_40M);
                }
                set_peri_reg_bits(
                    SPI0_USER1_REG,
                    SPI_USR_DUMMY_CYCLELEN_V,
                    spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_40M as u32,
                    SPI_USR_DUMMY_CYCLELEN_S,
                ); // DUMMY
                unsafe {
                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_CACHE_PORT);
                    esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);
                }
                // set drive ability for clock
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.flash_clk_io),
                    FUN_DRV,
                    2,
                    FUN_DRV_S,
                );
                set_peri_reg_bits(
                    gpio_pin_mux_reg(psram_io.psram_clk_io),
                    FUN_DRV,
                    2,
                    FUN_DRV_S,
                );
            }
        }
        set_peri_reg_mask(SPI0_USER_REG, SPI_USR_DUMMY); // dummy enable

        // In bootloader, all the signals are already configured,
        // We keep the following code in case the bootloader is some older version.
        unsafe {
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
        }

        // select pin function gpio
        if (psram_io.flash_clk_io == SPI_IOMUX_PIN_NUM_CLK)
            && (psram_io.flash_clk_io != psram_io.psram_clk_io)
        {
            // flash clock signal should come from IO MUX.
            gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.flash_clk_io), FUNC_SD_CLK_SPICLK);
        } else {
            // flash clock signal should come from GPIO matrix.
            gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.flash_clk_io), PIN_FUNC_GPIO);
        }
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.flash_cs_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_cs_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_clk_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_spiq_sd0_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_spid_sd1_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_spihd_sd2_io), PIN_FUNC_GPIO);
        gpio_hal_iomux_func_sel(gpio_pin_mux_reg(psram_io.psram_spiwp_sd3_io), PIN_FUNC_GPIO);

        let flash_id: u32 = unsafe { g_rom_flashchip.device_id };
        info!("Flash-ID = {}", flash_id);

        if flash_id == FLASH_ID_GD25LQ32C {
            // Set drive ability for 1.8v flash in 80Mhz.
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.flash_cs_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.flash_clk_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_cs_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_clk_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_spiq_sd0_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_spid_sd1_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_spihd_sd2_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
            set_peri_reg_bits(
                gpio_pin_mux_reg(psram_io.psram_spiwp_sd3_io),
                FUN_DRV_V,
                3,
                FUN_DRV_S,
            );
        }

        extra_dummy as u32
    }

    fn clear_peri_reg_mask(reg: u32, mask: u32) {
        unsafe {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
        }
    }

    fn set_peri_reg_mask(reg: u32, mask: u32) {
        unsafe {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
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

    fn write_peri_reg(reg: u32, val: u32) {
        unsafe {
            (reg as *mut u32).write_volatile(val);
        }
    }

    fn read_peri_reg(reg: u32) -> u32 {
        unsafe { (reg as *mut u32).read_volatile() }
    }
}
