static mut PSRAM_VADDR: u32 = 0x3C000000;

pub fn psram_vaddr_start() -> usize {
    unsafe { PSRAM_VADDR as usize }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "psram_2m")] {
        const PSRAM_SIZE: u32 = 2;
    } else if #[cfg(feature = "psram_4m")] {
        const PSRAM_SIZE: u32 = 4;
    } else if #[cfg(feature = "psram_8m")] {
        const PSRAM_SIZE: u32 = 8;
    } else {
        const PSRAM_SIZE: u32 = 0;
    }
}

pub const PSRAM_BYTES: usize = PSRAM_SIZE as usize * 1024 * 1024;

/// Initialize PSRAM to be used for data.
///
/// Currently only QSPI is supported.
#[cfg(any(feature = "psram_2m", feature = "psram_4m", feature = "psram_8m"))]
pub fn init_psram(_peripheral: impl crate::peripheral::Peripheral<P = crate::peripherals::PSRAM>) {
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE: u32 = 0x4000;
    const CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS: u8 = 8;
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE: u8 = 32;
    const CONFIG_ESP32S3_DATA_CACHE_SIZE: u32 = 0x8000;
    const CONFIG_ESP32S3_DCACHE_ASSOCIATED_WAYS: u8 = 8;
    const CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE: u8 = 32;
    const MMU_ACCESS_SPIRAM: u32 = 1 << 15;
    const START_PAGE: u32 = 0;

    extern "C" {
        fn rom_config_instruction_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );

        fn Cache_Suspend_DCache();

        fn rom_config_data_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );

        fn Cache_Resume_DCache(param: u32);

        /// Set DCache mmu mapping.
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
    unsafe {
        const MMU_PAGE_SIZE: u32 = 0x10000;
        const ICACHE_MMU_SIZE: usize = 0x800;
        const FLASH_MMU_TABLE_SIZE: usize = ICACHE_MMU_SIZE / core::mem::size_of::<u32>();
        const MMU_INVALID: u32 = 1 << 14;
        const DR_REG_MMU_TABLE: u32 = 0x600C5000;

        // calculate the PSRAM start address to map
        let mut start = PSRAM_VADDR;
        let mmu_table_ptr = DR_REG_MMU_TABLE as *const u32;
        for i in 0..FLASH_MMU_TABLE_SIZE {
            if mmu_table_ptr.add(i).read_volatile() != MMU_INVALID {
                start += MMU_PAGE_SIZE;
            } else {
                break;
            }
        }
        log::debug!("PSRAM start address = {:x}", start);
        PSRAM_VADDR = start;

        // Configure the mode of instruction cache : cache size, cache line size.
        rom_config_instruction_cache_mode(
            CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE,
            CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS,
            CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE,
        );

        // If we need use SPIRAM, we should use data cache.Connfigure the mode of data :
        // cache size, cache line size.
        Cache_Suspend_DCache();

        rom_config_data_cache_mode(
            CONFIG_ESP32S3_DATA_CACHE_SIZE,
            CONFIG_ESP32S3_DCACHE_ASSOCIATED_WAYS,
            CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE,
        );

        if cache_dbus_mmu_set(
            MMU_ACCESS_SPIRAM,
            PSRAM_VADDR,
            START_PAGE << 16,
            64,
            PSRAM_SIZE * 1024 / 64, // number of pages to map
            0,
        ) != 0
        {
            panic!("cache_dbus_mmu_set failed");
        }

        let extmem = &*esp32s3::EXTMEM::PTR;
        extmem.dcache_ctrl1.modify(|_, w| {
            w.dcache_shut_core0_bus()
                .clear_bit()
                .dcache_shut_core1_bus()
                .clear_bit()
        });

        Cache_Resume_DCache(0);
    }

    utils::psram_init();
}

#[cfg(any(feature = "psram_2m", feature = "psram_4m", feature = "psram_8m"))]
pub(crate) mod utils {
    use procmacros::ram;

    // these should probably be configurable, relates to https://github.com/esp-rs/esp-hal/issues/42
    // probably also the PSRAM size shouldn't get configured via features
    const SPI_TIMING_CORE_CLOCK: SpiTimingConfigCoreClock =
        SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m;
    const FLASH_FREQ: FlashFreq = FlashFreq::FlashFreq80m;
    const SPIRAM_SPEED: SpiRamFreq = SpiRamFreq::Freq40m;

    #[allow(unused)]
    enum FlashFreq {
        FlashFreq20m,
        FlashFreq40m,
        FlashFreq80m,
        FlashFreq120m,
    }

    #[allow(unused)]
    enum SpiRamFreq {
        Freq40m,
        Freq80m,
        Freq120m,
    }

    #[allow(unused)]
    #[derive(Debug)]
    enum SpiTimingConfigCoreClock {
        SpiTimingConfigCoreClock80m,
        SpiTimingConfigCoreClock120m,
        SpiTimingConfigCoreClock160m,
        SpiTimingConfigCoreClock240m,
    }

    impl SpiTimingConfigCoreClock {
        fn mhz(&self) -> u32 {
            match self {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m => 80,
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock120m => 120,
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m => 160,
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m => 240,
            }
        }
    }

    #[ram]
    pub(crate) fn psram_init() {
        psram_gpio_config();
        psram_set_cs_timing();

        // SPI1: send psram reset command
        psram_reset_mode_spi1();
        // SPI1: send QPI enable command
        psram_enable_qio_mode_spi1();

        // Do PSRAM timing tuning, we use SPI1 to do the tuning, and set the SPI0 PSRAM
        // timing related registers accordingly
        mspi_timing_psram_tuning();

        // Configure SPI0 PSRAM related SPI Phases
        config_psram_spi_phases();
        // Back to the high speed mode. Flash/PSRAM clocks are set to the clock that
        // user selected. SPI0/1 registers are all set correctly
        mspi_timing_enter_high_speed_mode(true);
    }

    const PSRAM_CS_IO: u8 = 26;
    const SPI_CS1_GPIO_NUM: u8 = 26;
    const FUNC_SPICS1_SPICS1: u8 = 0;
    const PIN_FUNC_GPIO: u8 = 2;
    const PSRAM_SPIWP_SD3_IO: u8 = 10;
    const ESP_ROM_EFUSE_FLASH_DEFAULT_SPI: u32 = 0;
    const SPICS1_OUT_IDX: u8 = 6;

    const DR_REG_SPI0_BASE: u32 = 0x60003000;
    const SPI0_MEM_SPI_SMEM_AC_REG: u32 = DR_REG_SPI0_BASE + 0xDC;
    const SPI_MEM_SPI_SMEM_CS_HOLD_TIME_V: u32 = 0x1F;
    const SPI_MEM_SPI_SMEM_CS_HOLD_TIME_S: u32 = 7;
    const SPI_MEM_SPI_SMEM_CS_SETUP_TIME_V: u32 = 0x1F;
    const SPI_MEM_SPI_SMEM_CS_SETUP_TIME_S: u32 = 2;
    const SPI_MEM_SPI_SMEM_CS_HOLD_M: u32 = 1 << 1;
    const SPI_MEM_SPI_SMEM_CS_SETUP_M: u32 = 1 << 0;
    const SPI0_MEM_CACHE_SCTRL_REG: u32 = DR_REG_SPI0_BASE + 0x40;
    const SPI0_MEM_SRAM_DWR_CMD_REG: u32 = DR_REG_SPI0_BASE + 0x4C;
    const SPI0_MEM_SRAM_DRD_CMD_REG: u32 = DR_REG_SPI0_BASE + 0x48;
    const SPI_MEM_USR_SRAM_DIO_M: u32 = 1 << 1;
    const SPI_MEM_USR_SRAM_QIO_M: u32 = 1 << 2;
    const SPI_MEM_CACHE_SRAM_USR_RCMD_M: u32 = 1 << 5;
    const SPI_MEM_CACHE_SRAM_USR_WCMD_M: u32 = 1 << 20;
    const SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN: u32 = 0x0000000F;
    const SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN_S: u32 = 28;
    const SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE: u32 = 0x0000FFFF;
    const SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE_S: u32 = 0;
    const PSRAM_QUAD_WRITE: u32 = 0x38;
    const SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_V: u32 = 0xF;
    const SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_S: u32 = 28;
    const SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_V: u32 = 0xFFFF;
    const PSRAM_FAST_READ_QUAD: u32 = 0xEB;
    const SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_S: u32 = 0;
    const SPI_MEM_SRAM_ADDR_BITLEN_V: u32 = 0x3F;
    const SPI_MEM_SRAM_ADDR_BITLEN_S: u32 = 14;
    const SPI_MEM_USR_RD_SRAM_DUMMY_M: u32 = 1 << 4;
    const SPI_MEM_SRAM_RDUMMY_CYCLELEN_V: u32 = 0x3F;
    const PSRAM_FAST_READ_QUAD_DUMMY: u32 = 6;
    const SPI_MEM_SRAM_RDUMMY_CYCLELEN_S: u32 = 6;
    const SPI0_MEM_MISC_REG: u32 = DR_REG_SPI0_BASE + 0x34;
    const SPI_MEM_CS1_DIS_M: u32 = 1 << 1;
    const SPI0_MEM_CORE_CLK_SEL_REG: u32 = DR_REG_SPI0_BASE + 0xEC;
    const SPI_MEM_CORE_CLK_SEL: u32 = 0x00000003;
    const DR_REG_SPI1_BASE: u32 = 0x60002000;
    const SPI0_MEM_CLOCK_REG: u32 = DR_REG_SPI0_BASE + 0x14;
    const SPI1_MEM_CLOCK_REG: u32 = DR_REG_SPI1_BASE + 0x14;
    const SPI0_MEM_SRAM_CLK_REG: u32 = DR_REG_SPI0_BASE + 0x50;
    const SPI_MEM_CLK_EQU_SYSCLK: u32 = 1 << 31;
    const SPI_MEM_SCLK_EQU_SYSCLK: u32 = 1 << 31;
    const SPI_MEM_CLKCNT_N_S: u32 = 16;
    const SPI_MEM_SCLKCNT_N_S: u32 = 16;
    const SPI_MEM_CLKCNT_H_S: u32 = 8;
    const SPI_MEM_SCLKCNT_H_S: u32 = 8;
    const SPI_MEM_CLKCNT_L_S: u32 = 0;
    const SPI_MEM_SCLKCNT_L_S: u32 = 0;

    extern "C" {
        fn esp_rom_efuse_get_flash_gpio_info() -> u32;

        fn esp_rom_efuse_get_flash_wp_gpio() -> u8;

        fn esp_rom_gpio_connect_out_signal(
            gpio_num: u8,
            signal_idx: u8,
            out_inv: bool,
            oen_inv: bool,
        );

        /// Enable Quad I/O pin functions
        ///
        /// Sets the HD & WP pin functions for Quad I/O modes, based on the
        /// efuse SPI pin configuration.
        ///
        /// [`wp_gpio_num`]: u8 Number of the WP pin to reconfigure for quad I/O
        /// [`spiconfig`]: u32 Pin configuration, as returned from ets_efuse_get_spiconfig().
        /// - If this parameter is 0, default SPI pins are used and wp_gpio_num
        ///   parameter is ignored.
        /// - If this parameter is 1, default HSPI pins are used and wp_gpio_num
        ///   parameter is ignored.
        /// - For other values, this parameter encodes the HD pin number and
        ///   also the CLK pin number. CLK pin selection is used to determine if
        ///   HSPI or SPI peripheral will be used (use HSPI if CLK pin is the
        ///   HSPI clock pin, otherwise use SPI).
        //   Both HD & WP pins are configured via GPIO matrix to map to the selected peripheral.
        fn esp_rom_spiflash_select_qio_pins(wp_gpio_num: u8, spiconfig: u32);
    }

    // Configure PSRAM SPI0 phase related registers here according to the PSRAM chip
    // requirement
    #[ram]
    fn config_psram_spi_phases() {
        // Config CMD phase
        clear_peri_reg_mask(SPI0_MEM_CACHE_SCTRL_REG, SPI_MEM_USR_SRAM_DIO_M); // disable dio mode for cache command
        set_peri_reg_mask(SPI0_MEM_CACHE_SCTRL_REG, SPI_MEM_USR_SRAM_QIO_M); // enable qio mode for cache command
        set_peri_reg_mask(SPI0_MEM_CACHE_SCTRL_REG, SPI_MEM_CACHE_SRAM_USR_RCMD_M); // enable cache read command
        set_peri_reg_mask(SPI0_MEM_CACHE_SCTRL_REG, SPI_MEM_CACHE_SRAM_USR_WCMD_M); // enable cache write command
        set_peri_reg_bits(
            SPI0_MEM_SRAM_DWR_CMD_REG,
            SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN,
            7,
            SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN_S,
        );
        set_peri_reg_bits(
            SPI0_MEM_SRAM_DWR_CMD_REG,
            SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE,
            PSRAM_QUAD_WRITE,
            SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE_S,
        ); // 0x38
        set_peri_reg_bits(
            SPI0_MEM_SRAM_DRD_CMD_REG,
            SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_V,
            7,
            SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_S,
        );
        set_peri_reg_bits(
            SPI0_MEM_SRAM_DRD_CMD_REG,
            SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_V,
            PSRAM_FAST_READ_QUAD,
            SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_S,
        ); // 0xEB

        // Config ADDR phase
        set_peri_reg_bits(
            SPI0_MEM_CACHE_SCTRL_REG,
            SPI_MEM_SRAM_ADDR_BITLEN_V,
            23,
            SPI_MEM_SRAM_ADDR_BITLEN_S,
        );

        // Dummy
        // We set the PSRAM chip required dummy here. If timing tuning is
        // needed, the dummy length will be updated in
        // `mspi_timing_enter_high_speed_mode()`
        set_peri_reg_mask(SPI0_MEM_CACHE_SCTRL_REG, SPI_MEM_USR_RD_SRAM_DUMMY_M); // enable cache read dummy
        set_peri_reg_bits(
            SPI0_MEM_CACHE_SCTRL_REG,
            SPI_MEM_SRAM_RDUMMY_CYCLELEN_V,
            PSRAM_FAST_READ_QUAD_DUMMY - 1,
            SPI_MEM_SRAM_RDUMMY_CYCLELEN_S,
        ); // dummy

        clear_peri_reg_mask(SPI0_MEM_MISC_REG, SPI_MEM_CS1_DIS_M); // ENABLE SPI0 CS1 TO PSRAM(CS0--FLASH; CS1--SRAM)
    }

    #[ram]
    fn mspi_timing_psram_tuning() {
        // currently we only support !SPI_TIMING_PSRAM_NEEDS_TUNING
        // see https://github.com/espressif/esp-idf/blob/4e24516ee2731eb55687182d4e061b5b93a9e33f/components/esp_hw_support/mspi_timing_tuning.c#L391-L415
    }

    /// Set SPI0 FLASH and PSRAM module clock, din_num, din_mode and extra
    /// dummy, according to the configuration got from timing tuning
    /// function (`calculate_best_flash_tuning_config`). iF control_spi1 ==
    /// 1, will also update SPI1 timing registers. Should only be set to 1 when
    /// do tuning.
    ///
    /// This function should always be called after `mspi_timing_flash_tuning`
    /// or `calculate_best_flash_tuning_config`
    #[ram]
    fn mspi_timing_enter_high_speed_mode(control_spi1: bool) {
        let core_clock: SpiTimingConfigCoreClock = get_mspi_core_clock();
        let flash_div: u32 = get_flash_clock_divider();
        let psram_div: u32 = get_psram_clock_divider();

        log::info!(
            "PSRAM core_clock {:?}, flash_div = {}, psram_div = {}",
            core_clock,
            flash_div,
            psram_div
        );

        // Set SPI01 core clock
        // SPI0 and SPI1 share the register for core clock. So we only set SPI0 here.
        // Set FLASH module clock
        spi0_timing_config_set_core_clock(core_clock);

        spi0_timing_config_set_flash_clock(flash_div);
        if control_spi1 {
            spi1_timing_config_set_flash_clock(flash_div);
        }
        // Set PSRAM module clock
        spi0_timing_config_set_psram_clock(psram_div);

        // #if SPI_TIMING_FLASH_NEEDS_TUNING || SPI_TIMING_PSRAM_NEEDS_TUNING
        //     set_timing_tuning_regs_as_required(true);
        // #endif
    }

    #[ram]
    fn spi0_timing_config_set_core_clock(core_clock: SpiTimingConfigCoreClock) {
        let reg_val = match core_clock {
            SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m => 0,
            SpiTimingConfigCoreClock::SpiTimingConfigCoreClock120m => 1,
            SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m => 2,
            SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m => 3,
        };

        set_peri_reg_bits(SPI0_MEM_CORE_CLK_SEL_REG, SPI_MEM_CORE_CLK_SEL, reg_val, 0);
    }

    #[ram]
    fn spi0_timing_config_set_flash_clock(freqdiv: u32) {
        if freqdiv == 1 {
            write_peri_reg(SPI0_MEM_CLOCK_REG, SPI_MEM_CLK_EQU_SYSCLK);
        } else {
            let freqbits: u32 = ((freqdiv - 1) << SPI_MEM_CLKCNT_N_S)
                | ((freqdiv / 2 - 1) << SPI_MEM_CLKCNT_H_S)
                | ((freqdiv - 1) << SPI_MEM_CLKCNT_L_S);
            write_peri_reg(SPI0_MEM_CLOCK_REG, freqbits);
        }
    }

    #[ram]
    fn spi1_timing_config_set_flash_clock(freqdiv: u32) {
        if freqdiv == 1 {
            write_peri_reg(SPI1_MEM_CLOCK_REG, SPI_MEM_CLK_EQU_SYSCLK);
        } else {
            let freqbits: u32 = ((freqdiv - 1) << SPI_MEM_CLKCNT_N_S)
                | ((freqdiv / 2 - 1) << SPI_MEM_CLKCNT_H_S)
                | ((freqdiv - 1) << SPI_MEM_CLKCNT_L_S);
            write_peri_reg(SPI1_MEM_CLOCK_REG, freqbits);
        }
    }

    #[ram]
    fn spi0_timing_config_set_psram_clock(freqdiv: u32) {
        if freqdiv == 1 {
            write_peri_reg(SPI0_MEM_SRAM_CLK_REG, SPI_MEM_SCLK_EQU_SYSCLK);
        } else {
            let freqbits: u32 = ((freqdiv - 1) << SPI_MEM_SCLKCNT_N_S)
                | ((freqdiv / 2 - 1) << SPI_MEM_SCLKCNT_H_S)
                | ((freqdiv - 1) << SPI_MEM_SCLKCNT_L_S);
            write_peri_reg(SPI0_MEM_SRAM_CLK_REG, freqbits);
        }
    }

    #[ram]
    fn get_mspi_core_clock() -> SpiTimingConfigCoreClock {
        SPI_TIMING_CORE_CLOCK
    }

    #[ram]
    fn get_flash_clock_divider() -> u32 {
        match FLASH_FREQ {
            FlashFreq::FlashFreq20m => SPI_TIMING_CORE_CLOCK.mhz() / 20,
            FlashFreq::FlashFreq40m => SPI_TIMING_CORE_CLOCK.mhz() / 40,
            FlashFreq::FlashFreq80m => SPI_TIMING_CORE_CLOCK.mhz() / 80,
            FlashFreq::FlashFreq120m => SPI_TIMING_CORE_CLOCK.mhz() / 120,
        }
    }

    #[ram]
    fn get_psram_clock_divider() -> u32 {
        match SPIRAM_SPEED {
            SpiRamFreq::Freq40m => SPI_TIMING_CORE_CLOCK.mhz() / 40,
            SpiRamFreq::Freq80m => SPI_TIMING_CORE_CLOCK.mhz() / 80,
            SpiRamFreq::Freq120m => SPI_TIMING_CORE_CLOCK.mhz() / 120,
        }
    }

    // send reset command to psram, in spi mode
    #[ram]
    fn psram_reset_mode_spi1() {
        const PSRAM_RESET_EN: u16 = 0x66;
        const PSRAM_RESET: u16 = 0x99;
        const CS_PSRAM_SEL: u8 = 1 << 1;

        psram_exec_cmd(
            CommandMode::PsramCmdSpi,
            PSRAM_RESET_EN,
            8, // command and command bit len
            0,
            0, // address and address bit len
            0, // dummy bit len
            core::ptr::null(),
            0, // tx data and tx bit len
            core::ptr::null_mut(),
            0,            // rx data and rx bit len
            CS_PSRAM_SEL, // cs bit mask
            false,
        ); // whether is program/erase operation

        psram_exec_cmd(
            CommandMode::PsramCmdSpi,
            PSRAM_RESET,
            8, // command and command bit len
            0,
            0, // address and address bit len
            0, // dummy bit len
            core::ptr::null(),
            0, // tx data and tx bit len
            core::ptr::null_mut(),
            0,            // rx data and rx bit len
            CS_PSRAM_SEL, // cs bit mask
            false,
        ); // whether is program/erase operation
    }

    #[derive(PartialEq)]
    #[allow(unused)]
    enum CommandMode {
        PsramCmdQpi = 0,
        PsramCmdSpi = 1,
    }

    #[ram]
    fn psram_exec_cmd(
        mode: CommandMode,
        cmd: u16,
        cmd_bit_len: u16,
        addr: u32,
        addr_bit_len: u32,
        dummy_bits: u32,
        mosi_data: *const u8,
        mosi_bit_len: u32,
        miso_data: *mut u8,
        miso_bit_len: u32,
        cs_mask: u8,
        is_write_erase_operation: bool,
    ) {
        extern "C" {
            ///  Start a spi user command sequence
            ///  [`spi_num`] spi port
            ///  [`rx_buf`] buffer pointer to receive data
            ///  [`rx_len`] receive data length in byte
            ///  [`cs_en_mask`] decide which cs to use, 0 for cs0, 1 for cs1
            ///  [`is_write_erase`] to indicate whether this is a write or erase
            /// operation, since the CPU would check permission
            fn esp_rom_spi_cmd_start(
                spi_num: u32,
                rx_buf: *const u8,
                rx_len: u16,
                cs_en_mask: u8,
                is_write_erase: bool,
            );
        }

        unsafe {
            let spi1 = &*esp32s3::SPI1::PTR;
            let backup_usr = spi1.user.read().bits();
            let backup_usr1 = spi1.user1.read().bits();
            let backup_usr2 = spi1.user2.read().bits();
            let backup_ctrl = spi1.ctrl.read().bits();
            psram_set_op_mode(mode);
            _psram_exec_cmd(
                cmd,
                cmd_bit_len,
                addr,
                addr_bit_len,
                dummy_bits,
                mosi_data,
                mosi_bit_len,
                miso_data,
                miso_bit_len,
            );
            esp_rom_spi_cmd_start(
                1,
                miso_data,
                (miso_bit_len / 8) as u16,
                cs_mask,
                is_write_erase_operation,
            );

            spi1.user.write(|w| w.bits(backup_usr));
            spi1.user1.write(|w| w.bits(backup_usr1));
            spi1.user2.write(|w| w.bits(backup_usr2));
            spi1.ctrl.write(|w| w.bits(backup_ctrl));
        }
    }

    #[ram]
    fn _psram_exec_cmd(
        cmd: u16,
        cmd_bit_len: u16,
        addr: u32,
        addr_bit_len: u32,
        dummy_bits: u32,
        mosi_data: *const u8,
        mosi_bit_len: u32,
        miso_data: *mut u8,
        miso_bit_len: u32,
    ) {
        #[repr(C)]
        struct esp_rom_spi_cmd_t {
            cmd: u16,             // Command value
            cmd_bit_len: u16,     // Command byte length
            addr: *const u32,     // Point to address value
            addr_bit_len: u32,    // Address byte length
            tx_data: *const u32,  // Point to send data buffer
            tx_data_bit_len: u32, // Send data byte length.
            rx_data: *mut u32,    // Point to recevie data buffer
            rx_data_bit_len: u32, // Recevie Data byte length.
            dummy_bit_len: u32,
        }

        extern "C" {
            /// Config the spi user command
            /// [`spi_num`] spi port
            /// [`pcmd`] pointer to accept the spi command struct
            fn esp_rom_spi_cmd_config(spi_num: u32, pcmd: *const esp_rom_spi_cmd_t);
        }

        let conf = esp_rom_spi_cmd_t {
            cmd,
            cmd_bit_len,
            addr: addr as *const u32,
            addr_bit_len,
            tx_data: mosi_data as *const u32,
            tx_data_bit_len: mosi_bit_len,
            rx_data: miso_data as *mut u32,
            rx_data_bit_len: miso_bit_len,
            dummy_bit_len: dummy_bits,
        };

        unsafe {
            esp_rom_spi_cmd_config(1, &conf);
        }
    }

    #[ram]
    fn psram_set_op_mode(mode: CommandMode) {
        extern "C" {
            fn esp_rom_spi_set_op_mode(spi: u32, mode: u32);
        }

        const ESP_ROM_SPIFLASH_QIO_MODE: u32 = 0;
        const ESP_ROM_SPIFLASH_SLOWRD_MODE: u32 = 5;

        unsafe {
            match mode {
                CommandMode::PsramCmdQpi => {
                    esp_rom_spi_set_op_mode(1, ESP_ROM_SPIFLASH_QIO_MODE);
                    let spi1 = &*esp32s3::SPI1::PTR;
                    spi1.ctrl.modify(|_, w| w.fcmd_quad().set_bit());
                }
                CommandMode::PsramCmdSpi => {
                    esp_rom_spi_set_op_mode(1, ESP_ROM_SPIFLASH_SLOWRD_MODE);
                }
            }
        }
    }

    /// Enter QPI mode
    #[ram]
    fn psram_enable_qio_mode_spi1() {
        const PSRAM_ENTER_QMODE: u16 = 0x35;
        const CS_PSRAM_SEL: u8 = 1 << 1;

        psram_exec_cmd(
            CommandMode::PsramCmdSpi,
            PSRAM_ENTER_QMODE,
            8, // command and command bit len
            0,
            0, // address and address bit len
            0, // dummy bit len
            core::ptr::null(),
            0, // tx data and tx bit len
            core::ptr::null_mut(),
            0,            // rx data and rx bit len
            CS_PSRAM_SEL, // cs bit mask
            false,        // whether is program/erase operation
        );
    }

    #[ram]
    fn psram_set_cs_timing() {
        // SPI0/1 share the cs_hold / cs_setup, cd_hold_time / cd_setup_time registers
        // for PSRAM, so we only need to set SPI0 related registers here
        set_peri_reg_bits(
            SPI0_MEM_SPI_SMEM_AC_REG,
            SPI_MEM_SPI_SMEM_CS_HOLD_TIME_V,
            0,
            SPI_MEM_SPI_SMEM_CS_HOLD_TIME_S,
        );
        set_peri_reg_bits(
            SPI0_MEM_SPI_SMEM_AC_REG,
            SPI_MEM_SPI_SMEM_CS_SETUP_TIME_V,
            0,
            SPI_MEM_SPI_SMEM_CS_SETUP_TIME_S,
        );
        set_peri_reg_mask(
            SPI0_MEM_SPI_SMEM_AC_REG,
            SPI_MEM_SPI_SMEM_CS_HOLD_M | SPI_MEM_SPI_SMEM_CS_SETUP_M,
        );
    }

    #[ram]
    fn psram_gpio_config() {
        // CS1
        let cs1_io: u8 = PSRAM_CS_IO;
        if cs1_io == SPI_CS1_GPIO_NUM {
            unsafe {
                let iomux = &*esp32s3::IO_MUX::PTR;
                iomux.gpio[cs1_io as usize].modify(|_, w| w.mcu_sel().variant(FUNC_SPICS1_SPICS1))
            }
        } else {
            unsafe {
                esp_rom_gpio_connect_out_signal(cs1_io, SPICS1_OUT_IDX, false, false);

                let iomux = &*esp32s3::IO_MUX::PTR;
                iomux.gpio[cs1_io as usize].modify(|_, w| w.mcu_sel().variant(PIN_FUNC_GPIO))
            }
        }

        // WP HD
        let mut wp_io: u8 = PSRAM_SPIWP_SD3_IO;
        let spiconfig = unsafe { esp_rom_efuse_get_flash_gpio_info() };
        if spiconfig == ESP_ROM_EFUSE_FLASH_DEFAULT_SPI {
            // MSPI pins (except wp / hd) are all configured via IO_MUX in 1st
            // bootloader.
        } else {
            // MSPI pins (except wp / hd) are all configured via GPIO matrix in 1st
            // bootloader.
            wp_io = unsafe { esp_rom_efuse_get_flash_wp_gpio() };
        }
        // This ROM function will init both WP and HD pins.
        unsafe {
            esp_rom_spiflash_select_qio_pins(wp_io, spiconfig);
        }
    }

    #[inline(always)]
    fn clear_peri_reg_mask(reg: u32, mask: u32) {
        unsafe {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
        }
    }

    #[inline(always)]
    fn set_peri_reg_mask(reg: u32, mask: u32) {
        unsafe {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
        }
    }

    #[inline(always)]
    fn set_peri_reg_bits(reg: u32, bitmap: u32, value: u32, shift: u32) {
        unsafe {
            (reg as *mut u32).write_volatile(
                ((reg as *mut u32).read_volatile() & !(bitmap << shift))
                    | ((value & bitmap) << shift),
            );
        }
    }

    #[inline(always)]
    fn write_peri_reg(reg: u32, val: u32) {
        unsafe {
            (reg as *mut u32).write_volatile(val);
        }
    }
}
