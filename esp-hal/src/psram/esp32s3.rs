use core::ops::Range;

use procmacros::ram;

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::peripherals::{EXTMEM, IO_MUX, MMU_TABLE, SPI0, SPI1};

/// PSRAM interface mode
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PsramMode {
    /// Tries to detect the PSRAM mode. While convenient, not entirely reliable.
    #[default]
    Auto,

    /// The PSRAM is connected via Quad SPI.
    QuadSpi,

    /// The PSRAM is connected via Octal SPI.
    OctalSpi,
}

/// Frequency of flash memory
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashFreq {
    /// Flash frequency 20 MHz
    FlashFreq20m  = 20,
    /// Flash frequency 40 MHz
    FlashFreq40m  = 40,
    /// Flash frequency 80 MHz
    #[default]
    FlashFreq80m  = 80,
    /// Flash frequency 120 MHz
    /// This is not recommended, see <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/flash_psram_config.html>
    FlashFreq120m = 120,
}

/// Frequency of PSRAM memory
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiRamFreq {
    /// PSRAM frequency 40 MHz
    Freq40m  = 40,
    /// PSRAM frequency 80 MHz. Default.
    #[default]
    Freq80m  = 80,
    /// PSRAM frequency 120 MHz
    /// This is not recommended, see <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/flash_psram_config.html>
    Freq120m = 120,
}

/// Core timing configuration
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiTimingConfigCoreClock {
    /// Core clock 80 MHz
    #[default]
    SpiTimingConfigCoreClock80m  = 80,
    /// Core clock 120 MHz
    SpiTimingConfigCoreClock120m = 120,
    /// Core clock 160 MHz
    SpiTimingConfigCoreClock160m = 160,
    /// Core clock 240 MHz
    SpiTimingConfigCoreClock240m = 240,
}

/// PSRAM configuration
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramConfig {
    /// PSRAM interface mode
    pub mode: PsramMode,
    /// PSRAM size
    pub size: PsramSize,
    /// Core timing configuration
    pub core_clock: Option<SpiTimingConfigCoreClock>,
    /// Frequency of flash memory
    pub flash_frequency: FlashFreq,
    /// Frequency of PSRAM memory
    pub ram_frequency: SpiRamFreq,
}

/// Initializes PSRAM to be used for data.
#[procmacros::ram]
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    let success = match config.mode {
        PsramMode::Auto => {
            let mut success = octal_spi_impl::psram_init(config);

            if !success {
                success = quad_spi_impl::psram_init(config);
            }

            success
        }
        PsramMode::QuadSpi => quad_spi_impl::psram_init(config),
        PsramMode::OctalSpi => octal_spi_impl::psram_init(config),
    };

    if !success {
        warn!(
            "Failed to configure PSRAM. This may indicate a missing/inoperable PSRAM chip, or an incorrect PSRAM configuration. Check if the PSRAM chip is present and the configuration is correct."
        );
    }

    success
}

#[procmacros::ram]
pub(crate) fn map_psram(config: PsramConfig) -> Range<usize> {
    const MMU_ACCESS_SPIRAM: u32 = 1 << 15;
    const START_PAGE: u32 = 0;

    unsafe extern "C" {
        fn Cache_Suspend_DCache() -> u32;

        fn Cache_Resume_DCache(param: u32);

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

    const MMU_PAGE_SIZE: u32 = property!("mmu.page_size");

    fn mmu_entry_is_valid(entry_id: usize) -> bool {
        !MMU_TABLE::regs().entry(entry_id).read().invalid().bit()
    }

    // calculate the PSRAM start address to map
    // the linker scripts can produce a gap between mapped IROM and DROM segments
    // bigger than a flash page - i.e. we will see an unmapped memory slot
    // start from the end and find the last mapped flash page
    //
    // More general information about the MMU can be found here:
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/mm.html#introduction
    let mut mapped_pages = 0;

    // the bootloader is using the last page to access flash internally
    // (e.g. to read the app descriptor) so we just skip that
    let flash_mmu_table_size: usize = MMU_TABLE::regs().entry_iter().count();
    for i in (0..(flash_mmu_table_size - 1)).rev() {
        if mmu_entry_is_valid(i) {
            mapped_pages = (i + 1) as u32;
            break;
        }
    }
    let start = EXTMEM_ORIGIN as u32 + (MMU_PAGE_SIZE * mapped_pages);
    debug!("PSRAM start address = {:x}", start);

    // If we need use SPIRAM, we should use data cache.
    unsafe { Cache_Suspend_DCache() };

    let cache_dbus_mmu_set_res = unsafe {
        cache_dbus_mmu_set(
            MMU_ACCESS_SPIRAM,
            start,
            START_PAGE << 16,
            64,
            config.size.get() as u32 / 1024 / 64, // number of pages to map
            0,
        )
    };

    EXTMEM::regs().dcache_ctrl1().modify(|_, w| {
        w.dcache_shut_core0_bus().clear_bit();
        w.dcache_shut_core1_bus().clear_bit()
    });

    unsafe { Cache_Resume_DCache(0) };

    // panic AFTER resuming the cache
    if cache_dbus_mmu_set_res != 0 {
        panic!("cache_dbus_mmu_set failed");
    }

    start as usize..start as usize + config.size.get()
}

pub(crate) mod quad_spi_impl {
    use super::*;
    use crate::psram::quad_xtensa;

    const PSRAM_CS_IO: u8 = 26;
    const SPI_CS1_GPIO_NUM: u8 = 26;
    const FUNC_SPICS1_SPICS1: u8 = 0;
    const PIN_FUNC_GPIO: u8 = 2;
    const PSRAM_SPIWP_SD3_IO: u8 = 10;
    const ESP_ROM_EFUSE_FLASH_DEFAULT_SPI: u32 = 0;
    const SPICS1_OUT_IDX: u8 = 6;

    #[ram]
    pub(crate) fn psram_init(config: &mut PsramConfig) -> bool {
        psram_gpio_config();
        psram_set_cs_timing();

        if config.size.is_auto() {
            let Some(size) = quad_xtensa::detect_quad_size() else {
                return false;
            };
            config.size = PsramSize::Size(size);
        }

        if config.core_clock.is_none() {
            config.core_clock = Some(if config.ram_frequency == SpiRamFreq::Freq120m {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock120m
            } else {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m
            });
        }

        quad_xtensa::psram_reset_mode_spi1();
        quad_xtensa::psram_enable_qio_mode_spi1();

        // Timing tuning is not implemented. Quad 80 MHz does not require it.
        // See IDF `mspi_timing_psram_tuning`.
        mspi_timing_psram_tuning();

        quad_xtensa::config_psram_spi_phases();
        mspi_timing_enter_high_speed_mode(true, config);

        info!("PSRAM initialized successfully in Quad SPI mode");
        true
    }

    unsafe extern "C" {
        fn esp_rom_efuse_get_flash_gpio_info() -> u32;

        fn esp_rom_efuse_get_flash_wp_gpio() -> u8;

        fn esp_rom_gpio_connect_out_signal(
            gpio_num: u8,
            signal_idx: u8,
            out_inv: bool,
            oen_inv: bool,
        );

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
        ///   number. CLK pin selection is used to determine if HSPI or SPI peripheral will be used
        ///   (use HSPI if CLK pin is the HSPI clock pin, otherwise use SPI).
        //   Both HD & WP pins are configured via GPIO matrix to map to the selected peripheral.
        fn esp_rom_spiflash_select_qio_pins(wp_gpio_num: u8, spiconfig: u32);
    }

    #[ram]
    fn mspi_timing_psram_tuning() {
        // currently we only support !SPI_TIMING_PSRAM_NEEDS_TUNING
        // see https://github.com/espressif/esp-idf/blob/4e24516ee2731eb55687182d4e061b5b93a9e33f/components/esp_hw_support/mspi_timing_tuning.c#L391-L415
    }

    /// Sets SPI0 FLASH and PSRAM module clock, din_num, din_mode and extra
    /// dummy, according to the configuration got from timing tuning
    /// function (`calculate_best_flash_tuning_config`). iF control_spi1 ==
    /// 1, will also update SPI1 timing registers. Should only be set to 1 when
    /// do tuning.
    ///
    /// Must always be called after `mspi_timing_flash_tuning` or
    /// `calculate_best_flash_tuning_config`
    #[ram]
    fn mspi_timing_enter_high_speed_mode(control_spi1: bool, config: &PsramConfig) {
        let core_clock: SpiTimingConfigCoreClock = mspi_core_clock(config);
        let flash_div: u32 = flash_clock_divider(config);
        let psram_div: u32 = psram_clock_divider(config);

        info!(
            "PSRAM {} MHz, flash {} MHz, core_clock {:?}, flash_div = {}, psram_div = {}",
            config.ram_frequency as u32,
            config.flash_frequency as u32,
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
        quad_xtensa::spi0_timing_config_set_psram_clock(psram_div);
    }

    #[ram]
    fn spi0_timing_config_set_core_clock(core_clock: SpiTimingConfigCoreClock) {
        unsafe {
            SPI0::regs().core_clk_sel().modify(|_, w| {
                w.core_clk_sel().bits(match core_clock {
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m => 0,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock120m => 1,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m => 2,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m => 3,
                })
            });
        }
    }

    #[ram]
    fn mspi_core_clock(config: &PsramConfig) -> SpiTimingConfigCoreClock {
        config.core_clock.unwrap_or_default()
    }

    #[ram]
    fn flash_clock_divider(config: &PsramConfig) -> u32 {
        config.core_clock.unwrap_or_default() as u32 / config.flash_frequency as u32
    }

    #[ram]
    fn psram_clock_divider(config: &PsramConfig) -> u32 {
        config.core_clock.unwrap_or_default() as u32 / config.ram_frequency as u32
    }

    #[ram]
    fn psram_set_cs_timing() {
        // SPI0/1 share the cs_hold / cs_setup, cd_hold_time / cd_setup_time registers
        // for PSRAM, so we only need to set SPI0 related registers here
        SPI0::regs().spi_smem_ac().modify(|_, w| unsafe {
            w.spi_smem_cs_hold_time().bits(0);
            w.spi_smem_cs_setup_time().bits(0);
            w.spi_smem_cs_hold().set_bit();
            w.spi_smem_cs_setup().set_bit()
        });
    }

    #[ram]
    fn psram_gpio_config() {
        // CS1
        let cs1_io: u8 = PSRAM_CS_IO;
        let mcu_sel = if cs1_io == SPI_CS1_GPIO_NUM {
            FUNC_SPICS1_SPICS1
        } else {
            unsafe {
                esp_rom_gpio_connect_out_signal(cs1_io, SPICS1_OUT_IDX, false, false);
                PIN_FUNC_GPIO
            }
        };

        IO_MUX::regs()
            .gpio(cs1_io as usize)
            .modify(|_, w| unsafe { w.mcu_sel().bits(mcu_sel) });

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
}

pub(crate) mod octal_spi_impl {
    use super::*;
    use crate::psram::quad_xtensa;

    const OPI_PSRAM_SYNC_READ: u16 = 0x0000;
    const OPI_PSRAM_SYNC_WRITE: u16 = 0x8080;
    const OPI_PSRAM_REG_READ: u16 = 0x4040;
    const OPI_PSRAM_REG_WRITE: u16 = 0xC0C0;
    const OCT_PSRAM_RD_CMD_BITLEN: u8 = 16;
    const OCT_PSRAM_WR_CMD_BITLEN: u8 = 16;
    const OCT_PSRAM_ADDR_BITLEN: u8 = 32;
    const OCT_PSRAM_RD_DUMMY_BITLEN: u8 = 2 * (10 - 1);
    const OCT_PSRAM_WR_DUMMY_BITLEN: u8 = 2 * (5 - 1);
    const OCT_PSRAM_CS1_IO: u8 = SPI_CS1_GPIO_NUM;
    const OCT_PSRAM_VENDOR_ID: u8 = 0xD;

    const OCT_PSRAM_CS_SETUP_TIME: u8 = 3;
    const OCT_PSRAM_CS_HOLD_TIME: u8 = 3;
    const OCT_PSRAM_CS_HOLD_DELAY: u8 = 2;

    const SPI_CS1_GPIO_NUM: u8 = 26;
    const FUNC_SPICS1_SPICS1: u8 = 0;

    const ESP_ROM_SPIFLASH_OPI_DTR_MODE: u8 = 7;

    unsafe extern "C" {
        // @brief To execute a flash operation command
        // @param spi_num spi port
        // @param mode Flash Read Mode
        // @param cmd data to send in command field
        // @param cmd_bit_len bit length of command field
        // @param addr data to send in address field
        // @param addr_bit_len bit length of address field
        // @param dummy_bits bit length of dummy field
        // @param mosi_data data buffer to be sent in mosi field
        // @param mosi_bit_len bit length of data buffer to be sent in mosi field
        // @param miso_data data buffer to accept data in miso field
        // @param miso_bit_len bit length of data buffer to accept data in miso field
        // @param cs_mark decide which cs pin to use. 0: cs0, 1: cs1
        // @param is_write_erase_operation to indicate whether this a write or erase
        // flash operation
        fn esp_rom_opiflash_exec_cmd(
            spi_num: u32,
            mode: u8,
            cmd: u32,
            cmd_bit_len: u32,
            addr: u32,
            addr_bit_len: u32,
            dummy_bits: u32,
            mosi_data: *const u8,
            mosi_bit_len: u32,
            miso_data: *mut u8,
            miso_bit_len: u32,
            cs_mask: u32,
            is_write_erase_operation: bool,
        );

        // @brief Set data swap mode in DTR(DDR) mode
        // @param spi_num spi port
        // @param wr_swap to decide whether to swap fifo data in dtr write operation
        // @param rd_swap to decide whether to swap fifo data in dtr read operation
        fn esp_rom_spi_set_dtr_swap_mode(spi: u32, wr_swap: bool, rd_swap: bool);

        fn esp_rom_opiflash_pin_config();
    }

    /// Represents the operational mode registers of an OPI PSRAM.
    #[derive(Default)]
    #[repr(C)]
    struct OpiPsramModeReg {
        // Mode register 0 (MR0).
        pub mr0: u8,
        // Mode register 1 (MR1).
        pub mr1: u8,
        // Mode register 2 (MR2).
        pub mr2: u8,
        // Mode register 3 (MR3).
        pub mr3: u8,
        // Mode register 4 (MR4).
        pub mr4: u8,
        // Mode register 8 (MR8).
        pub mr8: u8,
    }

    #[allow(unused)]
    impl OpiPsramModeReg {
        fn drive_str(&self) -> u8 {
            self.mr0 & 0b11
        }

        fn set_drive_str(&mut self, value: u8) {
            self.mr0 &= !0b11;
            self.mr0 |= value & 0b11;
        }

        fn read_latency(&self) -> u8 {
            (self.mr0 >> 2) & 0b111
        }

        fn set_read_latency(&mut self, value: u8) {
            self.mr0 &= !(0b111 << 2);
            self.mr0 |= (value & 0b111) << 2;
        }

        fn lt(&self) -> u8 {
            (self.mr0 >> 5) & 0b1
        }

        fn set_lt(&mut self, value: u8) {
            self.mr0 &= !(0b1 << 5);
            self.mr0 |= (value & 0b1) << 5;
        }

        fn rsvd0_1(&self) -> u8 {
            (self.mr0 >> 6) & 0b11
        }

        fn set_rsvd0_1(&mut self, value: u8) {
            self.mr0 &= !(0b11 << 6);
            self.mr0 |= (value & 0b11) << 6;
        }

        fn vendor_id(&self) -> u8 {
            self.mr1 & 0b11111
        }

        fn set_vendor_id(&mut self, value: u8) {
            self.mr1 &= !0b11111;
            self.mr1 |= value & 0b11111;
        }

        fn rsvd0_2(&self) -> u8 {
            (self.mr1 >> 5) & 0b111
        }

        fn set_rsvd0_2(&mut self, value: u8) {
            self.mr1 &= !(0b111 << 5);
            self.mr1 |= (value & 0b111) << 5;
        }

        fn density(&self) -> u8 {
            self.mr2 & 0b111
        }

        fn set_density(&mut self, value: u8) {
            self.mr2 &= !0b111;
            self.mr2 |= value & 0b111;
        }

        fn dev_id(&self) -> u8 {
            (self.mr2 >> 3) & 0b11
        }

        fn set_dev_id(&mut self, value: u8) {
            self.mr2 &= !(0b11 << 3);
            self.mr2 |= (value & 0b11) << 3;
        }

        fn rsvd1_2(&self) -> u8 {
            (self.mr2 >> 5) & 0b11
        }

        fn set_rsvd1_2(&mut self, value: u8) {
            self.mr2 &= !(0b11 << 5);
            self.mr2 |= (value & 0b11) << 5;
        }

        fn gb(&self) -> u8 {
            (self.mr2 >> 7) & 0b1
        }

        fn set_gb(&mut self, value: u8) {
            self.mr2 &= !(0b1 << 7);
            self.mr2 |= (value & 0b1) << 7;
        }

        fn rsvd3_7(&self) -> u8 {
            self.mr3 & 0b11111
        }

        fn set_rsvd3_7(&mut self, value: u8) {
            self.mr3 &= !0b11111;
            self.mr3 |= value & 0b11111;
        }

        fn srf(&self) -> u8 {
            (self.mr3 >> 5) & 0b1
        }

        fn set_srf(&mut self, value: u8) {
            self.mr3 &= !(0b1 << 5);
            self.mr3 |= (value & 0b1) << 5;
        }

        fn vcc(&self) -> u8 {
            (self.mr3 >> 6) & 0b1
        }

        fn set_vcc(&mut self, value: u8) {
            self.mr3 &= !(0b1 << 6);
            self.mr3 |= (value & 0b1) << 6;
        }

        fn rsvd0(&self) -> u8 {
            (self.mr3 >> 7) & 0b1
        }

        fn set_rsvd0(&mut self, value: u8) {
            self.mr3 &= !(0b1 << 7);
            self.mr3 |= (value & 0b1) << 7;
        }

        fn pasr(&self) -> u8 {
            self.mr4 & 0b111
        }

        fn set_pasr(&mut self, value: u8) {
            self.mr4 &= !0b111;
            self.mr4 |= value & 0b111;
        }

        fn rf(&self) -> u8 {
            (self.mr4 >> 3) & 0b1
        }

        fn set_rf(&mut self, value: u8) {
            self.mr4 &= !(0b1 << 3);
            self.mr4 |= (value & 0b1) << 3;
        }

        fn rsvd3(&self) -> u8 {
            (self.mr4 >> 4) & 0b1
        }

        fn set_rsvd3(&mut self, value: u8) {
            self.mr4 &= !(0b1 << 4);
            self.mr4 |= (value & 0b1) << 4;
        }

        fn wr_latency(&self) -> u8 {
            (self.mr4 >> 5) & 0b111
        }

        fn set_wr_latency(&mut self, value: u8) {
            self.mr4 &= !(0b111 << 5);
            self.mr4 |= (value & 0b111) << 5;
        }

        fn bl(&self) -> u8 {
            self.mr8 & 0b11
        }

        fn set_bl(&mut self, value: u8) {
            self.mr8 &= !0b11;
            self.mr8 |= value & 0b11;
        }

        fn bt(&self) -> u8 {
            (self.mr8 >> 2) & 0b1
        }

        fn set_bt(&mut self, value: u8) {
            self.mr8 &= !(0b1 << 2);
            self.mr8 |= (value & 0b1) << 2;
        }

        fn rsvd0_4(&self) -> u8 {
            (self.mr8 >> 3) & 0b11111
        }

        fn set_rsvd0_4(&mut self, value: u8) {
            self.mr8 &= !(0b11111 << 3);
            self.mr8 |= (value & 0b11111) << 3;
        }
    }

    #[ram]
    pub(crate) fn psram_init(config: &mut PsramConfig) -> bool {
        mspi_pin_init();
        set_psram_cs_timing();

        // for now we don't support ECC
        // "s_configure_psram_ecc();"

        // enter MSPI slow mode to init PSRAM device registers
        spi_timing_enter_mspi_low_speed_mode(true);

        unsafe {
            // set to variable dummy mode
            SPI1::regs()
                .ddr()
                .modify(|_, w| w.spi_fmem_var_dummy().set_bit());
            esp_rom_spi_set_dtr_swap_mode(1, false, false);
        }

        // Set PSRAM read latency and drive strength
        let mut mode_reg = OpiPsramModeReg::default();
        mode_reg.set_lt(1);
        mode_reg.set_read_latency(2);
        mode_reg.set_drive_str(0);
        mode_reg.set_bl(3);
        mode_reg.set_bt(0);

        init_psram_mode_reg(1, &mode_reg);
        // Print PSRAM info
        psram_mode_reg(1, &mut mode_reg);

        print_psram_info(&mode_reg);

        if mode_reg.vendor_id() != OCT_PSRAM_VENDOR_ID {
            debug!(
                "Unknown PSRAM vendor ID: {:x}. PSRAM chip not found or not supported. Check if the interface mode is configured correctly.",
                mode_reg.vendor_id()
            );
            return false;
        }

        let psram_size = match mode_reg.density() {
            0x0 => 2 * 1024 * 1024,
            0x1 => 4 * 1024 * 1024,
            0x3 => 8 * 1024 * 1024,
            0x5 => 16 * 1024 * 1024,
            0x7 => 32 * 1024 * 1024,
            _ => 0,
        };
        info!("{} bytes of PSRAM", psram_size);

        if config.core_clock.is_none() {
            config.core_clock = Some(if config.ram_frequency == SpiRamFreq::Freq80m {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m
            } else if config.ram_frequency == SpiRamFreq::Freq120m {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m
            } else {
                SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m
            });
        }
        if config.size.is_auto() {
            config.size = PsramSize::Size(psram_size);
        }

        // Do PSRAM timing tuning, we use SPI1 to do the tuning, and set the
        // SPI0 PSRAM timing related registers accordingly
        // this is unsupported for now
        // spi_timing_psram_tuning();

        // Back to the high speed mode. Flash/PSRAM clocks are set to the clock
        // that user selected. SPI0/1 registers are all set correctly
        spi_timing_enter_mspi_high_speed_mode(true, config);

        // Tuning may change SPI1 regs, whereas legacy spi_flash APIs rely on
        // these regs. This function is to restore SPI1 init state.
        spi_flash_set_rom_required_regs();

        // Flash chip requires MSPI specifically, call this function to set them
        // this is unsupported for now
        // spi_flash_set_vendor_required_regs();

        config_psram_spi_phases();

        info!("PSRAM initialized successfully in Octal SPI mode");
        true
    }

    // Configure PSRAM SPI0 phase related registers here according to the PSRAM chip
    // requirement
    fn config_psram_spi_phases() {
        SPI0::regs().cache_sctrl().modify(|_, w| unsafe {
            w.sram_oct().set_bit();

            w.cache_sram_usr_rcmd().set_bit();
            w.cache_sram_usr_wcmd().set_bit();

            // Config ADDR phase
            w.sram_addr_bitlen().bits(OCT_PSRAM_ADDR_BITLEN - 1);
            w.cache_usr_scmd_4byte().set_bit();

            // Config RD/WR Dummy phase
            w.usr_rd_sram_dummy().set_bit();
            w.usr_wr_sram_dummy().set_bit();
            w.sram_rdummy_cyclelen().bits(OCT_PSRAM_RD_DUMMY_BITLEN - 1);
            w.sram_wdummy_cyclelen().bits(OCT_PSRAM_WR_DUMMY_BITLEN - 1)
        });

        // Config Write CMD phase for SPI0 to access PSRAM
        SPI0::regs().sram_dwr_cmd().modify(|_, w| unsafe {
            w.cache_sram_usr_wr_cmd_value().bits(OPI_PSRAM_SYNC_WRITE);
            w.cache_sram_usr_wr_cmd_bitlen()
                .bits(OCT_PSRAM_WR_CMD_BITLEN - 1)
        });

        // Config Read CMD phase for SPI0 to access PSRAM
        SPI0::regs().sram_drd_cmd().modify(|_, w| unsafe {
            w.cache_sram_usr_rd_cmd_bitlen()
                .bits(OCT_PSRAM_RD_CMD_BITLEN - 1);
            w.cache_sram_usr_rd_cmd_value().bits(OPI_PSRAM_SYNC_READ)
        });

        SPI0::regs().spi_smem_ddr().modify(|_, w| {
            w.spi_smem_var_dummy().set_bit();
            w.wdat_swp().clear_bit();
            w.rdat_swp().clear_bit();
            w.en().set_bit()
        });

        SPI0::regs().sram_cmd().modify(|_, w| {
            w.sdummy_out().set_bit();
            w.scmd_oct().set_bit();
            w.saddr_oct().set_bit();
            w.sdout_oct().set_bit();
            w.sdin_oct().set_bit()
        });
    }

    #[ram]
    fn spi_flash_set_rom_required_regs() {
        // Disable the variable dummy mode when doing timing tuning
        SPI1::regs()
            .ddr()
            .modify(|_, w| w.spi_fmem_var_dummy().clear_bit());
        // STR /DTR mode setting is done every time when
        // `esp_rom_opiflash_exec_cmd` is called
        //
        // Add any registers that are not set in ROM SPI flash functions here in
        // the future
    }

    #[ram]
    fn mspi_pin_init() {
        unsafe { esp_rom_opiflash_pin_config() };
        // Set F4R4 board pin drive strength. TODO: IDF-3663

        // For now, set them all to 3. Need to check after QVL test results are out.
        // TODO: IDF-3663 Set default clk
        SPI0::regs().date().modify(|_, w| unsafe {
            w.spi_spiclk_pad_drv_ctl_en().set_bit();
            w.spi_smem_spiclk_fun_drv().bits(3);
            w.spi_fmem_spiclk_fun_drv().bits(3)
        });

        // Set default mspi d0 ~ d7, dqs pin drive strength
        let pins = [
            OCT_PSRAM_CS1_IO as usize,
            27,
            28,
            31,
            32,
            33,
            34,
            35,
            36,
            37,
        ];
        for pin in pins {
            IO_MUX::regs()
                .gpio(pin)
                .modify(|_, w| unsafe { w.fun_drv().bits(3) });
        }

        IO_MUX::regs()
            .gpio(OCT_PSRAM_CS1_IO as usize)
            .modify(|_, w| unsafe { w.mcu_sel().bits(FUNC_SPICS1_SPICS1) });
    }

    fn spi_timing_enter_mspi_low_speed_mode(control_spi1: bool) {
        // Here we are going to slow the SPI1 frequency to 20Mhz, so we need to
        // set SPI1 din_num and din_mode regs.
        //
        // Because SPI0 and SPI1 share the din_num and din_mode regs, so if we
        // clear SPI1 din_num and din_mode to 0, if the SPI0 flash
        // module clock is still in high freq, it may not work correctly.
        //
        // Therefore, here we need to slow both the SPI0 and SPI1 and related
        // timing tuning regs to 20Mhz configuration.
        // Switch SPI1 and SPI0 clock as 20MHz, set its SPIMEM core clock as 80M and set
        // clock division as 4
        spi0_timing_config_set_core_clock(SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m); // SPI0 and SPI1 share the register for core clock. So we only set SPI0 here.
        spi0_timing_config_set_flash_clock(4);
        if control_spi1 {
            // After tuning, won't touch SPI1 again
            spi1_timing_config_set_flash_clock(4);
        }

        // Set PSRAM module clock
        quad_xtensa::spi0_timing_config_set_psram_clock(4);

        // for now we don't support tuning the timing
        // "clear_timing_tuning_regs(control_spi1);"
    }

    // Set SPI0 FLASH and PSRAM module clock, din_num, din_mode and extra dummy,
    // according to the configuration got from timing tuning function
    // (`calculate_best_flash_tuning_config`). iF control_spi1 == 1, will also
    // update SPI1 timing registers. Should only be set to 1 when do tuning.
    //
    // This function should always be called after `spi_timing_flash_tuning` or
    // `calculate_best_flash_tuning_config`
    fn spi_timing_enter_mspi_high_speed_mode(control_spi1: bool, config: &PsramConfig) {
        let flash_div: u32 = flash_clock_divider(config);
        let psram_div: u32 = psram_clock_divider(config);

        // Set SPI01 core clock
        spi0_timing_config_set_core_clock(config.core_clock.unwrap_or_default()); // SPI0 and SPI1 share the register for core clock. So we only set SPI0 here.
        // Set FLASH module clock
        spi0_timing_config_set_flash_clock(flash_div);
        if control_spi1 {
            spi1_timing_config_set_flash_clock(flash_div);
        }
        // Set PSRAM module clock
        quad_xtensa::spi0_timing_config_set_psram_clock(psram_div);

        // for now we don't support tuning the timing
        // "set_timing_tuning_regs_as_required(true);"
    }

    fn set_psram_cs_timing() {
        // SPI0/1 share the cs_hold / cs_setup, cd_hold_time / cd_setup_time,
        // cs_hold_delay registers for PSRAM, so we only need to set SPI0 related
        // registers here
        SPI0::regs().spi_smem_ac().modify(|_, w| unsafe {
            w.spi_smem_cs_hold().set_bit();
            w.spi_smem_cs_setup().set_bit();
            w.spi_smem_cs_hold_time().bits(OCT_PSRAM_CS_HOLD_TIME);
            w.spi_smem_cs_setup_time().bits(OCT_PSRAM_CS_SETUP_TIME);

            // CONFIG_SPIRAM_ECC_ENABLE unsupported for now
            // CS1 high time
            w.spi_smem_cs_hold_delay().bits(OCT_PSRAM_CS_HOLD_DELAY)
        });
    }

    fn psram_mode_reg(spi_num: u32, out_reg: &mut OpiPsramModeReg) {
        let mode = ESP_ROM_SPIFLASH_OPI_DTR_MODE;
        let cmd_len: u32 = 16;
        let addr_bit_len: u32 = 32;
        let dummy: u32 = OCT_PSRAM_RD_DUMMY_BITLEN as u32;
        let mut data_bit_len: u32 = 16;

        unsafe {
            // Read MR0~1 register
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_READ as u32,
                cmd_len,
                0x0,
                addr_bit_len,
                dummy,
                core::ptr::null(),
                0,
                &mut out_reg.mr0,
                data_bit_len,
                1 << 1,
                false,
            );
            // Read MR2~3 register
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_READ as u32,
                cmd_len,
                0x2,
                addr_bit_len,
                dummy,
                core::ptr::null(),
                0,
                &mut out_reg.mr2,
                data_bit_len,
                1 << 1,
                false,
            );
            data_bit_len = 8;
            // Read MR4 register
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_READ as u32,
                cmd_len,
                0x4,
                addr_bit_len,
                dummy,
                core::ptr::null(),
                0,
                &mut out_reg.mr4,
                data_bit_len,
                1 << 1,
                false,
            );
            // Read MR8 register
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_READ as u32,
                cmd_len,
                0x8,
                addr_bit_len,
                dummy,
                core::ptr::null(),
                0,
                &mut out_reg.mr8,
                data_bit_len,
                1 << 1,
                false,
            );
        }
    }

    // Initialise mode registers of the PSRAM
    fn init_psram_mode_reg(spi_num: u32, mode_reg_config: &OpiPsramModeReg) {
        let mode = ESP_ROM_SPIFLASH_OPI_DTR_MODE;
        let cmd_len: u32 = 16;
        let addr: u32 = 0x0; // 0x0 is the MR0 register
        let addr_bit_len: u32 = 32;
        let dummy = OCT_PSRAM_RD_DUMMY_BITLEN as u32;
        let mut mode_reg = OpiPsramModeReg::default();
        let data_bit_len: u32 = 16;

        // read
        unsafe {
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_READ as u32,
                cmd_len,
                addr,
                addr_bit_len,
                dummy,
                core::ptr::null(),
                0,
                &mut mode_reg.mr0,
                data_bit_len,
                1 << 1,
                false,
            );
        }

        // modify
        mode_reg.set_lt(mode_reg_config.lt());
        mode_reg.set_read_latency(mode_reg_config.read_latency());
        mode_reg.set_drive_str(mode_reg_config.drive_str());

        // write
        unsafe {
            esp_rom_opiflash_exec_cmd(
                spi_num,
                mode,
                OPI_PSRAM_REG_WRITE as u32,
                cmd_len,
                addr,
                addr_bit_len,
                0,
                &mode_reg.mr0,
                16,
                core::ptr::null_mut(),
                0,
                1 << 1,
                false,
            );
        }

        // CONFIG_SPIRAM_ECC_ENABLE not yet supported
    }

    fn print_psram_info(reg_val: &OpiPsramModeReg) {
        info!(
            "vendor id    : {:02x} ({})",
            reg_val.vendor_id(),
            if reg_val.vendor_id() == 0x0d {
                "AP"
            } else {
                "UNKNOWN"
            }
        );
        info!(
            "dev id       : {:02x} (generation {})",
            reg_val.dev_id(),
            reg_val.dev_id() + 1
        );
        info!(
            "density      : {:02x} ({} Mbit)",
            reg_val.density(),
            if reg_val.density() == 0x1 {
                32
            } else if reg_val.density() == 0x3 {
                64
            } else if reg_val.density() == 0x5 {
                128
            } else if reg_val.density() == 0x7 {
                256
            } else {
                0
            }
        );
        info!(
            "good-die     : {:02x} ({})",
            reg_val.gb(),
            if reg_val.gb() == 1 { "Pass" } else { "Fail" }
        );
        info!(
            "Latency      : {:02x} ({})",
            reg_val.lt(),
            if reg_val.lt() == 1 {
                "Fixed"
            } else {
                "Variable"
            }
        );
        info!(
            "VCC          : {:02x} ({})",
            reg_val.vcc(),
            if reg_val.vcc() == 1 { "3V" } else { "1.8V" }
        );
        info!(
            "SRF          : {:02x} ({} Refresh)",
            reg_val.srf(),
            if reg_val.srf() == 0x1 { "Fast" } else { "Slow" }
        );
        info!(
            "BurstType    : {:02x} ({} Wrap)",
            reg_val.bt(),
            if reg_val.bt() == 1 && reg_val.bl() != 3 {
                "Hybrid"
            } else {
                ""
            }
        );
        info!(
            "BurstLen     : {:02x} ({} Byte)",
            reg_val.bl(),
            if reg_val.bl() == 0x00 {
                16
            } else if reg_val.bl() == 0x01 {
                32
            } else if reg_val.bl() == 0x10 {
                64
            } else {
                1024
            }
        );
        info!(
            "Readlatency  : {:02x} ({} cycles@{})",
            reg_val.read_latency(),
            reg_val.read_latency() * 2 + 6,
            if reg_val.lt() == 1 {
                "Fixed"
            } else {
                "Variable"
            }
        );
        info!(
            "DriveStrength: {:02x} (1/{})",
            reg_val.drive_str(),
            if reg_val.drive_str() == 0x00 {
                1
            } else if reg_val.drive_str() == 0x01 {
                2
            } else if reg_val.drive_str() == 0x02 {
                4
            } else {
                8
            }
        );
    }

    #[ram]
    fn spi0_timing_config_set_core_clock(core_clock: SpiTimingConfigCoreClock) {
        unsafe {
            SPI0::regs().core_clk_sel().modify(|_, w| {
                w.core_clk_sel().bits(match core_clock {
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m => 0,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock120m => 1,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m => 2,
                    SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m => 3,
                })
            });
        }
    }

    #[ram]
    fn flash_clock_divider(config: &PsramConfig) -> u32 {
        config.core_clock.unwrap_or_default() as u32 / config.flash_frequency as u32
    }

    #[ram]
    fn psram_clock_divider(config: &PsramConfig) -> u32 {
        config.core_clock.unwrap_or_default() as u32 / config.ram_frequency as u32
    }
}

#[ram]
fn spi0_timing_config_set_flash_clock(freqdiv: u32) {
    if freqdiv == 1 {
        SPI0::regs().clock().write(|w| unsafe {
            w.clk_equ_sysclk().set_bit();
            w.clkcnt_h().bits(0);
            w.clkcnt_n().bits(0);
            w.clkcnt_l().bits(0)
        });
    } else {
        SPI0::regs().clock().write(|w| unsafe {
            w.clk_equ_sysclk().clear_bit();
            w.clkcnt_h().bits((freqdiv / 2 - 1) as u8);
            w.clkcnt_n().bits((freqdiv - 1) as u8);
            w.clkcnt_l().bits((freqdiv - 1) as u8)
        });
    }
}

#[ram]
fn spi1_timing_config_set_flash_clock(freqdiv: u32) {
    if freqdiv == 1 {
        SPI1::regs().clock().write(|w| unsafe {
            w.clk_equ_sysclk().set_bit();
            w.clkcnt_h().bits(0);
            w.clkcnt_n().bits(0);
            w.clkcnt_l().bits(0)
        });
    } else {
        SPI1::regs().clock().write(|w| unsafe {
            w.clk_equ_sysclk().clear_bit();
            w.clkcnt_h().bits((freqdiv / 2 - 1) as u8);
            w.clkcnt_n().bits((freqdiv - 1) as u8);
            w.clkcnt_l().bits((freqdiv - 1) as u8)
        });
    }
}
