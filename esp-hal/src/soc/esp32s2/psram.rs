//! # PSRAM "virtual peripheral" driver (ESP32-S2)
//!
//! ## Overview
//!
//! The `PSRAM` module provides support for accessing and controlling the
//! `Pseudo Static Random Access Memory (PSRAM)` on the `ESP32-S2`.
//!
//! The `PSRAM` module enables users to interface with the `PSRAM` memory
//! present on the `ESP32-S2` chip. `PSRAM` provides additional external memory
//! to supplement the internal memory of the `ESP32-S2`, allowing for increased
//! storage capacity and improved performance in certain applications.
//!
//! //! ## Examples
//!
//! ### Quad PSRAM
//! This example shows how to use PSRAM as heap-memory via esp-alloc.
//! You need an ESP32S2 with at least 2 MB of PSRAM memory.
//! Notice that PSRAM example **must** be built in release mode!
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # extern crate alloc;
//! # use alloc::{string::String, vec::Vec};
//! # use esp_alloc as _;
//! # use esp_hal::psram;
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
//!     psram::PsramConfig::default());
//!
//! init_psram_heap(start, size);
//!
//! let mut large_vec: Vec<u32> = Vec::with_capacity(500 * 1024 / 4);
//!
//! for i in 0..(500 * 1024 / 4) {
//!     large_vec.push((i & 0xff) as u32);
//! }
//!
//! let string = String::from("A string allocated in PSRAM");
//! # }
//! ```

pub use crate::soc::psram_common::*;

const EXTMEM_ORIGIN: usize = 0x3f500000;

// Cache Speed
#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum PsramCacheSpeed {
    PsramCacheS80m = 1,
    PsramCacheS40m,
    PsramCacheS26m,
    PsramCacheS20m,
    #[default]
    PsramCacheMax,
}

/// PSRAM configuration
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramConfig {
    /// PSRAM size
    pub size: PsramSize,
    /// Cache Speed
    pub speed: PsramCacheSpeed,
}

/// Initialize PSRAM to be used for data.
///
/// Returns the start of the mapped memory and the size
#[procmacros::ram]
pub(crate) fn init_psram(config: PsramConfig) {
    let mut config = config;
    utils::psram_init(&mut config);

    #[allow(unused)]
    enum CacheLayout {
        Invalid    = 0,
        ICacheLow  = 1 << 0,
        ICacheHigh = 1 << 1,
        DCacheLow  = 1 << 2,
        DCacheHigh = 1 << 3,
    }

    const MMU_ACCESS_SPIRAM: u32 = 1 << 16;

    const CACHE_SIZE_8KB: u32 = 0;
    const CACHE_4WAYS_ASSOC: u32 = 0;
    const CACHE_LINE_SIZE_16B: u32 = 0;

    extern "C" {
        /// Allocate memory to used by ICache and DCache.
        ///
        /// [`sram0_layout`]: u32 the usage of first 8KB internal memory block,
        /// can be CACHE_MEMORY_INVALID,
        /// CACHE_MEMORY_ICACHE_LOW,
        /// CACHE_MEMORY_ICACHE_HIGH, CACHE_MEMORY_DCACHE_LOW and
        /// CACHE_MEMORY_DCACHE_HIGH
        /// [`sram1_layout`]: the usage of second 8KB internal memory block,
        /// [`sram2_layout`]: the usage of third 8KB internal memory block
        /// [`sram3_layout`]: the usage of forth 8KB internal memory block
        fn Cache_Allocate_SRAM(
            sram0_layout: u32,
            sram1_layout: u32,
            sram2_layout: u32,
            sram3_layout: u32,
        );

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

        /// Set DCache modes: cache size, associate ways and cache line size.
        ///
        /// [`cache_size`]: u32 the cache size, can be CACHE_SIZE_HALF and CACHE_SIZE_FULL
        /// [`ways`]: u32 the associate ways of cache, can only be CACHE_4WAYS_ASSOC
        /// [`cache_line_size`]: u32 the cache line size, can be CACHE_LINE_SIZE_16B, CACHE_LINE_SIZE_32B
        fn Cache_Set_DCache_Mode(cache_size: u32, ways: u32, cache_line_size: u32);

        /// Invalidate all cache items in DCache.
        fn Cache_Invalidate_DCache_All();
    }

    unsafe {
        Cache_Allocate_SRAM(
            CacheLayout::ICacheLow as u32,
            CacheLayout::DCacheLow as u32,
            CacheLayout::Invalid as u32,
            CacheLayout::Invalid as u32,
        );
        Cache_Set_DCache_Mode(CACHE_SIZE_8KB, CACHE_4WAYS_ASSOC, CACHE_LINE_SIZE_16B);
        Cache_Invalidate_DCache_All();

        const START_PAGE: u32 = 0;

        if cache_dbus_mmu_set(
            MMU_ACCESS_SPIRAM,
            EXTMEM_ORIGIN as u32,
            START_PAGE << 16,
            64,
            config.size.get() as u32 / 1024 / 64, // number of pages to map
            0,
        ) != 0
        {
            panic!("cache_dbus_mmu_set failed");
        }

        let extmem = &*esp32s2::EXTMEM::PTR;
        extmem.pro_dcache_ctrl1().modify(|_, w| {
            w.pro_dcache_mask_bus0()
                .clear_bit()
                .pro_dcache_mask_bus1()
                .clear_bit()
                .pro_dcache_mask_bus2()
                .clear_bit()
        });
    }

    unsafe {
        crate::soc::MAPPED_PSRAM.memory_range = EXTMEM_ORIGIN..EXTMEM_ORIGIN + config.size.get();
    }
}

pub(crate) mod utils {
    use super::*;

    const PSRAM_RESET_EN: u16 = 0x66;
    const PSRAM_RESET: u16 = 0x99;
    const PSRAM_DEVICE_ID: u16 = 0x9F;
    const CS_PSRAM_SEL: u8 = 1 << 1;

    /// PS-RAM addressing mode
    #[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[allow(unused)]
    pub enum PsramVaddrMode {
        /// App and pro CPU use their own flash cache for external RAM access
        #[default]
        Normal = 0,
        /// App and pro CPU share external RAM caches: pro CPU has low2M, app
        /// CPU has high 2M
        Lowhigh,
        /// App and pro CPU share external RAM caches: pro CPU does even 32yte
        /// ranges, app does odd ones.
        Evenodd,
    }

    // Function initializes the PSRAM by configuring GPIO pins, resetting the PSRAM,
    // and enabling Quad I/O (QIO) mode. It also calls the psram_cache_init
    // function to configure cache parameters and read/write commands.
    pub(crate) fn psram_init(config: &mut PsramConfig) {
        psram_gpio_config();

        if config.size.is_auto() {
            // read chip id
            let mut dev_id = 0u32;
            psram_exec_cmd(
                CommandMode::PsramCmdSpi,
                PSRAM_DEVICE_ID,
                8, // command and command bit len
                0,
                24, // address and address bit len
                0,  // dummy bit len
                core::ptr::null(),
                0, // tx data and tx bit len
                &mut dev_id as *mut _ as *mut u8,
                24,           // rx data and rx bit len
                CS_PSRAM_SEL, // cs bit mask
                false,
            );
            info!("chip id = {:x}", dev_id);

            const PSRAM_ID_EID_S: u32 = 16;
            const PSRAM_ID_EID_M: u32 = 0xff;
            const PSRAM_EID_SIZE_M: u32 = 0x07;
            const PSRAM_EID_SIZE_S: u32 = 5;

            let size_id = (((dev_id) >> PSRAM_ID_EID_S) & PSRAM_ID_EID_M) >> PSRAM_EID_SIZE_S
                & PSRAM_EID_SIZE_M;

            const PSRAM_EID_SIZE_32MBITS: u32 = 1;
            const PSRAM_EID_SIZE_64MBITS: u32 = 2;

            let size = match size_id {
                PSRAM_EID_SIZE_64MBITS => 16 / 8 * 1024 * 1024,
                PSRAM_EID_SIZE_32MBITS => 16 / 8 * 1024 * 1024,
                _ => 16 / 8 * 1024 * 1024,
            };

            info!("size is {}", size);

            config.size = PsramSize::Size(size);
        }

        psram_reset_mode();
        psram_enable_qio_mode();

        psram_cache_init(config.speed, PsramVaddrMode::Normal);
    }

    // send reset command to psram, in spi mode
    fn psram_reset_mode() {
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

    /// Enter QPI mode
    fn psram_enable_qio_mode() {
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

    #[derive(PartialEq)]
    #[allow(unused)]
    enum CommandMode {
        PsramCmdQpi = 0,
        PsramCmdSpi = 1,
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
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
            let spi1 = &*esp32s2::SPI1::PTR;
            let backup_usr = spi1.user().read().bits();
            let backup_usr1 = spi1.user1().read().bits();
            let backup_usr2 = spi1.user2().read().bits();
            let backup_ctrl = spi1.ctrl().read().bits();
            psram_set_op_mode(mode);
            _psram_exec_cmd(
                cmd,
                cmd_bit_len,
                &addr,
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

            spi1.user().write(|w| w.bits(backup_usr));
            spi1.user1().write(|w| w.bits(backup_usr1));
            spi1.user2().write(|w| w.bits(backup_usr2));
            spi1.ctrl().write(|w| w.bits(backup_ctrl));
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn _psram_exec_cmd(
        cmd: u16,
        cmd_bit_len: u16,
        addr: *const u32,
        addr_bit_len: u32,
        dummy_bits: u32,
        mosi_data: *const u8,
        mosi_bit_len: u32,
        miso_data: *mut u8,
        miso_bit_len: u32,
    ) {
        #[repr(C)]
        #[allow(non_camel_case_types)]
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
            addr,
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
                    let spi1 = &*esp32s2::SPI1::PTR;
                    spi1.ctrl().modify(|_, w| w.fcmd_quad().set_bit());
                }
                CommandMode::PsramCmdSpi => {
                    esp_rom_spi_set_op_mode(1, ESP_ROM_SPIFLASH_SLOWRD_MODE);
                }
            }
        }
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

    fn psram_gpio_config() {
        extern "C" {
            fn esp_rom_efuse_get_flash_gpio_info() -> u32;

            /// Enable Quad I/O pin functions
            ///
            /// Sets the HD & WP pin functions for Quad I/O modes, based on the
            /// efuse SPI pin configuration.
            ///
            /// [`wp_gpio_num`]: u8 Number of the WP pin to reconfigure for quad I/O
            /// [`spiconfig`]: u32 Pin configuration, as returned from ets_efuse_get_spiconfig().
            /// - If this parameter is 0, default SPI pins are used and
            ///   wp_gpio_num parameter is ignored.
            /// - If this parameter is 1, default HSPI pins are used and
            ///   wp_gpio_num parameter is ignored.
            /// - For other values, this parameter encodes the HD pin number and
            ///   also the CLK pin number. CLK pin selection is used to
            ///   determine if HSPI or SPI peripheral will be used (use HSPI if
            ///   CLK pin is the HSPI clock pin, otherwise use SPI).
            //   Both HD & WP pins are configured via GPIO matrix to map to the selected peripheral.
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
                // this case is currently not yet supported
                panic!("Unsupported for now! The case 'FLASH pins are all configured via GPIO matrix in ROM.' is not yet supported.");

                // FLASH pins are all configured via GPIO matrix in ROM.
                // psram_io.flash_clk_io =
                // EFUSE_SPICONFIG_RET_SPICLK(spiconfig);
                // psram_io.flash_cs_io = EFUSE_SPICONFIG_RET_SPICS0(spiconfig);
                // psram_io.psram_spiq_sd0_io =
                // EFUSE_SPICONFIG_RET_SPIQ(spiconfig);
                // psram_io.psram_spid_sd1_io =
                // EFUSE_SPICONFIG_RET_SPID(spiconfig);
                // psram_io.psram_spihd_sd2_io =
                // EFUSE_SPICONFIG_RET_SPIHD(spiconfig);
                // psram_io.psram_spiwp_sd3_io =
                // esp_rom_efuse_get_flash_wp_gpio();
            }
            esp_rom_spiflash_select_qio_pins(psram_io.psram_spiwp_sd3_io, spiconfig);
            // s_psram_cs_io = psram_io.psram_cs_io;
        }
    }

    const PSRAM_IO_MATRIX_DUMMY_20M: u32 = 0;
    const PSRAM_IO_MATRIX_DUMMY_40M: u32 = 0;
    const PSRAM_IO_MATRIX_DUMMY_80M: u32 = 0;

    /// Register initialization for sram cache params and r/w commands
    fn psram_cache_init(psram_cache_mode: PsramCacheSpeed, _vaddrmode: PsramVaddrMode) {
        let mut extra_dummy = 0;
        match psram_cache_mode {
            PsramCacheSpeed::PsramCacheS80m => {
                psram_clock_set(1);
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_80M;
            }
            PsramCacheSpeed::PsramCacheS40m => {
                psram_clock_set(2);
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;
            }
            PsramCacheSpeed::PsramCacheS26m => {
                psram_clock_set(3);
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_20M;
            }
            PsramCacheSpeed::PsramCacheS20m => {
                psram_clock_set(4);
                extra_dummy = PSRAM_IO_MATRIX_DUMMY_20M;
            }
            _ => {
                psram_clock_set(2);
            }
        }

        const PSRAM_QUAD_WRITE: u32 = 0x38;
        const PSRAM_FAST_READ_QUAD: u32 = 0xEB;
        const PSRAM_FAST_READ_QUAD_DUMMY: u32 = 0x5;

        unsafe {
            let spi = &*crate::peripherals::SPI0::PTR;

            spi.cache_sctrl()
                .modify(|_, w| w.usr_sram_dio().clear_bit()); // disable dio mode for cache command

            spi.cache_sctrl().modify(|_, w| w.usr_sram_qio().set_bit()); // enable qio mode for cache command

            spi.cache_sctrl()
                .modify(|_, w| w.cache_sram_usr_rcmd().set_bit()); // enable cache read command

            spi.cache_sctrl()
                .modify(|_, w| w.cache_sram_usr_wcmd().set_bit()); // enable cache write command

            // write address for cache command.
            spi.cache_sctrl()
                .modify(|_, w| w.sram_addr_bitlen().bits(23));

            spi.cache_sctrl()
                .modify(|_, w| w.usr_rd_sram_dummy().set_bit()); // enable cache read dummy

            // config sram cache r/w command
            spi.sram_dwr_cmd()
                .modify(|_, w| w.cache_sram_usr_wr_cmd_bitlen().bits(7));

            spi.sram_dwr_cmd().modify(|_, w| {
                w.cache_sram_usr_wr_cmd_bitlen()
                    .bits(PSRAM_QUAD_WRITE as u8)
            });

            spi.sram_dwr_cmd().modify(|_, w| {
                w.cache_sram_usr_wr_cmd_value()
                    .bits(PSRAM_QUAD_WRITE as u16)
            });

            spi.sram_drd_cmd()
                .modify(|_, w| w.cache_sram_usr_rd_cmd_bitlen().bits(7));

            spi.sram_drd_cmd().modify(|_, w| {
                w.cache_sram_usr_rd_cmd_value()
                    .bits(PSRAM_FAST_READ_QUAD as u16)
            });

            // dummy, psram cache :  40m--+1dummy,80m--+2dummy
            spi.cache_sctrl().modify(|_, w| {
                w.sram_rdummy_cyclelen()
                    .bits((PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy) as u8)
            });

            // ESP-IDF has some code here to deal with `!CONFIG_FREERTOS_UNICORE` - not
            // needed for ESP32-S2

            // ENABLE SPI0 CS1 TO PSRAM(CS0--FLASH; CS1--SRAM)
            spi.misc().modify(|_, w| w.cs1_dis().clear_bit());
        }
    }

    fn psram_clock_set(freqdiv: i8) {
        const SPI_MEM_SCLKCNT_N_S: u32 = 16;
        const SPI_MEM_SCLKCNT_H_S: u32 = 8;
        const SPI_MEM_SCLKCNT_L_S: u32 = 0;

        let spi = unsafe { &*crate::peripherals::SPI0::PTR };

        if 1 >= freqdiv {
            spi.sram_clk().modify(|_, w| w.sclk_equ_sysclk().set_bit());
        } else {
            let freqbits: u32 = (((freqdiv - 1) as u32) << SPI_MEM_SCLKCNT_N_S)
                | (((freqdiv / 2 - 1) as u32) << SPI_MEM_SCLKCNT_H_S)
                | (((freqdiv - 1) as u32) << SPI_MEM_SCLKCNT_L_S);
            unsafe {
                spi.sram_clk().modify(|_, w| w.bits(freqbits));
            }
        }
    }
}
