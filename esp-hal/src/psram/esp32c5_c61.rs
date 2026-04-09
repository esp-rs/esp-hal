use super::PsramSize;
use crate::{
    peripherals::{SPI0, SPI1},
    psram::implem::quad::MspiTimingSpeedMode,
};

mod quad;

use procmacros::ram;
use quad::CommandMode;

const EXTMEM_ORIGIN: u32 = 0x42000000;

const FUNC_SPICS1_SPICS1: u8 = 0;
const PIN_FUNC_GPIO: u8 = 1;

cfg_if::cfg_if! {
    if #[cfg(esp32c61)] {
        const PSRAM_CS_IO: u8 = 14;
        const SPI_CS1_GPIO_NUM: u8 = 14;
        const SPICS1_OUT_IDX: u8 = 102;
        const PSRAM_SPIWP_SD3_IO: u8 = 17;
    } else if #[cfg(esp32c5)] {
        const PSRAM_CS_IO: u8 = 15;
        const SPI_CS1_GPIO_NUM: u8 = 15;
        const SPICS1_OUT_IDX: u8 = 101;
        const PSRAM_SPIWP_SD3_IO: u8 = 18;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum PsramLlCsIdT {
    PsramLlCsId0,
    PsramLlCsId1,
}

/// Frequency of flash memory
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashFreq {
    /// Flash frequency 40 MHz
    FlashFreq40m = 40,
    /// Flash frequency 80 MHz
    #[default]
    FlashFreq80m = 80,
}

impl FlashFreq {
    fn mhz(&self) -> u32 {
        match self {
            FlashFreq::FlashFreq40m => 40,
            FlashFreq::FlashFreq80m => 80,
        }
    }
}

/// Frequency of PSRAM memory
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum SpiRamFreq {
    /// PSRAM frequency 40 MHz
    #[default]
    Freq40m = 40,
    /// PSRAM frequency 80 MHz
    Freq80m = 80,
}

impl SpiRamFreq {
    pub(crate) fn need_timing_tuning(&self) -> bool {
        *self != SpiRamFreq::Freq40m
    }

    fn mhz(&self) -> u32 {
        match self {
            SpiRamFreq::Freq40m => 40,
            SpiRamFreq::Freq80m => 80,
        }
    }
}

/// MSPI timing tuning parameters.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct MspiTimingTuningParam {
    /// Input signal delay mode
    pub spi_din_mode: u8,
    /// Input signal delay number
    pub spi_din_num: u8,
    /// Extra dummy length
    pub extra_dummy_len: u8,
}

/// PSRAM configuration
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramConfig {
    /// PSRAM size
    pub size: PsramSize,
    /// Frequency of flash memory
    pub flash_frequency: FlashFreq,
    /// Flash timing tuning configuration.
    /// Used as needed.
    pub flash_tuning: MspiTimingTuningParam,
    /// Frequency of PSRAM memory
    pub ram_frequency: SpiRamFreq,
    /// PSRAM timing tuning configuration.
    /// Used as needed.
    pub ram_tuning: MspiTimingTuningParam,
}

impl Default for PsramConfig {
    fn default() -> Self {
        Self {
            size: Default::default(),
            flash_frequency: Default::default(),
            flash_tuning: MspiTimingTuningParam {
                spi_din_mode: 3,
                spi_din_num: 1,
                extra_dummy_len: 2,
            },
            ram_frequency: Default::default(),
            ram_tuning: MspiTimingTuningParam {
                spi_din_mode: 3,
                spi_din_num: 1,
                extra_dummy_len: 2,
            },
        }
    }
}

/// Initialize PSRAM to be used for data.
///
/// Returns the start of the mapped memory and the size
#[procmacros::ram]
pub(crate) fn init_psram(config: PsramConfig) {
    let mut config = config;

    quad::psram_init(&mut config);

    const MMU_ACCESS_SPIRAM: u32 = 1 << 9;

    let start = unsafe {
        const MMU_PAGE_SIZE: u32 = 0x10000;
        const FLASH_MMU_TABLE_SIZE: u32 = 512;
        const MMU_VALID: u32 = 1 << 10;

        // TODO add the used registers to the PAC and use them
        const SPI_MEM_MMU_ITEM_INDEX: *mut u32 = (0x60002000 + 0x380) as *mut u32;
        const SPI_MEM_MMU_ITEM_CONTENT: *mut u32 = (0x60002000 + 0x37c) as *mut u32;

        fn read_mmu_entry(i: u32) -> u32 {
            unsafe {
                SPI_MEM_MMU_ITEM_INDEX.write_volatile(i);
                SPI_MEM_MMU_ITEM_CONTENT.read_volatile()
            }
        }

        fn write_mmu_entry(i: u32, entry: u32) {
            unsafe {
                SPI_MEM_MMU_ITEM_INDEX.write_volatile(i);
                SPI_MEM_MMU_ITEM_CONTENT.write_volatile(entry);
            }
        }

        // calculate the PSRAM start address to map
        // the linker scripts can produce a gap between mapped IROM and DROM segments
        // bigger than a flash page - i.e. we will see an unmapped memory slot
        // start from the end and find the last mapped flash page
        let mut mapped_pages = 0;

        // the bootloader is using the last page to access flash internally
        // (e.g. to read the app descriptor) so we just skip that
        for i in (0..(FLASH_MMU_TABLE_SIZE - 1)).rev() {
            if (read_mmu_entry(i) & MMU_VALID) != 0 {
                mapped_pages = i + 1;
                break;
            }
        }
        let start = EXTMEM_ORIGIN + (MMU_PAGE_SIZE * mapped_pages);
        debug!("PSRAM start address = {:x}", start);

        for i in 0..config.size.get() as u32 / MMU_PAGE_SIZE {
            write_mmu_entry(i + mapped_pages, MMU_VALID + MMU_ACCESS_SPIRAM + i)
        }

        // enable busses
        const CACHE_L1_CACHE_CTRL_REG: *mut u32 = (0x600C8000 + 4) as *mut u32;
        const CACHE_L1_CACHE_SHUT_BUS0: u32 = 1 << 0;
        const CACHE_L1_CACHE_SHUT_BUS1: u32 = 1 << 1;
        CACHE_L1_CACHE_CTRL_REG.write_volatile(
            CACHE_L1_CACHE_CTRL_REG.read_volatile()
                & !(CACHE_L1_CACHE_SHUT_BUS0 | CACHE_L1_CACHE_SHUT_BUS1),
        );

        start
    };

    unsafe {
        super::MAPPED_PSRAM.memory_range = start as usize..start as usize + config.size.get();
    }
}

fn core_clock_for_mode(_speed_mode: MspiTimingSpeedMode) -> u32 {
    // in all cases (low and high speed) we use an MSPI clock of 80 MHz on this target
    80
}

mod ctrlr_ll {
    use super::*;

    pub(crate) const PSRAM_CTRLR_LL_MSPI_ID_0: u32 = 0;

    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_cs_hold(_mspi_id: u32, hold_n: u32) {
        assert!(hold_n > 0);

        SPI0::regs().smem_ac().modify(|_, w| {
            w.smem_cs_hold().set_bit();
            unsafe {
                w.smem_cs_hold_time().bits(hold_n as u8 - 1);
            }
            w
        });
    }

    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_cs_setup(_mspi_id: u32, setup_n: u32) {
        assert!(setup_n > 0);

        SPI0::regs().smem_ac().modify(|_, w| {
            w.smem_cs_setup().set_bit();
            unsafe {
                w.smem_cs_setup_time().bits(setup_n as u8 - 1);
            }
            w
        });
    }

    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_read_mode(mspi_id: u32, read_mode: CommandMode) {
        assert!(mspi_id == 0);

        SPI0::regs().cache_sctrl().modify(|_, w| {
            w.usr_sram_dio().bit(read_mode == CommandMode::PsramCmdSpi);
            w.usr_sram_qio().bit(read_mode == CommandMode::PsramCmdQpi);
            w
        });
    }

    /// Set PSRAM write cmd
    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_wr_cmd(_mspi_id: u32, cmd_bitlen: u32, cmd_val: u32) {
        assert!(cmd_bitlen > 0);
        SPI0::regs()
            .cache_sctrl()
            .modify(|_, w| w.cache_sram_usr_wcmd().set_bit());

        SPI0::regs().sram_dwr_cmd().modify(|_, w| {
            unsafe { w.cache_sram_usr_wr_cmd_bitlen().bits(cmd_bitlen as u8 - 1) };
            unsafe { w.cache_sram_usr_wr_cmd_value().bits(cmd_val as u16) };
            w
        });
    }

    /// Set PSRAM read cmd
    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_rd_cmd(_mspi_id: u32, cmd_bitlen: u32, cmd_val: u32) {
        assert!(cmd_bitlen > 0);

        SPI0::regs()
            .cache_sctrl()
            .modify(|_, w| w.cache_sram_usr_rcmd().set_bit());

        SPI0::regs().sram_drd_cmd().modify(|_, w| {
            unsafe { w.cache_sram_usr_rd_cmd_bitlen().bits(cmd_bitlen as u8 - 1) };
            unsafe { w.cache_sram_usr_rd_cmd_value().bits(cmd_val as u16) };
            w
        });
    }

    /// Set PSRAM addr bitlen
    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_addr_bitlen(_mspi_id: u32, addr_bitlen: u32) {
        assert!(addr_bitlen > 0);

        SPI0::regs()
            .cache_sctrl()
            .modify(|_, w| unsafe { w.sram_addr_bitlen().bits(addr_bitlen as u8 - 1) });
    }

    /// Set PSRAM read dummy
    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_rd_dummy(_mspi_id: u32, dummy_n: u32) {
        assert!(dummy_n > 0);

        SPI0::regs().cache_sctrl().modify(|_, w| {
            w.usr_rd_sram_dummy().set_bit();
            unsafe { w.sram_rdummy_cyclelen().bits(dummy_n as u8 - 1) };
            w
        });
    }

    /// Select which pin to use for the psram
    #[inline(always)]
    pub(crate) fn psram_ctrlr_ll_set_cs_pin(_mspi_id: u32, cs_id: PsramLlCsIdT) {
        SPI1::regs().misc().modify(|_, w| {
            w.cs0_dis().bit(cs_id == PsramLlCsIdT::PsramLlCsId0);
            w.cs1_dis().bit(cs_id == PsramLlCsIdT::PsramLlCsId1);
            w
        });
    }
}

#[ram]
fn mspi_timing_config_set_flash_clock(flash_freq_mhz: u32, speed_mode: quad::MspiTimingSpeedMode) {
    let core_clock_mhz = core_clock_for_mode(speed_mode);
    // SPI0 and SPI1 share the register for core clock. So we only set SPI0 here.
    mspi_timing_ll_set_core_clock(core_clock_mhz);

    let freqdiv: u32 = core_clock_mhz / flash_freq_mhz;
    debug!("flash freqdiv: {}", freqdiv);
    assert!(freqdiv > 0);
    let reg_val: u32 = mspi_timing_ll_calculate_clock_reg(freqdiv);
    mspi_timing_ll_set_flash_clock(0, reg_val);
    mspi_timing_ll_set_flash_clock(1, reg_val);
}

/// Set MSPI_FAST_CLK's high-speed divider (valid when SOC_ROOT clock source is PLL)
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_core_clock(core_clk_mhz: u32) {
    let divider = match core_clk_mhz {
        80 => 6,
        _ => panic!("Invalid core clock"),
    };

    crate::peripherals::PCR::regs()
        .mspi_clk_conf()
        .modify(|_, w| unsafe { w.mspi_fast_div_num().bits(divider - 1) });
}

/// Calculate spi_flash clock frequency division parameters for register.
#[inline(always)]
fn mspi_timing_ll_calculate_clock_reg(clkdiv: u32) -> u32 {
    if clkdiv == 1 {
        1 << 31
    } else {
        (clkdiv - 1) | ((((clkdiv - 1) / 2) & 0xff) << 8) | (((clkdiv - 1) & 0xff) << 16)
    }
}

/// Set Flash clock
#[inline(always)]
fn mspi_timing_ll_set_flash_clock(mspi_id: u32, clock_conf: u32) {
    if mspi_id == 0 {
        SPI0::regs()
            .clock()
            .write(|w| unsafe { w.bits(clock_conf) });
    } else {
        SPI1::regs()
            .clock()
            .write(|w| unsafe { w.bits(clock_conf) });
    }
}

#[ram]
fn mspi_timing_config_set_psram_clock(psram_freq_mhz: u32, speed_mode: quad::MspiTimingSpeedMode) {
    // bool control_both_mspi = true
    let core_clock_mhz = core_clock_for_mode(speed_mode);

    // SPI0 and SPI1 share the register for core clock. So we only set SPI0 here.
    mspi_timing_ll_set_core_clock(core_clock_mhz);

    let freqdiv: u32 = core_clock_mhz / psram_freq_mhz;
    debug!("psram freqdiv: {}", freqdiv);
    assert!(freqdiv > 0);
    let reg_val: u32 = mspi_timing_ll_calculate_clock_reg(freqdiv);
    mspi_timing_ll_set_psram_clock(reg_val);
}

/// Set PSRAM clock
#[inline(always)]
fn mspi_timing_ll_set_psram_clock(clock_conf: u32) {
    SPI0::regs()
        .sram_clk()
        .write(|w| unsafe { w.bits(clock_conf) });
}

/// Set MSPI Flash din mode
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_flash_din_mode(mspi_id: u8, din_mode: u8) {
    assert!(mspi_id == 0);
    assert!(din_mode <= 7);

    SPI0::regs().din_mode().modify(|_, w| {
        unsafe {
            w.din0_mode().bits(din_mode);
            w.din1_mode().bits(din_mode);
            w.din2_mode().bits(din_mode);
            w.din3_mode().bits(din_mode);
            w.din4_mode().bits(din_mode);
            w.din5_mode().bits(din_mode);
            w.din6_mode().bits(din_mode);
            w.din7_mode().bits(din_mode);
            w.dins_mode().bits(din_mode);
        }

        w
    });

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Set MSPI Flash din num
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_flash_din_num(mspi_id: u8, din_num: u8) {
    assert!(mspi_id == 0);
    assert!(din_num <= 3);

    SPI0::regs().din_num().modify(|_, w| {
        unsafe {
            w.din0_num().bits(din_num);
            w.din1_num().bits(din_num);
            w.din2_num().bits(din_num);
            w.din3_num().bits(din_num);
            w.din4_num().bits(din_num);
            w.din5_num().bits(din_num);
            w.din6_num().bits(din_num);
            w.din7_num().bits(din_num);
            w.dins_num().bits(din_num);
        }

        w
    });

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Set MSPI Flash extra dummy
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_flash_extra_dummy(mspi_id: u8, extra_dummy: u8) {
    if mspi_id == 0 {
        SPI0::regs().timing_cali().modify(|_, w| {
            w.timing_cali().bit(extra_dummy > 0);
            unsafe { w.extra_dummy_cyclelen().bits(extra_dummy) };
            w
        });
    } else {
        SPI1::regs().timing_cali().modify(|_, w| {
            w.timing_cali().bit(extra_dummy > 0);
            unsafe { w.extra_dummy_cyclelen().bits(extra_dummy) };
            w
        });
    }

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Set MSPI PSRAM din mode
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_psram_din_mode(mspi_id: u8, din_mode: u8) {
    assert!(mspi_id == 0);
    assert!(din_mode <= 7);

    SPI0::regs().smem_din_mode().modify(|_, w| {
        unsafe {
            w.smem_din0_mode().bits(din_mode);
            w.smem_din1_mode().bits(din_mode);
            w.smem_din2_mode().bits(din_mode);
            w.smem_din3_mode().bits(din_mode);
            w.smem_din4_mode().bits(din_mode);
            w.smem_din5_mode().bits(din_mode);
            w.smem_din6_mode().bits(din_mode);
            w.smem_din7_mode().bits(din_mode);
            w.smem_dins_mode().bits(din_mode);
        }

        w
    });

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Set MSPI PSRAM din num
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_psram_din_num(mspi_id: u8, din_num: u8) {
    assert!(mspi_id == 0);
    assert!(din_num <= 3);

    SPI0::regs().smem_din_num().modify(|_, w| {
        unsafe {
            w.smem_din0_num().bits(din_num);
            w.smem_din1_num().bits(din_num);
            w.smem_din2_num().bits(din_num);
            w.smem_din3_num().bits(din_num);
            w.smem_din4_num().bits(din_num);
            w.smem_din5_num().bits(din_num);
            w.smem_din6_num().bits(din_num);
            w.smem_din7_num().bits(din_num);
            w.smem_dins_num().bits(din_num);
        }

        w
    });

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Set MSPI PSRAM extra dummy
#[inline(always)]
pub(crate) fn mspi_timing_ll_set_psram_extra_dummy(mspi_id: u8, extra_dummy: u8) {
    assert!(mspi_id == 0);

    SPI0::regs().smem_timing_cali().modify(|_, w| {
        w.smem_timing_cali().bit(extra_dummy > 0);
        unsafe { w.smem_extra_dummy_cyclelen().bits(extra_dummy) };
        w
    });

    SPI0::regs()
        .timing_cali()
        .modify(|_, w| w.update().set_bit());
}

/// Get MSPI flash dummy info
#[inline(always)]
pub(crate) fn mspi_timing_ll_get_flash_dummy(mspi_id: u8) -> (u8, u8) {
    assert!(mspi_id <= 1);

    if mspi_id == 0 {
        let usr_dummy = SPI0::regs().user1().read().usr_dummy_cyclelen().bits();
        let extra_dummy = SPI0::regs()
            .timing_cali()
            .read()
            .extra_dummy_cyclelen()
            .bits();
        (usr_dummy, extra_dummy)
    } else {
        let usr_dummy = SPI1::regs().user1().read().usr_dummy_cyclelen().bits();
        let extra_dummy = SPI1::regs()
            .timing_cali()
            .read()
            .extra_dummy_cyclelen()
            .bits();
        (usr_dummy, extra_dummy)
    }
}

/// Get MSPI PSRAM dummy info
#[inline(always)]
pub(crate) fn mspi_timing_ll_get_psram_dummy(mspi_id: u8) -> (u8, u8) {
    assert!(mspi_id == 0);

    let usr_dummy = SPI0::regs()
        .cache_sctrl()
        .read()
        .sram_rdummy_cyclelen()
        .bits();
    let extra_dummy = SPI0::regs()
        .smem_timing_cali()
        .read()
        .smem_extra_dummy_cyclelen()
        .bits();
    (usr_dummy, extra_dummy)
}
