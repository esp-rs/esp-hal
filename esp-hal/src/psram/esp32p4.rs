//! PSRAM driver for ESP32-P4X (chip revision v3.x / ECO5+).
//!
//! P4 has two PSRAM controllers wired to the same DQ pads
//! The structure is looks same with DDR controller but not same.

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::{
    peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST, PMU},
    soc::regi2c,
};

/// MMU page size (64 KB)
const MMU_PAGE_SIZE: usize = 0x10000;

/// PSRAM_MSPI0 base, AXI cache controller.
const MSPI0_BASE: u32 = 0x5008_E000;

/// PSRAM_MSPI1 base, direct-command controller.
const MSPI1_BASE: u32 = 0x5008_F000;

/// Volatile 32-bit read.
#[inline(always)]
unsafe fn mmio_read_32(addr: u32) -> u32 {
    unsafe { (addr as *const u32).read_volatile() }
}

/// Volatile 32-bit write (full overwrite).
#[inline(always)]
unsafe fn mmio_write_32(addr: u32, val: u32) {
    unsafe { (addr as *mut u32).write_volatile(val) }
}

/// Read-modify-write: set every bit of `mask`. Equivalent to
/// `*addr |= mask`.
#[inline(always)]
unsafe fn mmio_setbits_32(addr: u32, mask: u32) {
    unsafe { mmio_write_32(addr, mmio_read_32(addr) | mask) }
}

/// Read-modify-write: clear `clear`-bits then set `set`-bits.
/// Equivalent to `*addr = (*addr & !clear) | set`.
#[inline(always)]
unsafe fn mmio_clrsetbits_32(addr: u32, clear: u32, set: u32) {
    unsafe { mmio_write_32(addr, (mmio_read_32(addr) & !clear) | set) }
}

/// PSRAM interface mode (line count of the data bus).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramMode {
    /// 16-line DDR, AP HEX PSRAM with MR8.x16 = 1. Default.
    #[default]
    Hex,
    // TODO; selecting `Oct`
    // requires the cache-side controller config (`mem_sdin_hex` /
    // `mem_sdout_hex` bits) and chip MR8.x16 to flip together. Wire that
    // path before exposing.
    // Oct,
}

/// PSRAM bus frequency.
///
/// Choice ties together MPLL freq + bus divider + chip-side read/write latency.
///
/// | Variant   | MPLL | div | MR0.RL | MR4.WL | RD dummy bits | Use case               |
/// |-----------|------|-----|--------|--------|---------------|------------------------|
/// | `Mhz20`   | 400  | 20  | 2      | 2      | 18            | Low-power / debug      |
/// | `Mhz80`   | 320  | 4   | 2      | 2      | 18            | Conservative SI margin |
/// | `Mhz200`  | 400  | 2   | 4      | 1      | 26            | IDF default            |
/// | `Mhz250`  | 500  | 2   | 6      | 3      | 34            | Overclock              |
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SpiRamFreq {
    /// 20 MHz bus (MPLL 400 / div 20).
    Mhz20  = 20,

    /// 80 MHz bus (MPLL 320 / div 4).
    ///
    /// Conservative; widest timing margin.
    Mhz80  = 80,

    /// 200 MHz bus (MPLL 400 / div 2).
    ///
    /// IDF default for AP HEX PSRAM.
    #[default]
    Mhz200 = 200,

    /// 250 MHz bus (MPLL 500 / div 2).
    Mhz250 = 250,
}

/// MPLL target frequency override.
///
/// IDF only programs three discrete MPLL frequencies for the PSRAM
/// clock domain (`MSPI_TIMING_MPLL_FREQ_MHZ` in
/// `mspi_timing_tuning_configs.h`):
///
///   - `Mhz320` -> paired with `SpiRamFreq::Mhz80`
///   - `Mhz400` -> paired with `SpiRamFreq::Mhz20` / `Mhz200`
///   - `Mhz500` -> paired with `SpiRamFreq::Mhz250` (silicon v3+)
///
/// The MPLL itself can in principle be set to any value that satisfies
/// the formula `XTAL(40) * (div+1) / (ref_div+1)`, but only these three
/// have been silicon-validated.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum MpllFreq {
    /// 320 MHz. IDF pairing: `SpiRamFreq::Mhz80`.
    Mhz320 = 320,
    /// 400 MHz. IDF pairing: `SpiRamFreq::Mhz20` or `Mhz200`. Default
    /// when `core_clock = None` and `ram_frequency = Mhz200`.
    Mhz400 = 400,
    /// 500 MHz. IDF pairing: `SpiRamFreq::Mhz250` (silicon v3.0+).
    Mhz500 = 500,
}

/// PSRAM configuration.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// PSRAM interface mode (Hex 16-line vs Oct 8-line). Default: Hex.
    pub mode: PsramMode,
    /// Size of PSRAM to map. Default: `AutoDetect` via MR2 density.
    pub size: PsramSize,
    /// MPLL override. `None` (default) derives the MPLL frequency
    /// from `ram_frequency` per the table on `SpiRamFreq`.
    pub core_clock: Option<MpllFreq>,
    /// PSRAM bus frequency. Default: 200 MHz.
    pub ram_frequency: SpiRamFreq,
    // TODO: ECC enable.
    // The MSPI0 controller has a ECC engine.
    // pub ecc: bool, // or any other enum.
}

/// Initialize PSRAM.
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    init_psram_inner(config);
    true
}

pub(crate) fn map_psram(config: PsramConfig) -> core::ops::Range<usize> {
    let start = EXTMEM_ORIGIN;
    start..start + config.size.get()
}

/// Per-speed timing parameters. Mirrors IDF's `#if CONFIG_SPIRAM_SPEED_*`
/// branches in `esp_psram_impl_ap_hex.c` + `mspi_timing_tuning_configs.h`.
#[derive(Copy, Clone)]
struct SpeedParams {
    /// MPLL freq the analog block must run at.
    mpll_mhz: u32,
    /// Bus clock divider applied to MPLL.
    bus_div: u32,
    /// MR0.read_latency field value (cycles = 2 * value + 6).
    mr0_rl: u8,
    /// MR4.wr_latency field value.
    mr4_wl: u8,
    /// Read dummy length in bits for sync data reads (cache path).
    rd_dummy_bits: u32,
    /// Write dummy length in bits for sync data writes (cache path).
    wr_dummy_bits: u32,
    /// Register-read dummy length for direct command path (MSPI3).
    reg_dummy_bits: u32,
}

impl SpeedParams {
    const fn for_freq(f: SpiRamFreq) -> Self {
        match f {
            SpiRamFreq::Mhz20 => Self {
                mpll_mhz: 400,
                bus_div: 20,
                mr0_rl: 2,
                mr4_wl: 2,
                rd_dummy_bits: 18, // 2*(10-1)
                wr_dummy_bits: 8,  // 2*(5-1)
                reg_dummy_bits: 8, // 2*(5-1)
            },
            SpiRamFreq::Mhz80 => Self {
                mpll_mhz: 320,
                bus_div: 4,
                mr0_rl: 2,
                mr4_wl: 2,
                rd_dummy_bits: 18,
                wr_dummy_bits: 8,
                reg_dummy_bits: 8,
            },
            SpiRamFreq::Mhz200 => Self {
                mpll_mhz: 400,
                bus_div: 2,
                mr0_rl: 4,
                mr4_wl: 1,
                rd_dummy_bits: 26, // 2*(14-1)
                wr_dummy_bits: 12, // 2*(7-1)
                reg_dummy_bits: 12,
            },
            SpiRamFreq::Mhz250 => Self {
                mpll_mhz: 500,
                bus_div: 2,
                mr0_rl: 6,
                mr4_wl: 3,
                rd_dummy_bits: 34, // 2*(18-1)
                wr_dummy_bits: 16, // 2*(9-1)
                reg_dummy_bits: 16,
            },
        }
    }
}

/// CS timing constants (matches IDF AP_HEX_PSRAM_CS_*). Independent of
/// `SpiRamFreq` -- IDF uses the same values across all speed branches.
const AP_HEX_CS_SETUP_TIME: u32 = 4;
const AP_HEX_CS_HOLD_TIME: u32 = 4;
const AP_HEX_CS_HOLD_DELAY: u32 = 3;

fn init_psram_inner(config: &mut PsramConfig) {
    // Resolve the speed parameter set from `ram_frequency`. The MPLL
    // override (`config.core_clock`) lets the caller bypass the default
    // pairing (e.g. force 400 MHz MPLL with `Mhz20` bus); `None`
    // selects the table-default for the chosen `ram_frequency`.
    let mut params = SpeedParams::for_freq(config.ram_frequency);
    if let Some(mpll) = config.core_clock {
        params.mpll_mhz = mpll as u32;
    }

    psram_phy_ldo_init(); // PMU EXT_LDO regulator setup for the MSPI PHY analog block.
    configure_mpll(params.mpll_mhz);

    // Module clock + clock source
    HP_SYS_CLKRST::regs()
        .soc_clk_ctrl0()
        .modify(|_, w| w.psram_sys_clk_en().set_bit());
    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl00()
        .modify(|_, w| unsafe {
            w.psram_pll_clk_en().set_bit();
            w.psram_core_clk_en().set_bit();
            w.psram_clk_src_sel().bits(1) // 1 = MPLL
        });

    // Controller + PHY pad bring-up.
    set_bus_clock(params.bus_div);
    enable_dll();
    psram_pad_init(); // required for DDR strobe latch
    set_cs_timing();

    // SoC MR init (via MSPI3 SPI direct)
    init_mr_registers(params);

    if config.size.is_auto() {
        config.size = PsramSize::Size(psram_detect_size(params));
    }

    configure_psram_mspi(params, MSPI0_BASE); // basic AXI configuration here
    mmu_map_psram(MSPI0_BASE, config); // MMU mapping here
}

/// Set bus-clock divider for both PSRAM_MSPI0 (SRAM_CLK at 0x50) and
/// PSRAM_MSPI1 (CLOCK at 0x14). For divider=2 the value is
/// (N=1)<<16 | (H=0)<<8 | (L=1)<<0 = 0x00010001. For divider=1 the
/// fast path bit `CLK_EQU_SYSCLK` (bit 31) is set instead.
fn set_bus_clock(div: u32) {
    const MSPI0_SRAM_CLK: u32 = MSPI0_BASE + 0x50;
    const MSPI1_CLOCK: u32 = MSPI1_BASE + 0x14;

    let val = if div == 1 {
        1u32 << 31
    } else {
        ((div - 1) << 16) | ((div / 2 - 1) << 8) | (div - 1)
    };
    unsafe {
        mmio_write_32(MSPI0_SRAM_CLK, val);
        mmio_write_32(MSPI1_CLOCK, val);
    }
}

/// Enable DLL timing calibration for both controllers. Both DLL bits
/// live in MSPI0's register space (per IDF `psram_ctrlr_ll_enable_dll`):
fn enable_dll() {
    /// MSPI3 DLL (0x5008_E180), bit 5
    const MEM_TIMING_CALI: u32 = MSPI0_BASE + 0x180;
    /// MSPI2 DLL (0x5008_E190), bit 5
    const SMEM_TIMING_CALI: u32 = MSPI0_BASE + 0x190;
    const DLL_BIT: u32 = 1 << 5;
    unsafe {
        mmio_setbits_32(MEM_TIMING_CALI, DLL_BIT);
        mmio_setbits_32(SMEM_TIMING_CALI, DLL_BIT);
    }
}

/// IOMUX_MSPI_PIN base. Each PSRAM pad has a `..._PIN0_REG` at this base
/// plus the pad-specific offset given by `PsramPad`.
const IOMUX_MSPI_BASE: u32 = 0x500E_1200;

/// PSRAM IOMUX pads. Discriminant = byte offset from `IOMUX_MSPI_BASE`
/// to that pad's `IOMUX_MSPI_PIN_PSRAM_<name>_PIN0_REG`. Per-pad DRV
/// field is bits [13:12]; DQS0 and DQS1 additionally have an XPD enable
/// at bit 0.
#[repr(u32)]
#[derive(Copy, Clone)]
enum PsramPad {
    /// DQ0. Matches IDF `IOMUX_MSPI_PIN_PSRAM_D_PIN0_REG` (legacy SPI
    /// MOSI naming retained on the IOMUX register itself).
    Dq0  = 0x1C,
    /// DQ1. IDF `IOMUX_MSPI_PIN_PSRAM_Q_PIN0_REG` (legacy SPI MISO).
    Dq1  = 0x20,
    /// DQ2. IDF `IOMUX_MSPI_PIN_PSRAM_WP_PIN0_REG` (legacy WP).
    Dq2  = 0x24,
    /// DQ3. IDF `IOMUX_MSPI_PIN_PSRAM_HOLD_PIN0_REG` (legacy HOLD).
    /// In OPI/HEX DDR mode the controller does not treat this as a
    /// hold/freeze input -- it is a plain data line.
    Dq3  = 0x28,
    Dq4  = 0x2C,
    Dq5  = 0x30,
    Dq6  = 0x34,
    Dq7  = 0x38,
    Dqs0 = 0x3C,
    Dq8  = 0x40,
    Dq9  = 0x44,
    Dq10 = 0x48,
    Dq11 = 0x4C,
    Dq12 = 0x50,
    Dq13 = 0x54,
    Dq14 = 0x58,
    Dq15 = 0x5C,
    Ck   = 0x60,
    Cs   = 0x64,
    Dqs1 = 0x68,
}

impl PsramPad {
    /// Iteration order matches IDF `mspi_timing_ll_pin_drv_set` (all
    /// 20 DDR-ish pads in numeric offset order). Add a new variant above
    /// and append it here; `set_drv_all` will then cover it
    /// automatically.
    const ALL: [Self; 20] = [
        // DQ0 ~ DQ7
        Self::Dq0,
        Self::Dq1,
        Self::Dq2,
        Self::Dq3,
        Self::Dq4,
        Self::Dq5,
        Self::Dq6,
        Self::Dq7,
        // Strobe0
        Self::Dqs0,
        // DQ8 ~ DQ15
        Self::Dq8,
        Self::Dq9,
        Self::Dq10,
        Self::Dq11,
        Self::Dq12,
        Self::Dq13,
        Self::Dq14,
        Self::Dq15,
        // Ck and Cs
        Self::Ck,
        Self::Cs,
        // Strobe1
        Self::Dqs1,
    ];

    /// Absolute MMIO address of this pad's `~_PIN0_REG`.
    fn reg_addr(self) -> u32 {
        IOMUX_MSPI_BASE + self as u32
    }

    /// Set this pad's drive-strength field (bits [13:12]) to `drv`,
    /// preserving the other bits.
    unsafe fn set_drv(self, drv: u32) {
        unsafe { mmio_clrsetbits_32(self.reg_addr(), 0x3 << 12, (drv & 0x3) << 12) };
    }

    /// Set drive strength on every pad in `Self::ALL`.
    unsafe fn set_drv_all(drv: u32) {
        for pad in Self::ALL {
            unsafe { pad.set_drv(drv) };
        }
    }

    /// Set bit 0 (XPD power-up enable) of this pad's PIN0 register.
    /// Used on strobe to power on the strobe input buffer needed.
    unsafe fn enable_xpd(self) {
        unsafe { mmio_setbits_32(self.reg_addr(), 1) };
    }
}

/// Configure PSRAM PHY pads.
///
/// Mirrors IDF `mspi_timing_ll_pin_drv_set(2)` +
/// `mspi_timing_ll_enable_dqs(true)`. Both DQS XPD bits must be on for
/// DDR reads to latch data; drive strength = 2 matches IDF default for
/// AP HEX PSRAM at 200 MHz.
fn psram_pad_init() {
    unsafe {
        PsramPad::set_drv_all(2);
        PsramPad::Dqs0.enable_xpd();
        PsramPad::Dqs1.enable_xpd();
    }
}

/// Set PSRAM CS timing on the AXI controller's SMEM_AC register
/// (0x5008_E1A0). SMEM_CS_SETUP=1, SMEM_CS_HOLD=1, setup_time=N-1,
/// hold_time=N-1, hold_delay=N-1, split_trans_en=1.
fn set_cs_timing() {
    const SMEM_AC: u32 = MSPI0_BASE + 0x1A0;
    unsafe {
        let mut val = mmio_read_32(SMEM_AC);
        //   bit 0  SMEM_CS_SETUP
        //   bit 1  SMEM_CS_HOLD
        //   bit 31 SPLIT_TRANS_EN
        val |= (1u32 << 31) | 0b11;
        //   bits 6..2   SETUP_TIME (5-bit, N-1)
        //   bits 11..7  HOLD_TIME  (5-bit, N-1)
        //   bits 30..25 HOLD_DELAY (6-bit, N-1)
        val = (val & !(0x1F << 2)) | ((AP_HEX_CS_SETUP_TIME - 1) << 2);
        val = (val & !(0x1F << 7)) | ((AP_HEX_CS_HOLD_TIME - 1) << 7);
        val = (val & !(0x3F << 25)) | ((AP_HEX_CS_HOLD_DELAY - 1) << 25);
        mmio_write_32(SMEM_AC, val);
    }
}

// MR (a.k.a Mode Register with single byte inside the AP HEX PSRAM chip.

const MR_ADDR_MR0_MR1: u32 = 0x0;
const MR_ADDR_MR2_MR3: u32 = 0x2;
const MR_ADDR_MR4_MR5: u32 = 0x4;

/// MR6 only used in current state, MR7 is unused or reserved.
#[allow(dead_code)] // half-sleep trigger; kept for future power-down support
const MR_ADDR_MR6_MR7: u32 = 0x6;
/// MR8 only used in current state, MR9 is unused or reserved.
const MR_ADDR_MR8_MR9: u32 = 0x8;

/// Read an 8-bit mode-register pair from the AP HEX PSRAM chip.
///
/// Returns `(low, high)` where:
///   - `low`  = the MR at `mr_addr`
///   - `high` = the MR at `mr_addr + 1`
///
/// For pair addresses where the high byte is a reserved slot
/// (MR_ADDR_MR4_MR5 / MR_ADDR_MR6 / MR_ADDR_MR8_MR9 — see the table at the
/// `MR_ADDR_*` constants), the `high` value is don't-care and the
/// caller should ignore it.
fn psram_mr_read(speed_params: SpeedParams, mr_addr: u32) -> (u8, u8) {
    let pair = mspi1_reg_read16(speed_params, mr_addr);
    ((pair & 0xFF) as u8, ((pair >> 8) & 0xFF) as u8)
}

/// Write an 8-bit mode-register pair to the AP HEX PSRAM chip.
///
/// `low` goes to the MR at `mr_addr`, `high` goes to the MR at
/// `mr_addr + 1`. For pair addresses whose high slot is reserved, pass
/// the value previously read back (read-modify-write) to keep the chip
/// state consistent.
fn psram_mr_write(mr_addr: u32, low: u8, high: u8) {
    let pair = (low as u16) | ((high as u16) << 8);
    mspi1_reg_write16(mr_addr, pair)
}

/// Initialize AP HEX PSRAM mode registers via PSRAM_MSPI1 OPI DTR
/// referenced IDF `hex_psram_mode_reg_t`), missing on TRM
/// MR0: drive_str[1:0], read_latency[4:2], lt[5]
/// MR4: wr_latency[7:5]
/// MR8: bl[1:0], bt[2], rbx[3], x16[6]
fn init_mr_registers(speed_params: SpeedParams) {
    // Read+modify+write MR0 (preserve MR1 high byte).
    let (mut mr0, mr1) = psram_mr_read(speed_params, MR_ADDR_MR0_MR1);
    mr0 &= !(0x3 | (0x7 << 2) | (1 << 5));
    mr0 |= ((speed_params.mr0_rl & 0x7) << 2) | (1 << 5);
    psram_mr_write(MR_ADDR_MR0_MR1, mr0, mr1);

    // Read+modify+write MR4 (preserve MR5 reserved high byte).
    let (mut mr4, mr5) = psram_mr_read(speed_params, MR_ADDR_MR4_MR5);
    mr4 &= !(0x7 << 5);
    mr4 |= (speed_params.mr4_wl & 0x7) << 5;
    psram_mr_write(MR_ADDR_MR4_MR5, mr4, mr5);

    // do nothing for MR6 and MR7

    // Read+modify+write MR8 (high byte is reserved/absent — pass 0).
    let (mut mr8, mr9) = psram_mr_read(speed_params, MR_ADDR_MR8_MR9); // MR9 is unused
    mr8 &= !(0x3 | (1 << 2) | (1 << 3) | (1 << 6));
    mr8 |= 3 // bt = 0
        | (1 << 3) // rbx = 1
        | (1 << 6); // x16 = 1
    psram_mr_write(MR_ADDR_MR8_MR9, mr8, mr9); // keep previous MR9
}

/// Mirror of IDF's `esp_rom_spi_cmd_t` (rom/opi_flash.h). Layout must match
/// the C struct exactly; the ROM driver reads it via the pointer.
#[repr(C)]
struct EspRomSpiCmd {
    cmd: u16,
    cmd_bit_len: u16,
    addr: *mut u32,
    addr_bit_len: u32,
    tx_data: *mut u32,
    tx_data_bit_len: u32,
    rx_data: *mut u32,
    rx_data_bit_len: u32,
    dummy_bit_len: u32,
}

/// `esp_rom_spiflash_read_mode_t`. Only OPI_DTR is used here.
const ESP_ROM_SPIFLASH_OPI_DTR_MODE: u32 = 7;
/// MSPI controller index used for direct (non-cache) PSRAM commands.
/// Maps to `PSRAM_CTRLR_LL_MSPI_ID_3` in IDF (`PSRAM_MSPI1` = 0x5008_F000).
const ROM_SPI_PSRAM_CMD_NUM: i32 = 3;
/// CS mask: PSRAM lives on CS1 (bit 1). Flash on CS0.
#[expect(dead_code)]
const ROM_SPI_PSRAM_CS_MASK: u8 = 1 << 1;

unsafe extern "C" {
    /// Set the controller's read mode (e.g. OPI-DTR). Configures cmd/addr/
    /// data line counts (8-line for OPI) and DDR mode bits in one call.
    /// Linked from `esp32p4.rom.ld`: `esp_rom_spi_set_op_mode = 0x4fc00110`.
    fn esp_rom_spi_set_op_mode(spi_num: i32, mode: u32);
    /// Configure command/addr/dummy/data phases for next transaction.
    /// Writes USR / USER1 / USER2 / ADDR / MOSI_DLEN / MISO_DLEN / W0..
    /// Linked from `esp32p4.rom.ld`: `esp_rom_spi_cmd_config = 0x4fc00108`.
    fn esp_rom_spi_cmd_config(spi_num: i32, pcmd: *mut EspRomSpiCmd);

    /// `Cache_Invalidate_All = 0x4fc00404`. Invalidates all cache levels.
    fn Cache_Invalidate_All();
}

unsafe fn rom_cache_invalidate_all() {
    unsafe { Cache_Invalidate_All() }
}

/// Kick the controller (set `SPI_USR` bit 18 in CMD_REG) and poll
/// bounded for completion. Replacement for ROM `esp_rom_spi_cmd_start`,
/// which polls forever; a hang there gives no diagnostic, while the
/// bounded variant surfaces a real failure as a returned error. After
/// the bit clears, copies MISO bytes from W0..W{N} into `rx_buf`.
fn mspi1_kick_and_collect(rx: &mut [u8]) -> Result<(), ()> {
    const SPI_CMD: u32 = MSPI1_BASE;
    const SPI_W0: u32 = MSPI1_BASE + 0x58;
    const SPI_USR_TRIGGER: u32 = 1 << 18;
    const MAX_ITERS: u32 = 1_000_000;

    // Select CS1 (PSRAM); leave CS0 (flash) disabled.
    //   bit 0  CS0_DIS = 1  (disable flash CS)
    //   bit 1  CS1_DIS = 0  (enable PSRAM CS)
    const MISC: u32 = MSPI1_BASE + 0x34;
    unsafe { mmio_clrsetbits_32(MISC, 0b10, 0b01) };

    // Kick.
    unsafe { mmio_write_32(SPI_CMD, SPI_USR_TRIGGER) };
    let mut t = MAX_ITERS;
    while unsafe { mmio_read_32(SPI_CMD) } & SPI_USR_TRIGGER != 0 {
        t -= 1;
        if t == 0 {
            return Err(());
        }
        core::hint::spin_loop();
    }

    // Copy MISO bytes from W0..W{N}.
    if !rx.is_empty() {
        let n_bytes = rx.len();
        let n_words = n_bytes.div_ceil(4);
        for i in 0..n_words {
            let word = unsafe { mmio_read_32(SPI_W0 + (i as u32) * 4) };
            for b in 0..4 {
                let off = i * 4 + b;
                if off >= n_bytes {
                    break;
                }
                rx[off] = ((word >> (b * 8)) & 0xFF) as u8;
            }
        }
    }
    Ok(())
}

/// Issue an AP HEX PSRAM register-read command (0x4040, 16-bit cmd,
/// 32-bit address, 16-bit MISO) through `PSRAM_MSPI1`. Setup via ROM
/// helpers (`set_op_mode` + `cmd_config`); kick + poll done manually
/// with a timeout (avoids ROM `cmd_start` poll-forever pitfall).
fn mspi1_reg_read16(speed_params: SpeedParams, addr: u32) -> u16 {
    const REG_READ_CMD: u16 = 0x4040;
    const CMD_BITLEN: u16 = 16;
    const ADDR_BITLEN: u32 = 32;
    const DATA_BITLEN: u32 = 16;

    let mut addr_local = addr;
    let mut rx: [u8; 2] = [0; 2];
    let mut conf = EspRomSpiCmd {
        cmd: REG_READ_CMD,
        cmd_bit_len: CMD_BITLEN,
        addr: &mut addr_local as *mut u32,
        addr_bit_len: ADDR_BITLEN,
        tx_data: core::ptr::null_mut(),
        tx_data_bit_len: 0,
        rx_data: rx.as_mut_ptr() as *mut u32,
        rx_data_bit_len: DATA_BITLEN,
        dummy_bit_len: speed_params.reg_dummy_bits,
    };

    unsafe {
        esp_rom_spi_set_op_mode(ROM_SPI_PSRAM_CMD_NUM, ESP_ROM_SPIFLASH_OPI_DTR_MODE);
        esp_rom_spi_cmd_config(ROM_SPI_PSRAM_CMD_NUM, &raw mut conf);
    }
    let _ = mspi1_kick_and_collect(&mut rx);
    u16::from_le_bytes(rx)
}

/// Issue an AP HEX PSRAM register-write command (0xC0C0) with 16-bit
/// data through `PSRAM_MSPI1`. Same setup-via-ROM + manual-kick pattern
/// as `mspi1_reg_read16`.
fn mspi1_reg_write16(addr: u32, data: u16) {
    const REG_WRITE_CMD: u16 = 0xC0C0;
    const CMD_BITLEN: u16 = 16;
    const ADDR_BITLEN: u32 = 32;
    const DATA_BITLEN: u32 = 16;

    let mut addr_local = addr;
    let mut tx = data.to_le_bytes();
    let mut conf = EspRomSpiCmd {
        cmd: REG_WRITE_CMD,
        cmd_bit_len: CMD_BITLEN,
        addr: &mut addr_local as *mut u32,
        addr_bit_len: ADDR_BITLEN,
        tx_data: tx.as_mut_ptr() as *mut u32,
        tx_data_bit_len: DATA_BITLEN,
        rx_data: core::ptr::null_mut(),
        rx_data_bit_len: 0,
        dummy_bit_len: 0,
    };

    unsafe {
        esp_rom_spi_set_op_mode(ROM_SPI_PSRAM_CMD_NUM, ESP_ROM_SPIFLASH_OPI_DTR_MODE);
        esp_rom_spi_cmd_config(ROM_SPI_PSRAM_CMD_NUM, &mut conf as *mut EspRomSpiCmd);
    }
    let mut empty: [u8; 0] = [];
    let _ = mspi1_kick_and_collect(&mut empty);
}

/// Detect PSRAM size by reading mode register MR2 via MSPI1.
fn psram_detect_size(speed_params: SpeedParams) -> usize {
    let (mr2, _mr3) = psram_mr_read(speed_params, MR_ADDR_MR2_MR3);
    match mr2 & 0x7 {
        0x1 => 4 * 1024 * 1024,  //  32 Mbit
        0x3 => 8 * 1024 * 1024,  //  64 Mbit
        0x5 => 16 * 1024 * 1024, // 128 Mbit
        0x6 => 64 * 1024 * 1024, // 512 Mbit
        0x7 => 32 * 1024 * 1024, // 256 Mbit
        _ => 32 * 1024 * 1024,   // unknown -> EV Board default
    }
}

/// Map PSRAM physical pages into the virtual address space via MMU.
///
/// ESP32-P4 has TWO independent MMUs:
///   - Flash MMU (id 0): registers in `SPI_MEM_C` (FLASH_SPI0)
///   - PSRAM MMU (id 1): registers in `SPI_MEM_S` (PSRAM_MSPI0) at the `MMU_ITEM_INDEX_REG` /
///     `MMU_ITEM_CONTENT_REG` offsets we use here.
///
/// Each PSRAM MMU entry is a 32-bit word:
///   - bits [9:0] : physical page number (`SOC_MMU_PSRAM_VALID_VAL_MASK = 0x3FF`)
///   - bit  10    : `SOC_MMU_ACCESS_PSRAM` (selects PSRAM vs flash)
///   - bit  11    : `SOC_MMU_PSRAM_VALID`
///   - bit  12    : `SOC_MMU_PSRAM_SENSITIVE` (set only when cache encryption is on)
///
/// Virtual base: `SOC_MMU_PSRAM_VADDR_BASE = 0x4800_0000`. The PSRAM MMU
/// is a separate table from the flash MMU, so PSRAM entries start at
/// **entry index 0**, not at some offset relative to the flash range.
/// `entry_id = (vaddr - 0x4800_0000) / 0x10000`.
fn mmu_map_psram(mspi_base: u32, config: &PsramConfig) {
    const MMU_ITEM_CONTENT: u32 = 0x37C;
    const MMU_ITEM_INDEX: u32 = 0x380;
    const MMU_PSRAM_VALID: u32 = 1 << 11;
    const MMU_ACCESS_PSRAM: u32 = 1 << 10;
    const MMU_PADDR_MASK: u32 = 0x3FF; // 10 bits of physical page number

    let page_count = config.size.get() / MMU_PAGE_SIZE;

    // Note: we do NOT call Cache_Suspend_*. The ROM helpers expect
    // interrupts/scheduler state we can't guarantee in init context, and
    // hang. Since at init time nothing else is reading PSRAM via cache
    // yet, we can write the MMU entries unsynchronized and then
    // invalidate to drop any stale prefetched lines.
    unsafe {
        let index_reg = mspi_base + MMU_ITEM_INDEX;
        let content_reg = mspi_base + MMU_ITEM_CONTENT;
        for page in 0..page_count {
            let entry_id = page as u32; // PSRAM table starts at entry 0
            let content = MMU_PSRAM_VALID | MMU_ACCESS_PSRAM | (page as u32 & MMU_PADDR_MASK);
            mmio_write_32(index_reg, entry_id);
            mmio_write_32(content_reg, content);
        }
        rom_cache_invalidate_all();
    }
}

#[allow(dead_code)]
pub(crate) fn cache_invalidate(addr: u32, size: u32) {
    let cache = unsafe { crate::pac::CACHE::steal() };
    cache.sync_addr().write(|w| unsafe { w.bits(addr) });
    cache.sync_size().write(|w| unsafe { w.bits(size) });
    cache
        .sync_ctrl()
        .modify(|_, w| w.invalidate_ena().set_bit());
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {
        core::hint::spin_loop();
    }
}

#[allow(dead_code)]
pub(crate) fn cache_writeback(addr: u32, size: u32) {
    let cache = unsafe { crate::pac::CACHE::steal() };
    cache.sync_addr().write(|w| unsafe { w.bits(addr) });
    cache.sync_size().write(|w| unsafe { w.bits(size) });
    cache.sync_ctrl().modify(|_, w| w.writeback_ena().set_bit());
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {
        core::hint::spin_loop();
    }
}

/// Program the PMU external LDO regulators for the MSPI PHY
fn psram_phy_ldo_init() {
    PMU::regs()
        .ext_ldo_p0_0p1a()
        .write(|w| unsafe { w.bits(0x4020_0100) });
    PMU::regs()
        .ext_ldo_p0_0p1a_ana()
        .write(|w| unsafe { w.bits(0xB100_0000) });

    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p0_0p3a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p0_0p3a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p1_0p1a()
        .write(|w| unsafe { w.bits(0x4020_0180) });
    PMU::regs()
        .ext_ldo_p1_0p1a_ana()
        .write(|w| unsafe { w.bits(0x5700_0000) });

    PMU::regs()
        .ext_ldo_p1_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p1_0p2a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p1_0p3a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p1_0p3a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    // Allow LDO output to settle before the MSPI PHY is exercised.
    crate::rom::ets_delay_us(50);
}

/// Configure MPLL to target frequency for PSRAM clock.
fn configure_mpll(freq_mhz: u32) {
    // Power up the MPLL analog block.
    PMU::regs()
        .rf_pwc()
        .modify(|_, w| w.mspi_phy_xpd().set_bit());
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| w.lp_aonclkrst_hp_mpll_500m_clk_en().set_bit());

    // div = freq_mhz/20 - 1, ref_div = 1 -> MPLL = XTAL(40) * (div+1) / (ref_div+1)
    let ref_div: u8 = 1;
    let div: u8 = (freq_mhz / 20).saturating_sub(1) as u8;
    let div_val: u8 = (div << 3) | ref_div;

    let dhref = regi2c::I2C_MPLL_DHREF.read();
    regi2c::I2C_MPLL_DHREF.write_reg(dhref | (3 << 4));

    let rstb = regi2c::I2C_MPLL_IR_CAL_RSTB.read();
    regi2c::I2C_MPLL_IR_CAL_RSTB.write_reg(rstb & 0xDF);
    regi2c::I2C_MPLL_IR_CAL_RSTB.write_reg(rstb | (1 << 5));
    regi2c::I2C_MPLL_DIV_REG_ADDR.write_reg(div_val);

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().clear_bit());
    let mut t = 1_000_000_u32;
    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .mspi_cal_end()
        .bit_is_set()
    {
        t = t.saturating_sub(1);
        if t == 0 {
            break;
        }
        core::hint::spin_loop();
    }
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().set_bit());
    // cal_end timeout (t == 0): surfaces later via `psram_detect_size` fallback.
    let _ = t; // if need print this for debug
    crate::rom::ets_delay_us(10);
}

/// Configure PSRAM_MSPI0 (AXI cache) for HEX/DDR access at 0x4800_0000.
///
/// Faithful port of IDF `s_config_mspi_for_psram` from
/// `esp_psram_impl_ap_hex.c`. Each register field is named after its
/// `psram_ctrlr_ll_*` setter so the mapping is straightforward to verify.
///
/// Field layout (PSRAM_MSPI0 / SPIMEM2 register bit positions):
///
/// `CACHE_SCTRL` (0x40):
///   - bit 0:      `cache_usr_saddr_4byte`
///   - bit 3:      `usr_wr_sram_dummy`
///   - bit 4:      `usr_rd_sram_dummy`
///   - bit 5:      `cache_sram_usr_rcmd`
///   - bits 11:6:  `sram_rdummy_cyclelen`
///   - bits 19:14: `sram_addr_bitlen`
///   - bit 20:     `cache_sram_usr_wcmd`
///   - bit 21:     `sram_oct`
///   - bits 27:22: `sram_wdummy_cyclelen`
///
/// `SRAM_CMD` (0x44):
///   - bit 18: `sdin_oct`
///   - bit 19: `sdout_oct`
///   - bit 20: `saddr_oct`
///   - bit 21: `scmd_oct`
///   - bit 22: `sdummy_rin`
///   - bit 23: `sdummy_wout`
///   - bit 26: `sdin_hex`
///   - bit 27: `sdout_hex`
///
/// `MEM_CTRL1` (0x70): `ar_splice_en` bit 25, `aw_splice_en` bit 26.
///
/// `SMEM_DDR` (0xD8): bit 0 `smem_ddr_en`, bit 1 `smem_var_dummy`,
///                    bit 2 `smem_ddr_rdat_swp`, bit 3 `smem_ddr_wdat_swp`.
///
/// `CACHE_FCTRL` (0x3C): bit 0 `mem_axi_req_en`, bit 1 `close_axi_inf_en`.
fn configure_psram_mspi(speed_params: SpeedParams, base: u32) {
    // Dummy bit-counts come from the active speed parameter table.
    let rd_dummy_n = speed_params.rd_dummy_bits;
    let wr_dummy_n = speed_params.wr_dummy_bits;

    unsafe {
        // CACHE_SCTRL.
        //   bit 0   cache_usr_saddr_4byte
        //   bit 3   usr_wr_sram_dummy
        //   bit 4   usr_rd_sram_dummy
        //   bit 5   cache_sram_usr_rcmd
        let sctrl_addr = base + 0x40;
        let mut val = mmio_read_32(sctrl_addr);
        val |= 0b011_1001;
        //   bits 11..6   sram_rdummy_cyclelen
        //   bits 19..14  sram_addr_bitlen (N-1 form, here 32-bit addr)
        //   bits 27..22  sram_wdummy_cyclelen
        val = (val & !(0x3F << 6)) | ((rd_dummy_n - 1) << 6);
        val = (val & !(0x3F << 14)) | ((32 - 1) << 14);
        val = (val & !(0x3F << 22)) | ((wr_dummy_n - 1) << 22);
        //   bit 20  cache_sram_usr_wcmd
        //   bit 21  sram_oct (octal mode for cache transactions)
        val |= 0b11 << 20;
        mmio_write_32(sctrl_addr, val);

        // SRAM_CMD: octal-line + hex-data + dummy-level control.
        //   bit 18  sdin_oct
        //   bit 19  sdout_oct
        //   bit 20  saddr_oct
        //   bit 21  scmd_oct
        //   bit 23  sdummy_wout      (write-dummy level control enable)
        //   bit 26  sdin_hex         (16-line read data)
        //   bit 27  sdout_hex        (16-line write data)
        mmio_setbits_32(base + 0x44, 0b1100_1011_1100 << 16); // bits 27, 26, 23, 21..18

        // SRAM_DRD_CMD / SRAM_DWR_CMD: 16-bit sync read/write commands.
        mmio_write_32(base + 0x48, (16 - 1) << 28); // sync read  0x0000
        mmio_write_32(base + 0x4C, ((16 - 1) << 28) | 0x8080); // sync write 0x8080

        // MEM_CTRL1: splice enables (AXI burst optimization).
        //   bit 25  ar_splice_en
        //   bit 26  aw_splice_en
        mmio_setbits_32(base + 0x70, 0b11 << 25);

        // SMEM_DDR: DDR mode + variable dummy.
        //   bit 0  smem_ddr_en       set
        //   bit 1  smem_var_dummy    set
        //   bit 2  smem_ddr_rdat_swp clear (default off)
        //   bit 3  smem_ddr_wdat_swp clear (default off)
        mmio_clrsetbits_32(base + 0xD8, 0b1100, 0b0011);

        // CACHE_FCTRL: enable AXI access.
        //   bit 0  : MEM_AXI_REQ_EN     = 1 (set)
        //   bit 31 : CLOSE_AXI_INF_EN   = 0 (clear -> open the AXI iface)
        mmio_clrsetbits_32(base + 0x3C, 1u32 << 31, 1);
    }
}
