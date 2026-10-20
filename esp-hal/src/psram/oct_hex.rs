use crate::{
    clock::ll::PsramInstance,
    pac::{generic::Reg, iomux_mspi_pin::psram_d_pin0::PSRAM_D_PIN0_SPEC},
    peripherals::{IOMUX_MSPI_PIN, MEMSPI2, MEMSPI3},
    psram::{EXTMEM_ORIGIN, PsramTimingParams},
};

/// Detect PSRAM size by reading mode register MR2 via MSPI1.
pub(super) fn psram_detect_size(speed_params: &PsramTimingParams) -> usize {
    let (mr2, _mr3) = psram_mr_read(speed_params, MR_ADDR_MR2_MR3);
    match mr2 & 0x7 {
        0x1 => 4 * 1024 * 1024,  //  32 Mbit
        0x3 => 8 * 1024 * 1024,  //  64 Mbit
        0x5 => 16 * 1024 * 1024, // 128 Mbit
        0x6 => 64 * 1024 * 1024, // 512 Mbit
        0x7 => 32 * 1024 * 1024, // 256 Mbit
        _ => {
            // unknown -> EV Board default
            warn!("Unknown PSRAM size: mr2={:#x}", mr2);
            32 * 1024 * 1024
        }
    }
}

pub(super) fn write_psram_mmu_entry(entry_id: u32, page: u16) {
    MEMSPI2::regs()
        .mmu_item_index()
        .write(|w| unsafe { w.mmu_item_index().bits(entry_id) });
    MEMSPI2::regs().mmu_item_content().write(|w| unsafe {
        w.paddr().bits(page);
        w.access_spiram().set_bit();
        w.spiram_valid().set_bit()
    });
}

/// Maps PSRAM physical pages into the virtual address space via MMU.
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
pub(super) fn mmu_map_psram(size: usize) {
    const MMU_PAGE_SIZE: usize = property!("mmu.page_size");

    let page_count = size / MMU_PAGE_SIZE;

    // Note: we do NOT call Cache_Suspend_*. The ROM helpers expect
    // interrupts/scheduler state we can't guarantee in init context, and
    // hang. Since at init time nothing else is reading PSRAM via cache
    // yet, we can write the MMU entries unsynchronized and then
    // invalidate to drop any stale prefetched lines.
    for page in 0..page_count {
        write_psram_mmu_entry(page as u32, page as u16);
    }

    // Invalidate the newly mapped PSRAM window.
    unsafe { crate::soc::cache_invalidate_addr(EXTMEM_ORIGIN as u32, size as u32) };
}

/// Configures PSRAM_MSPI0 (AXI cache) for (OCT or HEX)/DDR access.
///
/// Faithful port of IDF `s_config_mspi_for_psram` from
/// `esp_psram_impl_ap_oct.c`. Each register field is named after its
/// `psram_ctrlr_ll_*` setter so the mapping is straightforward to verify.
pub(super) fn configure_psram_mspi(timing: &PsramTimingParams, is_hex: bool) {
    // Dummy bit-counts come from the active speed parameter table.
    MEMSPI2::regs().cache_sctrl().modify(|_, w| unsafe {
        w.cache_usr_saddr_4byte().set_bit();
        w.usr_wr_sram_dummy().set_bit();
        w.usr_rd_sram_dummy().set_bit();
        w.cache_sram_usr_rcmd().set_bit();

        w.sram_rdummy_cyclelen().bits(timing.rd_dummy_bits);
        w.sram_wdummy_cyclelen().bits(timing.wr_dummy_bits);

        w.sram_addr_bitlen().bits(31);

        w.cache_sram_usr_wcmd().set_bit();
        w.sram_oct().set_bit();

        w
    });

    // SRAM_CMD: octal-line + oct-data + dummy-level control.
    MEMSPI2::regs().sram_cmd().modify(|_, w| {
        w.sdin_oct().set_bit();
        w.sdout_oct().set_bit();
        w.saddr_oct().set_bit();
        w.scmd_oct().set_bit();
        w.sdummy_wout().set_bit();
        w.sdin_hex().bit(is_hex);
        w.sdout_hex().bit(is_hex)
    });

    // SRAM_DRD_CMD / SRAM_DWR_CMD: 16-bit sync read/write commands.
    MEMSPI2::regs().sram_drd_cmd().write(|w| unsafe {
        // sync read  0x0000
        w.cache_sram_usr_rd_cmd_bitlen().bits(15);
        w.cache_sram_usr_rd_cmd_value().bits(0)
    });
    MEMSPI2::regs().sram_dwr_cmd().write(|w| unsafe {
        // sync write 0x8080
        w.cache_sram_usr_wr_cmd_bitlen().bits(15);
        w.cache_sram_usr_wr_cmd_value().bits(0x8080)
    });

    MEMSPI2::regs().ctrl1().modify(|_, w| {
        w.ar_splice_en().set_bit();
        w.aw_splice_en().set_bit()
    });

    // SMEM_DDR: DDR mode + variable dummy.
    MEMSPI2::regs().smem_ddr().modify(|_, w| {
        w.var_dummy().set_bit();
        w.rdat_swp().clear_bit();
        w.wdat_swp().clear_bit();
        w.en().set_bit()
    });

    // CACHE_FCTRL: enable AXI access.
    MEMSPI2::regs().cache_fctrl().modify(|_, w| {
        w.axi_req_en().set_bit();
        w.close_axi_inf_en().clear_bit()
    });
}

/// Sets bus-clock divider for both PSRAM_MSPI0 and PSRAM_MSPI1.
pub(super) fn set_bus_clock(clock: u32) -> bool {
    let source_mhz = PsramInstance::Psram.function_clock_frequency() / 1_000_000;
    if !source_mhz.is_multiple_of(clock) {
        error!(
            "PSRAM source clock frequency must be an integer multiple of PSRAM frequency. Source={} MHz, PSRAM={} MHz",
            source_mhz, clock
        );
        return false;
    }

    let div = (source_mhz / clock) as u8;

    if div <= 1 {
        MEMSPI2::regs().sram_clk().write(|w| unsafe {
            w.sclk_equ_sysclk().set_bit();
            w.sclkcnt_n().bits(0);
            w.sclkcnt_h().bits(0);
            w.sclkcnt_l().bits(0)
        });
        MEMSPI3::regs().clock().write(|w| unsafe {
            w.clk_equ_sysclk().set_bit();
            w.clkcnt_n().bits(0);
            w.clkcnt_h().bits(0);
            w.clkcnt_l().bits(0)
        });
    } else {
        MEMSPI2::regs().sram_clk().write(|w| unsafe {
            w.sclkcnt_n().bits(div - 1);
            w.sclkcnt_h().bits(div / 2 - 1);
            w.sclkcnt_l().bits(div - 1)
        });
        MEMSPI3::regs().clock().write(|w| unsafe {
            w.clkcnt_n().bits(div - 1);
            w.clkcnt_h().bits(div / 2 - 1);
            w.clkcnt_l().bits(div - 1)
        });
    };

    true
}

/// Enables DLL timing calibration for both controllers.
pub(super) fn enable_dll() {
    MEMSPI2::regs()
        .timing_cali()
        .modify(|_, w| w.dll_timing_cali().set_bit());
    MEMSPI2::regs()
        .smem_timing_cali()
        .modify(|_, w| w.dll_timing_cali().set_bit());
}

/// Configures PSRAM PHY pads.
///
/// Mirrors IDF `mspi_timing_ll_pin_drv_set(2)` +
/// `mspi_timing_ll_enable_dqs(true)`.
pub(super) fn psram_pad_init(_is_hex: bool) {
    fn init_pin_drv(reg: &Reg<PSRAM_D_PIN0_SPEC>) {
        reg.modify(|_, w| unsafe { w.drv().bits(2) });
    }

    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_d_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_q_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_wp_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_hold_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq4_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq5_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq6_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq7_pin0());

    IOMUX_MSPI_PIN::regs()
        .psram_dqs_0_pin0()
        .modify(|_, w| unsafe {
            w.drv().bits(2);
            w.xpd().set_bit()
        });

    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_ck_pin0());
    init_pin_drv(IOMUX_MSPI_PIN::regs().psram_cs_pin0());

    #[cfg(psram_hex_spi)]
    if _is_hex {
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq8_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq9_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq10_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq11_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq12_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq13_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq14_pin0());
        init_pin_drv(IOMUX_MSPI_PIN::regs().psram_dq15_pin0());

        IOMUX_MSPI_PIN::regs()
            .psram_dqs_1_pin0()
            .modify(|_, w| unsafe {
                w.drv().bits(2);
                w.xpd().set_bit()
            });
    }
}

/// Sets PSRAM CS timing on the AXI controller's SMEM_AC register.
/// SMEM_CS_SETUP=1, SMEM_CS_HOLD=1, setup_time=N-1,
/// hold_time=N-1, hold_delay=N-1, split_trans_en=1.
pub(super) fn set_cs_timing() {
    /// CS timing constants (matches IDF AP_OCT_PSRAM_CS_*). Independent of
    /// `SpiRamFreq` -- IDF uses the same values across all speed branches.
    const AP_CS_SETUP_TIME: u8 = 4;
    const AP_CS_HOLD_TIME: u8 = 4;
    const AP_CS_HOLD_DELAY: u8 = 3;

    MEMSPI2::regs().smem_ac().modify(|_, w| unsafe {
        w.cs_setup().set_bit();
        w.cs_hold().set_bit();
        w.split_trans_en().set_bit();

        w.cs_setup_time().bits(AP_CS_SETUP_TIME - 1);
        w.cs_hold_time().bits(AP_CS_HOLD_TIME - 1);
        w.cs_hold_delay().bits(AP_CS_HOLD_DELAY - 1);

        w
    });
}

// MR (a.k.a Mode Register with single byte inside the AP OCT PSRAM chip.

const MR_ADDR_MR0_MR1: u32 = 0x0;
const MR_ADDR_MR2_MR3: u32 = 0x2;
const MR_ADDR_MR4_MR5: u32 = 0x4;

/// MR6 only used in current state, MR7 is unused or reserved.
#[allow(dead_code)] // half-sleep trigger; kept for future power-down support
const MR_ADDR_MR6_MR7: u32 = 0x6;
/// MR8 only used in current state, MR9 is unused or reserved.
const MR_ADDR_MR8_MR9: u32 = 0x8;

/// Reads an 8-bit mode-register pair from the AP OCT PSRAM chip.
///
/// Returns `(low, high)` where:
///   - `low`  = the MR at `mr_addr`
///   - `high` = the MR at `mr_addr + 1`
fn psram_mr_read(timing: &PsramTimingParams, mr_addr: u32) -> (u8, u8) {
    let pair = mspi1_reg_read16(timing, mr_addr);
    ((pair & 0xFF) as u8, ((pair >> 8) & 0xFF) as u8)
}

/// Writes an 8-bit mode-register pair to the AP OCT PSRAM chip.
///
/// `low` goes to the MR at `mr_addr`, `high` goes to the MR at
/// `mr_addr + 1`. For pair addresses whose high slot is reserved, pass
/// the value previously read back (read-modify-write).
fn psram_mr_write(mr_addr: u32, low: u8, high: u8) {
    let pair = (low as u16) | ((high as u16) << 8);
    mspi1_reg_write16(mr_addr, pair)
}

/// Initializes AP OCT PSRAM mode registers via PSRAM_MSPI1 OPI DTR
/// referenced IDF `oct_psram_mode_reg_t`).
/// MR0: drive_str[1:0], read_latency[4:2], lt[5]
/// MR4: wr_latency[7:5]
/// MR8: bl[1:0], bt[2], rbx[3], x16[6].
pub(super) fn init_mr_registers(timing: &PsramTimingParams, is_hex: bool) {
    // Read+modify+write MR0 (preserve MR1 high byte).
    let (mut mr0, mr1) = psram_mr_read(timing, MR_ADDR_MR0_MR1);
    mr0 &= !(0x3 | (0x7 << 2) | (1 << 5));
    mr0 |= ((timing.mr0_rl & 0x7) << 2) | (1 << 5);
    psram_mr_write(MR_ADDR_MR0_MR1, mr0, mr1);

    // Read+modify+write MR4 (preserve MR5 reserved high byte).
    let (mut mr4, mr5) = psram_mr_read(timing, MR_ADDR_MR4_MR5);
    mr4 &= !(0x7 << 5);
    mr4 |= (timing.mr4_wl & 0x7) << 5;
    psram_mr_write(MR_ADDR_MR4_MR5, mr4, mr5);

    // do nothing for MR6 and MR7

    // Read+modify+write MR8 (high byte is reserved/absent — pass 0).
    let (mut mr8, mr9) = psram_mr_read(timing, MR_ADDR_MR8_MR9); // MR9 is unused
    mr8 &= !(0x3 | (1 << 2) | (1 << 3) | (1 << 6));
    mr8 |= 3 // bt = 0
        | (1 << 3); // rbx = 1
    if is_hex {
        mr8 |= 1 << 6; // x16 = 1
    }
    psram_mr_write(MR_ADDR_MR8_MR9, mr8, mr9); // keep previous MR9
}

/// Mirror of IDF's `esp_rom_spi_cmd_t` (rom/opi_flash.h).
///
/// Layout must match the C struct exactly; the ROM driver reads it via the pointer.
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
/// Maps to `PSRAM_CTRLR_LL_MSPI_ID_3` in IDF.
const ROM_SPI_PSRAM_CMD_NUM: i32 = 3;

unsafe extern "C" {
    /// Sets the controller's read mode (e.g. OPI-DTR). Configures cmd/addr/
    /// data line counts (8-line for OPI) and DDR mode bits in one call.
    /// Linked from `esp32s31.rom.ld`: `esp_rom_spi_set_op_mode = 0x4fc00110`.
    fn esp_rom_spi_set_op_mode(spi_num: i32, mode: u32);
    /// Configures command/addr/dummy/data phases for next transaction.
    /// Writes USR / USER1 / USER2 / ADDR / MOSI_DLEN / MISO_DLEN / W0..
    /// Linked from `esp32s31.rom.ld`: `esp_rom_spi_cmd_config = 0x4fc00108`.
    fn esp_rom_spi_cmd_config(spi_num: i32, pcmd: *mut EspRomSpiCmd);
}

/// Kicks the controller (sets `SPI_USR` bit 18 in CMD_REG) and polls
/// bounded for completion. Replacement for ROM `esp_rom_spi_cmd_start`,
/// which polls forever; a hang there gives no diagnostic, while the
/// bounded variant surfaces a real failure as a returned error. After
/// the bit clears, copies MISO bytes from W0..W{N} into `rx_buf`.
fn mspi1_kick_and_collect(rx: &mut [u8]) -> Result<(), ()> {
    const SPI_USR_TRIGGER: u32 = 1 << 18;
    const MAX_ITERS: u32 = 1_000_000;

    // Select PSRAM; leave flash disabled.
    MEMSPI3::regs().misc().modify(|_, w| {
        w.cs1_dis().clear_bit();
        w.cs0_dis().set_bit()
    });

    // Kick.
    MEMSPI3::regs()
        .cmd()
        .write(|w| unsafe { w.bits(SPI_USR_TRIGGER) });

    let mut t = MAX_ITERS;
    while MEMSPI3::regs().cmd().read().bits() & SPI_USR_TRIGGER != 0 {
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
            let word = MEMSPI3::regs().w(i).read().bits();
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

/// Issue an AP OCT PSRAM register-read command (0x4040, 16-bit cmd,
/// 32-bit address, 16-bit MISO) through `PSRAM_MSPI1`. Setup via ROM
/// helpers (`set_op_mode` + `cmd_config`); kick + poll done manually
/// with a timeout (avoids ROM `cmd_start` poll-forever pitfall).
fn mspi1_reg_read16(timing: &PsramTimingParams, addr: u32) -> u16 {
    const REG_READ_CMD: u16 = 0x4040;
    const CMD_BITLEN: u16 = 16;
    const ADDR_BITLEN: u32 = 32;
    const DATA_BITLEN: u32 = 16;

    let mut addr_local = addr;
    let mut rx: [u8; 2] = [0; 2];
    let mut conf = EspRomSpiCmd {
        cmd: REG_READ_CMD,
        cmd_bit_len: CMD_BITLEN,
        addr: &raw mut addr_local,
        addr_bit_len: ADDR_BITLEN,
        tx_data: core::ptr::null_mut(),
        tx_data_bit_len: 0,
        rx_data: rx.as_mut_ptr().cast::<u32>(),
        rx_data_bit_len: DATA_BITLEN,
        dummy_bit_len: timing.reg_dummy_bits,
    };

    unsafe {
        esp_rom_spi_set_op_mode(ROM_SPI_PSRAM_CMD_NUM, ESP_ROM_SPIFLASH_OPI_DTR_MODE);
        esp_rom_spi_cmd_config(ROM_SPI_PSRAM_CMD_NUM, &raw mut conf);
    }
    let _ = mspi1_kick_and_collect(&mut rx);
    u16::from_le_bytes(rx)
}

/// Issue an AP OCT PSRAM register-write command (0xC0C0) with 16-bit
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
        addr: &raw mut addr_local,
        addr_bit_len: ADDR_BITLEN,
        tx_data: tx.as_mut_ptr().cast::<u32>(),
        tx_data_bit_len: DATA_BITLEN,
        rx_data: core::ptr::null_mut(),
        rx_data_bit_len: 0,
        dummy_bit_len: 0,
    };

    unsafe {
        esp_rom_spi_set_op_mode(ROM_SPI_PSRAM_CMD_NUM, ESP_ROM_SPIFLASH_OPI_DTR_MODE);
        esp_rom_spi_cmd_config(ROM_SPI_PSRAM_CMD_NUM, &raw mut conf);
    }
    let mut empty: [u8; 0] = [];
    let _ = mspi1_kick_and_collect(&mut empty);
}
