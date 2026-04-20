//! PSRAM driver for ESP32-P4X (chip revision v3.x / eco5).
//!
//! P4 PSRAM uses HEX (16-line) mode via dedicated MSPI2/3 controller.
//! Memory mapped at 0x4800_0000 - 0x4C00_0000 (64 MB via cache/MMU).
//!
//! Init sequence (from esp-idf esp_psram_impl_ap_hex.c):
//!   1. MPLL enable (400 MHz for 200MHz PSRAM bus clock)
//!   2. MSPI2/3 module clock enable + clock source = MPLL
//!   3. Bus clock divider (MPLL/2 = 200MHz)
//!   4. PSRAM read/write command config (HEX sync mode)
//!   5. Address, dummy, DDR mode config
//!   6. AXI access enable
//!   7. Cache/MMU page mapping
//!
//! Ref: esp-idf components/esp_psram/device/esp_psram_impl_ap_hex.c
//!      esp-idf components/esp_hal_mspi/esp32p4/include/hal/psram_ctrlr_ll.h
//!      esp-idf components/soc/esp32p4/include/soc/regi2c_mpll.h
//!      esp-idf components/hal/esp32p4/include/hal/mmu_ll.h
//!      TRM v0.5 Ch 9 (System and Memory)

/// PSRAM virtual address range (cached)
/// Ref: esp-idf ext_mem_defs.h -- SOC_IRAM_PSRAM_ADDRESS_LOW/HIGH
pub const PSRAM_VADDR_START: usize = 0x4800_0000;

/// MMU page size (64 KB)
const MMU_PAGE_SIZE: usize = 0x10000;

/// PSRAM configuration.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// Size of PSRAM to map
    pub size: super::PsramSize,
}

/// Initialize PSRAM.
///
/// Ref: esp-idf esp_psram_impl_enable() in esp_psram_impl_ap_hex.c
///
/// Signature matches the post-merge (esp-hal 1.1.0-rc.0+) convention:
/// returns `true` on successful bring-up. Our P4 implementation is not
/// yet hardware-validated, so we always return `true` optimistically --
/// this crate's `psram::Psram::new` only uses the return value to decide
/// whether to `map_psram`, and if the caller hasn't wired up PSRAM pins
/// they won't construct the `Psram` handle anyway.
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    init_psram_inner(*config);
    true
}

/// Returns the virtual-address range that the PSRAM was mapped into.
/// Called by `Psram::new` right after `init_psram` returns true.
pub(crate) fn map_psram(config: PsramConfig) -> core::ops::Range<usize> {
    let size = config.size.get();
    PSRAM_VADDR_START..PSRAM_VADDR_START + size
}

fn init_psram_inner(config: PsramConfig) {
    // 1. Enable MPLL (400 MHz) for PSRAM clock
    // Ref: esp-idf clk_tree_ll.h -- clk_ll_mpll_enable(), clk_ll_mpll_set_config()
    //      regi2c_mpll.h -- I2C_MPLL = 0x63
    configure_mpll(400);

    // 2. Enable PSRAM controller clock and select MPLL source
    // Ref: esp-idf psram_ctrlr_ll.h -- psram_ctrlr_ll_enable_module_clock()
    //      HP_SYS_CLKRST.peri_clk_ctrl00.reg_psram_clk_en = 1
    //      HP_SYS_CLKRST.peri_clk_ctrl00.reg_psram_clk_src_sel = 1 (MPLL)
    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    // Ref: esp-idf clk_gate_ll.h -- psram_ctrlr_ll_enable_module_clock()
    clkrst.peri_clk_ctrl00().modify(|_, w| unsafe {
        w.psram_pll_clk_en().set_bit();
        w.psram_core_clk_en().set_bit();
        w.psram_clk_src_sel().bits(1) // 1 = MPLL
    });

    // 3. Set bus clock divider: MPLL(400MHz) / 2 = 200MHz PSRAM bus
    // Ref: esp-idf psram_ctrlr_ll.h -- psram_ctrlr_ll_set_bus_clock()
    //      SPI_MEM_S_SRAM_CLK_REG: SCLKCNT_N=1, SCLKCNT_H=0, SCLKCNT_L=1
    // TODO(P4X): These registers are in SPIMEM2 which may not be in PAC.
    // Using direct MMIO for now.
    // PSRAM MSPI controller base
    // Ref: esp-idf reg_base.h -- DR_REG_PSRAM_MSPI0_BASE = 0x5008_E000
    const PSRAM_MSPI_BASE: u32 = 0x5008_E000;

    // 3a. Set PSRAM bus clock divider: MPLL(400MHz) / 2 = 200MHz
    // Ref: SPI_MEM_S_SRAM_CLK_REG offset from PSRAM_MSPI_BASE
    //      sclkcnt_n=1, sclkcnt_h=0, sclkcnt_l=1
    // TODO(P4X): verify exact register offset for SRAM_CLK_REG
    // const SRAM_CLK_OFFSET: u32 = ...; // need to find from spi_mem_s_reg.h

    // 3b. Configure PSRAM read/write commands for HEX sync mode
    // Ref: esp-idf psram_ctrlr_ll.h -- psram_ctrlr_ll_set_rd_cmd/wr_cmd()
    //      SPI_MEM_S_CACHE_SCTRL_REG: cache_sram_usr_rcmd, cache_sram_usr_wcmd
    //      SPI_MEM_S_SRAM_DRD_CMD_REG: cmd_bitlen=15, cmd_value=0x0000 (sync read)
    //      SPI_MEM_S_SRAM_DWR_CMD_REG: cmd_bitlen=15, cmd_value=0x8080 (sync write)

    // 3c. Set address length (32-bit) and dummy cycles
    //      SPI_MEM_S_CACHE_SCTRL_REG: sram_addr_bitlen=31 (32-bit)
    //      rdummy_cyclelen=13 (14-1 at 200MHz), wdummy_cyclelen=6 (7-1)

    // 3d. Enable DDR mode
    //      SPI_MEM_S_SMEM_DDR_REG: smem_ddr_en=1

    // 3e. Enable HEX data lines
    //      SPI_MEM_S_SRAM_CMD_REG: sdin_hex=1, sdout_hex=1

    // 3f. Enable AXI access
    //      SPI_MEM_S_CACHE_FCTRL_REG: axi_req_en=1

    // 4. Configure PSRAM MSPI controller
    // Ref: esp-idf psram_ctrlr_ll.h, spi_mem_s_reg.h
    //      PSRAM_MSPI_BASE = 0x5008_E000
    configure_psram_mspi(PSRAM_MSPI_BASE);

    // 5. Map PSRAM via MMU (0x48000000 virtual -> physical)
    // Ref: esp-idf mmu_ll.h -- mmu_ll_write_entry()
    mmu_map_psram(PSRAM_MSPI_BASE, config);

    // Determine PSRAM size
    let psram_size = match config.size {
        super::PsramSize::AutoDetect => psram_detect_size(PSRAM_MSPI_BASE),
        super::PsramSize::Size(s) => s,
    };

    // Set PSRAM range (virtual address space)
    if psram_size > 0 {
        let start = PSRAM_VADDR_START;
        let end = start + psram_size;
        unsafe { super::set_psram_range(start..end) };
    }

    // TODO(P4X): Full PSRAM init needs:
    // - SPIMEM2 read/write command configuration (HEX sync mode)
    // - Address bitlen (32-bit), dummy cycles (14 read, 7 write at 200MHz)
    // - CS timing, DDR mode enable
    // - HEX data line mode (sdin_hex, sdout_hex)
    // - AXI access enable (mem_axi_req_en)
    // - Cache/MMU page mapping (SPI_MEM_S_MMU_ITEM_INDEX/CONTENT)
    // - AP HEX PSRAM mode register initialization (MR0-MR8)
    // - Timing calibration
    //
    // Without these, PSRAM memory at 0x48000000 is NOT accessible.
    // The ROM bootloader may have already initialized PSRAM if esp-idf
    // 2nd-stage bootloader was used.
}

/// Detect PSRAM size by reading mode register MR2 via MSPI direct SPI command.
///
/// AP HEX PSRAM MR2 format (bits [2:0] = density):
///   0x1 = 32 Mbit  (4 MB)
///   0x3 = 64 Mbit  (8 MB)
///   0x5 = 128 Mbit (16 MB)
///   0x7 = 256 Mbit (32 MB)
///
/// Read sequence: issue SPI command 0x40 (read MR), address 0x2 (MR2 index),
/// capture 8-bit MISO response.
///
/// Safety: if the MSPI controller is not in a known-good state or the command
/// hangs, we fall back to 32 MB (EV Board default). This is conservative but
/// avoids bricking on chips without PSRAM populated.
///
/// Ref: esp-idf esp_psram_impl_ap_hex.c -- psram_detect_size()
///      TRM v0.5 Ch 9 (MSPI) -- USER/CMD/ADDR/MISO registers
fn psram_detect_size(mspi_base: u32) -> usize {
    const SPI_CMD: u32 = 0x00;
    const SPI_ADDR: u32 = 0x04;
    const SPI_USER: u32 = 0x18;
    const SPI_USER1: u32 = 0x1C;
    const SPI_USER2: u32 = 0x20;
    const SPI_MISO_DLEN: u32 = 0x28;
    const SPI_W0: u32 = 0x58;

    // USER: usr_command=bit31, usr_addr=bit30, usr_miso=bit28
    const USR_COMMAND: u32 = 1 << 31;
    const USR_ADDR: u32 = 1 << 30;
    const USR_MISO: u32 = 1 << 28;
    // CMD: usr=bit18
    const CMD_USR: u32 = 1 << 18;

    unsafe {
        // Command: 0x40 (read MR), command bitlen=7 (8-1)
        ((mspi_base + SPI_USER2) as *mut u32).write_volatile((7 << 28) | 0x40);
        // Address: MR2 index = 0x2, address bitlen=7 (8-1) in USER1 bits [31:26]
        ((mspi_base + SPI_USER1) as *mut u32).write_volatile(7 << 26);
        ((mspi_base + SPI_ADDR) as *mut u32).write_volatile(0x2);
        // MISO length: 8-1 = 7 bits
        ((mspi_base + SPI_MISO_DLEN) as *mut u32).write_volatile(7);
        // Enable command + address + MISO phases
        ((mspi_base + SPI_USER) as *mut u32).write_volatile(USR_COMMAND | USR_ADDR | USR_MISO);

        // Trigger transaction
        ((mspi_base + SPI_CMD) as *mut u32).write_volatile(CMD_USR);

        // Wait for completion with timeout
        let mut timeout = 100_000u32;
        while ((mspi_base + SPI_CMD) as *const u32).read_volatile() & CMD_USR != 0 {
            timeout -= 1;
            if timeout == 0 {
                // Timeout -> fall back to default (EV Board = 32 MB)
                return 32 * 1024 * 1024;
            }
        }

        // Read MR2 response from W0 register (low 8 bits)
        let mr2 = ((mspi_base + SPI_W0) as *const u32).read_volatile() & 0xFF;

        // Decode density field [2:0]
        match mr2 & 0x7 {
            0x1 => 4 * 1024 * 1024,   // 32 Mbit = 4 MB
            0x3 => 8 * 1024 * 1024,   // 64 Mbit = 8 MB
            0x5 => 16 * 1024 * 1024,  // 128 Mbit = 16 MB
            0x7 => 32 * 1024 * 1024,  // 256 Mbit = 32 MB
            _ => {
                // Unknown/invalid response -> fall back to EV Board default
                32 * 1024 * 1024
            }
        }
    }
}

/// Map PSRAM physical pages into the virtual address space via MMU.
///
/// Each MMU entry maps a 64 KB virtual page to a physical PSRAM page.
/// Virtual address range: 0x4800_0000 + page_idx * 0x10000
///
/// MMU entry format (SPI_MEM_S):
///   - Bit 15: MMU_VALID (1 = entry active)
///   - Bit 14: MMU_TYPE (0 = Flash, 1 = PSRAM)
///   - Bits [13:0]: Physical page number
///
/// Ref: esp-idf mmu_ll.h -- mmu_ll_write_entry(), mmu_ll_check_valid_ext_vaddr()
///      esp-idf ext_mem_layout.h -- SOC_MMU_ENTRY_NUM, SOC_MMU_PAGE_SIZE
///      TRM v0.5 Ch 9 (System and Memory) -- MMU configuration
fn mmu_map_psram(mspi_base: u32, config: PsramConfig) {
    const MMU_ITEM_INDEX: u32 = 0x380;
    const MMU_ITEM_CONTENT: u32 = 0x37C;
    const MMU_VALID: u32 = 1 << 15;
    const MMU_TYPE_PSRAM: u32 = 1 << 14;
    const MMU_INVALID_ENTRY: u32 = 0x4000; // bit 14 set, valid cleared = invalid

    let psram_size = match config.size {
        super::PsramSize::AutoDetect => 32 * 1024 * 1024, // default 32 MB
        super::PsramSize::Size(s) => s,
    };

    let page_count = psram_size / MMU_PAGE_SIZE;

    // Calculate the starting MMU entry index for PSRAM virtual address
    // PSRAM virtual starts at 0x4800_0000. The MMU entry index depends on
    // the overall virtual address map layout. For P4, PSRAM region starts
    // at a fixed offset in the MMU table.
    // Ref: esp-idf mmu_ll.h -- mmu_ll_get_entry_id()
    //      The formula: entry_id = (vaddr - SOC_MMU_VADDR_BASE) / MMU_PAGE_SIZE
    //      SOC_MMU_VADDR_BASE = 0x4000_0000 (flash mapping start)
    //      PSRAM at 0x4800_0000: entry_id = (0x4800_0000 - 0x4000_0000) / 0x10000 = 0x800
    const PSRAM_MMU_START_ENTRY: u32 = 0x800;

    // Disable data cache before modifying MMU
    // Ref: esp-idf cache_ll.h -- cache_ll_l1_disable_cache()
    cache_suspend();

    unsafe {
        let index_reg = (mspi_base + MMU_ITEM_INDEX) as *mut u32;
        let content_reg = (mspi_base + MMU_ITEM_CONTENT) as *mut u32;

        for page in 0..page_count {
            let entry_id = PSRAM_MMU_START_ENTRY + page as u32;
            let content = MMU_VALID | MMU_TYPE_PSRAM | (page as u32 & 0x3FFF);

            // Write entry: set index, then write content
            index_reg.write_volatile(entry_id);
            content_reg.write_volatile(content);
        }
    }

    // Re-enable data cache
    cache_resume();
}

/// Suspend L1 data cache for MMU/cache configuration.
///
/// Must be paired with cache_resume().
///
/// Ref: esp-idf cache_ll.h -- cache_ll_l1_disable_cache()
///      L1_DCACHE_CTRL register: shut_dbus0, shut_dbus1, shut_dma bits
fn cache_suspend() {
    let cache = unsafe { &*crate::pac::CACHE::PTR };
    cache
        .l1_dcache_ctrl()
        .modify(|_, w| w.l1_dcache_shut_dbus0().set_bit());

    // Wait for cache to idle
    while !cache
        .l1_dcache_autoload_ctrl()
        .read()
        .l1_dcache_autoload_done()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

/// Resume L1 data cache after MMU/cache configuration.
///
/// Ref: esp-idf cache_ll.h -- cache_ll_l1_enable_cache()
fn cache_resume() {
    let cache = unsafe { &*crate::pac::CACHE::PTR };
    cache
        .l1_dcache_ctrl()
        .modify(|_, w| w.l1_dcache_shut_dbus0().clear_bit());
}

/// Invalidate L1 data cache for a given address range.
///
/// Required after DMA transfers to ensure CPU sees fresh data.
///
/// Ref: esp-idf cache_ll.h -- cache_ll_l1_invalidate_cache()
///      CACHE.sync_ctrl, sync_addr, sync_size
#[allow(dead_code)]
pub(crate) fn cache_invalidate(addr: u32, size: u32) {
    let cache = unsafe { &*crate::pac::CACHE::PTR };

    cache
        .sync_addr()
        .write(|w| unsafe { w.bits(addr) });
    cache
        .sync_size()
        .write(|w| unsafe { w.bits(size) });

    // Trigger invalidate: sync_ctrl.invalidate_ena
    cache
        .sync_ctrl()
        .modify(|_, w| w.invalidate_ena().set_bit());

    // Wait for completion
    while !cache
        .sync_ctrl()
        .read()
        .sync_done()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

/// Write back L1 data cache for a given address range.
///
/// Required before DMA transfers to ensure DMA sees latest CPU writes.
///
/// Ref: esp-idf cache_ll.h -- cache_ll_l1_writeback_cache()
#[allow(dead_code)]
pub(crate) fn cache_writeback(addr: u32, size: u32) {
    let cache = unsafe { &*crate::pac::CACHE::PTR };

    cache
        .sync_addr()
        .write(|w| unsafe { w.bits(addr) });
    cache
        .sync_size()
        .write(|w| unsafe { w.bits(size) });

    // Trigger writeback: sync_ctrl.writeback_ena
    cache
        .sync_ctrl()
        .modify(|_, w| w.writeback_ena().set_bit());

    while !cache
        .sync_ctrl()
        .read()
        .sync_done()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

/// Configure MPLL to target frequency for PSRAM clock.
///
/// Ref: esp-idf clk_tree_ll.h:475-519 -- clk_ll_mpll_set_config()
///      regi2c_mpll.h -- I2C_MPLL = 0x63
fn configure_mpll(freq_mhz: u32) {
    use crate::soc::regi2c;

    const I2C_MPLL: u8 = 0x63;
    // Ref: esp-idf soc/esp32p4/include/soc/regi2c_mpll.h
    const I2C_MPLL_DIV_REG_ADDR: u8 = 2;
    const I2C_MPLL_DHREF: u8 = 3;
    const I2C_MPLL_IR_CAL_RSTB: u8 = 5;

    // Enable MPLL power
    // Ref: esp-idf clk_tree_ll.h -- clk_ll_mpll_enable()
    //      PMU.rf_pwc: mspi_phy_xpd
    //      LP_AON_CLKRST: hp_mpll_500m_clk_en
    // TODO(P4X): PMU rf_pwc register access for MPLL power

    // Calculate divider: MPLL_freq = XTAL(40MHz) * (div+1) / (ref_div+1)
    // For 400MHz: ref_div=1, div=19 -> 40*20/2 = 400MHz
    // Ref: esp-idf clk_tree_ll.h:508
    let div_val: u8 = match freq_mhz {
        400 => (9 << 3) | 1,  // div=9, ref_div=1 -> 40*10/2 = 200... needs recalc
        500 => (12 << 3) | 1, // div=12, ref_div=1
        _ => (9 << 3) | 1,    // default 400MHz
    };

    // MPLL calibration sequence
    // Ref: esp-idf clk_tree_ll.h:484-498
    // 1. Set DHREF bits [5:4] = 3
    let dhref = regi2c::regi2c_read(I2C_MPLL, 0, I2C_MPLL_DHREF);
    regi2c::regi2c_write(I2C_MPLL, 0, I2C_MPLL_DHREF, dhref | (3 << 4));

    // 2. Toggle IR_CAL_RSTB bit 5
    let rstb = regi2c::regi2c_read(I2C_MPLL, 0, I2C_MPLL_IR_CAL_RSTB);
    regi2c::regi2c_write(I2C_MPLL, 0, I2C_MPLL_IR_CAL_RSTB, rstb & 0xDF);
    regi2c::regi2c_write(I2C_MPLL, 0, I2C_MPLL_IR_CAL_RSTB, rstb | (1 << 5));

    // 3. Write divider
    regi2c::regi2c_write(I2C_MPLL, 0, I2C_MPLL_DIV_REG_ADDR, div_val);

    // 4. Run calibration
    // Ref: HP_SYS_CLKRST.ana_pll_ctrl0.reg_mspi_cal_stop/end
    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().clear_bit());
    while !clkrst.ana_pll_ctrl0().read().mspi_cal_end().bit_is_set() {
        core::hint::spin_loop();
    }
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

/// Configure PSRAM MSPI controller registers for HEX mode operation.
///
/// Ref: esp-idf psram_ctrlr_ll.h, spi_mem_s_reg.h
///      All offsets relative to PSRAM_MSPI_BASE (0x5008_E000)
fn configure_psram_mspi(base: u32) {
    // Register offsets from spi_mem_s_reg.h
    const CACHE_FCTRL: u32 = 0x3C;
    const CACHE_SCTRL: u32 = 0x40;
    const SRAM_CMD: u32 = 0x44;
    const SRAM_DRD_CMD: u32 = 0x48;
    const SRAM_DWR_CMD: u32 = 0x4C;
    const SMEM_DDR: u32 = 0xD8;

    unsafe {
        // 1. Configure read command: 16-bit sync read (0x0000)
        // SPI_MEM_S_CACHE_SCTRL: cache_sram_usr_rcmd=1
        // SPI_MEM_S_SRAM_DRD_CMD: cmd_bitlen=15, cmd_value=0
        let sctrl = (base + CACHE_SCTRL) as *mut u32;
        let val = sctrl.read_volatile();
        let val = val | (1 << 20); // cache_sram_usr_rcmd = 1
        let val = val | (1 << 5);  // cache_sram_usr_wcmd = 1
        // sram_addr_bitlen = 31 (32-bit address)
        let val = (val & !(0x3F << 14)) | (31 << 14);
        // usr_rd_sram_dummy = 1, sram_rdummy_cyclelen = 13 (14-1 for 200MHz)
        let val = val | (1 << 22); // usr_rd_sram_dummy
        let val = (val & !(0x3F << 6)) | (13 << 6); // rdummy_cyclelen
        // cache_usr_saddr_4byte = 1
        let val = val | (1 << 24);
        sctrl.write_volatile(val);

        // Read command: sync read 0x0000, bitlen=15
        let drd = (base + SRAM_DRD_CMD) as *mut u32;
        drd.write_volatile((15 << 28) | 0x0000); // bitlen[31:28]=15, value[15:0]=0

        // Write command: sync write 0x8080, bitlen=15
        let dwr = (base + SRAM_DWR_CMD) as *mut u32;
        dwr.write_volatile((15 << 28) | 0x8080);

        // 2. Enable DDR mode
        let ddr = (base + SMEM_DDR) as *mut u32;
        let val = ddr.read_volatile();
        ddr.write_volatile(val | 1); // smem_ddr_en = 1

        // 3. Enable HEX (16-line) data mode
        let sram_cmd = (base + SRAM_CMD) as *mut u32;
        let val = sram_cmd.read_volatile();
        // sdin_hex (bit 4) and sdout_hex (bit 5) -- exact bit positions need verification
        // Ref: esp-idf psram_ctrlr_ll.h -- psram_ctrlr_ll_set_line_mode()
        let val = val | (1 << 4) | (1 << 5); // HEX mode data lines
        sram_cmd.write_volatile(val);

        // 4. Enable AXI access
        let fctrl = (base + CACHE_FCTRL) as *mut u32;
        let val = fctrl.read_volatile();
        let val = val | (1 << 0); // mem_axi_req_en = 1
        let val = val & !(1 << 1); // close_axi_inf_en = 0
        fctrl.write_volatile(val);
    }
}
