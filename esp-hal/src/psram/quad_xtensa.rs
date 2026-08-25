//! Shared Quad PSRAM helpers for ESP32-S2 and ESP32-S3.
//!
//! Command sequences, SPI1 ROM access, SPI0 cache phases, and the PSRAM module
//! clock register are the same on both chips. Pin mux, MSPI core clock, and MMU
//! mapping stay chip-specific.

use procmacros::ram;

use crate::peripherals::{SPI0, SPI1};

const CS_PSRAM_SEL: u8 = 1 << 1;

const PSRAM_RESET_EN: u16 = 0x66;
const PSRAM_RESET: u16 = 0x99;
const PSRAM_DEVICE_ID: u16 = 0x9F;
const PSRAM_ENTER_QMODE: u16 = 0x35;
const PSRAM_EXIT_QMODE: u16 = 0xF5;

const PSRAM_QUAD_WRITE: u32 = 0x38;
const PSRAM_FAST_READ_QUAD: u32 = 0xEB;
const PSRAM_FAST_READ_QUAD_DUMMY: u32 = 6;

#[derive(PartialEq)]
enum CommandMode {
    PsramCmdQpi = 0,
    PsramCmdSpi = 1,
}

/// Reads the AP-memory device ID and converts density to a byte size.
#[ram]
pub(crate) fn detect_quad_size() -> Option<usize> {
    psram_disable_qio_mode_spi1();

    let mut dev_id = 0u32;
    psram_exec_cmd(
        CommandMode::PsramCmdSpi,
        PSRAM_DEVICE_ID,
        8,
        0,
        24,
        0,
        core::ptr::null(),
        0,
        &mut dev_id as *mut _ as *mut u8,
        24,
        CS_PSRAM_SEL,
        false,
    );

    if dev_id == 0xffffff {
        debug!(
            "Unknown PSRAM chip ID: {:x}. PSRAM chip not found or not supported. Check if the interface mode is configured correctly.",
            dev_id
        );
        return None;
    }

    info!("chip id = {:x}", dev_id);

    const PSRAM_ID_EID_S: u32 = 16;
    const PSRAM_ID_EID_M: u32 = 0xff;
    const PSRAM_EID_SIZE_M: u32 = 0x07;
    const PSRAM_EID_SIZE_S: u32 = 5;
    const PSRAM_EID_SIZE_32MBITS: u32 = 1;
    const PSRAM_EID_SIZE_64MBITS: u32 = 2;

    let size_id =
        (((dev_id >> PSRAM_ID_EID_S) & PSRAM_ID_EID_M) >> PSRAM_EID_SIZE_S) & PSRAM_EID_SIZE_M;

    let size = match size_id {
        PSRAM_EID_SIZE_64MBITS => 8 * 1024 * 1024,
        PSRAM_EID_SIZE_32MBITS => 4 * 1024 * 1024,
        _ => 2 * 1024 * 1024,
    };

    info!("size is {}", size);
    Some(size)
}

/// Configures SPI0 cache command/address/dummy phases for Quad PSRAM.
#[ram]
pub(crate) fn config_psram_spi_phases() {
    SPI0::regs().cache_sctrl().modify(|_, w| unsafe {
        w.usr_sram_dio().clear_bit();
        w.usr_sram_qio().set_bit();
        w.cache_sram_usr_rcmd().set_bit();
        w.cache_sram_usr_wcmd().set_bit();

        w.sram_addr_bitlen().bits(23);
        w.usr_rd_sram_dummy().set_bit();

        w.sram_rdummy_cyclelen()
            .bits((PSRAM_FAST_READ_QUAD_DUMMY - 1) as u8)
    });

    SPI0::regs().sram_dwr_cmd().modify(|_, w| unsafe {
        w.cache_sram_usr_wr_cmd_bitlen().bits(7);
        w.cache_sram_usr_wr_cmd_value()
            .bits(PSRAM_QUAD_WRITE as u16)
    });

    SPI0::regs().sram_drd_cmd().modify(|_, w| unsafe {
        w.cache_sram_usr_rd_cmd_bitlen().bits(7);
        w.cache_sram_usr_rd_cmd_value()
            .bits(PSRAM_FAST_READ_QUAD as u16)
    });

    // CS0 is flash, CS1 is PSRAM.
    SPI0::regs().misc().modify(|_, w| w.cs1_dis().clear_bit());
}

/// Programs the SPI0 PSRAM module clock divider.
#[ram]
pub(crate) fn spi0_timing_config_set_psram_clock(freqdiv: u32) {
    if freqdiv == 1 {
        SPI0::regs().sram_clk().write(|w| unsafe {
            w.sclk_equ_sysclk().set_bit();
            w.sclkcnt_h().bits(0);
            w.sclkcnt_n().bits(0);
            w.sclkcnt_l().bits(0)
        });
    } else {
        SPI0::regs().sram_clk().write(|w| unsafe {
            w.sclk_equ_sysclk().clear_bit();
            w.sclkcnt_h().bits((freqdiv / 2 - 1) as u8);
            w.sclkcnt_n().bits((freqdiv - 1) as u8);
            w.sclkcnt_l().bits((freqdiv - 1) as u8)
        });
    }
}

#[ram]
pub(crate) fn psram_reset_mode_spi1() {
    psram_exec_cmd(
        CommandMode::PsramCmdSpi,
        PSRAM_RESET_EN,
        8,
        0,
        0,
        0,
        core::ptr::null(),
        0,
        core::ptr::null_mut(),
        0,
        CS_PSRAM_SEL,
        false,
    );
    psram_exec_cmd(
        CommandMode::PsramCmdSpi,
        PSRAM_RESET,
        8,
        0,
        0,
        0,
        core::ptr::null(),
        0,
        core::ptr::null_mut(),
        0,
        CS_PSRAM_SEL,
        false,
    );
}

#[ram]
pub(crate) fn psram_enable_qio_mode_spi1() {
    psram_exec_cmd(
        CommandMode::PsramCmdSpi,
        PSRAM_ENTER_QMODE,
        8,
        0,
        0,
        0,
        core::ptr::null(),
        0,
        core::ptr::null_mut(),
        0,
        CS_PSRAM_SEL,
        false,
    );
}

#[ram]
fn psram_disable_qio_mode_spi1() {
    psram_exec_cmd(
        CommandMode::PsramCmdQpi,
        PSRAM_EXIT_QMODE,
        8,
        0,
        0,
        0,
        core::ptr::null(),
        0,
        core::ptr::null_mut(),
        0,
        CS_PSRAM_SEL,
        false,
    );
}

#[expect(clippy::too_many_arguments)]
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
    unsafe extern "C" {
        fn esp_rom_spi_cmd_start(
            spi_num: u32,
            rx_buf: *const u8,
            rx_len: u16,
            cs_en_mask: u8,
            is_write_erase: bool,
        );
    }

    unsafe {
        let spi1 = SPI1::regs();
        let backup_usr = spi1.user().read().bits();
        let backup_usr1 = spi1.user1().read().bits();
        let backup_usr2 = spi1.user2().read().bits();
        let backup_ctrl = spi1.ctrl().read().bits();
        psram_set_op_mode(mode);
        psram_config_cmd(
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

        spi1.user().write(|w| w.bits(backup_usr));
        spi1.user1().write(|w| w.bits(backup_usr1));
        spi1.user2().write(|w| w.bits(backup_usr2));
        spi1.ctrl().write(|w| w.bits(backup_ctrl));
    }
}

#[expect(clippy::too_many_arguments)]
#[ram]
fn psram_config_cmd(
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
    struct EspRomSpiCmd {
        cmd: u16,
        cmd_bit_len: u16,
        addr: *const u32,
        addr_bit_len: u32,
        tx_data: *const u32,
        tx_data_bit_len: u32,
        rx_data: *mut u32,
        rx_data_bit_len: u32,
        dummy_bit_len: u32,
    }

    unsafe extern "C" {
        fn esp_rom_spi_cmd_config(spi_num: u32, pcmd: *const EspRomSpiCmd);
    }

    let conf = EspRomSpiCmd {
        cmd,
        cmd_bit_len,
        addr: &addr,
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
    unsafe extern "C" {
        fn esp_rom_spi_set_op_mode(spi: u32, mode: u32);
    }

    const ESP_ROM_SPIFLASH_QIO_MODE: u32 = 0;
    const ESP_ROM_SPIFLASH_SLOWRD_MODE: u32 = 5;

    unsafe {
        match mode {
            CommandMode::PsramCmdQpi => {
                esp_rom_spi_set_op_mode(1, ESP_ROM_SPIFLASH_QIO_MODE);
                SPI1::regs().ctrl().modify(|_, w| w.fcmd_quad().set_bit());
            }
            CommandMode::PsramCmdSpi => {
                esp_rom_spi_set_op_mode(1, ESP_ROM_SPIFLASH_SLOWRD_MODE);
            }
        }
    }
}
