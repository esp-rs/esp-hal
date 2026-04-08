use procmacros::ram;

use super::{PsramConfig, PsramLlCsIdT, PsramSize, ctrlr_ll::*};
use crate::peripherals::{IO_MUX, SPI1};

const CS_PSRAM_SEL: u8 = 1 << 1;

const PSRAM_QUAD_CS_HOLD_VAL: u32 = 1;
const PSRAM_QUAD_CS_SETUP_VAL: u32 = 1;

const PSRAM_QUAD_CMD_LENGTH: u32 = 8;
const PSRAM_QUAD_ADDR_LENGTH: u32 = 24;

// AP memory
const PSRAM_RESET_EN: u16 = 0x66;
const PSRAM_RESET: u16 = 0x99;
const PSRAM_DEVICE_ID: u16 = 0x9F;
const PSRAM_QUAD_FAST_READ_QUAD: u32 = 0xEB;
const PSRAM_QUAD_WRITE_QUAD: u32 = 0x38;
const PSRAM_QUAD_FAST_READ_QUAD_DUMMY: u32 = 6;

#[derive(PartialEq)]
#[allow(unused)]
pub(super) enum CommandMode {
    PsramCmdQpi = 0,
    PsramCmdSpi = 1,
}

pub(super) enum MspiTimingSpeedMode {
    /// Low performance speed mode, this mode is safe for all the scenarios, unless the MSPI
    /// attached devices (Flash, PSRAM) are powered down.
    ///
    /// As a tradeoff, the performance of the MSPI (devices) are switched to a very low speed
    MspiTimingSpeedModeLowPerf,
    /// Normal performance speed mode, MSPI speed is the same as you configured
    MspiTimingSpeedModeNormalPerf,
}

#[ram]
pub(crate) fn psram_init(config: &mut super::PsramConfig) {
    psram_gpio_config();
    psram_set_cs_timing();

    mspi_timing_enter_low_speed_mode();

    if config.size.is_auto() {
        psram_disable_qio_mode_spi1();

        // read chip id
        let mut dev_id = 0u64;
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
            48,           // rx data and rx bit len
            CS_PSRAM_SEL, // cs bit mask
            false,
        );

        if dev_id & u32::MAX as u64 == 0xffffff {
            warn!(
                "Unknown PSRAM chip ID: {:x}. PSRAM chip not found or not supported. Check if ESP_HAL_CONFIG_PSRAM_MODE is configured correctly.",
                dev_id
            );
            return;
        }

        info!("Detected PSRAM chip id = {:x}", dev_id);

        let density = (((dev_id >> 16) & 0xff) >> 5) & 7;
        info!("Detected PSRAM chip density = {}", density);

        let size = match density {
            0 => 2 * 1024 * 1024,
            1 => 4 * 1024 * 1024,
            // while density says 8m, esp-idf considers it 4m - so we do the same
            2 if dev_id == 0xa188145b5d0d => 4 * 1024 * 1024,
            2 => 8 * 1024 * 1024,
            _ => 2 * 1024 * 1024,
        };

        info!("PSRAM size is {}", size);

        config.size = PsramSize::Size(size);
    }

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
    mspi_timing_enter_high_speed_mode(config);
}

unsafe extern "C" {
    fn esp_rom_gpio_connect_out_signal(gpio_num: u8, signal_idx: u8, out_inv: bool, oen_inv: bool);

    /// Enable Quad I/O pin functions
    ///
    /// Sets the HD & WP pin functions for Quad I/O modes, based on the
    /// efuse SPI pin configuration.
    ///
    /// [`wp_gpio_num`]: u8 Number of the WP pin to reconfigure for quad I/O
    /// [`spiconfig`]: u32 Pin configuration, as returned from ets_efuse_get_spiconfig().
    /// - If this parameter is 0, default SPI pins are used and wp_gpio_num parameter is ignored.
    /// - If this parameter is 1, default HSPI pins are used and wp_gpio_num parameter is ignored.
    /// - For other values, this parameter encodes the HD pin number and also the CLK pin number.
    ///   CLK pin selection is used to determine if HSPI or SPI peripheral will be used (use HSPI if
    ///   CLK pin is the HSPI clock pin, otherwise use SPI). Both HD & WP pins are configured via
    ///   GPIO matrix to map to the selected peripheral.
    fn esp_rom_spiflash_select_qio_pins(wp_gpio_num: u8, spiconfig: u32);
}

// Configure PSRAM SPI0 phase related registers here according to the PSRAM chip
// requirement
#[ram]
fn config_psram_spi_phases() {
    psram_ctrlr_ll_set_read_mode(PSRAM_CTRLR_LL_MSPI_ID_0, CommandMode::PsramCmdQpi);
    psram_ctrlr_ll_set_wr_cmd(
        PSRAM_CTRLR_LL_MSPI_ID_0,
        PSRAM_QUAD_CMD_LENGTH,
        PSRAM_QUAD_WRITE_QUAD,
    );
    psram_ctrlr_ll_set_rd_cmd(
        PSRAM_CTRLR_LL_MSPI_ID_0,
        PSRAM_QUAD_CMD_LENGTH,
        PSRAM_QUAD_FAST_READ_QUAD,
    );
    psram_ctrlr_ll_set_addr_bitlen(PSRAM_CTRLR_LL_MSPI_ID_0, PSRAM_QUAD_ADDR_LENGTH);
    psram_ctrlr_ll_set_rd_dummy(PSRAM_CTRLR_LL_MSPI_ID_0, PSRAM_QUAD_FAST_READ_QUAD_DUMMY);
    psram_ctrlr_ll_set_cs_pin(PSRAM_CTRLR_LL_MSPI_ID_0, PsramLlCsIdT::PsramLlCsId1);
}

#[ram]
fn mspi_timing_psram_tuning() {
    // esp-idf will probe different settings and select a suitable one
    //
    // see mspi_timing_psram_tuning in
    // components\esp_hw_support\mspi_timing_tuning\mspi_timing_tuning.c
    //
    // for now we just use the user provided setting (which might be default settings)
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
fn mspi_timing_enter_high_speed_mode(config: &PsramConfig) {
    super::mspi_timing_config_set_flash_clock(
        config.flash_frequency.mhz(),
        MspiTimingSpeedMode::MspiTimingSpeedModeNormalPerf,
    );
    super::mspi_timing_config_set_psram_clock(
        config.ram_frequency.mhz(),
        MspiTimingSpeedMode::MspiTimingSpeedModeNormalPerf,
    );

    if config.ram_frequency.need_timing_tuning() {
        mspi_timing_flash_config_set_tuning_regs(config);
        mspi_timing_psram_config_set_tuning_regs(config);
    }
}

// send reset command to psram, in spi mode
#[ram]
fn psram_reset_mode_spi1() {
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

    unsafe extern "C" {
        /// Config the spi user command
        /// [`spi_num`] spi port
        /// [`pcmd`] pointer to accept the spi command struct
        fn esp_rom_spi_cmd_config(spi_num: u32, pcmd: *const esp_rom_spi_cmd_t);
    }

    let conf = esp_rom_spi_cmd_t {
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

/// Exit QPI mode
#[ram]
fn psram_disable_qio_mode_spi1() {
    const PSRAM_EXIT_QMODE: u16 = 0xF5;
    const CS_PSRAM_SEL: u8 = 1 << 1;

    psram_exec_cmd(
        CommandMode::PsramCmdQpi,
        PSRAM_EXIT_QMODE,
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
    psram_ctrlr_ll_set_cs_hold(PSRAM_CTRLR_LL_MSPI_ID_0, PSRAM_QUAD_CS_HOLD_VAL);
    psram_ctrlr_ll_set_cs_setup(PSRAM_CTRLR_LL_MSPI_ID_0, PSRAM_QUAD_CS_SETUP_VAL);
    // #if CONFIG_SPIRAM_ECC_ENABLE
    //     psram_ctrlr_ll_set_ecc_cs_hold(PSRAM_CTRLR_LL_MSPI_ID_0,
    // PSRAM_QUAD_CS_ECC_HOLD_TIME_VAL); #endif
}

#[ram]
fn psram_gpio_config() {
    // CS1
    let cs1_io: u8 = super::PSRAM_CS_IO;
    if cs1_io == super::SPI_CS1_GPIO_NUM {
        unsafe {
            IO_MUX::regs()
                .gpio(cs1_io as usize)
                .modify(|_, w| w.mcu_sel().bits(super::FUNC_SPICS1_SPICS1));
        }
    } else {
        unsafe {
            esp_rom_gpio_connect_out_signal(cs1_io, super::SPICS1_OUT_IDX, false, false);

            IO_MUX::regs()
                .gpio(cs1_io as usize)
                .modify(|_, w| w.mcu_sel().bits(super::PIN_FUNC_GPIO));
        }
    }

    let wp_io: u8 = super::PSRAM_SPIWP_SD3_IO;
    // SOC_SPI_MEM_SUPPORT_CONFIG_GPIO_BY_EFUSE not supported

    // This ROM function will init both WP and HD pins.
    unsafe {
        esp_rom_spiflash_select_qio_pins(wp_io, 0);
    }
}

fn mspi_timing_enter_low_speed_mode() {
    // Here we are going to slow the SPI1 frequency to 20Mhz, so we need to set SPI1 din_num and
    // din_mode regs.
    //
    // Because SPI0 and SPI1 share the din_num and din_mode regs, so if we clear SPI1 din_num and
    // din_mode to 0, if the SPI0 flash module clock is still in high freq, it may not work
    // correctly.
    //
    // Therefore, here we need to slow both the SPI0 and SPI1 and related timing tuning regs to
    // 20Mhz configuration.
    //
    // Currently we only need to change these clocks on chips with timing tuning
    // Should be extended to other no-timing-tuning chips if needed. e.g.:
    // we still need to turn down Flash / PSRAM clock speed at a certain period of time
    let low_speed_freq_mhz = 20;
    super::mspi_timing_config_set_flash_clock(
        low_speed_freq_mhz,
        MspiTimingSpeedMode::MspiTimingSpeedModeLowPerf,
    );
    super::mspi_timing_config_set_psram_clock(
        low_speed_freq_mhz,
        MspiTimingSpeedMode::MspiTimingSpeedModeLowPerf,
    );

    // #if MSPI_TIMING_FLASH_NEEDS_TUNING || MSPI_TIMING_PSRAM_NEEDS_TUNING
    mspi_timing_flash_config_clear_tuning_regs();
    mspi_timing_psram_config_clear_tuning_regs();
    // #endif  //#if MSPI_TIMING_FLASH_NEEDS_TUNING || MSPI_TIMING_PSRAM_NEEDS_TUNING
}

fn mspi_timing_flash_config_clear_tuning_regs() {
    // bool control_both_mspi = true
    s_set_flash_din_mode_num(0, 0, 0); //SPI0 and SPI1 share the registers for flash din mode and num setting, so we only set SPI0's reg
    s_set_flash_extra_dummy(0, 0);

    s_set_flash_extra_dummy(1, 0);
}

fn mspi_timing_psram_config_clear_tuning_regs() {
    // bool control_both_mspi = true
    s_set_psram_din_mode_num(0, 0, 0);
    s_set_psram_extra_dummy(0, 0);
}

fn s_set_flash_din_mode_num(mspi_id: u8, din_mode: u8, din_num: u8) {
    super::mspi_timing_ll_set_flash_din_mode(mspi_id, din_mode);
    super::mspi_timing_ll_set_flash_din_num(mspi_id, din_num);
}

fn s_set_flash_extra_dummy(mspi_id: u8, extra_dummy: u8) {
    super::mspi_timing_ll_set_flash_extra_dummy(mspi_id, extra_dummy);
}

fn s_set_psram_din_mode_num(mspi_id: u8, din_mode: u8, din_num: u8) {
    super::mspi_timing_ll_set_psram_din_mode(mspi_id, din_mode);
    super::mspi_timing_ll_set_psram_din_num(mspi_id, din_num);
}

fn s_set_psram_extra_dummy(mspi_id: u8, extra_dummy: u8) {
    super::mspi_timing_ll_set_psram_extra_dummy(mspi_id, extra_dummy);
}

fn mspi_timing_flash_config_set_tuning_regs(cfg: &PsramConfig) {
    // bool control_both_mspi = true

    let timing_tuning_config = cfg.flash_tuning;

    debug!(
        "mspi_timing_flash_config_set_tuning_regs spi_din_mode {}, spi_din_num {}, extra_dummy_len {}",
        timing_tuning_config.spi_din_mode,
        timing_tuning_config.spi_din_num,
        timing_tuning_config.extra_dummy_len
    );

    // SPI0 and SPI1 share the registers for flash din mode and num setting, so we only set SPI0's
    // reg
    s_set_flash_din_mode_num(
        0,
        timing_tuning_config.spi_din_mode,
        timing_tuning_config.spi_din_num,
    );
    s_set_flash_extra_dummy(0, timing_tuning_config.extra_dummy_len);
    s_set_flash_extra_dummy(1, timing_tuning_config.extra_dummy_len);

    let (spi0_usr_dummy, spi0_extra_dummy) = super::mspi_timing_ll_get_flash_dummy(0);
    let (spi1_usr_dummy, spi1_extra_dummy) = super::mspi_timing_ll_get_flash_dummy(1);
    debug!(
        "flash, spi0_usr_dummy: {}, spi0_extra_dummy: {}, spi1_usr_dummy: {}, spi1_extra_dummy: {}",
        spi0_usr_dummy, spi0_extra_dummy, spi1_usr_dummy, spi1_extra_dummy
    );
}

fn mspi_timing_psram_config_set_tuning_regs(cfg: &PsramConfig) {
    // bool control_both_mspi = true

    let timing_tuning_config = cfg.ram_tuning;

    debug!(
        "mspi_timing_psram_config_set_tuning_regs spi_din_mode {}, spi_din_num {}, extra_dummy_len {}",
        timing_tuning_config.spi_din_mode,
        timing_tuning_config.spi_din_num,
        timing_tuning_config.extra_dummy_len
    );

    s_set_psram_din_mode_num(
        0,
        timing_tuning_config.spi_din_mode,
        timing_tuning_config.spi_din_num,
    );
    s_set_psram_extra_dummy(0, timing_tuning_config.extra_dummy_len);

    let (spi0_usr_rdummy, spi0_extra_dummy) = super::mspi_timing_ll_get_psram_dummy(0);
    debug!(
        "psram, spi0_usr_rdummy: {}, spi0_extra_dummy: {}",
        spi0_usr_rdummy, spi0_extra_dummy
    );
}
