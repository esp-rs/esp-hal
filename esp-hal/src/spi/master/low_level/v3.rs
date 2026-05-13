use enumset::EnumSet;

use super::{
    super::{Config, DataMode, SpiInterrupt},
    Driver,
};
use crate::spi::{Error, Mode};

pub(super) fn abort_transfer(driver: &Driver) {
    driver.configure_datalen(1, 1);
}

pub(super) fn init(driver: &Driver) {
    driver.regs().ctrl().modify(|_, w| {
        w.q_pol().clear_bit();
        w.d_pol().clear_bit();
        w.hold_pol().clear_bit();
        w
    });
    driver.regs().misc().write(|w| unsafe { w.bits(0) });
}

pub(super) fn init_spi_data_mode(
    driver: &Driver,
    cmd_mode: DataMode,
    address_mode: DataMode,
    data_mode: DataMode,
) -> Result<(), Error> {
    driver.regs().ctrl().modify(|_, w| {
        w.fcmd_dual().bit(cmd_mode == DataMode::Dual);
        w.fcmd_quad().bit(cmd_mode == DataMode::Quad);
        w.faddr_dual().bit(address_mode == DataMode::Dual);
        w.faddr_quad().bit(address_mode == DataMode::Quad);
        w.fread_dual().bit(data_mode == DataMode::Dual);
        w.fread_quad().bit(data_mode == DataMode::Quad)
    });
    driver.regs().user().modify(|_, w| {
        w.fwrite_dual().bit(data_mode == DataMode::Dual);
        w.fwrite_quad().bit(data_mode == DataMode::Quad)
    });
    Ok(())
}

pub(super) fn apply_config(_driver: &Driver, _config: &Config) {}

pub(super) fn set_data_mode(driver: &Driver, data_mode: Mode) {
    driver.regs().misc().modify(|_, w| {
        w.ck_idle_edge()
            .bit(matches!(data_mode, Mode::_2 | Mode::_3))
    });
    driver.regs().user().modify(|_, w| {
        w.ck_out_edge()
            .bit(matches!(data_mode, Mode::_1 | Mode::_2))
    });
}

pub(super) fn setup_full_duplex(_driver: &Driver) {}

pub(super) fn prepare_half_duplex(_driver: &Driver, _is_write: bool, dummy: u8) -> u8 {
    dummy
}

pub(super) fn setup_half_duplex(driver: &Driver) {
    driver.regs().misc().write(|w| unsafe { w.bits(0) });
}

pub(super) fn write_address(driver: &Driver, addr: u32) {
    driver
        .regs()
        .addr()
        .write(|w| unsafe { w.usr_addr_value().bits(addr) });
}

pub(super) fn enable_listen(driver: &Driver, interrupts: EnumSet<SpiInterrupt>, enable: bool) {
    driver.regs().dma_int_ena().modify(|_, w| {
        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransferDone => w.trans_done().bit(enable),
                #[cfg(spi_master_has_dma_segmented_transfer)]
                SpiInterrupt::DmaSegmentedTransferDone => w.dma_seg_trans_done().bit(enable),
                #[cfg(spi_master_has_app_interrupts)]
                SpiInterrupt::App2 => w.app2().bit(enable),
                #[cfg(spi_master_has_app_interrupts)]
                SpiInterrupt::App1 => w.app1().bit(enable),
            };
        }
        w
    });
}

pub(super) fn interrupts(driver: &Driver) -> EnumSet<SpiInterrupt> {
    let ints = driver.regs().dma_int_raw().read();
    let mut res = EnumSet::new();

    if ints.trans_done().bit() {
        res.insert(SpiInterrupt::TransferDone);
    }
    #[cfg(spi_master_has_dma_segmented_transfer)]
    if ints.dma_seg_trans_done().bit() {
        res.insert(SpiInterrupt::DmaSegmentedTransferDone);
    }
    #[cfg(spi_master_has_app_interrupts)]
    if ints.app2().bit() {
        res.insert(SpiInterrupt::App2);
    }
    #[cfg(spi_master_has_app_interrupts)]
    if ints.app1().bit() {
        res.insert(SpiInterrupt::App1);
    }

    res
}

pub(super) fn clear_interrupts(driver: &Driver, interrupts: EnumSet<SpiInterrupt>) {
    driver.regs().dma_int_clr().write(|w| {
        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransferDone => w.trans_done().clear_bit_by_one(),
                #[cfg(spi_master_has_dma_segmented_transfer)]
                SpiInterrupt::DmaSegmentedTransferDone => w.dma_seg_trans_done().clear_bit_by_one(),
                #[cfg(spi_master_has_app_interrupts)]
                SpiInterrupt::App2 => w.app2().clear_bit_by_one(),
                #[cfg(spi_master_has_app_interrupts)]
                SpiInterrupt::App1 => w.app1().clear_bit_by_one(),
            };
        }
        w
    });
}

pub(super) fn configure_datalen(driver: &Driver, rx_len: u32, tx_len: u32) {
    driver
        .regs()
        .ms_dlen()
        .write(|w| unsafe { w.ms_data_bitlen().bits(rx_len.max(tx_len)) });
}
