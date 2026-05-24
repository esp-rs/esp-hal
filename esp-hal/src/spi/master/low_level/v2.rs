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
        w.wp().clear_bit();
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
    driver.regs().slave().modify(|_, w| {
        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransferDone => w.int_trans_done_en().bit(enable),
                SpiInterrupt::DmaSegmentedTransferDone => w.int_dma_seg_trans_en().bit(enable),
            };
        }
        w
    });
}

pub(super) fn interrupts(driver: &Driver) -> EnumSet<SpiInterrupt> {
    let mut res = EnumSet::new();
    if driver.regs().slave().read().trans_done().bit() {
        res.insert(SpiInterrupt::TransferDone);
    }
    if driver.regs().hold().read().dma_seg_trans_done().bit() {
        res.insert(SpiInterrupt::DmaSegmentedTransferDone);
    }
    res
}

pub(super) fn clear_interrupts(driver: &Driver, interrupts: EnumSet<SpiInterrupt>) {
    for interrupt in interrupts {
        match interrupt {
            SpiInterrupt::TransferDone => {
                driver
                    .regs()
                    .slave()
                    .modify(|_, w| w.trans_done().clear_bit());
            }
            SpiInterrupt::DmaSegmentedTransferDone => {
                driver
                    .regs()
                    .hold()
                    .modify(|_, w| w.dma_seg_trans_done().clear_bit());
            }
        }
    }
}

pub(super) fn configure_datalen(driver: &Driver, rx_len: u32, tx_len: u32) {
    driver
        .regs()
        .mosi_dlen()
        .write(|w| unsafe { w.usr_mosi_dbitlen().bits(tx_len) });

    driver
        .regs()
        .miso_dlen()
        .write(|w| unsafe { w.usr_miso_dbitlen().bits(rx_len) });
}
