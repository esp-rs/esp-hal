use super::{
    ConfigError,
    CtsConfig,
    HwFlowControl,
    Info,
    RegisterBlock,
    RtsConfig,
    StopBits,
    SwFlowControl,
};

#[inline(always)]
pub(super) fn sync_regs(register_block: &RegisterBlock) {
    let update_reg = register_block.reg_update();

    update_reg.modify(|_, w| w.reg_update().set_bit());

    while update_reg.read().reg_update().bit_is_set() {
        core::hint::spin_loop();
    }
}

pub(super) fn set_rx_timeout(
    info: &Info,
    timeout: Option<u8>,
    symbol_len: u8,
) -> Result<(), ConfigError> {
    const MAX_THRHD: u16 = 0x3FF; // 10 bits

    let register_block = info.regs();

    if let Some(timeout) = timeout {
        let timeout_reg = timeout as u16 * symbol_len as u16;

        if timeout_reg > MAX_THRHD {
            return Err(ConfigError::TimeoutTooLong);
        }

        register_block
            .tout_conf()
            .modify(|_, w| unsafe { w.rx_tout_thrhd().bits(timeout_reg) });
    }

    register_block
        .tout_conf()
        .modify(|_, w| w.rx_tout_en().bit(timeout.is_some()));

    info.sync_regs();

    Ok(())
}

pub(super) fn rx_timeout_enabled(info: &Info) -> bool {
    info.regs().tout_conf().read().rx_tout_en().bit_is_set()
}

pub(super) fn is_tx_idle(info: &Info) -> bool {
    info.regs().fsm_status().read().st_utx_out().bits() == 0x0
}

pub(super) fn change_stop_bits(info: &Info, stop_bits: StopBits) {
    info.regs()
        .conf0()
        .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8 + 1) });
}

pub(super) fn change_flow_control(
    info: &Info,
    sw_flow_ctrl: SwFlowControl,
    hw_flow_ctrl: HwFlowControl,
) {
    match sw_flow_ctrl {
        SwFlowControl::Enabled {
            xon_char,
            xoff_char,
            xon_threshold,
            xoff_threshold,
        } => {
            info.regs()
                .swfc_conf0()
                .modify(|_, w| w.xonoff_del().set_bit().sw_flow_con_en().set_bit());
            info.regs().swfc_conf1().modify(|_, w| unsafe {
                w.xon_threshold()
                    .bits(xon_threshold)
                    .xoff_threshold()
                    .bits(xoff_threshold)
            });
            info.regs()
                .swfc_conf0()
                .modify(|_, w| unsafe { w.xon_char().bits(xon_char).xoff_char().bits(xoff_char) });
        }
        SwFlowControl::Disabled => {
            let reg = info.regs().swfc_conf0();
            reg.modify(|_, w| w.sw_flow_con_en().clear_bit());
            reg.modify(|_, w| w.xonoff_del().clear_bit());
        }
    }

    info.regs().conf0().modify(|_, w| {
        w.tx_flow_en()
            .bit(matches!(hw_flow_ctrl.cts, CtsConfig::Enabled))
    });

    match hw_flow_ctrl.rts {
        RtsConfig::Enabled(threshold) => configure_rts_flow_ctrl(info, true, Some(threshold)),
        RtsConfig::Disabled => configure_rts_flow_ctrl(info, false, None),
    }

    sync_regs(info.regs());
}

fn configure_rts_flow_ctrl(info: &Info, enable: bool, threshold: Option<u8>) {
    if let Some(threshold) = threshold {
        info.regs()
            .hwfc_conf()
            .modify(|_, w| unsafe { w.rx_flow_thrhd().bits(threshold) });
    }

    info.regs()
        .hwfc_conf()
        .modify(|_, w| w.rx_flow_en().bit(enable));
}

pub(super) fn current_symbol_length(info: &Info) -> u8 {
    let conf0 = info.regs().conf0().read();
    let data_bits = conf0.bit_num().bits() + 5; // 5 data bits are encoded as variant 0
    let parity = conf0.parity_en().bit() as u8;
    let mut stop_bits = conf0.stop_bit_num().bits();

    match stop_bits {
        1 => {}
        // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
        _ => stop_bits = 2,
    }

    1 + data_bits + parity + stop_bits
}

pub(super) fn read_next_from_fifo(info: &Info) -> u8 {
    info.regs().fifo().read().rxfifo_rd_byte().bits()
}

#[allow(clippy::unnecessary_cast)]
pub(super) fn rx_fifo_count(info: &Info) -> u16 {
    info.regs().status().read().rxfifo_cnt().bits() as u16
}
