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
pub(super) fn sync_regs(_register_block: &RegisterBlock) {
    #[cfg(not(any(esp32, esp32s2)))]
    {
        let update_reg = _register_block.id();

        update_reg.modify(|_, w| w.reg_update().set_bit());

        while update_reg.read().reg_update().bit_is_set() {
            core::hint::spin_loop();
        }
    }
}

pub(super) fn set_rx_timeout(
    info: &Info,
    timeout: Option<u8>,
    _symbol_len: u8,
) -> Result<(), ConfigError> {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            const MAX_THRHD: u8 = 0x7F; // 7 bits
        } else {
            const MAX_THRHD: u16 = 0x3FF; // 10 bits
        }
    }

    if let Some(timeout) = timeout {
        #[cfg(esp32)]
        let timeout_reg = timeout;
        #[cfg(not(esp32))]
        let timeout_reg = timeout as u16 * _symbol_len as u16;

        if timeout_reg > MAX_THRHD {
            return Err(ConfigError::TimeoutTooLong);
        }

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let reg_thrhd = info.regs().conf1();
            } else {
                let reg_thrhd = info.regs().mem_conf();
            }
        }
        reg_thrhd.modify(|_, w| unsafe { w.rx_tout_thrhd().bits(timeout_reg) });
    }

    info.regs()
        .conf1()
        .modify(|_, w| w.rx_tout_en().bit(timeout.is_some()));

    info.sync_regs();

    Ok(())
}

pub(super) fn rx_timeout_enabled(info: &Info) -> bool {
    info.regs().conf1().read().rx_tout_en().bit_is_set()
}

pub(super) fn is_tx_idle(info: &Info) -> bool {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            let status = info.regs().status();
        } else {
            let status = info.regs().fsm_status();
        }
    }

    status.read().st_utx_out().bits() == 0x0
}

pub(super) fn change_stop_bits(info: &Info, stop_bits: StopBits) {
    #[cfg(esp32)]
    {
        // workaround for hardware issue, when UART stop bit set as 2-bit mode.
        if stop_bits == StopBits::_2 {
            info.regs()
                .rs485_conf()
                .modify(|_, w| w.dl1_en().bit(stop_bits == StopBits::_2));

            info.regs()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
        }
    }

    #[cfg(not(esp32))]
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
                .flow_conf()
                .modify(|_, w| w.xonoff_del().set_bit().sw_flow_con_en().set_bit());

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    info.regs().swfc_conf().modify(|_, w| unsafe {
                        w.xon_threshold().bits(xon_threshold).xoff_threshold().bits(xoff_threshold)
                    });
                    info.regs().swfc_conf().modify(|_, w| unsafe {
                        w.xon_char().bits(xon_char).xoff_char().bits(xoff_char)
                    });
                } else {
                    info.regs()
                        .swfc_conf1()
                        .modify(|_, w| unsafe { w.xon_threshold().bits(xon_threshold as u16) });
                    info.regs()
                        .swfc_conf0()
                        .modify(|_, w| unsafe { w.xoff_threshold().bits(xoff_threshold as u16) });
                    info.regs()
                        .swfc_conf1()
                        .modify(|_, w| unsafe { w.xon_char().bits(xon_char) });
                    info.regs()
                        .swfc_conf0()
                        .modify(|_, w| unsafe { w.xoff_char().bits(xoff_char) });
                }
            }
        }
        SwFlowControl::Disabled => {
            let reg = info.regs().flow_conf();
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
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                info.regs()
                    .conf1()
                    .modify(|_, w| unsafe { w.rx_flow_thrhd().bits(threshold) });
            } else {
                info.regs()
                    .mem_conf()
                    .modify(|_, w| unsafe { w.rx_flow_thrhd().bits(threshold as u16) });
            }
        }
    }

    info.regs()
        .conf1()
        .modify(|_, w| w.rx_flow_en().bit(enable));
}

pub(super) fn current_symbol_length(info: &Info) -> u8 {
    let conf0 = info.regs().conf0().read();
    let data_bits = conf0.bit_num().bits() + 5; // 5 data bits are encoded as variant 0
    let parity = conf0.parity_en().bit() as u8;
    let mut stop_bits = conf0.stop_bit_num().bits();

    match stop_bits {
        1 => {
            // workaround for hardware issue, when UART stop bit set as 2-bit mode.
            #[cfg(esp32)]
            if info.regs().rs485_conf().read().dl1_en().bit_is_set() {
                stop_bits = 2;
            }
        }
        // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
        _ => stop_bits = 2,
    }

    1 + data_bits + parity + stop_bits
}

pub(super) fn read_next_from_fifo(info: &Info) -> u8 {
    fn access_fifo_register<R>(f: impl Fn() -> R) -> R {
        // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/cpu-subsequent-access-halted-when-get-interrupted.html
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                crate::interrupt::free(f)
            } else {
                f()
            }
        }
    }

    let fifo_reg = info.regs().fifo();
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
            let fifo_reg = unsafe {
                &*fifo_reg.as_ptr().cast::<u8>().add(0x20C00000).cast::<crate::pac::uart0::FIFO>()
            };
        }
    }

    access_fifo_register(|| fifo_reg.read().rxfifo_rd_byte().bits())
}

#[allow(clippy::unnecessary_cast)]
pub(super) fn rx_fifo_count(info: &Info) -> u16 {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            let fifo_cnt = info.regs().status().read().rxfifo_cnt().bits();

            // Calculate the real count based on the FIFO read and write offset address:
            // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/uart-fifo-cnt-indicates-data-length-incorrectly.html
            let status = info.regs().mem_rx_status().read();
            let rd_addr: u16 = status.mem_rx_rd_addr().bits();
            let wr_addr: u16 = status.mem_rx_wr_addr().bits();

            if wr_addr > rd_addr {
                wr_addr - rd_addr
            } else if wr_addr < rd_addr {
                (wr_addr + Info::UART_FIFO_SIZE) - rd_addr
            } else if fifo_cnt > 0 {
                Info::UART_FIFO_SIZE
            } else {
                0
            }
        } else {
            info.regs().status().read().rxfifo_cnt().bits() as u16
        }
    }
}
