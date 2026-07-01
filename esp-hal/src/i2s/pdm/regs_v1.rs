//! PDM register programming for ESP32 (I2S hardware v1).

use super::{PdmConfig, PdmDataFormat, PdmSlotMode, clock};
use crate::i2s::master::{ConfigError, private::RegisterAccessPrivate};
#[cfg(soc_has_i2s0)]
use crate::pac::i2s0::RegisterBlock as I2s0RegisterBlock;

#[cfg(soc_has_i2s0)]
fn i2s0_regs() -> &'static I2s0RegisterBlock {
    use crate::peripherals::I2S0;
    unsafe { &*I2S0::PTR.cast::<I2s0RegisterBlock>() }
}

pub(crate) fn configure_pdm<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &PdmConfig,
) -> Result<(), ConfigError> {
    if i2s.peripheral() != crate::system::Peripheral::I2s0 {
        return Err(ConfigError::PdmUnsupportedInstance);
    }

    i2s0_regs().conf2().modify(|_, w| {
        w.camera_en().clear_bit();
        w.lcd_en().clear_bit()
    });

    if let Some(tx) = &config.tx {
        configure_tx(i2s, tx)?;
    }
    if let Some(rx) = &config.rx {
        configure_rx(i2s, rx)?;
    }

    i2s0_regs().fifo_conf().modify(|_, w| w.dscr_en().set_bit());
    i2s0_regs().pd_conf().modify(|_, w| {
        w.fifo_force_pu().set_bit();
        w.fifo_force_pd().clear_bit()
    });

    Ok(())
}

fn configure_tx<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &super::PdmTxConfig,
) -> Result<(), ConfigError> {
    config.validate()?;

    let pcm = config.slot.data_format == PdmDataFormat::Pcm;
    let clock = clock::calculate_tx_clock(&config.clock, pcm)?;
    i2s.set_clock(clock.dividers);

    let is_mono = config.slot.slot_mode == PdmSlotMode::Mono;
    let regs = i2s0_regs();

    regs.conf().modify(|_, w| {
        w.tx_msb_shift().clear_bit();
        w.tx_mono().bit(is_mono);
        w.tx_right_first().clear_bit();
        w.tx_msb_right().clear_bit()
    });

    regs.sample_rate_conf()
        .modify(|_, w| unsafe { w.tx_bits_mod().bits(16) });

    regs.fifo_conf().modify(|_, w| unsafe {
        w.tx_fifo_mod().bits(if is_mono { 1 } else { 0 });
        w.tx_fifo_mod_force_en().set_bit()
    });

    if pcm {
        regs.pdm_conf().modify(|_, w| unsafe {
            w.tx_pdm_prescale().bits(config.slot.sd_prescale);
            w.tx_pdm_sigmadelta_in_shift()
                .bits(config.slot.sd_scale.to_register());
            w.tx_pdm_hp_in_shift()
                .bits(config.slot.hp_scale.to_register());
            w.tx_pdm_lp_in_shift()
                .bits(config.slot.lp_scale.to_register());
            w.tx_pdm_sinc_in_shift()
                .bits(config.slot.sinc_scale.to_register())
        });

        if clock.over_sample_ratio > 0 {
            regs.pdm_conf()
                .modify(|_, w| unsafe { w.tx_pdm_sinc_osr2().bits(clock.over_sample_ratio as u8) });
        }

        regs.pdm_freq_conf().modify(|_, w| unsafe {
            w.tx_pdm_fp().bits(config.clock.up_sample_fp as u16);
            w.tx_pdm_fs().bits(config.clock.up_sample_fs as u16)
        });
    }

    let slot_mask = config.slot.slot_mask.bits();
    regs.conf_chan().modify(|_, w| unsafe {
        // IDF: `i2s_ll_tx_select_pdm_slot` (mono uses 3/4/1, stereo uses 1/2/0).
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32/include/hal/i2s_ll.h#L790-L820>
        let chan_mod = if is_mono {
            match slot_mask {
                0b01 => 3u8,
                0b10 => 4u8,
                _ => 1u8,
            }
        } else {
            match slot_mask {
                0b01 => 1u8,
                0b10 => 2u8,
                _ => 0u8,
            }
        };
        w.tx_chan_mod().bits(chan_mod)
    });

    regs.pdm_conf().modify(|_, w| {
        w.tx_pdm_en().set_bit();
        w.pcm2pdm_conv_en().bit(pcm)
    });

    Ok(())
}

fn configure_rx<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &super::PdmRxConfig,
) -> Result<(), ConfigError> {
    config.validate()?;

    let pcm = config.slot.data_format == PdmDataFormat::Pcm;
    #[cfg(i2s_supports_pdm2pcm)]
    let pdm2pcm = pcm;
    #[cfg(not(i2s_supports_pdm2pcm))]
    let pdm2pcm = false;

    let slot_mask = config.slot.slot_mask.bits();
    let clock = clock::calculate_rx_clock(&config.clock, pcm, slot_mask)?;
    i2s.set_clock(clock.dividers);

    let is_mono = config.slot.slot_mode == PdmSlotMode::Mono;
    let regs = i2s0_regs();

    regs.conf().modify(|_, w| {
        w.rx_msb_shift().clear_bit();
        w.rx_mono().bit(is_mono);
        w.rx_right_first().clear_bit();
        w.rx_msb_right().clear_bit()
    });

    regs.sample_rate_conf()
        .modify(|_, w| unsafe { w.rx_bits_mod().bits(16) });

    regs.fifo_conf().modify(|_, w| unsafe {
        w.rx_fifo_mod().bits(if is_mono { 1 } else { 0 });
        w.rx_fifo_mod_force_en().set_bit()
    });

    regs.conf_chan().modify(|_, w| unsafe {
        // IDF: `i2s_ll_rx_select_pdm_slot` (always 1/2/0, not TDM mono 3/4/0).
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32/include/hal/i2s_ll.h#L827-L837>
        let chan_mod = match slot_mask {
            0b01 => 1u8,
            0b10 => 2u8,
            _ => 0u8,
        };
        w.rx_chan_mod().bits(chan_mod)
    });

    regs.pdm_conf().modify(|_, w| {
        w.rx_pdm_en().set_bit();
        w.pdm2pcm_conv_en().bit(pdm2pcm);
        w.rx_pdm_sinc_dsr_16_en()
            .bit(config.clock.downsample_rate == super::PdmDownsampleRate::Dsr16s)
    });

    Ok(())
}
