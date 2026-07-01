//! PDM register programming for I2S hardware after ESP32 (v1).
//!
//! This module is used for all targets with `i2s_version` `"2"` or `"3"` (see
//! `i2s/pdm/mod.rs`). The filename reflects the first revision in this family;
//! chip-specific differences are handled via `#[cfg(i2s_version = "...")]`.

use super::{
    PdmConfig,
    PdmDataFormat,
    PdmSlotMode,
    PdmTxLineMode,
    clock,
    hp_filter::cut_off_coefficients,
};
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

    if let Some(tx) = &config.tx {
        configure_tx(i2s, tx)?;
    }
    if let Some(rx) = &config.rx {
        configure_rx(i2s, rx)?;
    }
    Ok(())
}

fn configure_tx<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &super::PdmTxConfig,
) -> Result<(), ConfigError> {
    config.validate()?;

    let pcm = config.slot.data_format == PdmDataFormat::Pcm;
    let clock = clock::calculate_tx_clock(&config.clock, pcm)?;

    let regs = i2s0_regs();
    let is_mono = config.slot.slot_mode == PdmSlotMode::Mono;

    // Slot/filter setup first (IDF: `i2s_pdm_tx_set_slot` before `i2s_pdm_tx_set_clock`).
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L231-L235>
    regs.tx_conf().modify(|_, w| w.tx_reset().set_bit());
    regs.tx_conf().modify(|_, w| {
        w.tx_slave_mod().clear_bit();
        w.tx_tdm_en().clear_bit();
        w.tx_pcm_bypass().clear_bit();
        // `i2s_ll_tx_pdm_dma_take_mode(..., is_mono, true)`.
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/i2s_hal.c#L218> and
        // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L1139-L1143>
        w.tx_mono().bit(is_mono);
        w.tx_mono_fst_vld().set_bit();
        #[cfg(i2s_version = "3")]
        w.tx_msb_shift().clear_bit();
        w.tx_ws_idle_pol().clear_bit()
    });
    regs.tx_conf().modify(|_, w| unsafe {
        // `i2s_ll_tx_pdm_slot_mode(..., is_mono, false, I2S_PDM_SLOT_BOTH)`.
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/i2s_hal.c#L219> and
        // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L1169-L1174>
        w.tx_chan_mod().bits(if is_mono { 3 } else { 0 })
    });

    #[cfg(i2s_version = "2")]
    regs.tx_conf1().modify(|_, w| w.tx_msb_shift().clear_bit());

    let slot_bits = if is_mono { 16 } else { 32 };
    regs.tx_conf1().modify(|_, w| unsafe {
        w.tx_bits_mod().bits(15);
        w.tx_tdm_chan_bits().bits(slot_bits - 1);
        w.tx_half_sample_bits().bits(15);
        w.tx_tdm_ws_width().bits(0)
    });

    if pcm {
        let freq_x10 = (config.slot.hp_cut_off_freq_hz * 10.0) as u32;
        let (param0, param5) = cut_off_coefficients(freq_x10);

        // Set fp/fs before clock enable (IDF: `i2s_pdm_tx_calculate_clock`).
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L63-L65>
        regs.tx_pcm2pdm_conf1().modify(|_, w| unsafe {
            w.tx_pdm_fp().bits(config.clock.up_sample_fp as u16);
            w.tx_pdm_fs().bits(config.clock.up_sample_fs as u16);
            w.tx_iir_hp_mult12_0().bits(param0 as u8);
            w.tx_iir_hp_mult12_5().bits(param5 as u8)
        });

        regs.tx_pcm2pdm_conf().modify(|_, w| unsafe {
            w.tx_pdm_sinc_osr2().bits(clock.over_sample_ratio as u8);
            w.tx_pdm_prescale().bits(config.slot.sd_prescale);
            w.tx_pdm_sigmadelta_in_shift()
                .bits(config.slot.sd_scale.to_register());
            w.tx_pdm_hp_in_shift()
                .bits(config.slot.hp_scale.to_register());
            w.tx_pdm_lp_in_shift()
                .bits(config.slot.lp_scale.to_register());
            w.tx_pdm_sinc_in_shift()
                .bits(config.slot.sinc_scale.to_register());
            w.tx_pdm_dac_mode_en()
                .bit(config.slot.line_mode != PdmTxLineMode::OneLineCodec);
            w.tx_pdm_dac_2out_en()
                .bit(config.slot.line_mode == PdmTxLineMode::TwoLineDac);
            w.tx_pdm_sigmadelta_dither().bit(config.slot.sd_dither != 0);
            w.tx_pdm_sigmadelta_dither2()
                .bit(config.slot.sd_dither2 != 0);
            w.tx_pdm_hp_bypass().bit(!config.slot.hp_en)
        });
    }

    // IDF init order: slot → gpio (`mclk_bind`) → clock.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L231-L235> and
    // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L200-L202>
    #[cfg(not(i2s_clock_configured_by_pcr))]
    regs.rx_clkm_conf().modify(|_, w| w.mclk_sel().clear_bit());
    #[cfg(i2s_clock_configured_by_pcr)]
    {
        use crate::peripherals::PCR;
        PCR::regs()
            .i2s_rx_clkm_conf()
            .modify(|_, w| w.i2s_mclk_sel().clear_bit());
    }

    set_pdm_tx_clock(i2s, &clock);

    // `i2s_ll_tx_enable_pdm`: PDM on, TDM off, PCM2PDM on for PCM data.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L801-L806>
    regs.tx_conf().modify(|_, w| {
        w.tx_pdm_en().set_bit();
        w.tx_tdm_en().clear_bit()
    });
    if pcm {
        regs.tx_pcm2pdm_conf()
            .modify(|_, w| w.pcm2pdm_conv_en().set_bit());
    } else {
        regs.tx_pcm2pdm_conf()
            .modify(|_, w| w.pcm2pdm_conv_en().clear_bit());
    }

    Ok(())
}

/// Expand a stereo slot mask to include both slots on every active PDM line.
fn stereo_pdm_rx_slot_mask(slot_mask: u16) -> u16 {
    let mut stereo_mask = 0u16;
    for i in 0..8 {
        let pair = 0b11u16 << (2 * i);
        if slot_mask & pair != 0 {
            stereo_mask |= pair;
        }
    }
    stereo_mask
}

fn configure_rx<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &super::PdmRxConfig,
) -> Result<(), ConfigError> {
    config.validate()?;

    let pcm = config.slot.data_format == PdmDataFormat::Pcm;
    let mut slot_mask = config.slot.slot_mask.bits();
    if config.slot.slot_mode == PdmSlotMode::Stereo {
        slot_mask = stereo_pdm_rx_slot_mask(slot_mask);
    }

    let clock = clock::calculate_rx_clock(&config.clock, pcm, slot_mask)?;

    let regs = i2s0_regs();

    // IDF: `i2s_ll_rx_reset` / `i2s_ll_rx_reset_fifo` — WT bits, write 1 to trigger.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L199-L224>
    regs.rx_conf().modify(|_, w| {
        w.rx_reset().set_bit();
        w.rx_fifo_reset().set_bit()
    });
    regs.rx_conf().modify(|_, w| {
        w.rx_slave_mod().clear_bit();
        w.rx_pcm_bypass().set_bit();
        // IDF `i2s_hal_pdm_set_rx_slot` (HW v2): mono mode stays disabled for PDM RX.
        // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/i2s_hal.c#L268>
        w.rx_mono().clear_bit();
        w
    });
    #[cfg(i2s_version = "3")]
    regs.rx_conf().modify(|_, w| w.rx_msb_shift().clear_bit());
    #[cfg(i2s_version = "2")]
    regs.rx_conf()
        .modify(|_, w| unsafe { w.rx_stop_mode().bits(2) });

    // IDF init order: slot → gpio (`mclk_bind`) → clock.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L582-L585> and
    // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L551-L553>
    #[cfg(i2s_version = "2")]
    regs.rx_conf1().modify(|_, w| w.rx_msb_shift().clear_bit());

    regs.rx_conf1().modify(|_, w| unsafe {
        w.rx_bits_mod().bits(15);
        w.rx_tdm_chan_bits().bits(15);
        w.rx_half_sample_bits().bits(15)
    });

    // PDM always uses a 2-slot frame; mono DMA expects one active slot only.
    // IDF `i2s_ll_rx_select_std_slot` sets `tot_chan_num = 1` before the mask.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L689>
    regs.rx_tdm_ctrl()
        .modify(|_, w| unsafe { w.rx_tdm_tot_chan_num().bits(1) });
    regs.rx_tdm_ctrl()
        .modify(|r, w| unsafe { w.bits((r.bits() & 0xFFFF0000) | u32::from(slot_mask)) });

    #[cfg(not(i2s_clock_configured_by_pcr))]
    regs.rx_clkm_conf().modify(|_, w| w.mclk_sel().set_bit());
    #[cfg(i2s_clock_configured_by_pcr)]
    {
        use crate::peripherals::PCR;
        PCR::regs()
            .i2s_rx_clkm_conf()
            .modify(|_, w| w.i2s_mclk_sel().set_bit());
    }
    i2s.set_rx_clock(clock.dividers);

    #[cfg(all(i2s_supports_pdm2pcm, esp32p4))]
    {
        let dsr16 = config.clock.downsample_rate == super::PdmDownsampleRate::Dsr16s;
        let freq_x10 = (config.slot.hp_cut_off_freq_hz * 10.0) as u32;
        let (param0, param5) = cut_off_coefficients(freq_x10);
        regs.rx_pdm2pcm_conf().modify(|_, w| {
            w.rx_pdm2pcm_en().bit(pcm);
            w.rx_pdm_sinc_dsr_16_en().bit(dsr16);
            w.rx_pdm_hp_bypass().bit(!config.slot.hp_en);
            w.rx_pdm2pcm_amplify_num()
                .bits(config.slot.amplify_num.max(1).min(15) as u8);
            w.rx_iir_hp_mult12_0().bits(param0 as u8);
            w.rx_iir_hp_mult12_5().bits(param5 as u8)
        });
    }

    // IDF init order ends with `i2s_ll_rx_enable_pdm` after slot, GPIO, and clock.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L590> and
    // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L814-L818>
    regs.rx_conf().modify(|_, w| {
        w.rx_pdm_en().set_bit();
        w.rx_tdm_en().clear_bit();
        #[cfg(all(i2s_supports_pdm2pcm, not(esp32p4)))]
        {
            w.rx_pdm2pcm_en().bit(pcm);
            w.rx_pdm_sinc_dsr_16_en()
                .bit(config.clock.downsample_rate == super::PdmDownsampleRate::Dsr16s);
        }
        w
    });

    Ok(())
}

#[cfg(not(i2s_clock_configured_by_pcr))]
fn set_pdm_tx_clock<I: RegisterAccessPrivate + ?Sized>(i2s: &I, clock: &clock::PdmTxClockResult) {
    set_pdm_tx_clock_common(i2s, clock);
    apply_tx_pdm_clock_workaround(clock.dividers.mclk_divider);
}

#[cfg(i2s_clock_configured_by_pcr)]
fn set_pdm_tx_clock<I: RegisterAccessPrivate + ?Sized>(i2s: &I, clock: &clock::PdmTxClockResult) {
    set_pdm_tx_clock_common(i2s, clock);
}

fn set_pdm_tx_clock_common<I: RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    clock: &clock::PdmTxClockResult,
) {
    // IDF: precise MCLK first; non-PCR chips then apply the PDM raw-div workaround.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L100-L102> and
    // <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_hal_i2s/esp32c3/include/hal/i2s_ll.h#L328-L344>
    use crate::i2s::master::private::I2sClockDividers;
    let dividers = I2sClockDividers {
        mclk_divider: clock.dividers.mclk_divider,
        bclk_divider: clock.dividers.bclk_divider,
        denominator: clock.dividers.denominator,
        numerator: clock.dividers.numerator,
    };
    i2s.set_tx_clock(dividers);
}

#[cfg(not(i2s_clock_configured_by_pcr))]
fn apply_tx_pdm_clock_workaround(mclk_div: u32) {
    let regs = i2s0_regs();
    regs.tx_clkm_conf()
        .modify(|_, w| unsafe { w.tx_clkm_div_num().bits(2) });
    regs.tx_clkm_div_conf().modify(|_, w| unsafe {
        w.tx_clkm_div_yn1().clear_bit();
        w.tx_clkm_div_y().bits(1);
        w.tx_clkm_div_z().bits(0);
        w.tx_clkm_div_x().bits(0)
    });
    regs.tx_clkm_div_conf().modify(|_, w| unsafe {
        w.tx_clkm_div_yn1().clear_bit();
        w.tx_clkm_div_y().bits(1);
        w.tx_clkm_div_z().bits(0);
        w.tx_clkm_div_x().bits(1)
    });
    regs.tx_clkm_conf()
        .modify(|_, w| unsafe { w.tx_clkm_div_num().bits(mclk_div as u8) });
}
