//! PDM clock calculations.

use super::{PdmDownsampleRate, PdmError, PdmRxClockConfig, PdmTxClockConfig};
use crate::{i2s::master::private::I2sClockDividers, soc, time::Rate};

pub(crate) const PDM_BCK_FACTOR: u32 = 64;
pub(crate) const PDM_TX_BCLK_DIV_MIN: u32 = 8;
pub(crate) const PDM_RX_BCLK_DIV_MIN: u32 = 8;

pub(crate) struct PdmTxClockResult {
    pub dividers: I2sClockDividers,
    pub over_sample_ratio: u32,
}

pub(crate) struct PdmRxClockResult {
    pub dividers: I2sClockDividers,
}

fn calculate_mclk_dividers(sclk: u32, mclk: u32) -> Result<I2sClockDividers, PdmError> {
    // Require `sclk > mclk * 1.99` — use integer math to avoid soft-float.
    if (sclk as u64) * 100 <= (mclk as u64) * 199 {
        return Err(PdmError::InvalidClock);
    }

    let dividers = I2sClockDividers::from_frequencies(sclk, mclk, 0);

    if dividers.mclk_divider >= 256 {
        return Err(PdmError::InvalidClock);
    }

    Ok(dividers)
}

pub(crate) fn calculate_tx_clock(
    clk: &PdmTxClockConfig,
    pcm: bool,
) -> Result<PdmTxClockResult, PdmError> {
    if clk.up_sample_fs > 480 {
        return Err(PdmError::InvalidClock);
    }

    let rate = clk.sample_rate.as_hz();
    let bclk_div = clk.bclk_div.max(PDM_TX_BCLK_DIV_MIN);

    let (bclk, over_sample_ratio) = if pcm {
        let over_sample_ratio = clk.up_sample_fp / clk.up_sample_fs.max(1);
        (rate * PDM_BCK_FACTOR * over_sample_ratio, over_sample_ratio)
    } else {
        (rate * 2, 0)
    };

    let mclk = bclk * bclk_div;
    let sclk = soc::i2s_sclk_frequency();
    let mut dividers = calculate_mclk_dividers(sclk, mclk)?;
    dividers.bclk_divider = bclk_div;

    Ok(PdmTxClockResult {
        dividers,
        over_sample_ratio,
    })
}

const PDM_RX_CLK_LIMIT_COEFF: u32 = 128;

pub(crate) fn calculate_rx_clock(
    clk: &PdmRxClockConfig,
    pcm: bool,
    slot_mask: u16,
) -> Result<PdmRxClockResult, PdmError> {
    let rate = clk.sample_rate.as_hz();
    let dn_sample_factor = PDM_BCK_FACTOR * clk.downsample_rate.factor();
    let slot_num = slot_mask.count_ones().max(1);

    let bclk = if pcm {
        rate * dn_sample_factor
    } else {
        rate * 2
    };

    let bclk_limit = (PDM_RX_CLK_LIMIT_COEFF * slot_num).div_ceil(dn_sample_factor);
    let bclk_div = clk.bclk_div.max(PDM_RX_BCLK_DIV_MIN).max(bclk_limit);

    let mclk = bclk * bclk_div;
    let sclk = soc::i2s_sclk_frequency();
    let mut dividers = calculate_mclk_dividers(sclk, mclk)?;
    dividers.bclk_divider = bclk_div;

    Ok(PdmRxClockResult { dividers })
}

impl PdmDownsampleRate {
    fn factor(self) -> u32 {
        match self {
            PdmDownsampleRate::Dsr8s => 1,
            PdmDownsampleRate::Dsr16s => 2,
        }
    }
}

impl PdmTxClockConfig {
    /// Default codec-line PDM TX clock (`I2S_PDM_TX_CLK_DEFAULT_CONFIG`)
    pub fn codec_default(sample_rate: Rate) -> Self {
        Self {
            sample_rate,
            up_sample_fp: 960,
            up_sample_fs: 480,
            bclk_div: PDM_TX_BCLK_DIV_MIN,
        }
    }

    /// Default DAC-line PDM TX clock (`I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG`)
    pub fn dac_default(sample_rate: Rate) -> Self {
        Self {
            sample_rate,
            up_sample_fp: 960,
            up_sample_fs: sample_rate.as_hz() / 100,
            bclk_div: 13,
        }
    }
}

impl PdmRxClockConfig {
    /// Default PDM RX clock (`I2S_PDM_RX_CLK_DEFAULT_CONFIG`)
    pub fn default(sample_rate: Rate) -> Self {
        Self {
            sample_rate,
            downsample_rate: PdmDownsampleRate::Dsr8s,
            bclk_div: PDM_RX_BCLK_DIV_MIN,
        }
    }
}
