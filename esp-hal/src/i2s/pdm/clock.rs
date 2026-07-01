//! PDM clock calculations ported from ESP-IDF `i2s_pdm.c`
//! (<https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c>).

use super::{PdmDownsampleRate, PdmRxClockConfig, PdmTxClockConfig};
use crate::{
    i2s::master::{ConfigError, I2S_LL_MCLK_DIVIDER_MAX, private::I2sClockDividers},
    soc,
    time::Rate,
};

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

fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        (a, b) = (b, a % b);
    }
    a
}

fn calculate_mclk_dividers(sclk: u32, mclk: u32) -> Result<I2sClockDividers, ConfigError> {
    // IDF: `(float)sclk > mclk * 1.99` — use integer math to avoid soft-float.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L59>
    if (sclk as u64) * 100 <= (mclk as u64) * 199 {
        return Err(ConfigError::InvalidPdmClock);
    }

    let mut mclk_divider = sclk / mclk;
    if mclk_divider >= 256 {
        return Err(ConfigError::InvalidPdmClock);
    }

    let freq_diff = sclk.abs_diff(mclk * mclk_divider);
    let mut denominator = 0u32;
    let mut numerator = 0u32;

    if freq_diff != 0 {
        let decimal = freq_diff as u64 * 10000 / mclk as u64;
        if decimal > 1250000 / 126 {
            mclk_divider += 1;
            if mclk_divider >= 256 {
                return Err(ConfigError::InvalidPdmClock);
            }
        } else {
            let max_fract = I2S_LL_MCLK_DIVIDER_MAX as u32;
            let common = gcd(freq_diff, mclk);
            let exact_num = freq_diff / common;
            let exact_den = mclk / common;
            if (2..=max_fract).contains(&exact_den) {
                numerator = exact_num;
                denominator = exact_den;
            } else {
                let mut min = u32::MAX;
                for a in 2..=max_fract {
                    // Best rational approximation of `freq_diff / mclk` with denominator `a`.
                    let b = ((freq_diff as u64 * a as u64) + mclk as u64 / 2) / mclk as u64;
                    if b == 0 || b >= a as u64 {
                        continue;
                    }
                    let ma = freq_diff as u64 * a as u64;
                    let mb = mclk as u64 * b;
                    if ma == mb {
                        denominator = a;
                        numerator = b as u32;
                        break;
                    }
                    let err = ma.abs_diff(mb);
                    if err < min as u64 {
                        denominator = a;
                        numerator = b as u32;
                        min = err as u32;
                    }
                }
            }
        }
    }

    Ok(I2sClockDividers {
        mclk_divider,
        bclk_divider: 0,
        denominator,
        numerator,
    })
}

// see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L34-L70>
pub(crate) fn calculate_tx_clock(
    clk: &PdmTxClockConfig,
    pcm: bool,
) -> Result<PdmTxClockResult, ConfigError> {
    if clk.up_sample_fs > 480 {
        return Err(ConfigError::InvalidPdmClock);
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
) -> Result<PdmRxClockResult, ConfigError> {
    let rate = clk.sample_rate.as_hz();
    let dn_sample_factor = PDM_BCK_FACTOR * clk.downsample_rate.factor();
    let slot_num = slot_mask.count_ones().max(1);

    // IDF `i2s_pdm_rx_calculate_clock`.
    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L387-L422>
    let bclk = if pcm {
        rate * dn_sample_factor
    } else {
        rate * 2
    };

    // see <https://github.com/espressif/esp-idf/blob/04f7908b1207d945b3fce94aca661379c8ab7afb/components/esp_driver_i2s/i2s_pdm.c#L407-L408>
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
    /// Default codec-line PDM TX clock (`I2S_PDM_TX_CLK_DEFAULT_CONFIG`).
    pub fn codec_default(sample_rate: Rate) -> Self {
        Self {
            sample_rate,
            up_sample_fp: 960,
            up_sample_fs: 480,
            bclk_div: PDM_TX_BCLK_DIV_MIN,
        }
    }

    /// Default DAC-line PDM TX clock (`I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG`).
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
    /// Default PDM RX clock (`I2S_PDM_RX_CLK_DEFAULT_CONFIG`).
    pub fn default(sample_rate: Rate) -> Self {
        Self {
            sample_rate,
            downsample_rate: PdmDownsampleRate::Dsr8s,
            bclk_div: PDM_RX_BCLK_DIV_MIN,
        }
    }
}
