//! PDM (pulse-density modulation) configuration for the I2S master driver.
//!
//! Default configurations and clock calculations follow Espressif's I2S PDM driver
//! (<https://github.com/espressif/esp-idf/tree/master/components/esp_driver_i2s>).

mod clock;
#[cfg(not(i2s_version = "1"))]
mod hp_filter;
#[cfg_attr(i2s_version = "1", path = "regs_v1.rs")]
#[cfg_attr(i2s_version = "2", path = "regs_v2.rs")]
#[cfg_attr(i2s_version = "3", path = "regs_v2.rs")]
mod ll;

use super::master::{ConfigError, Instance};
use crate::time::Rate;

/// PDM configuration errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmError {
    /// PDM is not supported on this I2S peripheral instance (I2S1).
    UnsupportedInstance,
    /// PCM format requested but hardware PCM2PDM/PDM2PCM is unavailable.
    PcmFormatUnsupported,
    /// PDM clock configuration is invalid.
    InvalidClock,
    /// PDM slot mask selects no active channels.
    InvalidSlotMask,
    /// PDM config must enable at least one of TX or RX.
    DirectionMissing,
    /// Simultaneous PDM TX and RX is not supported.
    DuplexUnsupported,
    /// PDM data line index is not available on this chip.
    InvalidLine,
}

impl core::error::Error for PdmError {}

impl core::fmt::Display for PdmError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedInstance => write!(f, "PDM mode is only supported on I2S0"),
            Self::PcmFormatUnsupported => write!(
                f,
                "PCM PDM format is not supported on this chip; use raw PDM format"
            ),
            Self::InvalidClock => write!(f, "PDM clock configuration is out of supported range"),
            Self::InvalidSlotMask => {
                write!(f, "PDM slot mask must select at least one active slot")
            }
            Self::DirectionMissing => {
                write!(f, "PDM configuration must include TX and/or RX settings")
            }
            Self::DuplexUnsupported => {
                write!(f, "PDM full duplex (TX and RX together) is not supported")
            }
            Self::InvalidLine => write!(f, "PDM data line index is not available on this chip"),
        }
    }
}

/// A peripheral singleton that supports PDM mode (I2S0 only).
#[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
pub trait PdmInstance: Instance {}

// TODO on ESP32-P4 instances other than I2S0 can support PDM, but those DON'T support
// PCM2PDM/PDM2PCM
#[cfg(all(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx), soc_has_i2s0))]
impl PdmInstance for crate::peripherals::I2S0<'_> {}

/// PDM data format: PCM samples in software vs raw PDM bitstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmDataFormat {
    /// Hardware filter converts between PCM and PDM when supported.
    #[default]
    Pcm,
    /// Raw PDM samples; no hardware PCM conversion.
    Raw,
}

/// Mono or stereo slot mode (fixed two hardware slots).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmSlotMode {
    /// Single active slot (left by default).
    #[default]
    Mono,
    /// Both slots active (stereo).
    Stereo,
}

/// Active PDM slot selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmSlotMask(u16);

impl PdmSlotMask {
    /// Left slot only.
    pub const LEFT: Self = Self(1 << 1);
    /// Right slot only.
    pub const RIGHT: Self = Self(1 << 0);
    /// Both slots active.
    pub const BOTH: Self = Self(0b11);

    #[cfg(all(i2s_supports_pdm_rx, not(esp32)))]
    /// Line 0, left slot.
    pub const LINE0_LEFT: Self = Self(1 << 1);
    #[cfg(all(i2s_supports_pdm_rx, not(esp32)))]
    /// Line 0, right slot.
    pub const LINE0_RIGHT: Self = Self(1 << 0);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 1, left slot.
    pub const LINE1_LEFT: Self = Self(1 << 3);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 1, right slot.
    pub const LINE1_RIGHT: Self = Self(1 << 2);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 2, left slot.
    pub const LINE2_LEFT: Self = Self(1 << 5);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 2, right slot.
    pub const LINE2_RIGHT: Self = Self(1 << 4);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 3, left slot.
    pub const LINE3_LEFT: Self = Self(1 << 7);
    #[cfg(all(i2s_supports_pdm_rx, esp32s3))]
    /// Line 3, right slot.
    pub const LINE3_RIGHT: Self = Self(1 << 6);

    /// Create a mask from raw slot bits.
    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    /// Raw slot mask bits.
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Default mask for the given mono/stereo mode.
    pub fn for_mode(mode: PdmSlotMode) -> Self {
        match mode {
            PdmSlotMode::Mono => Self::LEFT,
            PdmSlotMode::Stereo => Self::BOTH,
        }
    }
}

/// PDM RX downsample rate (PDM2PCM path).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmDownsampleRate {
    /// 64x downsample (DSR 8s).
    #[default]
    Dsr8s,
    /// 128x downsample (DSR 16s).
    Dsr16s,
}

/// PDM TX sigma-delta filter scaling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmSigScaling {
    /// Divide input by 2.
    Div2,
    /// Multiply input by 1.
    #[default]
    Mul1,
    /// Multiply input by 2.
    Mul2,
    /// Multiply input by 4.
    Mul4,
}

impl PdmSigScaling {
    pub(crate) fn to_register(self) -> u8 {
        match self {
            Self::Div2 => 0,
            Self::Mul1 => 1,
            Self::Mul2 => 2,
            Self::Mul4 => 3,
        }
    }
}

/// PDM TX line routing (HW v2+).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PdmTxLineMode {
    /// Single-line codec output (default).
    #[default]
    OneLineCodec,
    /// Single-line DAC output.
    OneLineDac,
    /// Two-line DAC output (stereo).
    TwoLineDac,
}

/// PDM TX clock configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmTxClockConfig {
    /// Target PCM sample rate.
    pub sample_rate: Rate,
    /// Upsampling factor numerator (`fp`).
    pub up_sample_fp: u32,
    /// Upsampling factor denominator (`fs`).
    pub up_sample_fs: u32,
    /// Bit clock divider.
    pub bclk_div: u32,
}

/// PDM TX slot / filter configuration.
#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PdmTxSlotConfig {
    /// Mono or stereo slot mode.
    pub slot_mode: PdmSlotMode,
    /// PCM or raw PDM data format.
    pub data_format: PdmDataFormat,
    /// Sigma-delta prescale value.
    pub sd_prescale: u8,
    /// Sigma-delta filter input scaling.
    pub sd_scale: PdmSigScaling,
    /// High-pass filter input scaling.
    pub hp_scale: PdmSigScaling,
    /// Low-pass filter input scaling.
    pub lp_scale: PdmSigScaling,
    /// Sinc filter input scaling.
    pub sinc_scale: PdmSigScaling,
    #[cfg(not(i2s_version = "1"))]
    /// Output line routing mode.
    pub line_mode: PdmTxLineMode,
    /// Enable the TX high-pass filter.
    pub hp_en: bool,
    /// High-pass filter cut-off frequency in Hz.
    pub hp_cut_off_freq_hz: f32,
    #[cfg(not(i2s_version = "1"))]
    /// Sigma-delta dither bit 0.
    pub sd_dither: u8,
    #[cfg(not(i2s_version = "1"))]
    /// Sigma-delta dither bit 1.
    pub sd_dither2: u8,
    #[cfg(i2s_version = "1")]
    /// Active TX slot mask (ESP32 only).
    pub slot_mask: PdmSlotMask,
}

impl PdmTxSlotConfig {
    fn codec_pcm_default(mode: PdmSlotMode) -> Self {
        Self {
            slot_mode: mode,
            data_format: PdmDataFormat::Pcm,
            sd_prescale: 0,
            sd_scale: PdmSigScaling::Mul1,
            hp_scale: PdmSigScaling::Div2,
            lp_scale: PdmSigScaling::Mul1,
            sinc_scale: PdmSigScaling::Mul1,
            #[cfg(not(i2s_version = "1"))]
            line_mode: PdmTxLineMode::OneLineCodec,
            hp_en: true,
            hp_cut_off_freq_hz: 35.5,
            #[cfg(not(i2s_version = "1"))]
            sd_dither: 0,
            #[cfg(not(i2s_version = "1"))]
            sd_dither2: 1,
            #[cfg(i2s_version = "1")]
            slot_mask: PdmSlotMask::BOTH,
        }
    }

    #[cfg(not(i2s_version = "1"))]
    fn dac_pcm_default(mode: PdmSlotMode) -> Self {
        let mut cfg = Self::codec_pcm_default(mode);
        cfg.hp_scale = PdmSigScaling::Mul1;
        cfg.lp_scale = PdmSigScaling::Mul1;
        cfg.sinc_scale = PdmSigScaling::Mul1;
        #[cfg(not(i2s_version = "1"))]
        {
            cfg.line_mode = if mode == PdmSlotMode::Mono {
                PdmTxLineMode::OneLineDac
            } else {
                PdmTxLineMode::TwoLineDac
            };
        }
        cfg
    }

    #[cfg(not(i2s_supports_pcm2pdm))]
    fn raw_default(mode: PdmSlotMode) -> Self {
        let mut cfg = Self::codec_pcm_default(mode);
        cfg.data_format = PdmDataFormat::Raw;
        cfg
    }
}

/// PDM RX clock configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmRxClockConfig {
    /// Target PCM sample rate.
    pub sample_rate: Rate,
    /// PDM2PCM downsample rate.
    pub downsample_rate: PdmDownsampleRate,
    /// Bit clock divider.
    pub bclk_div: u32,
}

/// PDM RX slot configuration.
#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PdmRxSlotConfig {
    /// Mono or stereo slot mode.
    pub slot_mode: PdmSlotMode,
    /// Active RX slot mask.
    pub slot_mask: PdmSlotMask,
    /// PCM or raw PDM data format.
    pub data_format: PdmDataFormat,
    #[cfg(i2s_supports_pdm_rx_hp_filter)]
    /// Enable the RX high-pass filter.
    pub hp_en: bool,
    #[cfg(i2s_supports_pdm_rx_hp_filter)]
    /// High-pass filter cut-off frequency in Hz.
    pub hp_cut_off_freq_hz: f32,
    #[cfg(i2s_supports_pdm_rx_hp_filter)]
    /// RX amplification factor (1–15).
    pub amplify_num: u32,
}

impl PdmRxSlotConfig {
    fn pcm_default(mode: PdmSlotMode) -> Self {
        Self {
            slot_mode: mode,
            slot_mask: PdmSlotMask::for_mode(mode),
            data_format: PdmDataFormat::Pcm,
            #[cfg(i2s_supports_pdm_rx_hp_filter)]
            hp_en: true,
            #[cfg(i2s_supports_pdm_rx_hp_filter)]
            hp_cut_off_freq_hz: 35.5,
            #[cfg(i2s_supports_pdm_rx_hp_filter)]
            amplify_num: 1,
        }
    }

    fn raw_default(mode: PdmSlotMode) -> Self {
        let mut cfg = Self::pcm_default(mode);
        cfg.data_format = PdmDataFormat::Raw;
        #[cfg(i2s_supports_pdm_rx_hp_filter)]
        {
            cfg.hp_en = false;
        }
        cfg
    }
}

/// Full PDM TX unit configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmTxConfig {
    /// TX clock settings.
    pub clock: PdmTxClockConfig,
    /// TX slot and filter settings.
    pub slot: PdmTxSlotConfig,
}

impl PdmTxConfig {
    /// Codec-line defaults (`I2S_PDM_TX_*_DEFAULT_CONFIG`).
    pub fn new_codec_default(sample_rate: Rate, mode: PdmSlotMode) -> Self {
        Self {
            clock: PdmTxClockConfig::codec_default(sample_rate),
            slot: default_tx_slot(mode),
        }
    }

    /// DAC-line defaults (`I2S_PDM_TX_*_DAC_DEFAULT_CONFIG`, HW v2+).
    #[cfg(not(i2s_version = "1"))]
    pub fn new_dac_default(sample_rate: Rate, mode: PdmSlotMode) -> Self {
        Self {
            clock: PdmTxClockConfig::dac_default(sample_rate),
            slot: PdmTxSlotConfig::dac_pcm_default(mode),
        }
    }

    /// Validate TX configuration against hardware capabilities.
    pub fn validate(&self) -> Result<(), PdmError> {
        #[cfg(not(i2s_supports_pcm2pdm))]
        if self.slot.data_format == PdmDataFormat::Pcm {
            return Err(PdmError::PcmFormatUnsupported);
        }
        if self.clock.up_sample_fs > 480 {
            return Err(PdmError::InvalidClock);
        }
        Ok(())
    }
}

fn default_tx_slot(mode: PdmSlotMode) -> PdmTxSlotConfig {
    cfg_select! {
        i2s_supports_pcm2pdm => PdmTxSlotConfig::codec_pcm_default(mode),
        _ => PdmTxSlotConfig::raw_default(mode),
    }
}

/// Full PDM RX unit configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmRxConfig {
    /// RX clock settings.
    pub clock: PdmRxClockConfig,
    /// RX slot settings.
    pub slot: PdmRxSlotConfig,
}

impl PdmRxConfig {
    /// Default RX config; uses PCM when PDM2PCM is available, otherwise raw.
    pub fn new_default(sample_rate: Rate, mode: PdmSlotMode) -> Self {
        Self {
            clock: PdmRxClockConfig::default(sample_rate),
            slot: default_rx_slot(mode),
        }
    }

    /// PCM RX defaults (requires hardware PDM2PCM support).
    pub fn new_pcm_default(sample_rate: Rate, mode: PdmSlotMode) -> Self {
        Self {
            clock: PdmRxClockConfig::default(sample_rate),
            slot: PdmRxSlotConfig::pcm_default(mode),
        }
    }

    /// Raw PDM RX defaults (no hardware PCM conversion).
    pub fn new_raw_default(sample_rate: Rate, mode: PdmSlotMode) -> Self {
        Self {
            clock: PdmRxClockConfig::default(sample_rate),
            slot: PdmRxSlotConfig::raw_default(mode),
        }
    }

    /// Validate RX configuration against hardware capabilities.
    pub fn validate(&self) -> Result<(), PdmError> {
        #[cfg(not(i2s_supports_pdm2pcm))]
        if self.slot.data_format == PdmDataFormat::Pcm {
            return Err(PdmError::PcmFormatUnsupported);
        }
        if self.slot.slot_mask.bits() == 0 {
            return Err(PdmError::InvalidSlotMask);
        }
        Ok(())
    }
}

fn default_rx_slot(mode: PdmSlotMode) -> PdmRxSlotConfig {
    cfg_select! {
        i2s_supports_pdm2pcm => PdmRxSlotConfig::pcm_default(mode),
        _ => PdmRxSlotConfig::raw_default(mode),
    }
}

/// PDM mode configuration (simplex TX and/or RX).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdmConfig {
    /// Optional TX unit configuration.
    pub tx: Option<PdmTxConfig>,
    /// Optional RX unit configuration.
    pub rx: Option<PdmRxConfig>,
}

impl PdmConfig {
    /// PDM TX only (recommended simplex setup).
    #[cfg(i2s_supports_pdm_tx)]
    pub fn tx_only(tx: PdmTxConfig) -> Self {
        Self {
            tx: Some(tx),
            rx: None,
        }
    }

    /// PDM RX only (recommended simplex setup).
    #[cfg(i2s_supports_pdm_rx)]
    pub fn rx_only(rx: PdmRxConfig) -> Self {
        Self {
            tx: None,
            rx: Some(rx),
        }
    }

    /// Validate that exactly one direction is configured and settings are valid.
    pub fn validate(&self) -> Result<(), PdmError> {
        if self.tx.is_none() && self.rx.is_none() {
            return Err(PdmError::DirectionMissing);
        }
        if self.tx.is_some() && self.rx.is_some() {
            return Err(PdmError::DuplexUnsupported);
        }
        if let Some(tx) = &self.tx {
            tx.validate()?;
        }
        if let Some(rx) = &self.rx {
            rx.validate()?;
        }
        Ok(())
    }
}

pub(crate) fn configure_pdm<I: super::master::private::RegisterAccessPrivate + ?Sized>(
    i2s: &I,
    config: &PdmConfig,
) -> Result<(), ConfigError> {
    config.validate().map_err(ConfigError::Pdm)?;
    ll::configure_pdm(i2s, config).map_err(ConfigError::Pdm)
}
