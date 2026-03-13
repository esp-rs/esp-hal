//! Clock tree definitions and implementations for ESP32-C5.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (32K OSC_SLOW, 130k RC_SLOW and 20M RC_FAST) are not calibrated here,
//!   this system can only give a rough estimate of their frequency. They can be calibrated
//!   separately using a known crystal frequency.
//! - Some of the SOC capabilities are not implemented: I2S external pad clock source, external 32k
//!   oscillator, others.
#![allow(dead_code, reason = "Some of this is bound to be unused")]
#![allow(missing_docs, reason = "Experimental")]
define_clock_tree_types!();

/// Clock configuration options.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// Default CPU clock
    #[default]
    _80MHz  = 80,

    /// 160 MHz CPU clock
    _160MHz = 160,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {};
    const PRESET_160: ClockConfig = ClockConfig {};
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_80MHz => CpuClock::PRESET_80, // TODO: Add preset for 80MHz
            CpuClock::_160MHz => CpuClock::PRESET_160, // TODO: Add preset for 160MHz
        }
    }
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self::from(CpuClock::default())
    }
}

impl ClockConfig {
    pub(crate) fn try_get_preset(self) -> Option<CpuClock> {
        match self {
            v if v == CpuClock::PRESET_80 => Some(CpuClock::_80MHz),
            v if v == CpuClock::PRESET_160 => Some(CpuClock::_160MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self) {
        self.apply();
    }
}
