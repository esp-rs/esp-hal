use fugit::HertzU32;
use strum::FromRepr;

use crate::clock::Clock;

pub(crate) fn init() {
    todo!()
}

pub(crate) fn configure_clock() {
    todo!()
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy)]
pub(crate) enum RtcFastClock {}

impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        todo!()
    }
}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RtcSlowClock {}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> HertzU32 {
        todo!()
    }
}

/// RTC Watchdog Timer
pub struct RtcClock;

/// RTC Watchdog Timer driver
impl RtcClock {
    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        todo!()
    }
}
