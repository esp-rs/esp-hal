use strum::FromRepr;

use crate::{
    peripherals::LPWR,
    rtc_cntl::{RtcCalSel, RtcClock, RtcFastClock, RtcSlowClock},
};

pub(crate) fn init() {}

pub(crate) fn configure_clock() {
    RtcClock::set_fast_freq(RtcFastClock::RtcFastClock8m);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRtc);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let rtc_cntl = &*LPWR::ptr();
        rtc_cntl.store1().write(|w| w.bits(cal_val));
    }
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
/// SOC Reset Reason.
pub enum SocResetReason {
    /// Power on reset
    ChipPowerOn   = 0x01,
    /// Software resets the digital core
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    /// SDIO module resets the digital core
    CoreSdio      = 0x06,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU
    ///
    /// In ESP-IDF there are `Cpu0Mwdt1` and `Cpu1Mwdt1`, however they have the
    /// same values.
    CpuMwdt0      = 0x0B,
    /// Software resets CPU
    ///
    /// In ESP-IDF there are `Cpu0Sw` and `Cpu1Sw`, however they have the same
    /// values.
    Cpu0Sw        = 0x0C,
    /// RTC watch dog resets CPU
    ///
    /// In ESP-IDF there are `Cpu0RtcWdt` and `Cpu1RtcWdt`, however they have
    /// the same values.
    Cpu0RtcWdt    = 0x0D,
    /// CPU0 resets CPU1 by DPORT_APPCPU_RESETTING
    Cpu1Cpu0      = 0x0E,
    /// Reset when the VDD voltage is not stable
    SysBrownOut   = 0x0F,
    /// RTC watch dog resets digital core and rtc module
    SysRtcWdt     = 0x10,
}
