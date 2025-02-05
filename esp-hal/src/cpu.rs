//! CPU related functionality

#[cfg(any(esp32, esp32s3))]
#[allow(unused_imports)]
pub use crate::soc::cpu_control::*;

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, Copy, Clone, PartialEq, Eq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu = 1,
}

impl Cpu {
    /// The number of available cores.
    pub const COUNT: usize = 1 + cfg!(multi_core) as usize;

    /// Returns the core the application is currently executing on
    #[inline(always)]
    pub fn current() -> Self {
        // This works for both RISCV and Xtensa because both
        // get_raw_core functions return zero, _or_ something
        // greater than zero; 1 in the case of RISCV and 0x2000
        // in the case of Xtensa.
        match raw_core() {
            0 => Cpu::ProCpu,
            #[cfg(all(multi_core, riscv))]
            1 => Cpu::AppCpu,
            #[cfg(all(multi_core, xtensa))]
            0x2000 => Cpu::AppCpu,
            _ => unreachable!(),
        }
    }

    /// Returns an iterator over the "other" cores.
    #[inline(always)]
    pub(crate) fn other() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                match Self::current() {
                    Cpu::ProCpu => [Cpu::AppCpu].into_iter(),
                    Cpu::AppCpu => [Cpu::ProCpu].into_iter(),
                }
            } else {
                [].into_iter()
            }
        }
    }

    /// Returns an iterator over all cores.
    #[inline(always)]
    pub(crate) fn all() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                [Cpu::ProCpu, Cpu::AppCpu].into_iter()
            } else {
                [Cpu::ProCpu].into_iter()
            }
        }
    }
}

/// Returns the raw value of the mhartid register.
///
/// On RISC-V, this is the hardware thread ID.
///
/// On Xtensa, this returns the result of reading the PRID register logically
/// ANDed with 0x2000, the 13th bit in the register. Espressif Xtensa chips use
/// this bit to determine the core id.
#[inline(always)]
pub(crate) fn raw_core() -> usize {
    // This method must never return UNUSED_THREAD_ID_VALUE
    cfg_if::cfg_if! {
        if #[cfg(all(multi_core, riscv))] {
            riscv::register::mhartid::read()
        } else if #[cfg(all(multi_core, xtensa))] {
            (xtensa_lx::get_processor_id() & 0x2000) as usize
        } else {
            0
        }
    }
}

use crate::rtc_cntl::SocResetReason;

/// Source of the wakeup event
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SleepSource {
    /// In case of deep sleep, reset was not caused by exit from deep sleep
    Undefined = 0,
    /// Not a wakeup cause, used to disable all wakeup sources with
    /// esp_sleep_disable_wakeup_source
    All,
    /// Wakeup caused by external signal using RTC_IO
    Ext0,
    /// Wakeup caused by external signal using RTC_CNTL
    Ext1,
    /// Wakeup caused by timer
    Timer,
    /// Wakeup caused by touchpad
    TouchPad,
    /// Wakeup caused by ULP program
    Ulp,
    /// Wakeup caused by GPIO (light sleep only on ESP32, S2 and S3)
    Gpio,
    /// Wakeup caused by UART (light sleep only)
    Uart,
    /// Wakeup caused by WIFI (light sleep only)
    Wifi,
    /// Wakeup caused by COCPU int
    Cocpu,
    /// Wakeup caused by COCPU crash
    CocpuTrapTrig,
    /// Wakeup caused by BT (light sleep only)
    BT,
}

bitflags::bitflags! {
    #[allow(unused)]
    pub(crate) struct WakeupReason: u32 {
        const NoSleep         = 0;
        #[cfg(pm_support_ext0_wakeup)]
        /// EXT0 GPIO wakeup
        const ExtEvent0Trig   = 1 << 0;
        #[cfg(pm_support_ext1_wakeup)]
        /// EXT1 GPIO wakeup
        const ExtEvent1Trig   = 1 << 1;
        /// GPIO wakeup (light sleep only)
        const GpioTrigEn      = 1 << 2;
        #[cfg(not(any(esp32c6, esp32h2)))]
        /// Timer wakeup
        const TimerTrigEn     = 1 << 3;
        #[cfg(any(esp32c6, esp32h2))]
        /// Timer wakeup
        const TimerTrigEn     = 1 << 4;
        #[cfg(pm_support_wifi_wakeup)]
        /// MAC wakeup (light sleep only)
        const WifiTrigEn      = 1 << 5;
        /// UART0 wakeup (light sleep only)
        const Uart0TrigEn     = 1 << 6;
        /// UART1 wakeup (light sleep only)
        const Uart1TrigEn     = 1 << 7;
        #[cfg(pm_support_touch_sensor_wakeup)]
        /// Touch wakeup
        const TouchTrigEn     = 1 << 8;
        #[cfg(ulp_supported)]
        /// ULP wakeup
        const UlpTrigEn       = 1 << 9;
        #[cfg(pm_support_bt_wakeup)]
        /// BT wakeup (light sleep only)
        const BtTrigEn        = 1 << 10;
        #[cfg(riscv_coproc_supported)]
        const CocpuTrigEn     = 1 << 11;
        #[cfg(riscv_coproc_supported)]
        const CocpuTrapTrigEn = 1 << 13;
    }
}

/// Performs a software reset on the chip.
pub fn software_reset() {
    crate::rom::software_reset();
}

/// Resets the given CPU, leaving peripherals unchanged.
#[instability::unstable]
pub fn software_reset_cpu(cpu: Cpu) {
    crate::rom::software_reset_cpu(cpu as u32);
}

/// Retrieves the reason for the last reset as a SocResetReason enum value.
/// Returns `None` if the reset reason cannot be determined.
#[instability::unstable]
pub fn reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::reset_reason(Cpu::current())
}

/// Retrieves the cause of the last wakeup event as a SleepSource enum value.
#[instability::unstable]
pub fn wakeup_cause() -> SleepSource {
    crate::rtc_cntl::wakeup_cause()
}
