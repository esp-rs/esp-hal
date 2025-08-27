#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Real-Time Control and Low-power Management (RTC_CNTL)
//!
//! ## Overview
//!
//! The RTC_CNTL peripheral is responsible for managing the low-power modes on
//! the chip.
//!
//! ## Configuration
//!
//! It also includes the necessary configurations and constants for clock
//! sources and low-power management. The driver provides the following features
//! and functionalities:
//!
//! * Clock Configuration
//! * Calibration
//! * Low-Power Management
//! * Handling Watchdog Timers
//!
//! ## Examples
//!
//! ### Get time in ms from the RTC Timer
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use core::time::Duration;
//! # use esp_hal::{delay::Delay, rtc_cntl::Rtc};
//!
//! let rtc = Rtc::new(peripherals.LPWR);
//! let delay = Delay::new();
//!
//! loop {
//!     // Print the current RTC time in milliseconds
//!     let time_ms = rtc.current_time_us() / 1000;
//!     delay.delay_millis(1000);
//!
//!     // Set the time to half a second in the past
//!     let new_time = rtc.current_time_us() - 500_000;
//!     rtc.set_current_time_us(new_time);
//! }
//! # }
//! ```
//!
//! ### RWDT usage
//! ```rust, no_run
//! # {before_snippet}
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use esp_hal::delay::Delay;
//! # use esp_hal::rtc_cntl::Rtc;
//! # use esp_hal::rtc_cntl::Rwdt;
//! # use esp_hal::rtc_cntl::RwdtStage;
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
//!
//! let mut delay = Delay::new();
//! let mut rtc = Rtc::new(peripherals.LPWR);
//!
//! rtc.set_interrupt_handler(interrupt_handler);
//! rtc.rwdt
//!     .set_timeout(RwdtStage::Stage0, Duration::from_millis(2000));
//! rtc.rwdt.listen();
//!
//! critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));
//! # {after_snippet}
//!
//! // Where the `LP_WDT` interrupt handler is defined as:
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use esp_hal::rtc_cntl::Rwdt;
//! # use esp_hal::rtc_cntl::RwdtStage;
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
//!
//! // Handle the corresponding interrupt
//! #[handler]
//! fn interrupt_handler() {
//!     critical_section::with(|cs| {
//!         println!("RWDT Interrupt");
//!
//!         let mut rwdt = RWDT.borrow_ref_mut(cs);
//!         if let Some(rwdt) = rwdt.as_mut() {
//!             rwdt.clear_interrupt();
//!
//!             println!("Restarting in 5 seconds...");
//!
//!             rwdt.set_timeout(RwdtStage::Stage0, Duration::from_millis(5000));
//!             rwdt.unlisten();
//!         }
//!     });
//! }
//! ```
//!
//! ### Get time in ms from the RTC Timer
//! ```rust, no_run
//! # {before_snippet}
//! # use core::time::Duration;
//! # use esp_hal::{delay::Delay, rtc_cntl::Rtc};
//!
//! let rtc = Rtc::new(peripherals.LPWR);
//! let delay = Delay::new();
//!
//! loop {
//!     // Get the current RTC time in milliseconds
//!     let time_ms = rtc.current_time_us() * 1000;
//!     delay.delay_millis(1000);
//!
//!     // Set the time to half a second in the past
//!     let new_time = rtc.current_time_us() - 500_000;
//!     rtc.set_current_time_us(new_time);
//! }
//! # }
//! ```

use esp_rom_sys::rom::ets_delay_us;

pub use self::rtc::SocResetReason;
#[cfg(not(esp32))]
use crate::efuse::Efuse;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2))]
use crate::rtc_cntl::sleep::{RtcSleepConfig, WakeSource, WakeTriggers};
use crate::{
    clock::{Clock, XtalClock},
    interrupt::{self, InterruptHandler},
    peripherals::{Interrupt, LPWR, TIMG0},
    system::{Cpu, SleepSource},
    time::{Duration, Rate},
};
// only include sleep where it's been implemented
#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2))]
pub mod sleep;

#[cfg_attr(esp32, path = "rtc/esp32.rs")]
#[cfg_attr(esp32c2, path = "rtc/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "rtc/esp32c3.rs")]
#[cfg_attr(esp32c6, path = "rtc/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "rtc/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "rtc/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "rtc/esp32s3.rs")]
pub(crate) mod rtc;

cfg_if::cfg_if! {
    if #[cfg(any(esp32c6, esp32h2))] {
        use crate::peripherals::LP_WDT;
        use crate::peripherals::LP_TIMER;
        use crate::peripherals::LP_AON;
    } else {
        use crate::peripherals::LPWR as LP_WDT;
        use crate::peripherals::LPWR as LP_TIMER;
        use crate::peripherals::LPWR as LP_AON;
    }
}

bitflags::bitflags! {
    #[allow(unused)]
    struct WakeupReason: u32 {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)] // FIXME: resolve this
/// RTC FAST_CLK frequency values
pub(crate) enum RtcFastClock {
    /// Main XTAL, divided by 4
    #[cfg(not(any(esp32c6, esp32h2)))]
    #[expect(unused)]
    RtcFastClockXtalD4,

    /// Select XTAL_D2_CLK as RTC_FAST_CLK source
    #[cfg(any(esp32c6, esp32h2))]
    #[expect(unused)]
    RtcFastClockXtalD2,

    /// Internal fast RC oscillator
    RtcFastClockRcFast,
}

impl Clock for RtcFastClock {
    fn frequency(&self) -> Rate {
        match self {
            #[cfg(not(any(esp32c6, esp32h2)))]
            RtcFastClock::RtcFastClockXtalD4 => Rate::from_hz(40_000_000 / 4),

            #[cfg(any(esp32c6, esp32h2))]
            RtcFastClock::RtcFastClockXtalD2 => Rate::from_hz(property!("soc.xtal_frequency") / 2),

            RtcFastClock::RtcFastClockRcFast => Rate::from_hz(property!("soc.rc_fast_clk_default")),
        }
    }
}

#[cfg(not(any(esp32c6, esp32h2)))]
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)] // FIXME: resolve this
/// RTC SLOW_CLK frequency values
pub enum RtcSlowClock {
    /// Internal slow RC oscillator
    RtcSlowClockRcSlow  = 0,
    /// External 32 KHz XTAL
    RtcSlowClock32kXtal = 1,
    /// Internal fast RC oscillator, divided by 256
    RtcSlowClock8mD256  = 2,
}

/// RTC SLOW_CLK frequency values
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
#[cfg(any(esp32c6, esp32h2))]
pub enum RtcSlowClock {
    /// Select RC_SLOW_CLK as RTC_SLOW_CLK source
    RtcSlowClockRcSlow  = 0,
    /// Select XTAL32K_CLK as RTC_SLOW_CLK source
    RtcSlowClock32kXtal = 1,
    /// Select RC32K_CLK as RTC_SLOW_CLK source
    RtcSlowClock32kRc   = 2,
    /// Select OSC_SLOW_CLK (external slow clock) as RTC_SLOW_CLK source
    RtcSlowOscSlow      = 3,
}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> Rate {
        match self {
            RtcSlowClock::RtcSlowClockRcSlow => Rate::from_hz(property!("soc.rc_slow_clock")),

            RtcSlowClock::RtcSlowClock32kXtal => Rate::from_hz(32_768),

            #[cfg(not(any(esp32c6, esp32h2)))]
            RtcSlowClock::RtcSlowClock8mD256 => RtcFastClock::RtcFastClockRcFast.frequency() / 256,

            #[cfg(any(esp32c6, esp32h2))]
            RtcSlowClock::RtcSlowClock32kRc => Rate::from_hz(32_768),

            #[cfg(any(esp32c6, esp32h2))]
            RtcSlowClock::RtcSlowOscSlow => Rate::from_hz(32_768),
        }
    }
}

#[allow(unused)]
#[cfg(not(any(esp32c6, esp32h2)))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)] // FIXME: resolve this
/// Clock source to be calibrated using rtc_clk_cal function
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcCalRtcMux      = 0,
    /// Internal 8 MHz RC oscillator, divided by 256
    RtcCal8mD256      = 1,
    /// External 32 KHz XTAL
    RtcCal32kXtal     = 2,
    #[cfg(not(esp32))]
    /// Internal 150 KHz RC oscillator
    RtcCalInternalOsc = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg(any(esp32c6, esp32h2))]
/// Clock source to be calibrated using rtc_clk_cal function
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcCalRtcMux     = -1,
    /// Internal 150kHz RC oscillator
    RtcCalRcSlow     = 0,
    /// External 32kHz XTAL, as one type of 32k clock
    RtcCal32kXtal    = 1,
    /// Internal 32kHz RC oscillator, as one type of 32k clock
    RtcCal32kRc      = 2,
    /// External slow clock signal input by lp_pad_gpio0, as one type of 32k
    /// clock
    RtcCal32kOscSlow = 3,
    /// Internal MHz-range RC oscillator
    RtcCalRcFast,
}

/// Low-power Management
pub struct Rtc<'d> {
    _inner: crate::peripherals::LPWR<'d>,
    /// Reset Watchdog Timer.
    pub rwdt: Rwdt,
    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    /// Super Watchdog
    pub swd: Swd,
}

impl<'d> Rtc<'d> {
    /// Create a new instance in [crate::Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rtc_cntl: crate::peripherals::LPWR<'d>) -> Self {
        Self {
            _inner: rtc_cntl,
            rwdt: Rwdt::new(),
            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            swd: Swd::new(),
        }
    }

    /// Return estimated XTAL frequency in MHz.
    pub fn estimate_xtal_frequency(&mut self) -> u32 {
        RtcClock::estimate_xtal_frequency()
    }

    /// Get the time since boot in the raw register units.
    fn time_since_boot_raw(&self) -> u64 {
        let rtc_cntl = LP_TIMER::regs();

        #[cfg(esp32)]
        let (l, h) = {
            rtc_cntl.time_update().write(|w| w.time_update().set_bit());
            while rtc_cntl.time_update().read().time_valid().bit_is_clear() {
                // might take 1 RTC slowclk period, don't flood RTC bus
                crate::rom::ets_delay_us(1);
            }
            let h = rtc_cntl.time1().read().time_hi().bits();
            let l = rtc_cntl.time0().read().time_lo().bits();
            (l, h)
        };
        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let (l, h) = {
            rtc_cntl.time_update().write(|w| w.time_update().set_bit());
            let h = rtc_cntl.time_high0().read().timer_value0_high().bits();
            let l = rtc_cntl.time_low0().read().timer_value0_low().bits();
            (l, h)
        };
        #[cfg(any(esp32c6, esp32h2))]
        let (l, h) = {
            rtc_cntl.update().write(|w| w.main_timer_update().set_bit());
            let h = rtc_cntl
                .main_buf0_high()
                .read()
                .main_timer_buf0_high()
                .bits();
            let l = rtc_cntl.main_buf0_low().read().main_timer_buf0_low().bits();
            (l, h)
        };
        ((h as u64) << 32) | (l as u64)
    }

    /// Get the time since boot.
    pub fn time_since_boot(&self) -> Duration {
        Duration::from_micros(
            self.time_since_boot_raw() * 1_000_000
                / RtcClock::slow_freq().frequency().as_hz() as u64,
        )
    }

    /// Read the current value of the boot time registers in microseconds.
    fn boot_time_us(&self) -> u64 {
        // For more info on about how RTC setting works and what it has to do with boot time, see https://github.com/esp-rs/esp-hal/pull/1883

        // In terms of registers, STORE2 and STORE3 are used on all current chips
        // (esp32, esp32p4, esp32h2, esp32c2, esp32c3, esp32c5, esp32c6, esp32c61,
        // esp32s2, esp32s3)

        // In terms of peripherals:

        // - LPWR is used on the following chips: esp32, esp32p4, esp32c2, esp32c3, esp32s2, esp32s3

        // - LP_AON is used on the following chips: esp32c5, esp32c6, esp32c61, esp32h2

        // For registers and peripherals used in esp-idf, see https://github.com/search?q=repo%3Aespressif%2Fesp-idf+RTC_BOOT_TIME_LOW_REG+RTC_BOOT_TIME_HIGH_REG+path%3A**%2Frtc.h&type=code

        let rtc_cntl = LP_AON::regs();

        let (l, h) = (rtc_cntl.store2(), rtc_cntl.store3());

        let l = l.read().bits() as u64;
        let h = h.read().bits() as u64;

        // https://github.com/espressif/esp-idf/blob/23e4823f17a8349b5e03536ff7653e3e584c9351/components/newlib/port/esp_time_impl.c#L115
        l + (h << 32)
    }

    /// Set the current value of the boot time registers in microseconds.
    fn set_boot_time_us(&self, boot_time_us: u64) {
        // Please see `boot_time_us` for documentation on registers and peripherals
        // used for certain SOCs.

        let rtc_cntl = LP_AON::regs();

        let (l, h) = (rtc_cntl.store2(), rtc_cntl.store3());

        // https://github.com/espressif/esp-idf/blob/23e4823f17a8349b5e03536ff7653e3e584c9351/components/newlib/port/esp_time_impl.c#L102-L103
        l.write(|w| unsafe { w.bits((boot_time_us & 0xffffffff) as u32) });
        h.write(|w| unsafe { w.bits((boot_time_us >> 32) as u32) });
    }

    #[procmacros::doc_replace]
    /// Get the current time in microseconds.
    ///
    /// # Example
    ///
    /// This example shows how to get the weekday of the current time in
    /// New York using the `jiff` crate. This example works in core-only
    /// environments without dynamic memory allocation.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::rtc_cntl::Rtc;
    /// use jiff::{
    ///     Timestamp,
    ///     tz::{self, TimeZone},
    /// };
    ///
    /// static TZ: TimeZone = tz::get!("America/New_York");
    ///
    /// let rtc = Rtc::new(peripherals.LPWR);
    /// let now = Timestamp::from_microsecond(rtc.current_time_us() as i64)?;
    /// let weekday_in_new_york = now.to_zoned(TZ.clone()).weekday();
    /// # {after_snippet}
    /// ```
    pub fn current_time_us(&self) -> u64 {
        // Current time is boot time + time since boot

        let rtc_time_us = self.time_since_boot().as_micros();
        let boot_time_us = self.boot_time_us();
        let wrapped_boot_time_us = u64::MAX - boot_time_us;

        // We can detect if we wrapped the boot time by checking if rtc time is greater
        // than the amount of time we would've wrapped.
        if rtc_time_us > wrapped_boot_time_us {
            // We also just checked that this won't overflow
            rtc_time_us - wrapped_boot_time_us
        } else {
            boot_time_us + rtc_time_us
        }
    }

    /// Set the current time in microseconds.
    pub fn set_current_time_us(&self, current_time_us: u64) {
        // Current time is boot time + time since boot (rtc time)
        // So boot time = current time - time since boot (rtc time)

        let rtc_time_us = self.time_since_boot().as_micros();
        if current_time_us < rtc_time_us {
            // An overflow would happen if we subtracted rtc_time_us from current_time_us.
            // To work around this, we can wrap around u64::MAX by subtracting the
            // difference between the current time and the time since boot.
            // Subtracting time since boot and adding current new time is equivalent and
            // avoids overflow. We just checked that rtc_time_us is less than time_us
            // so this won't overflow.
            self.set_boot_time_us(u64::MAX - rtc_time_us + current_time_us)
        } else {
            self.set_boot_time_us(current_time_us - rtc_time_us)
        }
    }

    /// Enter deep sleep and wake with the provided `wake_sources`.
    ///
    /// In Deep-sleep mode, the CPUs, most of the RAM, and all digital
    /// peripherals that are clocked from APB_CLK are powered off.
    ///
    /// You can use the [`#[esp_hal::ram(persistent)]`][procmacros::ram]
    /// attribute to persist a variable though deep sleep.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2))]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2))]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2))]
    pub fn sleep(&mut self, config: &RtcSleepConfig, wake_sources: &[&dyn WakeSource]) {
        let mut config = *config;
        let mut wakeup_triggers = WakeTriggers::default();
        for wake_source in wake_sources {
            wake_source.apply(self, &mut wakeup_triggers, &mut config)
        }

        config.apply();

        config.start_sleep(wakeup_triggers);
        config.finish_sleep();
    }

    const RTC_DISABLE_ROM_LOG: u32 = 1;

    /// Temporarily disable log messages of the ROM bootloader.
    ///
    /// If you need to permanently disable the ROM bootloader messages, you'll
    /// need to set the corresponding eFuse.
    #[cfg(any(esp32s3, esp32h2))]
    pub fn disable_rom_message_printing(&self) {
        // Corresponding documentation:
        // ESP32-S3: TRM v1.5 chapter 8.3
        // ESP32-H2: TRM v0.5 chapter 8.2.3

        let rtc_cntl = LP_AON::regs();
        rtc_cntl
            .store4()
            .modify(|r, w| unsafe { w.bits(r.bits() | Self::RTC_DISABLE_ROM_LOG) });
    }

    /// Register an interrupt handler for the RTC.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let interrupt = Interrupt::LP_WDT;
            } else {
                let interrupt = Interrupt::RTC_CORE;
            }
        }
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        unsafe { interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }
}
impl crate::private::Sealed for Rtc<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Rtc<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

/// Clocks.
// TODO: this type belongs in `esp_hal::clock`.
pub struct RtcClock;

/// RTC Watchdog Timer driver.
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    pub(crate) fn update_xtal_freq_mhz(mhz: u32) {
        let half = mhz & 0xFFFF;
        let combined = half | half << 16;

        let xtal_freq_reg = LP_AON::regs().store4().read().bits();
        let disable_log_bit = xtal_freq_reg & Rtc::RTC_DISABLE_ROM_LOG;
        debug!("Storing {}MHz to retention register", mhz);

        LP_AON::regs()
            .store4()
            .write(|w| unsafe { w.bits(combined | disable_log_bit) });
    }

    pub(crate) fn read_xtal_freq_mhz() -> Option<u32> {
        let xtal_freq_reg = LP_AON::regs().store4().read().bits();

        // RTC_XTAL_FREQ is stored as two copies in lower and upper 16-bit halves
        // need to mask out the RTC_DISABLE_ROM_LOG bit which is also stored in the same
        // register
        let xtal_freq = (xtal_freq_reg & !Rtc::RTC_DISABLE_ROM_LOG) as u16;
        let xtal_freq_copy = (xtal_freq_reg >> 16) as u16;

        if xtal_freq == xtal_freq_copy && xtal_freq != 0 && xtal_freq != u16::MAX {
            debug!("Read stored {}MHz from retention register", xtal_freq);
            Some(xtal_freq as u32)
        } else {
            None
        }
    }

    /// Get main XTAL frequency.
    ///
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG during esp-hal startup.
    pub fn xtal_freq() -> XtalClock {
        match Self::read_xtal_freq_mhz() {
            Some(mhz) => XtalClock::closest_from_mhz(mhz),
            None => panic!("Clocks have not been initialized yet"),
        }
    }

    /// Get the RTC_SLOW_CLK source.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub fn slow_freq() -> RtcSlowClock {
        match LPWR::regs().clk_conf().read().ana_clk_rtc_sel().bits() {
            0 => RtcSlowClock::RtcSlowClockRcSlow,
            1 => RtcSlowClock::RtcSlowClock32kXtal,
            2 => RtcSlowClock::RtcSlowClock8mD256,
            _ => unreachable!(),
        }
    }

    /// Select source for RTC_SLOW_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn set_slow_freq(slow_freq: RtcSlowClock) {
        LPWR::regs().clk_conf().modify(|_, w| {
            unsafe {
                w.ana_clk_rtc_sel().bits(slow_freq as u8);
            }

            // Why we need to connect this clock to digital?
            // Or maybe this clock should be connected to digital when
            // XTAL 32k clock is enabled instead?
            w.dig_xtal32k_en()
                .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kXtal));

            // The clk_8m_d256 will be closed when rtc_state in SLEEP,
            // so if the slow_clk is 8md256, clk_8m must be force power on
            w.ck8m_force_pu()
                .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock8mD256))
        });

        crate::rom::ets_delay_us(300u32);
    }

    /// Select source for RTC_FAST_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn set_fast_freq(fast_freq: RtcFastClock) {
        LPWR::regs().clk_conf().modify(|_, w| {
            w.fast_clk_rtc_sel().bit(match fast_freq {
                RtcFastClock::RtcFastClockRcFast => true,
                RtcFastClock::RtcFastClockXtalD4 => false,
            })
        });

        crate::rom::ets_delay_us(3u32);
    }

    // TODO: IDF-5781 Some of esp32c6 SOC_RTC_FAST_CLK_SRC_XTAL_D2 rtc_fast clock
    // has timing issue. Force to use SOC_RTC_FAST_CLK_SRC_RC_FAST since 2nd
    // stage bootloader https://github.com/espressif/esp-idf/blob/master/components/bootloader_support/src/bootloader_clock_init.c#L65-L67
    #[cfg(any(esp32h2, esp32c6))]
    fn set_fast_freq(fast_freq: RtcFastClock) {
        #[cfg(esp32h2)]
        LPWR::regs().lp_clk_conf().modify(|_, w| unsafe {
            w.fast_clk_sel().bits(match fast_freq {
                RtcFastClock::RtcFastClockRcFast => 0b00,
                RtcFastClock::RtcFastClockXtalD2 => 0b01,
            })
        });

        #[cfg(esp32c6)]
        LPWR::regs().lp_clk_conf().modify(|_, w| {
            w.fast_clk_sel().bit(match fast_freq {
                RtcFastClock::RtcFastClockRcFast => false,
                RtcFastClock::RtcFastClockXtalD2 => true,
            })
        });

        crate::rom::ets_delay_us(3);
    }

    #[cfg(any(esp32h2, esp32c6))]
    fn set_slow_freq(slow_freq: RtcSlowClock) {
        LPWR::regs()
            .lp_clk_conf()
            .modify(|_, w| unsafe { w.slow_clk_sel().bits(slow_freq as u8) });

        LPWR::regs().clk_to_hp().modify(|_, w| {
            w.icg_hp_xtal32k()
                .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kXtal));
            w.icg_hp_osc32k()
                .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kRc))
        });
    }

    /// Get the RTC_SLOW_CLK source
    #[cfg(any(esp32h2, esp32c6))]
    pub fn slow_freq() -> RtcSlowClock {
        match LPWR::regs().lp_clk_conf().read().slow_clk_sel().bits() {
            0 => RtcSlowClock::RtcSlowClockRcSlow,
            1 => RtcSlowClock::RtcSlowClock32kXtal,
            2 => RtcSlowClock::RtcSlowClock32kRc,
            3 => RtcSlowClock::RtcSlowOscSlow,
            _ => unreachable!(),
        }
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    #[cfg(not(any(esp32c6, esp32h2)))] // TODO: merge with C6/H2 impl
    fn calibrate_internal(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        // Except for ESP32, choosing RTC_CAL_RTC_MUX results in calibration of
        // the 150k RTC clock (90k on ESP32-S2) regardless of the currently selected
        // SLOW_CLK. On the ESP32, it uses the currently selected SLOW_CLK.
        // The following code emulates ESP32 behavior for the other chips:
        #[cfg(not(esp32))]
        let cal_clk = match cal_clk {
            RtcCalSel::RtcCalRtcMux => match RtcClock::slow_freq() {
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock8mD256 => RtcCalSel::RtcCal8mD256,
                _ => cal_clk,
            },
            RtcCalSel::RtcCalInternalOsc => RtcCalSel::RtcCalRtcMux,
            _ => cal_clk,
        };
        let rtc_cntl = LPWR::regs();
        let timg0 = TIMG0::regs();

        // Enable requested clock (150k clock is always on)
        let dig_32k_xtal_enabled = rtc_cntl.clk_conf().read().dig_xtal32k_en().bit_is_set();
        let dig_clk8m_d256_enabled = rtc_cntl.clk_conf().read().dig_clk8m_d256_en().bit_is_set();

        rtc_cntl.clk_conf().modify(|_, w| {
            if matches!(cal_clk, RtcCalSel::RtcCal32kXtal) {
                w.dig_xtal32k_en().set_bit();
            }

            if matches!(cal_clk, RtcCalSel::RtcCal8mD256) {
                w.dig_clk8m_d256_en().set_bit();
            }

            w
        });

        // There may be another calibration process already running during we
        // call this function, so we should wait the last process is done.
        #[cfg(not(esp32))]
        if timg0
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            // Set a small timeout threshold to accelerate the generation of timeout.
            // The internal circuit will be reset when the timeout occurs and will not
            // affect the next calibration.
            timg0
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            while timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_clear()
                && timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_clear()
            {}
        }

        // Prepare calibration
        timg0.rtccalicfg().modify(|_, w| unsafe {
            w.rtc_cali_clk_sel().bits(cal_clk as u8);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_max().bits(slowclk_cycles as u16)
        });

        // Figure out how long to wait for calibration to finish
        // Set timeout reg and expect time delay
        let expected_freq = match cal_clk {
            RtcCalSel::RtcCal32kXtal => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::RtcSlowClock32kXtal
            }
            RtcCalSel::RtcCal8mD256 => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::RtcSlowClock8mD256
            }
            _ => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                RtcSlowClock::RtcSlowClockRcSlow
            }
        };

        let us_time_estimate = Rate::from_mhz(slowclk_cycles) / expected_freq.frequency();

        // Start calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit().rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        crate::rom::ets_delay_us(us_time_estimate);

        #[cfg(esp32)]
        let mut timeout_us = us_time_estimate;

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                break timg0.rtccalicfg1().read().rtc_cali_value().bits();
            }

            #[cfg(not(esp32))]
            if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }

            #[cfg(esp32)]
            if timeout_us > 0 {
                timeout_us -= 1;
                crate::rom::ets_delay_us(1);
            } else {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        rtc_cntl.clk_conf().modify(|_, w| {
            w.dig_xtal32k_en().bit(dig_32k_xtal_enabled);
            w.dig_clk8m_d256_en().bit(dig_clk8m_d256_enabled)
        });

        cal_val
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    #[cfg(any(esp32c6, esp32h2))]
    pub(crate) fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        use crate::peripherals::{LP_CLKRST, PCR, PMU};

        #[derive(Clone, Copy)]
        enum RtcCaliClkSel {
            CaliClkRcSlow = 0,
            CaliClkRcFast = 1,
            CaliClk32k    = 2,
        }

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match cal_clk {
                RtcCalSel::RtcCalRtcMux => match RtcClock::slow_freq() {
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                    _ => cal_clk,
                },
                RtcCalSel::RtcCal32kOscSlow => RtcCalSel::RtcCalRtcMux,
                _ => cal_clk,
            };
        }

        let clk_src = RtcClock::slow_freq();

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
            };
        }

        let cali_clk_sel;
        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCalRcSlow,
            }
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcFast;
        } else if cal_clk == RtcCalSel::RtcCalRcSlow {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcSlow;
        } else {
            cali_clk_sel = RtcCaliClkSel::CaliClk32k;

            match cal_clk {
                RtcCalSel::RtcCalRtcMux | RtcCalSel::RtcCalRcSlow | RtcCalSel::RtcCalRcFast => {}
                RtcCalSel::RtcCal32kRc => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) });
                }
                RtcCalSel::RtcCal32kXtal => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) });
                }
                RtcCalSel::RtcCal32kOscSlow => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) });
                }
            }
        }

        // Enable requested clock (150k is always on)
        // Some delay is required before the time is stable
        // Only enable if originaly was disabled
        // If clock is already on, do nothing

        let dig_32k_xtal_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_xtal32k()
            .bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        }

        // TODO: very hacky - icg_hp_xtal32k is already set in the above condition?
        // in ESP-IDF these are not called in this function but the fields are set
        LP_CLKRST::regs()
            .clk_to_hp()
            .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        PMU::regs().hp_sleep_lp_ck_power().modify(|_, w| {
            w.hp_sleep_xpd_xtal32k().set_bit();
            w.hp_sleep_xpd_rc32k().set_bit()
        });

        let rc_fast_enabled = PMU::regs()
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_fosc_clk()
            .bit_is_set();
        let dig_rc_fast_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_fosc()
            .bit_is_set();

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if !rc_fast_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if !dig_rc_fast_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        let rc32k_enabled = PMU::regs()
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_rc32k()
            .bit_is_set();
        let dig_rc32k_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_osc32k()
            .bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if !rc32k_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }

            if !dig_rc32k_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        // Check if there is already running calibration process
        // TODO: &mut TIMG0 for calibration
        let timg0 = TIMG0::regs();

        if timg0
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            timg0
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            // Set small timeout threshold to accelerate the generation of timeot
            // Internal circuit will be reset when timeout occurs and will not affect the
            // next calibration
            while !timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set()
                && !timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set()
            {}
        }

        // Prepare calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(cali_clk_sel as u8) });
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_max().bits(slowclk_cycles as u16) });

        let expected_frequency = match cali_clk_sel {
            RtcCaliClkSel::CaliClk32k => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::RtcSlowClock32kXtal.frequency()
            }
            RtcCaliClkSel::CaliClkRcFast => {
                timg0
                    .rtccalicfg2()
                    .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(0x01FFFFFF) });
                RtcFastClock::RtcFastClockRcFast.frequency()
            }
            RtcCaliClkSel::CaliClkRcSlow => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                RtcSlowClock::RtcSlowClockRcSlow.frequency()
            }
        };

        let us_time_estimate = Rate::from_mhz(slowclk_cycles) / expected_frequency;

        // Start calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        crate::rom::ets_delay_us(us_time_estimate);

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
                // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
                // 32 times. And ensure that this modification will not affect
                // ECO0.
                // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
                #[cfg(esp32c6)]
                if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RtcCalRcFast {
                    break timg0.rtccalicfg1().read().rtc_cali_value().bits() >> 5;
                }
                break timg0.rtccalicfg1().read().rtc_cali_value().bits();
            }

            if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().clear_bit());
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if rc_fast_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if dig_rc_fast_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if rc32k_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }
            if dig_rc32k_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        cal_val
    }

    /// Measure RTC slow clock's period, based on main XTAL frequency
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::xtal_freq();

        #[cfg(esp32c6)]
        let slowclk_cycles = if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RtcCalRcFast {
            // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
            // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
            // chips by 32 to avoid excessive calibration time.
            slowclk_cycles >> 5
        } else {
            slowclk_cycles
        };

        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let calibration_clock = match RtcClock::slow_freq() {
                    RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRtcMux,
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                    RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
                    // RtcSlowClock::RtcCalRcFast => RtcCalSel::RtcCalRcFast,
                };
            } else {
                let calibration_clock = match RtcClock::slow_freq() {
                    RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRtcMux,
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock8mD256 => RtcCalSel::RtcCal8mD256,
                };
            }
        }

        // TODO: store the result somewhere
        let period_13q19 = RtcClock::calibrate(calibration_clock, 1024);

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
    }

    /// Return estimated XTAL frequency in MHz.
    pub(crate) fn estimate_xtal_frequency() -> u32 {
        // TODO: this could reuse Self::calibrate_internal
        const SLOW_CLOCK_CYCLES: u32 = 100;

        let calibration_clock = RtcSlowClock::RtcSlowClockRcSlow;

        // Make sure the process doesn't time out due to some spooky configuration.
        #[cfg(not(esp32))]
        TIMG0::regs().rtccalicfg2().reset();

        TIMG0::regs().rtccalicfg().write(|w| unsafe {
            w.rtc_cali_clk_sel().bits(calibration_clock as u8);
            w.rtc_cali_max().bits(SLOW_CLOCK_CYCLES as u16);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_start().set_bit()
        });

        // Delay, otherwise the CPU may read back the previous state of the completion flag and skip
        // waiting.
        ets_delay_us(SLOW_CLOCK_CYCLES * 1_000_000 / calibration_clock.frequency().as_hz());

        // Wait for the calibration to finish
        while TIMG0::regs()
            .rtccalicfg()
            .read()
            .rtc_cali_rdy()
            .bit_is_clear()
        {}

        let cali_value = TIMG0::regs().rtccalicfg1().read().rtc_cali_value().bits();

        (cali_value * (calibration_clock.frequency().as_hz() / SLOW_CLOCK_CYCLES)) / 1_000_000
    }
}

/// Behavior of the RWDT stage if it times out.
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum RwdtStageAction {
    /// No effect on the system.
    Off         = 0,
    /// Trigger an interrupt.
    Interrupt   = 1,
    /// Reset the CPU core.
    ResetCpu    = 2,
    /// Reset the main system.
    /// The power management unit and RTC peripherals will not be reset.
    ResetCore   = 3,
    /// Reset the main system, power management unit and RTC peripherals.
    ResetSystem = 4,
}

/// RWDT stages.
///
/// Timer stages allow for a timer to have a series of different timeout values
/// and corresponding expiry action.
#[derive(Debug, Clone, Copy)]
pub enum RwdtStage {
    /// RWDT stage 0.
    Stage0,
    /// RWDT stage 1.
    Stage1,
    /// RWDT stage 2.
    Stage2,
    /// RWDT stage 3.
    Stage3,
}

/// RTC Watchdog Timer.
pub struct Rwdt;

impl Default for Rwdt {
    fn default() -> Self {
        Self::new()
    }
}

/// RTC Watchdog Timer driver.
impl Rwdt {
    /// Create a new RTC watchdog timer instance
    pub fn new() -> Self {
        Self
    }

    /// Enable the watchdog timer instance.
    /// Watchdog starts with default settings (`stage 0` resets the system, the
    /// others are deactivated)
    pub fn enable(&mut self) {
        self.set_enabled(true);
    }

    /// Disable the watchdog timer instance.
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    /// Listen for interrupts on stage 0.
    pub fn listen(&mut self) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);

        // Configure STAGE0 to trigger an interrupt upon expiration
        rtc_cntl
            .wdtconfig0()
            .modify(|_, w| unsafe { w.wdt_stg0().bits(RwdtStageAction::Interrupt as u8) });

        rtc_cntl.int_ena().modify(|_, w| w.wdt().set_bit());

        self.set_write_protection(true);
    }

    /// Stop listening for interrupts on stage 0.
    pub fn unlisten(&mut self) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);

        // Configure STAGE0 to reset the main system and the RTC upon expiration.
        rtc_cntl
            .wdtconfig0()
            .modify(|_, w| unsafe { w.wdt_stg0().bits(RwdtStageAction::ResetSystem as u8) });

        rtc_cntl.int_ena().modify(|_, w| w.wdt().clear_bit());

        self.set_write_protection(true);
    }

    /// Clear interrupt.
    pub fn clear_interrupt(&mut self) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);

        rtc_cntl.int_clr().write(|w| w.wdt().clear_bit_by_one());

        self.set_write_protection(true);
    }

    /// Check if the interrupt is set.
    pub fn is_interrupt_set(&self) -> bool {
        let rtc_cntl = LP_WDT::regs();

        rtc_cntl.int_st().read().wdt().bit_is_set()
    }

    /// Feed the watchdog timer.
    pub fn feed(&mut self) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);
        rtc_cntl.wdtfeed().write(|w| w.wdt_feed().set_bit());
        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        let rtc_cntl = LP_WDT::regs();

        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl.wdtwprotect().write(|w| unsafe { w.bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);

        if !enable {
            rtc_cntl.wdtconfig0().modify(|_, w| unsafe { w.bits(0) });
        } else {
            rtc_cntl
                .wdtconfig0()
                .write(|w| w.wdt_flashboot_mod_en().bit(false));

            rtc_cntl
                .wdtconfig0()
                .modify(|_, w| w.wdt_en().bit(enable).wdt_pause_in_slp().bit(enable));

            // Apply default settings for WDT
            unsafe {
                rtc_cntl.wdtconfig0().modify(|_, w| {
                    w.wdt_stg0().bits(RwdtStageAction::ResetSystem as u8);
                    w.wdt_cpu_reset_length().bits(7);
                    w.wdt_sys_reset_length().bits(7);
                    w.wdt_stg1().bits(RwdtStageAction::Off as u8);
                    w.wdt_stg2().bits(RwdtStageAction::Off as u8);
                    w.wdt_stg3().bits(RwdtStageAction::Off as u8);
                    w.wdt_en().set_bit()
                });
            }
        }

        self.set_write_protection(true);
    }

    /// Configure timeout value in ms for the selected stage.
    pub fn set_timeout(&mut self, stage: RwdtStage, timeout: Duration) {
        let rtc_cntl = LP_WDT::regs();

        let timeout_raw = (timeout.as_millis() * (RtcClock::cycles_to_1ms() as u64)) as u32;
        self.set_write_protection(false);

        let config_reg = match stage {
            RwdtStage::Stage0 => rtc_cntl.wdtconfig1(),
            RwdtStage::Stage1 => rtc_cntl.wdtconfig2(),
            RwdtStage::Stage2 => rtc_cntl.wdtconfig3(),
            RwdtStage::Stage3 => rtc_cntl.wdtconfig4(),
        };

        #[cfg(not(esp32))]
        let timeout_raw = timeout_raw >> (1 + Efuse::rwdt_multiplier());

        config_reg.modify(|_, w| unsafe { w.hold().bits(timeout_raw) });

        self.set_write_protection(true);
    }

    /// Set the action for a specific stage.
    pub fn set_stage_action(&mut self, stage: RwdtStage, action: RwdtStageAction) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);
        rtc_cntl.wdtconfig0().modify(|_, w| unsafe {
            match stage {
                RwdtStage::Stage0 => w.wdt_stg0().bits(action as u8),
                RwdtStage::Stage1 => w.wdt_stg1().bits(action as u8),
                RwdtStage::Stage2 => w.wdt_stg2().bits(action as u8),
                RwdtStage::Stage3 => w.wdt_stg3().bits(action as u8),
            }
        });

        self.set_write_protection(true);
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
/// Super Watchdog
pub struct Swd;

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
/// Super Watchdog driver
impl Swd {
    /// Create a new super watchdog timer instance
    pub fn new() -> Self {
        Self
    }

    /// Enable the watchdog timer instance
    pub fn enable(&mut self) {
        self.set_enabled(true);
    }

    /// Disable the watchdog timer instance
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    /// Enable/disable write protection for WDT registers
    fn set_write_protection(&mut self, enable: bool) {
        let rtc_cntl = LP_WDT::regs();

        #[cfg(not(any(esp32c6, esp32h2)))]
        let wkey = if enable { 0u32 } else { 0x8F1D_312A };
        #[cfg(any(esp32c6, esp32h2))]
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        let rtc_cntl = LP_WDT::regs();

        self.set_write_protection(false);
        rtc_cntl
            .swd_conf()
            .write(|w| w.swd_auto_feed_en().bit(!enable));
        self.set_write_protection(true);
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
impl Default for Swd {
    fn default() -> Self {
        Self::new()
    }
}

/// Return reset reason.
pub fn reset_reason(cpu: Cpu) -> Option<SocResetReason> {
    let reason = crate::rom::rtc_get_reset_reason(cpu as u32);

    SocResetReason::from_repr(reason as usize)
}

/// Return wakeup reason.
pub fn wakeup_cause() -> SleepSource {
    if reset_reason(Cpu::ProCpu) != Some(SocResetReason::CoreDeepSleep) {
        return SleepSource::Undefined;
    }

    #[cfg(any(esp32c6, esp32h2))]
    let wakeup_cause = WakeupReason::from_bits_retain(
        crate::peripherals::PMU::regs()
            .slp_wakeup_status0()
            .read()
            .wakeup_cause()
            .bits(),
    );
    #[cfg(not(any(esp32, esp32c6, esp32h2)))]
    let wakeup_cause = WakeupReason::from_bits_retain(
        LPWR::regs().slp_wakeup_cause().read().wakeup_cause().bits(),
    );
    #[cfg(esp32)]
    let wakeup_cause = WakeupReason::from_bits_retain(
        LPWR::regs().wakeup_state().read().wakeup_cause().bits() as u32,
    );

    if wakeup_cause.contains(WakeupReason::TimerTrigEn) {
        return SleepSource::Timer;
    }
    if wakeup_cause.contains(WakeupReason::GpioTrigEn) {
        return SleepSource::Gpio;
    }
    if wakeup_cause.intersects(WakeupReason::Uart0TrigEn | WakeupReason::Uart1TrigEn) {
        return SleepSource::Uart;
    }

    #[cfg(pm_support_ext0_wakeup)]
    if wakeup_cause.contains(WakeupReason::ExtEvent0Trig) {
        return SleepSource::Ext0;
    }
    #[cfg(pm_support_ext1_wakeup)]
    if wakeup_cause.contains(WakeupReason::ExtEvent1Trig) {
        return SleepSource::Ext1;
    }

    #[cfg(pm_support_touch_sensor_wakeup)]
    if wakeup_cause.contains(WakeupReason::TouchTrigEn) {
        return SleepSource::TouchPad;
    }

    #[cfg(ulp_supported)]
    if wakeup_cause.contains(WakeupReason::UlpTrigEn) {
        return SleepSource::Ulp;
    }

    #[cfg(pm_support_wifi_wakeup)]
    if wakeup_cause.contains(WakeupReason::WifiTrigEn) {
        return SleepSource::Wifi;
    }

    #[cfg(pm_support_bt_wakeup)]
    if wakeup_cause.contains(WakeupReason::BtTrigEn) {
        return SleepSource::BT;
    }

    #[cfg(riscv_coproc_supported)]
    if wakeup_cause.contains(WakeupReason::CocpuTrigEn) {
        return SleepSource::Ulp;
    } else if wakeup_cause.contains(WakeupReason::CocpuTrapTrigEn) {
        return SleepSource::CocpuTrapTrig;
    }

    SleepSource::Undefined
}
