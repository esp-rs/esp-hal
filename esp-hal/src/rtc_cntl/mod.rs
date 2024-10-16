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
//!    * Clock Configuration
//!    * Calibration
//!    * Low-Power Management
//!    * Handling Watchdog Timers
//!
//! ## Example
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use core::cell::RefCell;
//!
//! # use critical_section::Mutex;
//! # use esp_hal::delay::Delay;
//! # use esp_hal::rtc_cntl::Rtc;
//! # use esp_hal::rtc_cntl::Rwdt;
//! # use crate::esp_hal::InterruptConfigurable;
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
//! let mut delay = Delay::new();
//!
//! let mut rtc = Rtc::new(peripherals.LPWR);
//!
//! rtc.set_interrupt_handler(interrupt_handler);
//! rtc.rwdt.set_timeout(0, 2000.millis());
//! rtc.rwdt.listen();
//!
//! critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));
//! # }
//!
//! // Where the `LP_WDT` interrupt handler is defined as:
//! # use core::cell::RefCell;
//!
//! # use critical_section::Mutex;
//! # use esp_hal::rtc_cntl::Rwdt;
//!
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
//!
//! // Handle the corresponding interrupt
//! #[handler]
//! fn interrupt_handler() {
//!     critical_section::with(|cs| {
//!         // esp_println::println!("RWDT Interrupt");
//!
//!         let mut rwdt = RWDT.borrow_ref_mut(cs);
//!         let rwdt = rwdt.as_mut().unwrap();
//!         rwdt.clear_interrupt();
//!
//!         // esp_println::println!("Restarting in 5 seconds...");
//!
//!         rwdt.set_timeout(0, 5000u64.millis());
//!         rwdt.unlisten();
//!     });
//! }
//! ```

use chrono::{DateTime, NaiveDateTime};
#[cfg(not(any(esp32c6, esp32h2)))]
use fugit::HertzU32;
use fugit::MicrosDurationU64;

pub use self::rtc::SocResetReason;
#[cfg(not(any(esp32c6, esp32h2)))]
use crate::clock::XtalClock;
#[cfg(not(esp32))]
use crate::efuse::Efuse;
#[cfg(not(any(esp32c6, esp32h2)))]
use crate::peripherals::{LPWR, TIMG0};
#[cfg(any(esp32c6, esp32h2))]
use crate::peripherals::{LP_AON, LP_TIMER, LP_WDT};
#[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
use crate::rtc_cntl::sleep::{RtcSleepConfig, WakeSource, WakeTriggers};
use crate::{
    clock::Clock,
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    reset::{SleepSource, WakeupReason},
    Cpu,
    InterruptConfigurable,
};
// only include sleep where it's been implemented
#[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
pub mod sleep;

#[cfg_attr(esp32, path = "rtc/esp32.rs")]
#[cfg_attr(esp32c2, path = "rtc/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "rtc/esp32c3.rs")]
#[cfg_attr(esp32c6, path = "rtc/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "rtc/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "rtc/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "rtc/esp32s3.rs")]
pub(crate) mod rtc;

#[cfg(any(esp32c6, esp32h2))]
unsafe fn lp_aon<'a>() -> &'a <LP_AON as core::ops::Deref>::Target {
    &*LP_AON::PTR
}

#[cfg(not(any(esp32c6, esp32h2)))]
unsafe fn lp_aon<'a>() -> &'a <LPWR as core::ops::Deref>::Target {
    &*LPWR::PTR
}

#[cfg(any(esp32c6, esp32h2))]
unsafe fn lp_timer<'a>() -> &'a <LP_TIMER as core::ops::Deref>::Target {
    &*LP_TIMER::PTR
}

#[cfg(not(any(esp32c6, esp32h2)))]
unsafe fn lp_timer<'a>() -> &'a <LPWR as core::ops::Deref>::Target {
    &*LPWR::PTR
}

#[cfg(any(esp32c6, esp32h2))]
unsafe fn lp_wdt<'a>() -> &'a <LP_WDT as core::ops::Deref>::Target {
    &*LP_WDT::PTR
}

#[cfg(not(any(esp32c6, esp32h2)))]
unsafe fn lp_wdt<'a>() -> &'a <LPWR as core::ops::Deref>::Target {
    &*LPWR::PTR
}

#[cfg(not(any(esp32c6, esp32h2)))]
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
/// RTC SLOW_CLK frequency values
pub(crate) enum RtcFastClock {
    /// Main XTAL, divided by 4
    RtcFastClockXtalD4 = 0,
    /// Internal fast RC oscillator
    RtcFastClock8m     = 1,
}

#[cfg(not(any(esp32c6, esp32h2)))]
impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcFastClock::RtcFastClockXtalD4 => HertzU32::Hz(40_000_000 / 4),
            #[cfg(any(esp32, esp32s2))]
            RtcFastClock::RtcFastClock8m => HertzU32::Hz(8_500_000),
            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            RtcFastClock::RtcFastClock8m => HertzU32::Hz(17_500_000),
        }
    }
}

#[cfg(not(any(esp32c6, esp32h2)))]
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
/// RTC SLOW_CLK frequency values
pub enum RtcSlowClock {
    /// Internal slow RC oscillator
    RtcSlowClockRtc     = 0,
    /// External 32 KHz XTAL
    RtcSlowClock32kXtal = 1,
    /// Internal fast RC oscillator, divided by 256
    RtcSlowClock8mD256  = 2,
}

#[cfg(not(any(esp32c6, esp32h2)))]
impl Clock for RtcSlowClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(esp32)]
            RtcSlowClock::RtcSlowClockRtc => HertzU32::Hz(150_000),
            #[cfg(esp32s2)]
            RtcSlowClock::RtcSlowClockRtc => HertzU32::Hz(90_000),
            #[cfg(any(esp32c2, esp32c3, esp32s3))]
            RtcSlowClock::RtcSlowClockRtc => HertzU32::Hz(136_000),
            RtcSlowClock::RtcSlowClock32kXtal => HertzU32::Hz(32_768),
            #[cfg(any(esp32, esp32s2))]
            RtcSlowClock::RtcSlowClock8mD256 => HertzU32::Hz(8_500_000 / 256),
            #[cfg(any(esp32c2, esp32c3, esp32s3))]
            RtcSlowClock::RtcSlowClock8mD256 => HertzU32::Hz(17_500_000 / 256),
        }
    }
}

#[allow(unused)]
#[cfg(not(any(esp32c6, esp32h2)))]
#[derive(Debug, Clone, Copy)]
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

/// Low-power Management
pub struct Rtc<'d> {
    _inner: PeripheralRef<'d, crate::peripherals::LPWR>,
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
    pub fn new(rtc_cntl: impl Peripheral<P = crate::peripherals::LPWR> + 'd) -> Self {
        rtc::init();
        rtc::configure_clock();

        let this = Self {
            _inner: rtc_cntl.into_ref(),
            rwdt: Rwdt::new(),
            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            swd: Swd::new(),
        };

        #[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
        RtcSleepConfig::base_settings(&this);

        this
    }

    /// Return estimated XTAL frequency in MHz.
    pub fn estimate_xtal_frequency(&mut self) -> u32 {
        RtcClock::estimate_xtal_frequency()
    }

    /// Get the time since boot in the raw register units.
    fn time_since_boot_raw(&self) -> u64 {
        let rtc_cntl = unsafe { lp_timer() };

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
        #[cfg(any(esp32c2, esp32c3, esp32s3, esp32s2))]
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
    pub fn time_since_boot(&self) -> MicrosDurationU64 {
        MicrosDurationU64::micros(
            self.time_since_boot_raw() * 1_000_000
                / RtcClock::get_slow_freq().frequency().to_Hz() as u64,
        )
    }

    /// Read the current value of the boot time registers in microseconds.
    fn boot_time_us(&self) -> u64 {
        // For more info on about how RTC setting works and what it has to do with boot time, see https://github.com/esp-rs/esp-hal/pull/1883

        // In terms of registers, STORE2 and STORE3 are used on all current chips
        // (esp32, esp32p4, esp32h2, esp32c2, esp32c3, esp32c5, esp32c6, esp32c61,
        // esp32s2, esp32s3)

        // In terms of peripherals:

        // - LPWR is used on the following chips: esp32, esp32p4, esp32c2, esp32c3,
        //   esp32s2, esp32s3

        // - LP_AON is used on the following chips: esp32c5, esp32c6, esp32c61, esp32h2

        // For registers and peripherals used in esp-idf, see https://github.com/search?q=repo%3Aespressif%2Fesp-idf+RTC_BOOT_TIME_LOW_REG+RTC_BOOT_TIME_HIGH_REG+path%3A**%2Frtc.h&type=code

        let rtc_cntl = unsafe { lp_aon() };

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

        let rtc_cntl = unsafe { lp_aon() };

        let (l, h) = (rtc_cntl.store2(), rtc_cntl.store3());

        // https://github.com/espressif/esp-idf/blob/23e4823f17a8349b5e03536ff7653e3e584c9351/components/newlib/port/esp_time_impl.c#L102-L103
        l.write(|w| unsafe { w.bits((boot_time_us & 0xffffffff) as u32) });
        h.write(|w| unsafe { w.bits((boot_time_us >> 32) as u32) });
    }

    /// Get the current time.
    pub fn current_time(&self) -> NaiveDateTime {
        // Current time is boot time + time since boot

        let rtc_time_us = self.time_since_boot().to_micros();
        let boot_time_us = self.boot_time_us();
        let wrapped_boot_time_us = u64::MAX - boot_time_us;

        // We can detect if we wrapped the boot time by checking if rtc time is greater
        // than the amount of time we would've wrapped.
        let current_time_us = if rtc_time_us > wrapped_boot_time_us {
            // We also just checked that this won't overflow
            rtc_time_us - wrapped_boot_time_us
        } else {
            boot_time_us + rtc_time_us
        };

        DateTime::from_timestamp_micros(current_time_us as i64)
            .unwrap()
            .naive_utc()
    }

    /// Set the current time.
    ///
    /// # Panics
    ///
    /// Panics if `current_time` is before the Unix epoch (meaning the
    /// underlying timestamp is negative).
    pub fn set_current_time(&self, current_time: NaiveDateTime) {
        let current_time_us: u64 = current_time
            .and_utc()
            .timestamp_micros()
            .try_into()
            .expect("current_time is negative");

        // Current time is boot time + time since boot (rtc time)
        // So boot time = current time - time since boot (rtc time)

        let rtc_time_us = self.time_since_boot().to_micros();
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
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6, esp32c2))]
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

        let rtc_cntl = unsafe { lp_aon() };
        rtc_cntl
            .store4()
            .modify(|r, w| unsafe { w.bits(r.bits() | Self::RTC_DISABLE_ROM_LOG) });
    }
}
impl<'d> crate::private::Sealed for Rtc<'d> {}

impl<'d> InterruptConfigurable for Rtc<'d> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            interrupt::bind_interrupt(
                #[cfg(any(esp32c6, esp32h2))]
                Interrupt::LP_WDT,
                #[cfg(not(any(esp32c6, esp32h2)))]
                Interrupt::RTC_CORE,
                handler.handler(),
            );
            interrupt::enable(
                #[cfg(any(esp32c6, esp32h2))]
                Interrupt::LP_WDT,
                #[cfg(not(any(esp32c6, esp32h2)))]
                Interrupt::RTC_CORE,
                handler.priority(),
            )
            .unwrap();
        }
    }
}

/// RTC Watchdog Timer.
pub struct RtcClock;

/// RTC Watchdog Timer driver.
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    /// Enable or disable 8 MHz internal oscillator.
    ///
    /// Output from 8 MHz internal oscillator is passed into a configurable
    /// divider, which by default divides the input clock frequency by 256.
    /// Output of the divider may be used as RTC_SLOW_CLK source.
    /// Output of the divider is referred to in register descriptions and code
    /// as 8md256 or simply d256. Divider values other than 256 may be
    /// configured, but this facility is not currently needed, so is not
    /// exposed in the code.
    ///
    /// When 8MHz/256 divided output is not needed, the divider should be
    /// disabled to reduce power consumption.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn enable_8m(clk_8m_en: bool, d256_en: bool) {
        let rtc_cntl = unsafe { &*LPWR::PTR };

        if clk_8m_en {
            rtc_cntl.clk_conf().modify(|_, w| w.enb_ck8m().clear_bit());
            unsafe {
                rtc_cntl.timer1().modify(|_, w| w.ck8m_wait().bits(5));
            }
            crate::rom::ets_delay_us(50);
        } else {
            rtc_cntl.clk_conf().modify(|_, w| w.enb_ck8m().set_bit());
            rtc_cntl
                .timer1()
                .modify(|_, w| unsafe { w.ck8m_wait().bits(20) });
        }

        if d256_en {
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.enb_ck8m_div().clear_bit());
        } else {
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.enb_ck8m_div().set_bit());
        }
    }

    pub(crate) fn read_xtal_freq_mhz() -> Option<u32> {
        let xtal_freq_reg = unsafe { lp_aon() }.store4().read().bits();

        // RTC_XTAL_FREQ is stored as two copies in lower and upper 16-bit halves
        // need to mask out the RTC_DISABLE_ROM_LOG bit which is also stored in the same
        // register
        let xtal_freq = (xtal_freq_reg & !Rtc::RTC_DISABLE_ROM_LOG) as u16;
        let xtal_freq_copy = (xtal_freq_reg >> 16) as u16;

        if xtal_freq == xtal_freq_copy && xtal_freq != 0 && xtal_freq != u16::MAX {
            Some(xtal_freq as u32)
        } else {
            None
        }
    }

    /// Get main XTAL frequency.
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG by the
    /// bootloader, as passed to rtc_clk_init function.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub fn get_xtal_freq() -> XtalClock {
        match Self::read_xtal_freq_mhz() {
            None | Some(40) => XtalClock::RtcXtalFreq40M,
            #[cfg(any(esp32c3, esp32s3))]
            Some(32) => XtalClock::RtcXtalFreq32M,
            #[cfg(any(esp32, esp32c2))]
            Some(26) => XtalClock::RtcXtalFreq26M,
            Some(other) => XtalClock::RtcXtalFreqOther(other),
        }
    }

    /// Get the RTC_SLOW_CLK source.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub fn get_slow_freq() -> RtcSlowClock {
        let rtc_cntl = unsafe { &*LPWR::PTR };
        let slow_freq = rtc_cntl.clk_conf().read().ana_clk_rtc_sel().bits();
        match slow_freq {
            0 => RtcSlowClock::RtcSlowClockRtc,
            1 => RtcSlowClock::RtcSlowClock32kXtal,
            2 => RtcSlowClock::RtcSlowClock8mD256,
            _ => unreachable!(),
        }
    }

    /// Select source for RTC_SLOW_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn set_slow_freq(slow_freq: RtcSlowClock) {
        unsafe {
            let rtc_cntl = &*LPWR::PTR;
            rtc_cntl.clk_conf().modify(|_, w| {
                w.ana_clk_rtc_sel()
                    .bits(slow_freq as u8)
                    // Why we need to connect this clock to digital?
                    // Or maybe this clock should be connected to digital when
                    // XTAL 32k clock is enabled instead?
                    .dig_xtal32k_en()
                    .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kXtal))
                    // The clk_8m_d256 will be closed when rtc_state in SLEEP,
                    // so if the slow_clk is 8md256, clk_8m must be force power on
                    .ck8m_force_pu()
                    .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock8mD256))
            });
        };

        crate::rom::ets_delay_us(300u32);
    }

    /// Select source for RTC_FAST_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn set_fast_freq(fast_freq: RtcFastClock) {
        unsafe {
            let rtc_cntl = &*LPWR::PTR;
            rtc_cntl.clk_conf().modify(|_, w| {
                w.fast_clk_rtc_sel().bit(match fast_freq {
                    RtcFastClock::RtcFastClock8m => true,
                    RtcFastClock::RtcFastClockXtalD4 => false,
                })
            });
        };

        crate::rom::ets_delay_us(3u32);
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn calibrate_internal(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        // Except for ESP32, choosing RTC_CAL_RTC_MUX results in calibration of
        // the 150k RTC clock (90k on ESP32-S2) regardless of the currently selected
        // SLOW_CLK. On the ESP32, it uses the currently selected SLOW_CLK.
        // The following code emulates ESP32 behavior for the other chips:
        #[cfg(not(esp32))]
        let cal_clk = match cal_clk {
            RtcCalSel::RtcCalRtcMux => match RtcClock::get_slow_freq() {
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock8mD256 => RtcCalSel::RtcCal8mD256,
                _ => cal_clk,
            },
            RtcCalSel::RtcCalInternalOsc => RtcCalSel::RtcCalRtcMux,
            _ => cal_clk,
        };
        let rtc_cntl = unsafe { &*LPWR::PTR };
        let timg0 = unsafe { &*TIMG0::PTR };

        // Enable requested clock (150k clock is always on)
        let dig_32k_xtal_enabled = rtc_cntl.clk_conf().read().dig_xtal32k_en().bit_is_set();

        if matches!(cal_clk, RtcCalSel::RtcCal32kXtal) && !dig_32k_xtal_enabled {
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.dig_xtal32k_en().set_bit());
        }

        if matches!(cal_clk, RtcCalSel::RtcCal8mD256) {
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.dig_clk8m_d256_en().set_bit());
        }

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
            w.rtc_cali_clk_sel()
                .bits(cal_clk as u8)
                .rtc_cali_start_cycling()
                .clear_bit()
                .rtc_cali_max()
                .bits(slowclk_cycles as u16)
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
                RtcSlowClock::RtcSlowClockRtc
            }
        };

        let us_time_estimate = HertzU32::MHz(slowclk_cycles) / expected_freq.frequency();

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
        rtc_cntl
            .clk_conf()
            .modify(|_, w| w.dig_xtal32k_en().bit(dig_32k_xtal_enabled));

        if matches!(cal_clk, RtcCalSel::RtcCal8mD256) {
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.dig_clk8m_d256_en().clear_bit());
        }

        cal_val
    }

    /// Measure ratio between XTAL frequency and RTC slow clock frequency.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn get_calibration_ratio(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let ratio = (xtal_cycles << RtcClock::CAL_FRACT) / slowclk_cycles as u64;

        (ratio & (u32::MAX as u64)) as u32
    }

    /// Measure RTC slow clock's period, based on main XTAL frequency.
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::get_xtal_freq();
        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    #[cfg(not(any(esp32c6, esp32h2)))]
    fn cycles_to_1ms() -> u16 {
        let period_13q19 = RtcClock::calibrate(
            match RtcClock::get_slow_freq() {
                RtcSlowClock::RtcSlowClockRtc => RtcCalSel::RtcCalRtcMux,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                #[cfg(not(any(esp32c6, esp32h2)))]
                RtcSlowClock::RtcSlowClock8mD256 => RtcCalSel::RtcCal8mD256,
            },
            1024,
        );

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
    }

    /// Return estimated XTAL frequency in MHz.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub(crate) fn estimate_xtal_frequency() -> u32 {
        // Number of 8M/256 clock cycles to use for XTAL frequency estimation.
        const XTAL_FREQ_EST_CYCLES: u32 = 10;

        let rtc_cntl = unsafe { &*LPWR::PTR };
        let clk_8m_enabled = rtc_cntl.clk_conf().read().enb_ck8m().bit_is_clear();
        let clk_8md256_enabled = rtc_cntl.clk_conf().read().enb_ck8m_div().bit_is_clear();

        if !clk_8md256_enabled {
            RtcClock::enable_8m(true, true);
        }

        let ratio = RtcClock::get_calibration_ratio(RtcCalSel::RtcCal8mD256, XTAL_FREQ_EST_CYCLES);
        let freq_mhz =
            ((ratio as u64 * RtcFastClock::RtcFastClock8m.hz() as u64 / 1_000_000u64 / 256u64)
                >> RtcClock::CAL_FRACT) as u32;

        RtcClock::enable_8m(clk_8m_enabled, clk_8md256_enabled);

        freq_mhz
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

/// RWDT related errors.
#[derive(Debug)]
pub enum RwdtError {
    /// Trying to configure the wrong stage.
    InvalidStage,
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
        let rtc_cntl = unsafe { lp_wdt() };

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
        let rtc_cntl = unsafe { lp_wdt() };

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
        let rtc_cntl = unsafe { lp_wdt() };

        self.set_write_protection(false);

        rtc_cntl.int_clr().write(|w| w.wdt().clear_bit_by_one());

        self.set_write_protection(true);
    }

    /// Check if the interrupt is set.
    pub fn is_interrupt_set(&self) -> bool {
        let rtc_cntl = unsafe { lp_wdt() };

        rtc_cntl.int_st().read().wdt().bit_is_set()
    }

    /// Feed the watchdog timer.
    pub fn feed(&mut self) {
        let rtc_cntl = unsafe { lp_wdt() };

        self.set_write_protection(false);
        rtc_cntl.wdtfeed().write(|w| w.wdt_feed().set_bit());
        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        let rtc_cntl = unsafe { lp_wdt() };

        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl.wdtwprotect().write(|w| unsafe { w.bits(wkey) });
    }

        let rtc_cntl = unsafe { lpwr() };
        let rtc_cntl = unsafe { lp_wdt() };
    fn set_enabled(&mut self, enable: bool) {
        let rtc_cntl = unsafe { lp_wdt() };

        self.set_write_protection(false);

        if !enabled {
            rtc_cntl.wdtconfig0().modify(|_, w| unsafe { w.bits(0) });
        } else {
            rtc_cntl
                .wdtconfig0()
                .write(|w| w.wdt_flashboot_mod_en().bit(false));

            rtc_cntl
                .wdtconfig0()
                .modify(|_, w| w.wdt_en().bit(enabled).wdt_pause_in_slp().bit(enabled));

            // Apply default settings for WDT
            unsafe {
                rtc_cntl.wdtconfig0().modify(|_, w| {
                    w.wdt_stg0()
                        .bits(RwdtStageAction::ResetSystem as u8)
                        .wdt_cpu_reset_length()
                        .bits(7)
                        .wdt_sys_reset_length()
                        .bits(7)
                        .wdt_stg1()
                        .bits(RwdtStageAction::Off as u8)
                        .wdt_stg2()
                        .bits(RwdtStageAction::Off as u8)
                        .wdt_stg3()
                        .bits(RwdtStageAction::Off as u8)
                        .wdt_en()
                        .set_bit()
                });
            }
        }

        self.set_write_protection(true);
    }

    /// Configure timeout value in ms for the selected stage.
    pub fn set_timeout(
        &mut self,
        stage: usize,
        timeout: MicrosDurationU64,
    ) -> Result<(), RwdtError> {
        let rtc_cntl = unsafe { lp_wdt() };

        let timeout_raw = (timeout.to_millis() * (RtcClock::cycles_to_1ms() as u64)) as u32;
        self.set_write_protection(false);

        unsafe {
            #[cfg(esp32)]
            match stage {
                0 => rtc_cntl
                    .wdtconfig1()
                    .modify(|_, w| w.wdt_stg0_hold().bits(timeout_raw)),
                1 => rtc_cntl
                    .wdtconfig2()
                    .modify(|_, w| w.wdt_stg1_hold().bits(timeout_raw)),
                2 => rtc_cntl
                    .wdtconfig3()
                    .modify(|_, w| w.wdt_stg2_hold().bits(timeout_raw)),
                3 => rtc_cntl
                    .wdtconfig4()
                    .modify(|_, w| w.wdt_stg3_hold().bits(timeout_raw)),
                _ => return Err(RwdtError::InvalidStage),
            }

            #[cfg(any(esp32c6, esp32h2))]
            match stage {
                0 => rtc_cntl.config1().modify(|_, w| {
                    w.wdt_stg0_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                1 => rtc_cntl.config2().modify(|_, w| {
                    w.wdt_stg1_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                2 => rtc_cntl.config3().modify(|_, w| {
                    w.wdt_stg2_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                3 => rtc_cntl.config4().modify(|_, w| {
                    w.wdt_stg3_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                _ => return Err(RwdtError::InvalidStage),
            }

            #[cfg(not(any(esp32, esp32c6, esp32h2)))]
            match stage {
                0 => rtc_cntl.wdtconfig1().modify(|_, w| {
                    w.wdt_stg0_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                1 => rtc_cntl.wdtconfig2().modify(|_, w| {
                    w.wdt_stg1_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                2 => rtc_cntl.wdtconfig3().modify(|_, w| {
                    w.wdt_stg2_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                3 => rtc_cntl.wdtconfig4().modify(|_, w| {
                    w.wdt_stg3_hold()
                        .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
                }),
                _ => return Err(RwdtError::InvalidStage),
            }
        }

        self.set_write_protection(true);

        Ok(())
    }

    /// Set the action for a specific stage.
    pub fn set_stage_action(
        &mut self,
        stage: usize,
        action: RwdtStageAction,
    ) -> Result<(), RwdtError> {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.set_write_protection(false);

        match stage {
            0 => {
                rtc_cntl
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg0().bits(action as u8) });
            }
            1 => {
                rtc_cntl
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg1().bits(action as u8) });
            }
            2 => {
                rtc_cntl
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg2().bits(action as u8) });
            }
            3 => {
                rtc_cntl
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg3().bits(action as u8) });
            }
            _ => {
                return Err(RwdtError::InvalidStage);
            }
        }

        self.set_write_protection(true);

        Ok(())
    }
}

impl embedded_hal_02::watchdog::WatchdogDisable for Rwdt {
    fn disable(&mut self) {
        self.disable();
    }
}

impl embedded_hal_02::watchdog::WatchdogEnable for Rwdt {
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.set_timeout(0, period.into()).unwrap();
    }
}

impl embedded_hal_02::watchdog::Watchdog for Rwdt {
    fn feed(&mut self) {
        self.feed();
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
        let rtc_cntl = unsafe { lp_wdt() };

        #[cfg(not(any(esp32c6, esp32h2)))]
        let wkey = if enable { 0u32 } else { 0x8F1D_312A };
        #[cfg(any(esp32c6, esp32h2))]
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        let rtc_cntl = unsafe { lp_wdt() };

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

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
impl embedded_hal_02::watchdog::WatchdogDisable for Swd {
    fn disable(&mut self) {
        self.disable();
    }
}

/// Return reset reason.
pub fn get_reset_reason(cpu: Cpu) -> Option<SocResetReason> {
    let reason = crate::rom::rtc_get_reset_reason(cpu as u32);

    SocResetReason::from_repr(reason as usize)
}

/// Return wakeup reason.
pub fn get_wakeup_cause() -> SleepSource {
    if get_reset_reason(Cpu::ProCpu) != Some(SocResetReason::CoreDeepSleep) {
        return SleepSource::Undefined;
    }

    #[cfg(any(esp32c6, esp32h2))]
    let wakeup_cause = WakeupReason::from_bits_retain(unsafe {
        (*crate::peripherals::PMU::PTR)
            .slp_wakeup_status0()
            .read()
            .wakeup_cause()
            .bits()
    });
    #[cfg(not(any(esp32, esp32c6, esp32h2)))]
    let wakeup_cause = WakeupReason::from_bits_retain(unsafe {
        (*LPWR::PTR).slp_wakeup_cause().read().wakeup_cause().bits()
    });
    #[cfg(esp32)]
    let wakeup_cause = WakeupReason::from_bits_retain(unsafe {
        (*LPWR::PTR).wakeup_state().read().wakeup_cause().bits() as u32
    });

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

// libphy.a can pull this in on some chips, we provide it here in the hal
// so that either ieee or esp-wifi gets it for free without duplicating in both
#[no_mangle]
extern "C" fn rtc_clk_xtal_freq_get() -> i32 {
    let xtal = RtcClock::get_xtal_freq();
    xtal.mhz() as i32
}
