//! # Real-Time Clock Control and Low-power Management (RTC_CNTL)
//!
//! ## Overview
//! The RTC_CNTL peripheral is responsible for managing the real-time clock and
//! low-power modes on the chip.
//!
//! ## Configuration
//!  It also includes the necessary configurations and constants for clock
//! sources and low-power management. The driver provides the following features
//! and functionalities:
//!    * Clock Configuration
//!    * Calibration
//!    * Low-Power Management
//!    * Real-Time Clock
//!    * Handling Watchdog Timers
//!
//! ## Examples
//! ### Print Time in Milliseconds From the RTC Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use core::cell::RefCell;
//!
//! # use critical_section::Mutex;
//! # use esp_hal::delay::Delay;
//! # use esp_hal::rtc_cntl::Rtc;
//! # use esp_hal::rtc_cntl::Rwdt;
//! # use crate::esp_hal::prelude::_fugit_ExtU64;
//! # use crate::esp_hal::InterruptConfigurable;
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
//! let mut delay = Delay::new(&clocks);
//!
//! let mut rtc = Rtc::new(peripherals.LPWR);
//! rtc.set_interrupt_handler(interrupt_handler);
//! rtc.rwdt.set_timeout(2000.millis());
//! rtc.rwdt.listen();
//!
//! critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));
//!
//!
//! loop {}
//! # }
//!
//! // Where the `LP_WDT` interrupt handler is defined as:
//! // Handle the corresponding interrupt
//! # use core::cell::RefCell;
//!
//! # use critical_section::Mutex;
//! # use esp_hal::prelude::handler;
//! # use esp_hal::interrupt::InterruptHandler;
//! # use esp_hal::interrupt;
//! # use esp_hal::interrupt::Priority;
//! # use crate::esp_hal::prelude::_fugit_ExtU64;
//! # use esp_hal::rtc_cntl::Rwdt;
//! static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));
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
//!         rwdt.set_timeout(5000u64.millis());
//!         rwdt.unlisten();
//!     });
//! }
//! ```

#![allow(missing_docs)] // TODO: Remove when able

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
use crate::peripherals::{LP_TIMER, LP_WDT};
#[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
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
// only include sleep where its been implemented
#[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
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
pub use rtc::RtcClock;

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
    pub rwdt: Rwdt,
    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
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
            rwdt: Rwdt::default(),
            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            swd: Swd::new(),
        };

        #[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
        RtcSleepConfig::base_settings(&this);

        this
    }

    /// Return estimated XTAL frequency in MHz.
    pub fn estimate_xtal_frequency(&mut self) -> u32 {
        RtcClock::estimate_xtal_frequency()
    }

    /// Read the current value of the rtc time registers.
    pub fn get_time_raw(&self) -> u64 {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::ptr() };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_TIMER::ptr() };

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

    /// Read the current value of the rtc time registers in microseconds.
    pub fn get_time_us(&self) -> u64 {
        self.get_time_raw() * 1_000_000 / RtcClock::get_slow_freq().frequency().to_Hz() as u64
    }

    /// Read the current value of the rtc time registers in milliseconds.
    pub fn get_time_ms(&self) -> u64 {
        self.get_time_raw() * 1_000 / RtcClock::get_slow_freq().frequency().to_Hz() as u64
    }

    /// Enter deep sleep and wake with the provided `wake_sources`.
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
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

#[cfg(not(any(esp32c6, esp32h2)))]
/// RTC Watchdog Timer.
pub struct RtcClock;

#[cfg(not(any(esp32c6, esp32h2)))]
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

    /// Get main XTAL frequency.
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG by the
    /// bootloader, as passed to rtc_clk_init function.
    pub fn get_xtal_freq() -> XtalClock {
        let xtal_freq_reg = unsafe { &*LPWR::PTR }.store4().read().bits();

        // Values of RTC_XTAL_FREQ_REG and RTC_APB_FREQ_REG are stored as two copies in
        // lower and upper 16-bit halves. These are the routines to work with such a
        // representation.
        let clk_val_is_valid = |val| {
            (val & 0xffffu32) == ((val >> 16u32) & 0xffffu32) && val != 0u32 && val != u32::MAX
        };
        let reg_val_to_clk_val = |val| val & u16::MAX as u32;

        if !clk_val_is_valid(xtal_freq_reg) {
            return XtalClock::RtcXtalFreq40M;
        }

        match reg_val_to_clk_val(xtal_freq_reg) {
            40 => XtalClock::RtcXtalFreq40M,
            #[cfg(any(esp32c3, esp32s3))]
            32 => XtalClock::RtcXtalFreq32M,
            #[cfg(any(esp32, esp32c2))]
            26 => XtalClock::RtcXtalFreq26M,
            other => XtalClock::RtcXtalFreqOther(other),
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
    fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::get_xtal_freq();
        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
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
enum RwdtStageAction {
    Off         = 0,
    Interrupt   = 1,
    ResetCpu    = 2,
    ResetSystem = 3,
    ResetRtc    = 4,
}

/// RTC Watchdog Timer.
pub struct Rwdt {
    stg0_action: RwdtStageAction,
    stg1_action: RwdtStageAction,
    stg2_action: RwdtStageAction,
    stg3_action: RwdtStageAction,
}

impl Default for Rwdt {
    fn default() -> Self {
        Self {
            stg0_action: RwdtStageAction::ResetRtc,
            stg1_action: RwdtStageAction::Off,
            stg2_action: RwdtStageAction::Off,
            stg3_action: RwdtStageAction::Off,
        }
    }
}

/// RTC Watchdog Timer driver.
impl Rwdt {
    /// Enable the watchdog timer instance.
    pub fn enable(&mut self) {
        self.set_enabled(true);
    }

    /// Disable the watchdog timer instance.
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    /// Listen for interrupts.
    pub fn listen(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.stg0_action = RwdtStageAction::Interrupt;

        self.set_write_protection(false);

        // Configure STAGE0 to trigger an interrupt upon expiration
        rtc_cntl
            .wdtconfig0()
            .modify(|_, w| unsafe { w.wdt_stg0().bits(self.stg0_action as u8) });

        rtc_cntl.int_ena().modify(|_, w| w.wdt().set_bit());

        self.set_write_protection(true);
    }

    /// Stop listening for interrupts.
    pub fn unlisten(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.stg0_action = RwdtStageAction::ResetRtc;

        self.set_write_protection(false);

        // Configure STAGE0 to reset the main system and the RTC upon expiration.
        rtc_cntl
            .wdtconfig0()
            .modify(|_, w| unsafe { w.wdt_stg0().bits(self.stg0_action as u8) });

        rtc_cntl.int_ena().modify(|_, w| w.wdt().clear_bit());

        self.set_write_protection(true);
    }

    /// Clear interrupt.
    pub fn clear_interrupt(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.set_write_protection(false);

        rtc_cntl.int_clr().write(|w| w.wdt().clear_bit_by_one());

        self.set_write_protection(true);
    }

    /// Check if the interrupt is set.
    pub fn is_interrupt_set(&self) -> bool {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        rtc_cntl.int_st().read().wdt().bit_is_set()
    }

    /// Feed the watchdog timer.
    pub fn feed(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.set_write_protection(false);
        rtc_cntl.wdtfeed().write(|w| w.wdt_feed().set_bit());
        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl.wdtwprotect().write(|w| unsafe { w.bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        self.set_write_protection(false);

        rtc_cntl
            .wdtconfig0()
            .modify(|_, w| w.wdt_en().bit(enable).wdt_flashboot_mod_en().bit(enable));

        self.set_write_protection(true);
    }

    /// Configure timeout value in ms.
    pub fn set_timeout(&mut self, timeout: MicrosDurationU64) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        let timeout_raw = (timeout.to_millis() * (RtcClock::cycles_to_1ms() as u64)) as u32;
        self.set_write_protection(false);

        unsafe {
            #[cfg(esp32)]
            rtc_cntl
                .wdtconfig1()
                .modify(|_, w| w.wdt_stg0_hold().bits(timeout_raw));

            #[cfg(any(esp32c6, esp32h2))]
            rtc_cntl.config1().modify(|_, w| {
                w.wdt_stg0_hold()
                    .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
            });

            #[cfg(not(any(esp32, esp32c6, esp32h2)))]
            rtc_cntl.wdtconfig1().modify(|_, w| {
                w.wdt_stg0_hold()
                    .bits(timeout_raw >> (1 + Efuse::get_rwdt_multiplier()))
            });

            rtc_cntl.wdtconfig0().modify(|_, w| {
                w.wdt_stg0()
                    .bits(self.stg0_action as u8)
                    .wdt_cpu_reset_length()
                    .bits(7)
                    .wdt_sys_reset_length()
                    .bits(7)
                    .wdt_stg1()
                    .bits(self.stg1_action as u8)
                    .wdt_stg2()
                    .bits(self.stg2_action as u8)
                    .wdt_stg3()
                    .bits(self.stg3_action as u8)
                    .wdt_en()
                    .set_bit()
            });
        }

        self.set_write_protection(true);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::watchdog::WatchdogDisable for Rwdt {
    fn disable(&mut self) {
        self.disable();
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::watchdog::WatchdogEnable for Rwdt {
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.set_timeout(period.into());
    }
}

#[cfg(feature = "embedded-hal-02")]
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

    /// Disable the watchdog timer instance
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    /// Enable/disable write protection for WDT registers
    fn set_write_protection(&mut self, enable: bool) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

        #[cfg(not(any(esp32c6, esp32h2)))]
        let wkey = if enable { 0u32 } else { 0x8F1D_312A };
        #[cfg(any(esp32c6, esp32h2))]
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        rtc_cntl
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let rtc_cntl = unsafe { &*LPWR::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let rtc_cntl = unsafe { &*LP_WDT::PTR };

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

#[cfg(all(
    any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3),
    feature = "embedded-hal-02"
))]
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
