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

pub use self::rtc::SocResetReason;
#[cfg(not(esp32))]
use crate::efuse::Efuse;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2, esp32h2))]
use crate::rtc_cntl::sleep::{RtcSleepConfig, WakeSource, WakeTriggers};
use crate::{
    clock::{Clock, RtcClock},
    interrupt::{self, InterruptHandler},
    peripherals::{Interrupt, LPWR},
    system::{Cpu, SleepSource},
    time::Duration,
};
// only include sleep where it's been implemented
#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2, esp32h2))]
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

/// Clock source to be calibrated using `rtc_clk_cal` function
#[allow(unused)]
#[cfg(not(any(esp32c6, esp32h2)))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcMux      = 0,
    /// Internal 8 MHz RC oscillator, divided by 256
    _8mD256     = 1,
    /// External 32 KHz XTAL
    _32kXtal    = 2,
    /// Internal 150 KHz RC oscillator
    #[cfg(not(esp32))]
    InternalOsc = 3,
}

/// Clock source to be calibrated using `rtc_clk_cal` function
#[cfg(any(esp32c6, esp32h2))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcMux      = -1,
    /// Internal 150kHz RC oscillator
    RcSlow      = 0,
    /// External 32kHz XTAL, as one type of 32k clock
    _32kXtal    = 1,
    /// Internal 32kHz RC oscillator, as one type of 32k clock
    _32kRc      = 2,
    /// External slow clock signal input by lp_pad_gpio0, as one type of 32k
    /// clock
    _32kOscSlow = 3,
    /// Internal MHz-range RC oscillator
    RcFast,
}

/// Low-power Management
pub struct Rtc<'d> {
    _inner: LPWR<'d>,
    /// Reset Watchdog Timer.
    pub rwdt: Rwdt,
    /// Super Watchdog
    #[cfg(swd)]
    pub swd: Swd,
}

impl<'d> Rtc<'d> {
    /// Create a new instance in [crate::Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rtc_cntl: LPWR<'d>) -> Self {
        Self {
            _inner: rtc_cntl,
            rwdt: Rwdt(()),
            #[cfg(swd)]
            swd: Swd(()),
        }
    }

    /// Return estimated XTAL frequency in MHz.
    pub fn estimate_xtal_frequency(&mut self) -> u32 {
        RtcClock::estimate_xtal_frequency()
    }

    /// Get the time since boot in the raw register units.
    fn time_since_boot_raw(&self) -> u64 {
        let rtc_cntl = LP_TIMER::regs();

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                rtc_cntl.time_update().write(|w| w.time_update().set_bit());
                while rtc_cntl.time_update().read().time_valid().bit_is_clear() {
                    // Might take 1 RTC slowclk period, don't flood RTC bus
                    crate::rom::ets_delay_us(1);
                }

                let h = rtc_cntl.time1().read().time_hi().bits();
                let l = rtc_cntl.time0().read().time_lo().bits();
            } else if #[cfg(any(esp32c6, esp32h2))] {
                rtc_cntl.update().write(|w| w.main_timer_update().set_bit());

                let h = rtc_cntl
                    .main_buf0_high()
                    .read()
                    .main_timer_buf0_high()
                    .bits();
                let l = rtc_cntl.main_buf0_low().read().main_timer_buf0_low().bits();
            } else {
                rtc_cntl.time_update().write(|w| w.time_update().set_bit());

                let h = rtc_cntl.time_high0().read().timer_value0_high().bits();
                let l = rtc_cntl.time_low0().read().timer_value0_low().bits();
            }
        }

        ((h as u64) << 32) | (l as u64)
    }

    /// Get the time elapsed since the last power-on reset.
    ///
    /// It should be noted that any reset or sleep, other than a power-up reset, will not stop or
    /// reset the RTC timer.
    pub fn time_since_power_up(&self) -> Duration {
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

        let l = rtc_cntl.store2().read().bits() as u64;
        let h = rtc_cntl.store3().read().bits() as u64;

        // https://github.com/espressif/esp-idf/blob/23e4823f17a8349b5e03536ff7653e3e584c9351/components/newlib/port/esp_time_impl.c#L115
        l + (h << 32)
    }

    /// Set the current value of the boot time registers in microseconds.
    fn set_boot_time_us(&self, boot_time_us: u64) {
        // Please see `boot_time_us` for documentation on registers and peripherals
        // used for certain SOCs.

        let rtc_cntl = LP_AON::regs();

        // https://github.com/espressif/esp-idf/blob/23e4823/components/newlib/port/esp_time_impl.c#L102-L103
        rtc_cntl
            .store2() // Low bits
            .write(|w| unsafe { w.bits((boot_time_us & 0xffff_ffff) as u32) });
        rtc_cntl
            .store3() // High bits
            .write(|w| unsafe { w.bits((boot_time_us >> 32) as u32) });
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

        let rtc_time_us = self.time_since_power_up().as_micros();
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

        let rtc_time_us = self.time_since_power_up().as_micros();
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
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2, esp32h2))]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2, esp32h2))]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32c2, esp32h2))]
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

    pub(crate) const RTC_DISABLE_ROM_LOG: u32 = 1;

    /// Temporarily disable log messages of the ROM bootloader.
    ///
    /// If you need to permanently disable the ROM bootloader messages, you'll
    /// need to set the corresponding eFuse.
    pub fn disable_rom_message_printing(&self) {
        // Corresponding documentation:
        // ESP32-S3: TRM v1.5 chapter 8.3
        // ESP32-H2: TRM v0.5 chapter 8.2.3

        LP_AON::regs()
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
pub struct Rwdt(());

/// RTC Watchdog Timer driver.
impl Rwdt {
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
        self.set_write_protection(false);

        LP_WDT::regs()
            .int_clr()
            .write(|w| w.wdt().clear_bit_by_one());

        self.set_write_protection(true);
    }

    /// Check if the interrupt is set.
    pub fn is_interrupt_set(&self) -> bool {
        LP_WDT::regs().int_st().read().wdt().bit_is_set()
    }

    /// Feed the watchdog timer.
    pub fn feed(&mut self) {
        self.set_write_protection(false);
        LP_WDT::regs().wdtfeed().write(|w| w.wdt_feed().set_bit());
        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        LP_WDT::regs()
            .wdtwprotect()
            .write(|w| unsafe { w.bits(wkey) });
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
        self.set_write_protection(false);

        LP_WDT::regs().wdtconfig0().modify(|_, w| unsafe {
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

/// Super Watchdog
#[cfg(swd)]
pub struct Swd(());

/// Super Watchdog driver
#[cfg(swd)]
impl Swd {
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
        #[cfg(not(any(esp32c6, esp32h2)))]
        let wkey = if enable { 0u32 } else { 0x8F1D_312A };
        #[cfg(any(esp32c6, esp32h2))]
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        LP_WDT::regs()
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        self.set_write_protection(false);

        LP_WDT::regs()
            .swd_conf()
            .write(|w| w.swd_auto_feed_en().bit(!enable));

        self.set_write_protection(true);
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

    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            let wakeup_cause_bits = LPWR::regs().wakeup_state().read().wakeup_cause().bits() as u32;
        } else if #[cfg(any(esp32c6, esp32h2))] {
            let wakeup_cause_bits = crate::peripherals::PMU::regs()
                .slp_wakeup_status0()
                .read()
                .wakeup_cause()
                .bits();
        } else {
            let wakeup_cause_bits = LPWR::regs().slp_wakeup_cause().read().wakeup_cause().bits();
        }
    }

    let wakeup_cause = WakeupReason::from_bits_retain(wakeup_cause_bits);

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
