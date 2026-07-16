//! # RTC Control Sleep Module
//!
//! ## Overview
//! The `sleep` module allows configuring various wakeup sources and setting up
//! the sleep behavior based on those sources. The supported wakeup sources
//! include:
//!    * `GPIO` pins - light sleep only
//!    * timers
//!    * `SDIO (Secure Digital Input/Output) - light sleep only`
//!    * `MAC (Media Access Control)` wake - light sleep only
//!    * `UART0` - light sleep only
//!    * `UART1` - light sleep only
//!    * `touch`
//!    * `ULP (Ultra-Low Power)` wake
//!    * `BT (Bluetooth) wake` - light sleep only

use core::cell::RefCell;

use enumset::EnumSet;

#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::gpio::RtcPin as RtcIoWakeupPinType;
#[cfg(any(esp32c2, esp32c3, esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
use crate::gpio::RtcPinWithResistors as RtcIoWakeupPinType;
use crate::{
    peripherals::LPWR,
    rtc_cntl::{Rtc, WakeupSource},
};

#[cfg(soc_has_pmu)]
mod pmu_common;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c5, path = "esp32c5.rs")]
#[cfg_attr(esp32c61, path = "esp32c61.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
#[cfg_attr(esp32h2, path = "esp32h2.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
mod sleep_impl;
pub use sleep_impl::*;

mod wakeup_sources;
pub use wakeup_sources::*;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
/// Level at which a wake-up event is triggered
pub enum WakeupLevel {
    /// The wake-up event is triggered when the pin is low.
    Low,
    #[default]
    ///  The wake-up event is triggered when the pin is high.
    High,
}

#[procmacros::doc_replace]
/// External wake-up source (Ext0).
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext0WakeupSource, LowPower, TimerWakeupSource, WakeupLevel}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull};
/// # use esp_hal::time::Duration;
///
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_input = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin_4_input);
/// let ext0 = Ext0WakeupSource::new(pin_4, WakeupLevel::High);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext0]);
///
/// # }
/// ```
#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct Ext0WakeupSource<P: RtcIoWakeupPinType> {
    /// The pin used as the wake-up source.
    pin: RefCell<P>,
    /// The level at which the wake-up event is triggered.
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<P: RtcIoWakeupPinType> Ext0WakeupSource<P> {
    /// Creates a new external wake-up source (Ext0``) with the specified pin
    /// and wake-up level.
    pub fn new(pin: P, level: WakeupLevel) -> Self {
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

#[procmacros::doc_replace]
/// External wake-up source (Ext1).
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource, WakeupLevel}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull, RtcPin};
/// # use esp_hal::time::Duration;
///
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_2 = peripherals.GPIO2;
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_driver = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// // Drop the driver to access `pin_4`
/// core::mem::drop(pin_4_driver);
///
/// let mut wakeup_pins: [&mut dyn RtcPin; 2] = [&mut pin_4, &mut pin_2];
///
/// let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext1]);
///
/// # }
/// ```
#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct Ext1WakeupSource<'a, 'b> {
    /// A collection of pins used as wake-up sources.
    pins: RefCell<&'a mut [&'b mut dyn RtcIoWakeupPinType]>,
    /// The level at which the wake-up event is triggered across all pins.
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [&'b mut dyn RtcIoWakeupPinType], level: WakeupLevel) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
        }
    }
}

#[procmacros::doc_replace(
    "pin_low" => {
        cfg(esp32h2) => "GPIO9",
        _ => "GPIO2",
    },
    "pin_high" => {
        cfg(esp32h2) => "GPIO10",
        _ => "GPIO3",
    },
)]
/// External wake-up source (Ext1).
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource, WakeupLevel}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull, RtcPinWithResistors};
/// # use esp_hal::time::Duration;
/// #
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_low_input = Input::new(peripherals.__pin_low__.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin_low_input);
///
/// let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] =
/// &mut [
///     (&mut peripherals.__pin_low__, WakeupLevel::Low),
///     (&mut peripherals.__pin_high__, WakeupLevel::High),
/// ];
///
/// let ext1 = Ext1WakeupSource::new(wakeup_pins);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext1]);
///
/// # {after_snippet}
/// ```
#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

#[procmacros::doc_replace(
    "pin0" => {
        cfg(any(esp32c3, esp32c2)) => "GPIO2",
        cfg(any(esp32s2, esp32s3)) => "GPIO17"
    },
    "pin1" => {
        cfg(any(esp32c3, esp32c2)) => "GPIO3",
        cfg(any(esp32s2, esp32s3)) => "GPIO18"
    },
    "rtc_pin_trait" => {
        cfg(any(esp32c3, esp32c2)) => "gpio::RtcPinWithResistors",
        cfg(any(esp32s2, esp32s3)) => "gpio::RtcPin"
    },
)]
/// RTC_IO wakeup source
///
/// RTC_IO wakeup allows configuring any combination of RTC_IO pins with
/// arbitrary wakeup levels to wake up the chip from sleep. This wakeup source
/// can be used to wake up from both light and deep sleep.
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::gpio::{self, Input, InputConfig, Pull};
/// # use esp_hal::rtc_cntl::{reset_reason,
/// #   sleep::{LowPower, RtcioWakeupSource, TimerWakeupSource, WakeupLevel},
/// #   wakeup_cause, SocResetReason
/// # };
/// # use esp_hal::system::Cpu;
/// # use esp_hal::time::Duration;
///
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let delay = Delay::new();
/// let timer = TimerWakeupSource::new(Duration::from_secs(10));
/// let wakeup_pins: &mut [(&mut dyn __rtc_pin_trait__, WakeupLevel)] = &mut [
///     (&mut peripherals.__pin0__, WakeupLevel::Low),
///     (&mut peripherals.__pin1__, WakeupLevel::High),
/// ];
///
/// let rtcio = RtcioWakeupSource::new(wakeup_pins);
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &rtcio]);
///
/// # {after_snippet}
/// ```
#[cfg(any(esp32c3, esp32s2, esp32s3, esp32c2))]
pub struct RtcioWakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(any(esp32c3, esp32s2, esp32s3, esp32c2))]
impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    /// Creates a new external GPIO wake-up source.
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

/// LP Core wakeup source
///
/// Wake up from LP core. This wakeup source
/// can be used to wake up from both light and deep sleep.
#[cfg(esp32c6)]
pub struct WakeFromLpCoreWakeupSource {}

#[cfg(esp32c6)]
impl WakeFromLpCoreWakeupSource {
    /// Create a new instance of `WakeFromLpCoreWakeupSource`
    pub fn new() -> Self {
        Self {}
    }
}

#[cfg(esp32c6)]
impl Default for WakeFromLpCoreWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

/// ULP wakeup source
///
/// Wake up from ULP software interrupt, and/or ULP-RISCV Trap condition.
/// Both of these triggers are enabled by default.
/// This source will clear any outstanding software interrupts prior to entering sleep, by default.
///
/// S2 supports the following triggers (Refer to ESP32-S2 Technical Reference Manual, Table 9.4-3.
/// Wakeup Source)
///  - ULP-FSM software interrupt (unsure if this ALSO supports ULP-RISCV software interrupt)
///  - ULP-RISCV Trap
///
/// S3 supports the following triggers (Refer to ESP32-S3 Technical Reference Manual, Table 10.4-3.
/// Wakeup Source)
///  - ULP-FSM software interrupt and ULP-RISCV software interrupt
///  - ULP-RISCV Trap
///
/// This wakeup source can be used to wake up from both light and deep sleep.
#[cfg(any(esp32s2, esp32s3))]
pub struct UlpWakeupSource {
    wake_on_interrupt: bool,
    wake_on_trap: bool,
    clear_interrupts_on_sleep: bool,
}

#[cfg(any(esp32s2, esp32s3))]
impl UlpWakeupSource {
    /// Create a new instance of `WakeFromUlpWakeupSource`
    pub const fn new() -> Self {
        Self {
            wake_on_interrupt: true,
            wake_on_trap: true,
            clear_interrupts_on_sleep: true,
        }
    }

    /// Enable wakeup triggered by software interrupt from ULP-FSM or ULP-RISCV
    pub fn set_wake_on_interrupt(mut self, value: bool) -> Self {
        self.wake_on_interrupt = value;
        self
    }

    /// Enable wakeup triggered by ULP-RISCV Trap
    pub fn set_wake_on_trap(mut self, value: bool) -> Self {
        self.wake_on_trap = value;
        self
    }

    /// Enable clearing of latched wake-up interrupts prior to entering sleep
    pub fn set_clear_interrupts_on_sleep(mut self, value: bool) -> Self {
        self.clear_interrupts_on_sleep = value;
        self
    }

    /// Clears the wake-up interrupts
    pub fn clear_interrupts(&self) {
        crate::peripherals::LPWR::regs().int_clr().write(|w| {
            w.cocpu_trap().clear_bit_by_one();
            w.cocpu().clear_bit_by_one();
            w.ulp_cp().clear_bit_by_one()
        });
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Default for UlpWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

/// GPIO wakeup source
///
/// Wake up from GPIO high or low level. Any pin can be used with this wake up
/// source. Configure the pin for wake up via
/// [crate::gpio::Input::wakeup_enable].
///
/// This wakeup source can be used to wake up from light sleep only.
pub struct GpioWakeupSource {}

impl GpioWakeupSource {
    /// Create a new instance of [GpioWakeupSource]
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for GpioWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

impl WakeSource for GpioWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.insert(WakeupSource::Gpio);
    }
}

macro_rules! uart_wakeup_impl {
    ($num:literal) => {
        paste::paste! {
            #[doc = concat!("UART", $num, " wakeup source")]
            ///
            /// The chip can be woken up by reverting RXD for multiple cycles until the
            /// number of rising edges is equal to or greater than the given value.
            ///
            /// Note that the character which triggers wakeup (and any characters before
            /// it) will not be received by the UART after wakeup. This means that the
            /// external device typically needs to send an extra character to trigger
            /// wakeup before sending the data.
            ///
            /// After waking-up from UART, you should send some extra data through the UART
            /// port in Active mode, so that the internal wakeup indication signal can be
            /// cleared. Otherwise, the next UART wake-up would trigger with two less
            /// rising edges than the configured threshold value.
            ///
            /// Wakeup from light sleep takes some time, so not every character sent to the
            /// UART can be received by the application.
            ///
            /// This wakeup source can be used to wake up from light sleep only.
            pub struct [< Uart $num WakeupSource >] {
                threshold: u16,
            }

            impl [< Uart $num WakeupSource >] {
                #[doc = concat!("Create a new instance of UART", $num, " wakeup source>") ]
                ///
                /// # Panics
                ///
                /// Panics if `threshold` is out of bounds.
                pub fn new(threshold: u16) -> Self {
                    if threshold > 1023 {
                        panic!("Invalid threshold");
                    }
                    Self { threshold }
                }
            }

            impl WakeSource for [< Uart $num WakeupSource >] {
                fn apply(&self, _rtc: &Rtc<'_>, triggers: &mut WakeTriggers, _sleep_config: &mut RtcSleepConfig) {
                    triggers.insert(WakeupSource::[< Uart $num >]);
                    let uart = crate::peripherals::[< UART $num >]::regs();

                    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
                    uart.sleep_conf()
                        .modify(|_, w| unsafe { w.active_threshold().bits(self.threshold) });

                    #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3)))]
                    uart.sleep_conf2().modify(|_, w| unsafe {
                        w.wk_mode_sel().bits(0);
                        w.active_threshold().bits(self.threshold)
                    });
                }
            }
        }
    };
}

uart_wakeup_impl!(0);
uart_wakeup_impl!(1);

/// The set of wakeup sources configured to end a sleep.
///
/// This is a thin wrapper around a set of [`WakeupSource`]s. Wakeup source implementations enable
/// the sources they need via [`WakeTriggers::insert`] from within [`WakeSource::apply`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WakeTriggers(EnumSet<WakeupSource>);

impl WakeTriggers {
    /// Enables the given source as a wakeup trigger.
    pub fn insert(&mut self, source: WakeupSource) {
        self.0.insert(source);
    }

    /// Returns `true` if the given source is enabled as a wakeup trigger.
    pub fn contains(&self, source: WakeupSource) -> bool {
        self.0.contains(source)
    }

    /// Returns the raw wakeup-enable register bitmask.
    pub(crate) fn as_u32(&self) -> u32 {
        self.0.as_u32()
    }
}

/// Trait representing a wakeup source.
pub trait WakeSource {
    /// Configures the RTC and applies the wakeup triggers.
    fn apply(&self, rtc: &Rtc<'_>, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig);
}

/// Low-power management.
#[instability::unstable]
pub struct LowPower<'d> {
    _inner: LPWR<'d>,
}

impl<'d> LowPower<'d> {
    /// Creates a new `LowPower` driver.
    pub fn new(lpwr: LPWR<'d>) -> Self {
        Self { _inner: lpwr }
    }

    /// Enter deep sleep and wake with the provided `wake_sources`.
    ///
    /// In Deep-sleep mode, the CPUs, most of the RAM, and all digital
    /// peripherals that are clocked from APB_CLK are powered off.
    ///
    /// You can use the [`#[esp_hal::ram(persistent)]`][procmacros::ram]
    /// attribute to persist a variable though deep sleep.
    #[cfg(sleep_deep_sleep)]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(sleep_light_sleep)]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(sleep_driver_supported)]
    #[crate::ram]
    pub fn sleep(&mut self, config: &RtcSleepConfig, wake_sources: &[&dyn WakeSource]) {
        let rtc = Rtc::new(unsafe { crate::peripherals::RTC_TIMER::steal() });

        let mut config = *config;
        let mut wakeup_triggers = WakeTriggers::default();
        for wake_source in wake_sources {
            wake_source.apply(&rtc, &mut wakeup_triggers, &mut config)
        }

        config.apply();

        sleep_uart_prepare();

        // Latch the systimer value *before* sleeping. The systimer keeps running during
        // the sleep enter/exit sequences, so we must not advance from the post-wake
        // value (that would count the enter/exit time twice). Instead we set an absolute
        // target of `before + slept`, measured by the always-running LP timer.
        let before_ticks = crate::time::implem::raw_counter();
        let before = rtc.time_since_boot_raw();

        let _uart0_sclk_guard = crate::system::ensure_uart0_sclk_enabled();
        config.start_sleep(wakeup_triggers);

        if config.is_deep_sleep() {
            // Because RTC is in a slower clock domain than the CPU, it
            // can take several CPU cycles for the sleep mode to start.
            loop {
                core::hint::spin_loop();
            }
        }

        config.finish_sleep();

        let after = rtc.time_since_boot_raw();

        let slept_us = crate::clock::rtc_ticks_to_us(after.wrapping_sub(before));
        let slept_ticks = crate::time::implem::us_to_ticks(slept_us);

        unsafe { crate::time::implem::update_counter(before_ticks + slept_ticks) };
        sleep_uart_resume();

        // Unlike deep sleep, light sleep does not reset the chip, so `wakeup_cause` cannot rely on
        // the reset reason to tell whether a wakeup occurred.
        // https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/esp_hw_support/sleep_modes.c#L2158
        if !config.deep_slp() {
            super::LIGHT_SLEEP_WAKEUP.store(true, portable_atomic::Ordering::Relaxed);
        }
    }
}

#[cfg(sleep_driver_supported)]
fn sleep_uart_prepare() {
    use crate::uart::Instance;
    for_each_uart! {
        ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident) => {
            unsafe {
                crate::peripherals::$inst::steal().info().suspend_for_sleep();
            }
        };
    }
}

#[cfg(sleep_driver_supported)]
fn sleep_uart_resume() {
    use crate::uart::Instance;
    for_each_uart! {
        ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident) => {
            unsafe {
                crate::peripherals::$inst::steal().info().resume_from_sleep();
            }
        };
    }
}
