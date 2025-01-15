//! # Motor Control Pulse Width Modulator (MCPWM)
//!
//! ## Overview
//!
//! The MCPWM peripheral is a versatile PWM generator, which contains various
//! submodules to make it a key element in power electronic applications like
//! motor control, digital power, and so on. Typically, the MCPWM peripheral can
//! be used in the following scenarios:
//! - Digital motor control, e.g., brushed/brushless DC motor, RC servo motor
//! - Switch mode-based digital power conversion
//! - Power DAC, where the duty cycle is equivalent to a DAC analog value
//! - Calculate external pulse width, and convert it into other analog values
//!   like speed, distance
//! - Generate Space Vector PWM (SVPWM) signals for Field Oriented Control (FOC)
//!
//! ## Configuration
//!
//! * PWM Timers 0, 1 and 2
//!     * Every PWM timer has a dedicated 8-bit clock prescaler.
//!     * The 16-bit counter in the PWM timer can work in count-up mode,
//!       count-down mode or count-up-down mode.
//!     * A hardware sync or software sync can trigger a reload on the PWM timer
//!       with a phase register (Not yet implemented)
//! * PWM Operators 0, 1 and 2
//!     * Every PWM operator has two PWM outputs: PWMxA and PWMxB. They can work
//!       independently, in symmetric and asymmetric configuration.
//!     * Software, asynchronously override control of PWM signals.
//!     * Configurable dead-time on rising and falling edges; each set up
//!       independently. (Not yet implemented)
//!     * All events can trigger CPU interrupts. (Not yet implemented)
//!     * Modulating of PWM output by high-frequency carrier signals, useful
//!       when gate drivers are insulated with a transformer. (Not yet
//!       implemented)
//!     * Period, time stamps and important control registers have shadow
//!       registers with flexible updating methods.
//! * Fault Detection Module (Not yet implemented)
//! * Capture Module (Not yet implemented)
#![doc = ""]
#![cfg_attr(esp32, doc = "Clock source is PWM_CLOCK")]
#![cfg_attr(esp32s3, doc = "Clock source is CRYPTO_PWM_CLOCK")]
#![cfg_attr(esp32c6, doc = "Clock source is CRYPTO_CLOCK")]
#![cfg_attr(esp32h2, doc = "Clock source is XTAL")]
#![doc = ""]
//! ## Examples
//!
//! ### Output a 20 kHz signal
//!
//! This example uses timer0 and operator0 of the MCPWM0 peripheral to output a
//! 50% duty signal at 20 kHz. The signal will be output to the pin assigned to
//! `pin`.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::mcpwm::{operator::{DeadTimeCfg, PWMStream, PwmPinConfig}, timer::PwmWorkingMode, McPwm, PeripheralClockConfig};
//! # let pin = peripherals.GPIO0;
//!
//! // initialize peripheral
#![cfg_attr(
    esp32h2,
    doc = "let clock_cfg = PeripheralClockConfig::with_frequency(40.MHz()).unwrap();"
)]
#![cfg_attr(
    not(esp32h2),
    doc = "let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();"
)]
//! let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
//!
//! // connect operator0 to timer0
//! mcpwm.operator0.set_timer(&mcpwm.timer0);
//! // connect operator0 to pin
//! let mut pwm_pin = mcpwm
//!     .operator0
//!     .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);
//!
//! // start timer with timestamp values in the range of 0..=99 and a frequency
//! // of 20 kHz
//! let timer_clock_cfg = clock_cfg
//!     .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20.kHz())
//!     .unwrap();
//! mcpwm.timer0.start(timer_clock_cfg);
//!
//! // pin will be high 50% of the time
//! pwm_pin.set_timestamp(50);
//! # }
//! ```

use core::ops::Deref;

use fugit::HertzU32;
use operator::Operator;
use timer::Timer;

use crate::{
    clock::Clocks,
    gpio::OutputSignal,
    peripheral::{Peripheral, PeripheralRef},
    system::{self, PeripheralGuard},
};

/// MCPWM operators
pub mod operator;
/// MCPWM timers
pub mod timer;

type RegisterBlock = crate::pac::mcpwm0::RegisterBlock;

/// The MCPWM peripheral
#[non_exhaustive]
pub struct McPwm<'d, PWM> {
    _inner: PeripheralRef<'d, PWM>,
    /// Timer0
    pub timer0: Timer<0, PWM>,
    /// Timer1
    pub timer1: Timer<1, PWM>,
    /// Timer2
    pub timer2: Timer<2, PWM>,
    /// Operator0
    pub operator0: Operator<'d, 0, PWM>,
    /// Operator1
    pub operator1: Operator<'d, 1, PWM>,
    /// Operator2
    pub operator2: Operator<'d, 2, PWM>,
    _guard: PeripheralGuard,
}

impl<'d, PWM: PwmPeripheral> McPwm<'d, PWM> {
    /// `pwm_clk = clocks.crypto_pwm_clock / (prescaler + 1)`
    // clocks.crypto_pwm_clock normally is 160 MHz
    pub fn new(
        peripheral: impl Peripheral<P = PWM> + 'd,
        peripheral_clock: PeripheralClockConfig,
    ) -> Self {
        crate::into_ref!(peripheral);

        let guard = PeripheralGuard::new(PWM::peripheral());

        #[cfg(not(esp32c6))]
        {
            // set prescaler
            peripheral
                .clk_cfg()
                .write(|w| unsafe { w.clk_prescale().bits(peripheral_clock.prescaler) });

            // enable clock
            peripheral.clk().write(|w| w.en().set_bit());
        }

        #[cfg(esp32c6)]
        {
            unsafe { &*crate::peripherals::PCR::PTR }
                .pwm_clk_conf()
                .modify(|_, w| unsafe {
                    w.pwm_div_num()
                        .bits(peripheral_clock.prescaler)
                        .pwm_clkm_en()
                        .set_bit()
                        .pwm_clkm_sel()
                        .bits(1)
                });
        }

        #[cfg(esp32h2)]
        {
            unsafe { &*crate::peripherals::PCR::PTR }
                .pwm_clk_conf()
                .modify(|_, w| unsafe {
                    w.pwm_div_num()
                        .bits(peripheral_clock.prescaler)
                        .pwm_clkm_en()
                        .set_bit()
                        .pwm_clkm_sel()
                        .bits(0)
                });
        }

        Self {
            _inner: peripheral,
            timer0: Timer::new(),
            timer1: Timer::new(),
            timer2: Timer::new(),
            operator0: Operator::new(),
            operator1: Operator::new(),
            operator2: Operator::new(),
            _guard: guard,
        }
    }
}

/// Clock configuration of the MCPWM peripheral
#[derive(Copy, Clone)]
pub struct PeripheralClockConfig {
    frequency: HertzU32,
    prescaler: u8,
}

impl PeripheralClockConfig {
    /// Get a clock configuration with the given prescaler.
    ///
    /// With standard system clock configurations the input clock to the MCPWM
    /// peripheral is `160 MHz`.
    ///
    /// The peripheral clock frequency is calculated as:
    /// `peripheral_clock = input_clock / (prescaler + 1)`
    pub fn with_prescaler(prescaler: u8) -> Self {
        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let source_clock = clocks.pwm_clock;
            } else if #[cfg(esp32c6)] {
                let source_clock = clocks.crypto_clock;
            } else if #[cfg(esp32s3)] {
                let source_clock = clocks.crypto_pwm_clock;
            } else if #[cfg(esp32h2)] {
                let source_clock = clocks.xtal_clock;
            }
        }

        Self {
            frequency: source_clock / (prescaler as u32 + 1),
            prescaler,
        }
    }

    /// Get a clock configuration with the given frequency.
    ///
    /// ### Note:
    /// This will try to select an appropriate prescaler for the
    /// [`PeripheralClockConfig::with_prescaler`] method.
    /// If the calculated prescaler is not in the range `0..u8::MAX`
    /// [`FrequencyError`] will be returned.
    ///
    /// With standard system clock configurations the input clock to the MCPWM
    /// peripheral is `160 MHz`.
    ///
    /// Only divisors of the input clock (`160 Mhz / 1`, `160 Mhz / 2`, ...,
    /// `160 Mhz / 256`) are representable exactly. Other target frequencies
    /// will be rounded up to the next divisor.
    pub fn with_frequency(target_freq: HertzU32) -> Result<Self, FrequencyError> {
        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let source_clock = clocks.pwm_clock;
            } else if #[cfg(esp32c6)] {
                let source_clock = clocks.crypto_clock;
            } else if #[cfg(esp32s3)] {
                let source_clock = clocks.crypto_pwm_clock;
            } else if #[cfg(esp32h2)] {
                let source_clock = clocks.xtal_clock;
            }
        }

        if target_freq.raw() == 0 || target_freq > source_clock {
            return Err(FrequencyError);
        }

        let prescaler = source_clock / target_freq - 1;
        if prescaler > u8::MAX as u32 {
            return Err(FrequencyError);
        }

        Ok(Self::with_prescaler(prescaler as u8))
    }

    /// Get the peripheral clock frequency.
    ///
    /// ### Note:
    /// The actual value is rounded down to the nearest `u32` value
    pub fn frequency(&self) -> HertzU32 {
        self.frequency
    }

    /// Get a timer clock configuration with the given prescaler.
    ///
    /// The resulting timer frequency depends on the chosen
    /// [`timer::PwmWorkingMode`].
    ///
    /// #### `PwmWorkingMode::Increase` or `PwmWorkingMode::Decrease`
    /// `timer_frequency = peripheral_clock / (prescaler + 1) / (period + 1)`
    /// #### `PwmWorkingMode::UpDown`
    /// `timer_frequency = peripheral_clock / (prescaler + 1) / (2 * period)`
    pub fn timer_clock_with_prescaler(
        &self,
        period: u16,
        mode: timer::PwmWorkingMode,
        prescaler: u8,
    ) -> timer::TimerClockConfig {
        timer::TimerClockConfig::with_prescaler(self, period, mode, prescaler)
    }

    /// Get a timer clock configuration with the given frequency.
    ///
    /// ### Note:
    /// This will try to select an appropriate prescaler for the timer.
    /// If the calculated prescaler is not in the range `0..u8::MAX`
    /// [`FrequencyError`] will be returned.
    ///
    /// See [`PeripheralClockConfig::timer_clock_with_prescaler`] for how the
    /// frequency is calculated.
    pub fn timer_clock_with_frequency(
        &self,
        period: u16,
        mode: timer::PwmWorkingMode,
        target_freq: HertzU32,
    ) -> Result<timer::TimerClockConfig, FrequencyError> {
        timer::TimerClockConfig::with_frequency(self, period, mode, target_freq)
    }
}

/// Target frequency could not be set.
/// Check how the frequency is calculated in the corresponding method docs.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrequencyError;

/// A MCPWM peripheral
pub trait PwmPeripheral: Deref<Target = RegisterBlock> + crate::private::Sealed {
    /// Get a pointer to the peripheral RegisterBlock
    fn block() -> *const RegisterBlock;
    /// Get operator GPIO mux output signal
    fn output_signal<const OP: u8, const IS_A: bool>() -> OutputSignal;
    /// Peripheral
    fn peripheral() -> system::Peripheral;
}

#[cfg(mcpwm0)]
impl PwmPeripheral for crate::peripherals::MCPWM0 {
    fn block() -> *const RegisterBlock {
        Self::PTR
    }

    fn output_signal<const OP: u8, const IS_A: bool>() -> OutputSignal {
        match (OP, IS_A) {
            (0, true) => OutputSignal::PWM0_0A,
            (1, true) => OutputSignal::PWM0_1A,
            (2, true) => OutputSignal::PWM0_2A,
            (0, false) => OutputSignal::PWM0_0B,
            (1, false) => OutputSignal::PWM0_1B,
            (2, false) => OutputSignal::PWM0_2B,
            _ => unreachable!(),
        }
    }

    fn peripheral() -> system::Peripheral {
        system::Peripheral::Mcpwm0
    }
}

#[cfg(mcpwm1)]
impl PwmPeripheral for crate::peripherals::MCPWM1 {
    fn block() -> *const RegisterBlock {
        Self::PTR
    }

    fn output_signal<const OP: u8, const IS_A: bool>() -> OutputSignal {
        match (OP, IS_A) {
            (0, true) => OutputSignal::PWM1_0A,
            (1, true) => OutputSignal::PWM1_1A,
            (2, true) => OutputSignal::PWM1_2A,
            (0, false) => OutputSignal::PWM1_0B,
            (1, false) => OutputSignal::PWM1_1B,
            (2, false) => OutputSignal::PWM1_2B,
            _ => unreachable!(),
        }
    }

    fn peripheral() -> system::Peripheral {
        system::Peripheral::Mcpwm1
    }
}
