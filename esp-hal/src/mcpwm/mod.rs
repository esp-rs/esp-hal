#![cfg_attr(docsrs, procmacros::doc_replace(
    "mcpwm_freq" => {
        cfg(not(esp32h2)) => "40",
        cfg(esp32h2) => "32"
    },
    "clock_src" => {
        cfg(esp32) => "PLL_F160M (160 MHz)",
        cfg(esp32s3) => "CRYPTO_PWM_CLK (160 MHz)",
        cfg(esp32c6) => "PLL_F160M (160 MHz)",
        cfg(esp32h2) => "PLL_F96M_CLK (96 MHz)",
    },
))]
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
//! - Calculate external pulse width, and convert it into other analog values like speed, distance
//! - Generate Space Vector PWM (SVPWM) signals for Field Oriented Control (FOC)
//!
//! ## Configuration
//! * PWM Timers 0, 1 and 2
//!     * Every PWM timer has a dedicated 8-bit clock prescaler.
//!     * The 16-bit counter in the PWM timer can work in count-up mode, count-down mode or
//!       count-up-down mode.
//!     * A hardware sync or software sync can trigger a reload on the PWM timer with a phase
//!       register.
//!     * Timers run until a preconfigured [`timer::StopCondition`], this enum also specifies
//!       running continuously.
//!     * Timers can generate [`timer::TimerEvent`] interrupts based on hardware events.
//!     * Each timer can produce hardware sync events based on [`timer::SyncOutSelect`].
//!     * Timers can be configured to receive hardware sync events from other timers, or external
//!       GPIO pins.
//! * PWM Operators 0, 1 and 2
//!     * Every PWM operator has two PWM outputs: PWMxA and PWMxB. They can work independently, in
//!       symmetric and asymmetric configuration.
//!     * Software, asynchronously override control of PWM signals.
//!     * Configurable dead-time on rising and falling edges; each set up independently. (Not yet
//!       implemented)
//!     * All events can trigger CPU interrupts. (Not yet implemented)
//!     * Period, time stamps and important control registers have shadow registers with flexible
//!       updating methods.
//! * Capture Channels 0, 1 and 2
//!     * Every capture channel has one signal input. With an optional invert filter
//!     * Each capture module can be configured to detect rising and (or), falling edges on an
//!       external signal.
//!     * Capture channels can produce hardware capture events based on [`capture::CaptureEvent`].
//!     * To record the current value of the capture counter. A software trigger can be used to
//!       trigger a capture event. Note: The edge that is captured is UNSPECIFIED for software
//!       captures.
//!     * Each capture channel can be configured with a 8-bit pre-scaler. Which only triggers
//!       capture events every Nth edge captured. ( Useful for high frequencies )
//! * Capture Timer:
//!     * Capture timer can be configured with a sync in source.
//!     * A hardware sync or software sync can trigger a reload of the capture timer's counter with
//!       the set value called sync phase.
#![cfg_attr(
    mcpwm_capture_clk_from_group,
    doc = "     * Capture timer's clock source is the same as the PWM timers clock source"
)]
#![cfg_attr(
    not(mcpwm_capture_clk_from_group),
    doc = "     * Capture timer has its own independent clock source from the MCPWM peripheral."
)]
//! * Fault Detection Module (Not yet implemented)
#![cfg_attr(
    not(mcpwm_capture_clk_from_group),
    doc = "\nCapture clock source is `ADB-CLK (80 MHz)` by default.\n"
)]
//! Clock source is `__clock_src__` by default.
//!
//! ## Examples
//!
//! ### Output a 20 kHz signal
//!
//! This example uses timer0 and operator0 of the MCPWM0 peripheral to output a
//! 50% duty signal at 20 kHz. The signal will be output to the pin assigned to
//! `pin`.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::mcpwm::{operator::{DeadTimeCfg, PWMStream, PwmPinConfig}, timer::PwmWorkingMode, McPwm, AnyMcPwm, PeripheralClockConfig};
//! # let pin = peripherals.GPIO0;
//!
//! // initialize peripheral
//! let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(__mcpwm_freq__))?;
//! let mut mcpwm = McPwm::new(AnyMcPwm::from(peripherals.MCPWM0), clock_cfg);
//!
//! // connect operator0 to timer0
//! mcpwm.operator0.set_timer(&mcpwm.timer0);
//!
//! // connect operator0 to pin
//! let mut pwm_pin = mcpwm
//!     .operator0
//!     .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);
//!
//! // start timer with timestamp values in the range of 0..=99 and a frequency
//! // of 20 kHz
//! let timer_clock_cfg = clock_cfg
//!     .timer_clock_with_frequency(99, PwmWorkingMode::Increase,
//! Rate::from_khz(20))?;
//! mcpwm.timer0.apply_config(timer_clock_cfg)?;
//! mcpwm.timer0.start();
//!
//! // pin will be high 50% of the time
//! pwm_pin.set_timestamp(50);
//! # {after_snippet}
//! ```

use enumset::{EnumSet, EnumSetType};

#[cfg(soc_has_mcpwm0)]
use crate::mcpwm::{
    capture::{CaptureChannel, CaptureTimer},
    operator::Operator,
    sync::SyncLine,
    timer::Timer,
};
use crate::{
    gpio::{InputSignal, OutputSignal},
    interrupt::{self, InterruptHandler},
    mcpwm::capture::CaptureChannelConfig,
    pac,
    private::DropGuard,
    soc::clocks::{self, ClockTree},
    system::{Peripheral, PeripheralGuard},
    time::Rate,
};

/// MCPWM capture channels
pub mod capture;
/// MCPWM operators
pub mod operator;
/// Sync
pub mod sync;
/// MCPWM timers
pub mod timer;

crate::any_peripheral! {
    /// Any MCPWM peripheral.
    pub peripheral AnyMcPwm<'d> {
        #[cfg(soc_has_mcpwm0)]
        Mcpwm0(crate::peripherals::MCPWM0<'d>),
        #[cfg(soc_has_mcpwm1)]
        Mcpwm1(crate::peripherals::MCPWM1<'d>),
    }
}

impl Instance for AnyMcPwm<'_> {
    fn info(&self) -> &'static Info {
        any::delegate!(self, mcpwm => { mcpwm.info() })
    }
}

/// The MCPWM peripheral
#[non_exhaustive]
pub struct McPwm<'d> {
    mcpwm: AnyMcPwm<'d>,

    /// Timer0
    pub timer0: Timer<'d>,
    /// Timer1
    pub timer1: Timer<'d>,
    /// Timer2
    pub timer2: Timer<'d>,
    /// Capture Timer
    pub capture_timer: CaptureTimer<'d>,

    /// Operator0
    pub operator0: Operator<'d>,
    /// Operator1
    pub operator1: Operator<'d>,
    /// Operator2
    pub operator2: Operator<'d>,

    /// Capture0
    pub capture0: CaptureChannel<'d>,
    /// Capture1
    pub capture1: CaptureChannel<'d>,
    /// Capture2
    pub capture2: CaptureChannel<'d>,

    /// Sync line 0
    pub sync0: SyncLine,
    /// Sync line 1
    pub sync1: SyncLine,
    /// Sync line 2
    pub sync2: SyncLine,
}

impl<'d> McPwm<'d> {
    /// Create a new instance generics
    pub fn new(mcpwm: AnyMcPwm<'d>, peripheral_clock: PeripheralClockConfig) -> Self {
        let info = mcpwm.info();
        let guard = PeripheralGuard::new(info.peripheral());

        // set prescaler for timer (0-2)
        info.regs()
            .clk_cfg()
            .write(|w| unsafe { w.clk_prescale().bits(peripheral_clock.prescaler) });

        // enable clock
        info.regs().clk().write(|w| w.en().set_bit());

        let info = mcpwm.info();
        Self {
            mcpwm,
            timer0: Timer::new(guard.clone(), info, 0, &peripheral_clock),
            timer1: Timer::new(guard.clone(), info, 1, &peripheral_clock),
            timer2: Timer::new(guard.clone(), info, 2, &peripheral_clock),
            operator0: Operator::new(guard.clone(), 0, info),
            operator1: Operator::new(guard.clone(), 1, info),
            operator2: Operator::new(guard.clone(), 2, info),
            capture_timer: CaptureTimer::new(guard.clone(), info),
            capture0: CaptureChannel::new(guard.clone(), info, 0, CaptureChannelConfig::default()),
            capture1: CaptureChannel::new(guard.clone(), info, 1, CaptureChannelConfig::default()),
            capture2: CaptureChannel::new(guard, info, 2, CaptureChannelConfig::default()),
            sync0: SyncLine::new(0, info),
            sync1: SyncLine::new(1, info),
            sync2: SyncLine::new(2, info),
        }
    }

    /// Set the interrupt handler for the MCPWM peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let info = self.mcpwm.info();
        let interrupt = info.interrupt();

        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        interrupt::bind_handler(interrupt, handler);
    }
}

/// Clock configuration of the MCPWM peripheral
#[derive(Copy, Clone)]
pub struct PeripheralClockConfig {
    frequency: Rate,
    prescaler: u8,
}

impl PeripheralClockConfig {
    fn source_clock() -> Rate {
        // FIXME: this works right now because we configure the default clock source during startup.
        // Needs to be revisited when refactoring the MCPWM driver before stabilization.
        Rate::from_hz(clocks::McpwmInstance::Mcpwm0.function_clock_frequency())
    }

    /// Get a clock configuration with the given prescaler.
    ///
    /// With standard system clock configurations the input clock to the MCPWM
    /// peripheral is `160 MHz`.
    ///
    /// The peripheral clock frequency is calculated as:
    /// `peripheral_clock = input_clock / (prescaler + 1)`
    pub fn with_prescaler(prescaler: u8) -> Self {
        let source_clock = Self::source_clock();

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
    pub fn with_frequency(target_freq: Rate) -> Result<Self, FrequencyError> {
        let source_clock = Self::source_clock();

        if target_freq.as_hz() == 0 || target_freq > source_clock {
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
    pub fn frequency(&self) -> Rate {
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
        target_freq: Rate,
    ) -> Result<timer::TimerClockConfig, FrequencyError> {
        timer::TimerClockConfig::with_frequency(self, period, mode, target_freq)
    }

    /// Get a timer clock configuration with the default values.
    ///
    /// ### Note:
    /// - Prescaler defaults to the minimum value of `0`.
    /// - Period defaults to the maximum value of `u16::MAX`.
    /// - PWM working mode defaults to `PwmWorkingMode::Increase`.
    ///
    /// The frequency is calculated with the formula described in
    ///   [`PeripheralClockConfig::timer_clock_with_prescaler`] with the default prescaler value.
    pub fn timer_clock_default(&self) -> timer::TimerClockConfig {
        timer::TimerClockConfig::default(self)
    }
}

/// Target frequency could not be set.
/// Check how the frequency is calculated in the corresponding method docs.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrequencyError;

impl core::fmt::Display for FrequencyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Target frequency could not be set")
    }
}

impl core::error::Error for FrequencyError {}

type RegisterBlock = pac::mcpwm0::RegisterBlock;

/// Peripheral info for an MCPWM instance.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
#[allow(private_interfaces, reason = "Unstable details")]
pub struct Info {
    /// Register block
    register_block: *const RegisterBlock,
    /// System peripheral marker.
    _peripheral: crate::system::Peripheral,
    /// Interrupt marker
    _interrupt: crate::peripherals::Interrupt,
    /// Sync inputs
    sync_input: [InputSignal; 3],
    /// Capture inputs
    capture_input: [InputSignal; 3],
    /// Operator A outputs
    operator_a_output: [OutputSignal; 3],
    /// Operator B outputs
    operator_b_output: [OutputSignal; 3],
}

/// Dispatches event to register bit getter for interrupt status checks
macro_rules! dispatch_event_bit {
    ($int_register:expr, $event:expr, $unit:expr) => {
        match $event {
            Event::TimerStop => $int_register.timer_stop($unit).bit(),
            Event::TimerEqualZero => $int_register.timer_tez($unit).bit(),
            Event::TimerEqualPeriod => $int_register.timer_tep($unit).bit(),
            Event::Capture => $int_register.cap($unit).bit(),
            Event::CompareA => $int_register.cmpr_tea($unit).bit(),
            Event::CompareB => $int_register.cmpr_teb($unit).bit(),
            Event::Fault => $int_register.fault($unit).bit(),
            Event::FaultClear => $int_register.fault_clr($unit).bit(),
            Event::FaultCycleByCycle => $int_register.tz_cbc($unit).bit(),
            Event::FaultOneShotMode => $int_register.tz_ost($unit).bit(),
        }
    };
}

/// Dispatches event to register bit setter for interrupt control
macro_rules! dispatch_event_write {
    ($int_register:expr, $event:expr, $unit:expr, $value:expr) => {
        match $event {
            Event::TimerStop => $int_register.timer_stop($unit).bit($value),
            Event::TimerEqualZero => $int_register.timer_tez($unit).bit($value),
            Event::TimerEqualPeriod => $int_register.timer_tep($unit).bit($value),
            Event::Capture => $int_register.cap($unit).bit($value),
            Event::CompareA => $int_register.cmpr_tea($unit).bit($value),
            Event::CompareB => $int_register.cmpr_teb($unit).bit($value),
            Event::Fault => $int_register.fault($unit).bit($value),
            Event::FaultClear => $int_register.fault_clr($unit).bit($value),
            Event::FaultCycleByCycle => $int_register.tz_cbc($unit).bit($value),
            Event::FaultOneShotMode => $int_register.tz_ost($unit).bit($value),
        }
    };
}

/// Event types for MCPWM
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum Event {
    /// Event for when a timer stops
    TimerStop,
    /// Event for when a timer equals zero
    TimerEqualZero,
    /// Event for when a timer equals period
    TimerEqualPeriod,
    /// Event for a fault
    Fault,
    /// Event for a fault clear
    FaultClear,
    /// Event for a compare A
    CompareA,
    /// Event for a compare B
    CompareB,
    /// Event for a fault cycle by cycle
    FaultCycleByCycle,
    /// Event for a fault one shot mode
    FaultOneShotMode,
    /// Event for a capture
    Capture,
}

/// A peripheral singleton compatible with the MCPWM driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    /// Returns the peripheral data describing this instance.
    #[doc(hidden)]
    fn info(&self) -> &'static Info;
}

impl Info {
    /// Returns the register block for this PWM instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Returns the peripheral
    pub fn peripheral(&self) -> crate::system::Peripheral {
        self._peripheral
    }

    /// Returns the interrupt marker
    pub fn interrupt(&self) -> crate::peripherals::Interrupt {
        self._interrupt
    }

    /// Returns the output signal for operators
    pub fn operator_output_signal(&self, operator: u8, is_a: bool) -> OutputSignal {
        match is_a {
            true => self.operator_a_output[operator as usize],
            false => self.operator_b_output[operator as usize],
        }
    }

    /// Returns the sync input signal
    pub fn sync_input_signal(&self, sync: u8) -> InputSignal {
        self.sync_input[sync as usize]
    }

    /// Returns the capture input signal
    pub fn capture_input_signal(&self, chan: u8) -> InputSignal {
        self.capture_input[chan as usize]
    }

    /// Return if the interrupt for an event is set
    pub fn interrupt_set(&self, unit: u8, event: Event) -> bool {
        let regs = self.regs();
        let int_st = regs.int_st().read();
        dispatch_event_bit!(int_st, event, unit)
    }

    /// Clear the interrupt for an event on a specific UNIT #
    pub fn clear_interrupt(&self, unit: u8, event: Event) {
        let regs = self.regs();
        regs.int_clr().write(|w| {
            dispatch_event_write!(w, event, unit, true);
            w
        });
    }

    /// Enables listening for an event on a specific UNIT #
    pub fn enable_listen(&self, unit: u8, events: EnumSet<Event>, value: bool) {
        let regs = self.regs();
        regs.int_ena().modify(|_, w| {
            for event in events {
                dispatch_event_write!(w, event, unit, value);
            }
            w
        });
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

// Create an `Instance` impl for each MCPWM peripheral
for_each_mcpwm!(
    ($id:literal, $inst:ident, $sys:ident) => {
        paste::paste! {
            impl Instance for crate::peripherals::$inst<'_> {
                /// Returns peripheral data for MCPWM $id
                fn info(&self) -> &'static Info {
                    static INFO: Info = Info {
                        register_block: crate::peripherals::$inst::regs(),
                        _peripheral: crate::system::Peripheral::$sys,
                        _interrupt: crate::peripherals::Interrupt::$inst,
                        sync_input: [
                            InputSignal::[<PWM $id _SYNC0>],
                            InputSignal::[<PWM $id _SYNC1>],
                            InputSignal::[<PWM $id _SYNC2>],
                        ],
                        capture_input: [
                            InputSignal::[<PWM $id _CAP0>],
                            InputSignal::[<PWM $id _CAP1>],
                            InputSignal::[<PWM $id _CAP2>],
                        ],
                        operator_a_output: [
                            OutputSignal::[<PWM $id _0A>],
                            OutputSignal::[<PWM $id _1A>],
                            OutputSignal::[<PWM $id _2A>],
                        ],
                        operator_b_output: [
                            OutputSignal::[<PWM $id _0B>],
                            OutputSignal::[<PWM $id _1B>],
                            OutputSignal::[<PWM $id _2B>],
                        ],
                    };

                    &INFO
                }
            }
    }
    };
);

#[allow(dead_code)] // Field is seemingly unused but we rely on its Drop impl
struct PwmClockGuard(DropGuard<clocks::McpwmInstance, fn(clocks::McpwmInstance)>);

impl PwmClockGuard {
    fn instance(mcpwm_info: &'static Info) -> clocks::McpwmInstance {
        match mcpwm_info.peripheral() {
            Peripheral::Mcpwm0 => clocks::McpwmInstance::Mcpwm0,
            #[cfg(soc_has_mcpwm1)]
            Peripheral::Mcpwm1 => clocks::McpwmInstance::Mcpwm1,
            _ => unreachable!(),
        }
    }

    pub fn new(mcpwm_info: &'static Info) -> Self {
        ClockTree::with(move |clocks| Self::instance(mcpwm_info).request_function_clock(clocks));

        Self(DropGuard::new(Self::instance(mcpwm_info), move |mcpwm| {
            ClockTree::with(move |clocks| mcpwm.release_function_clock(clocks));
        }))
    }
}

impl<'d> crate::private::Sealed for McPwm<'d> {}
#[instability::unstable]
impl<'d> crate::interrupt::InterruptConfigurable for McPwm<'d> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}
