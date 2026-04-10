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
//!     * Timers can generate [`timer::TimerEvent`] during specific conditions.
//!     * Timers [`sync::SyncOut`] can be configured to fire at specific conditions configured by
//!       [`timer::SyncOutSelect`]
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
//!     * Each capture channel can trigger software capture event recording the capture counter. The
//!       edge that is captured is UNSPECIFIED for software captures.
//!     * Each capture channel can be configured with a 8-bit pre-scaler. Which only triggers
//!       capture events every Nth edge captured. ( Useful for high frequencies )
//! * Capture Timer:
//!     * Capture timer can be configured with a sync in source.
//!     * A hardware sync or software sync can trigger a reload on the capture timer with the set
//!       phase.
#![cfg_attr(
    soc_has_mcpwm_capture_clk_from_group,
    doc = "     * Capture timer's clock source is the same as the PWM timers clock source"
)]
#![cfg_attr(
    not(soc_has_mcpwm_capture_clk_from_group),
    doc = "     * Capture timer has its own independent clock source from the MCPWM peripheral."
)]
//! * Fault Detection Module (Not yet implemented)
#![cfg_attr(
    not(soc_has_mcpwm_capture_clk_from_group),
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
//! # use esp_hal::mcpwm::{operator::{DeadTimeCfg, PWMStream, PwmPinConfig}, timer::PwmWorkingMode, McPwm, PeripheralClockConfig};
//! # let pin = peripherals.GPIO0;
//!
//! // initialize peripheral
//! let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(__mcpwm_freq__))?;
//! let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
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
//! mcpwm.timer0.set_config(timer_clock_cfg);
//! mcpwm.timer0.start();
//!
//! // pin will be high 50% of the time
//! pwm_pin.set_timestamp(50);
//! # {after_snippet}
//! ```

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

#[cfg(soc_has_mcpwm0)]
use crate::mcpwm::{
    capture::{CaptureChannelCreator, CaptureTimer},
    operator::Operator,
    sync::SyncLine,
    timer::Timer,
};
use crate::{
    gpio::{InputSignal, OutputSignal},
    interrupt::{self, InterruptHandler},
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

/// Provides nice types for public API
#[cfg(soc_has_mcpwm0)]
pub mod mcpwm0 {
    use crate::{mcpwm::*, peripherals::MCPWM0 as MCPWMPeripheral};

    /// MCPWM Driver for MCPWM0
    pub type McPwm<'d> = super::McPwm<'d, MCPWMPeripheral<'d>>;
    /// Capture channel creator for MCPWM0
    pub type CaptureChannelCreator<'d, const NUM: u8> =
        capture::CaptureChannelCreator<'d, NUM, MCPWMPeripheral<'d>>;
    /// Capture Channel for MCPWM0
    pub type CaptureChannel<'d, const NUM: u8> =
        capture::CaptureChannel<'d, NUM, MCPWMPeripheral<'d>>;
    /// Capture timer for MCPWM0
    pub type CaptureTimer<'d> = capture::CaptureTimer<'d, MCPWMPeripheral<'d>>;
    /// Timer for MCPWM0
    pub type Timer<'d, const NUM: u8> = timer::Timer<'d, NUM, MCPWMPeripheral<'d>>;
    /// Operator for MCPWM0
    pub type Operator<'d, const NUM: u8> = operator::Operator<'d, NUM, MCPWMPeripheral<'d>>;
    /// Pwm Pin for MCPWM0
    pub type PwmPin<'d, const NUM: u8, const IS_A: bool> =
        operator::PwmPin<'d, MCPWMPeripheral<'d>, NUM, IS_A>;
    /// Linked Pins for MCPWM0
    pub type LinkedPins<'d, const NUM: u8> = operator::LinkedPins<'d, MCPWMPeripheral<'d>, NUM>;
    /// Sync source for MCPWM0
    pub type SyncSource<'d> = dyn sync::SyncSource<MCPWMPeripheral<'d>>;
}

/// Provides nice types for public API
#[cfg(soc_has_mcpwm1)]
pub mod mcpwm1 {
    use crate::{mcpwm::*, peripherals::MCPWM1 as MCPWMPeripheral};
    /// MCPWM Driver for MCPWM1
    pub type McPwm<'d> = super::McPwm<'d, MCPWMPeripheral<'d>>;
    /// Capture channel creator for MCPWM1
    pub type CaptureChannelCreator<'d, const NUM: u8> =
        capture::CaptureChannelCreator<'d, NUM, MCPWMPeripheral<'d>>;
    /// Capture Channel for MCPWM1
    pub type CaptureChannel<'d, const NUM: u8> =
        capture::CaptureChannel<'d, NUM, MCPWMPeripheral<'d>>;
    /// Capture timer for MCPWM1
    pub type CaptureTimer<'d> = capture::CaptureTimer<'d, MCPWMPeripheral<'d>>;
    /// Timer for MCPWM1
    pub type Timer<'d, const NUM: u8> = timer::Timer<'d, NUM, MCPWMPeripheral<'d>>;
    /// Operator for MCPWM1
    pub type Operator<'d, const NUM: u8> = operator::Operator<'d, NUM, MCPWMPeripheral<'d>>;
    /// Pwm Pin for MCPWM1
    pub type PwmPin<'d, const NUM: u8, const IS_A: bool> =
        operator::PwmPin<'d, MCPWMPeripheral<'d>, NUM, IS_A>;
    /// Linked Pins for MCPWM1
    pub type LinkedPins<'d, const NUM: u8> = operator::LinkedPins<'d, MCPWMPeripheral<'d>, NUM>;
    /// Sync source for MCPWM1
    pub type SyncSource<'d> = dyn sync::SyncSource<MCPWMPeripheral<'d>>;
}

/// The MCPWM peripheral
#[non_exhaustive]
pub struct McPwm<'d, PWM: Instance> {
    _phantom: PhantomData<&'d PWM>,
    _instance: PWM,

    /// Timer0
    pub timer0: Timer<'d, 0, PWM>,
    /// Timer1
    pub timer1: Timer<'d, 1, PWM>,
    /// Timer2
    pub timer2: Timer<'d, 2, PWM>,
    /// Capture Timer
    pub capture_timer: CaptureTimer<'d, PWM>,

    /// Operator0
    pub operator0: Operator<'d, 0, PWM>,
    /// Operator1
    pub operator1: Operator<'d, 1, PWM>,
    /// Operator2
    pub operator2: Operator<'d, 2, PWM>,

    /// Capture0
    pub capture0: CaptureChannelCreator<'d, 0, PWM>,
    /// Capture1
    pub capture1: CaptureChannelCreator<'d, 1, PWM>,
    /// Capture2
    pub capture2: CaptureChannelCreator<'d, 2, PWM>,

    /// Sync0
    pub sync0: SyncLine<'d, 0, PWM>,
    /// Sync1
    pub sync1: SyncLine<'d, 1, PWM>,
    /// Sync2
    pub sync2: SyncLine<'d, 2, PWM>,
}

impl<'d, PWM: Instance> McPwm<'d, PWM> {
    /// Create a new instance generics
    pub fn new(_peripheral: PWM, peripheral_clock: PeripheralClockConfig) -> Self {
        let info = PWM::info();
        let guard = PeripheralGuard::new(info.peripheral());

        // set prescaler for timer (0-2)
        info.regs()
            .clk_cfg()
            .write(|w| unsafe { w.clk_prescale().bits(peripheral_clock.prescaler) });

        // enable clock
        info.regs().clk().write(|w| w.en().set_bit());

        Self {
            _phantom: PhantomData,
            _instance: _peripheral,
            timer0: Timer::new(guard.clone(), &peripheral_clock),
            timer1: Timer::new(guard.clone(), &peripheral_clock),
            timer2: Timer::new(guard.clone(), &peripheral_clock),
            operator0: Operator::new(guard.clone()),
            operator1: Operator::new(guard.clone()),
            operator2: Operator::new(guard.clone()),
            capture_timer: CaptureTimer::new(guard.clone()),
            capture0: CaptureChannelCreator::new(guard.clone()),
            capture1: CaptureChannelCreator::new(guard.clone()),
            capture2: CaptureChannelCreator::new(guard),
            sync0: SyncLine::new(),
            sync1: SyncLine::new(),
            sync2: SyncLine::new(),
        }
    }

    /// Set the interrupt handler for the MCPWM peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let info = PWM::info();
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
        ClockTree::with(|clocks| {
            Rate::from_hz(clocks::McpwmInstance::Mcpwm0.function_clock_frequency(clocks))
        })
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

impl Info {
    /// Returns the register block for this PWM instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Returns the periperal
    pub fn peripheral(&self) -> crate::system::Peripheral {
        self._peripheral
    }

    /// Returns the interrupt marker
    pub fn interrupt(&self) -> crate::peripherals::Interrupt {
        self._interrupt
    }

    /// Returns the output signal for operators
    pub fn operator_output_signal<const OP: u8, const IS_A: bool>(&self) -> OutputSignal {
        match IS_A {
            true => self.operator_a_output[OP as usize],
            false => self.operator_b_output[OP as usize],
        }
    }

    /// Returns the sync input signal
    pub fn sync_input_signal<const SYNC: u8>(&self) -> InputSignal {
        self.sync_input[SYNC as usize]
    }

    /// Returns the capture input signal
    pub fn capture_input_signal<const CHAN: u8>(&self) -> InputSignal {
        self.capture_input[CHAN as usize]
    }

    /// Return if the interrupt for an event is set
    pub fn interrupt_set<const UNIT: u8>(&self, event: Event) -> bool {
        let regs = self.regs();
        let int_st = regs.int_st().read();
        dispatch_event_bit!(int_st, event, UNIT)
    }

    /// Clear the interrupt for an event on a specific UNIT #
    pub fn clear_interrupt<const UNIT: u8>(&self, event: Event) {
        let regs = self.regs();
        regs.int_clr().write(|w| {
            dispatch_event_write!(w, event, UNIT, true);
            w
        });
    }

    /// Enables listening for an event on a specific UNIT #
    pub fn enable_listen<const UNIT: u8>(&self, events: EnumSet<Event>, value: bool) {
        let regs = self.regs();
        critical_section::with(|_| {
            regs.int_ena().modify(|r, w| {
                for event in events {
                    dispatch_event_write!(w, event, UNIT, value);
                }
                w
            });
        });
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

/// Represents a MCPWM peripheral
pub trait Instance: crate::private::Sealed {
    #[doc(hidden)]
    /// Returns the peripheral data and state.
    fn info() -> &'static Info;
}

#[cfg(soc_has_mcpwm0)]
impl Instance for crate::peripherals::MCPWM0<'_> {
    /// Returns peripheral data for MCPWM 0
    fn info() -> &'static Info {
        static INFO: Info = Info {
            register_block: crate::peripherals::MCPWM0::regs(),
            _peripheral: crate::system::Peripheral::Mcpwm0,
            _interrupt: crate::peripherals::Interrupt::MCPWM0,
            sync_input: [
                InputSignal::PWM0_SYNC0,
                InputSignal::PWM0_SYNC1,
                InputSignal::PWM0_SYNC2,
            ],
            capture_input: [
                InputSignal::PWM0_CAP0,
                InputSignal::PWM0_CAP1,
                InputSignal::PWM0_CAP2,
            ],
            operator_a_output: [
                OutputSignal::PWM0_0A,
                OutputSignal::PWM0_1A,
                OutputSignal::PWM0_2A,
            ],
            operator_b_output: [
                OutputSignal::PWM0_0B,
                OutputSignal::PWM0_1B,
                OutputSignal::PWM0_2B,
            ],
        };

        &INFO
    }
}

#[cfg(soc_has_mcpwm1)]
impl Instance for crate::peripherals::MCPWM1<'_> {
    fn info() -> &'static Info {
        static INFO: Info = Info {
            register_block: crate::peripherals::MCPWM1::regs(),
            _peripheral: crate::system::Peripheral::Mcpwm1,
            _interrupt: crate::peripherals::Interrupt::MCPWM1,
            sync_input: [
                InputSignal::PWM1_SYNC0,
                InputSignal::PWM1_SYNC1,
                InputSignal::PWM1_SYNC2,
            ],
            capture_input: [
                InputSignal::PWM1_CAP0,
                InputSignal::PWM1_CAP1,
                InputSignal::PWM1_CAP2,
            ],
            operator_a_output: [
                OutputSignal::PWM1_0A,
                OutputSignal::PWM1_1A,
                OutputSignal::PWM1_2A,
            ],
            operator_b_output: [
                OutputSignal::PWM1_0B,
                OutputSignal::PWM1_1B,
                OutputSignal::PWM1_2B,
            ],
        };

        &INFO
    }
}

#[allow(dead_code)] // Field is seemingly unused but we rely on its Drop impl
struct PwmClockGuard(DropGuard<(), fn(())>);

impl PwmClockGuard {
    fn instance<PWM: Instance>() -> clocks::McpwmInstance {
        let info = PWM::info();
        match info.peripheral() {
            Peripheral::Mcpwm0 => clocks::McpwmInstance::Mcpwm0,
            #[cfg(soc_has_mcpwm1)]
            Peripheral::Mcpwm1 => clocks::McpwmInstance::Mcpwm1,
            _ => unreachable!(),
        }
    }

    pub fn new<PWM: Instance>() -> Self {
        ClockTree::with(move |clocks| Self::instance::<PWM>().request_function_clock(clocks));

        Self(DropGuard::new((), |_| {
            ClockTree::with(move |clocks| Self::instance::<PWM>().release_function_clock(clocks));
        }))
    }
}

impl<'d, PWM: Instance + 'd> crate::private::Sealed for McPwm<'d, PWM> {}
#[instability::unstable]
impl<'d, PWM: Instance + 'd> crate::interrupt::InterruptConfigurable for McPwm<'d, PWM> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}
