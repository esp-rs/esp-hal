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
//!
//! * PWM Timers 0, 1 and 2
//!     * Every PWM timer has a dedicated 8-bit clock prescaler.
//!     * The 16-bit counter in the PWM timer can work in count-up mode, count-down mode or
//!       count-up-down mode.
//!     * A hardware sync or software sync can trigger a reload on the PWM timer with a phase
//!       register.
//!     * Timers run until a perconfigured [`timer::StopCondition`], this enum also specifies
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
//!     * Modulating of PWM output by high-frequency carrier signals, useful when gate drivers are
//!       insulated with a transformer. (Not yet implemented)
//!     * Period, time stamps and important control registers have shadow registers with flexible
//!       updating methods.
//! * Capture Channels 0, 1 and 2
//!     * Every capture channel has one signal input. With an optional invert filter
//!     * Each capture module can be configured to detect rising and (or), falling edges on an
//!       external signal.
//!     * Each capture channel can trigger software capture event recoding the capture counter. The
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
    doc = "     * Capture timer's has it's own independent clock source from the MCPWM peripheral."
)]
//! * Fault Detection Module (Not yet implemented)
#![cfg_attr(
    not(soc_has_mcpwm_capture_clk_from_group),
    doc = "\nCapture clock source is `ADB-CLK (80 MHz)` by default.\n"
)]
//! Clock source is `__clock_src__`` by default.
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
//! Rate::from_khz(20))?; mcpwm.timer0.start(timer_clock_cfg);
//!
//! // pin will be high 50% of the time
//! pwm_pin.set_timestamp(50);
//! # {after_snippet}
//! ```

use core::{cell::Cell, marker::PhantomData};

use critical_section::Mutex;
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
    private::OnDrop,
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

/// The MCPWM peripheral
#[non_exhaustive]
pub struct McPwm<'d, PWM: Instance> {
    _phantom: PhantomData<&'d PWM>,

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
    /// `pwm_clk = clocks.crypto_pwm_clock / (prescaler + 1)`
    pub fn new(_peripheral: PWM, peripheral_clock: PeripheralClockConfig) -> Self {
        let (info, _) = PWM::split();
        let guard = PeripheralGuard::new(info.peripheral());

        // set prescaler for timer (0-2)
        info.regs()
            .clk_cfg()
            .write(|w| unsafe { w.clk_prescale().bits(peripheral_clock.prescaler) });

        // enable clock
        info.regs().clk().write(|w| w.en().set_bit());

        Self {
            _phantom: PhantomData,
            timer0: Timer::new(guard.clone()),
            timer1: Timer::new(guard.clone()),
            timer2: Timer::new(guard.clone()),
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
        let (info, _) = PWM::split();
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
}

/// Target frequency could not be set.
/// Check how the frequency is calculated in the corresponding method docs.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrequencyError;

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

/// Peripheral state for an MCPWM instance.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
pub struct State {
    int_ena: Mutex<Cell<u32>>,
}

unsafe impl Sync for State {}
impl State {
    /// Return if the interrupt for an event is set
    pub fn interrupt_set<const UNIT: u8>(&self, info: &Info, event: Event) -> bool {
        let regs = info.regs();
        let int_st = regs.int_st().read();
        match event {
            Event::TimerStop => int_st.timer_stop(UNIT).bit(),
            Event::TimerEqualPeriod => int_st.timer_tep(UNIT).bit(),
            Event::TimerEqualZero => int_st.timer_tez(UNIT).bit(),
            Event::Capture => int_st.cap(UNIT).bit(),
            Event::CompareA => int_st.cmpr_tea(UNIT).bit(),
            Event::CompareB => int_st.cmpr_teb(UNIT).bit(),
            Event::Fault => int_st.fault(UNIT).bit(),
            Event::FaultClear => int_st.fault_clr(UNIT).bit(),
            Event::FaultCycleByCycle => int_st.tz_cbc(UNIT).bit(),
            Event::FaultOneShotMode => int_st.tz_ost(UNIT).bit(),
        }
    }

    /// Clear the interrupt for an event on a specific UNIT #
    pub fn clear_interrupt<const UNIT: u8>(&self, info: &Info, event: Event) {
        let regs = info.regs();
        regs.int_clr().write(|w| match event {
            Event::TimerStop => w.timer_stop(UNIT).bit(true),
            Event::TimerEqualPeriod => w.timer_tep(UNIT).bit(true),
            Event::TimerEqualZero => w.timer_tez(UNIT).bit(true),
            Event::Capture => w.cap(UNIT).bit(true),
            Event::CompareA => w.cmpr_tea(UNIT).bit(true),
            Event::CompareB => w.cmpr_teb(UNIT).bit(true),
            Event::Fault => w.fault(UNIT).bit(true),
            Event::FaultClear => w.fault_clr(UNIT).bit(true),
            Event::FaultCycleByCycle => w.tz_cbc(UNIT).bit(true),
            Event::FaultOneShotMode => w.tz_ost(UNIT).bit(true),
        });
    }

    /// Enables listening for an event on a specific UNIT #
    pub fn enable_listen<const UNIT: u8>(&self, info: &Info, events: EnumSet<Event>, value: bool) {
        critical_section::with(|cs| {
            let int_ena = self.int_ena.borrow(cs);
            let regs = info.regs();

            int_ena.set(regs.int_ena().write(|w| {
                unsafe { w.bits(int_ena.take()) };

                for event in events {
                    match event {
                        Event::TimerStop => w.timer_stop(UNIT).bit(value),
                        Event::TimerEqualPeriod => w.timer_tep(UNIT).bit(value),
                        Event::TimerEqualZero => w.timer_tez(UNIT).bit(value),
                        Event::Capture => w.cap(UNIT).bit(value),
                        Event::CompareA => w.cmpr_tea(UNIT).bit(value),
                        Event::CompareB => w.cmpr_teb(UNIT).bit(value),
                        Event::Fault => w.fault(UNIT).bit(value),
                        Event::FaultClear => w.fault_clr(UNIT).bit(value),
                        Event::FaultCycleByCycle => w.tz_cbc(UNIT).bit(value),
                        Event::FaultOneShotMode => w.tz_ost(UNIT).bit(value),
                    };
                }

                w
            }));
        });
    }
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
    fn split() -> (&'static Info, &'static State);
}

#[cfg(soc_has_mcpwm0)]
impl Instance for crate::peripherals::MCPWM0<'_> {
    /// Returns peripheral data for MCPWM 0
    fn split() -> (&'static Info, &'static State) {
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

        static STATE: State = State {
            int_ena: Mutex::new(Cell::new(0)),
        };

        (&INFO, &STATE)
    }
}

#[cfg(soc_has_mcpwm1)]
impl Instance for crate::peripherals::MCPWM1<'_> {
    fn split() -> (&'static Info, &'static State) {
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

        static STATE: State = State {
            int_ena: Mutex::new(Cell::new(0)),
        };

        (&INFO, &STATE)
    }
}

#[allow(dead_code)] // Field is seemingly unused but we rely on its Drop impl
struct PwmClockGuard(OnDrop<fn()>);

impl PwmClockGuard {
    fn instance<PWM: Instance>() -> clocks::McpwmInstance {
        let (info, _) = PWM::split();
        match info.peripheral() {
            Peripheral::Mcpwm0 => clocks::McpwmInstance::Mcpwm0,
            #[cfg(soc_has_mcpwm1)]
            Peripheral::Mcpwm1 => clocks::McpwmInstance::Mcpwm1,
            _ => unreachable!(),
        }
    }

    pub fn new<PWM: Instance>() -> Self {
        ClockTree::with(move |clocks| Self::instance::<PWM>().request_function_clock(clocks));

        Self(OnDrop::new(|| {
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
