#![cfg_attr(docsrs, procmacros::doc_replace(
    "mcpwm_freq" => {
        cfg(not(esp32h2)) => "40",
        cfg(esp32h2) => "32"
    },
    "clock_src" => {
        cfg(esp32) => "ADB-CLK (80 MHz)",
        cfg(esp32s3) => "ADB-CLK (80 MHz)",
        cfg(esp32c6) => "ADB-CLK (80 MHz)",
        cfg(esp32h2) => "ADB-CLK (80 MHz)",
    }
))]
//! # MCPWM Capture Module
//!
//! ## Overview
//! The `Capture` is responsible for managing the recording of the
//! capture timer during specific software or hardware triggered
//! 'capture' events.
//!
//! ### Capture events can be triggered in 2 ways:
//! 1. During a falling and/or Rising edge of a preconfigured GPIO pin.
//! 2. Software triggered captures though [`CaptureChannel::trigger_capture`].
//!
//! ### Configuration
//! This module provides the flexibility of configuring any GPIO pin as an input
//! for capturing the rising and/or falling edge of a signal. This module allows
//! for the ability to trigger software captures.
//!
//! ## Example
//!
//! This example shows configuring MCPWM for receiving
//! rising and falling edges from a GPIO pin.
//!
//! ```rust, no_run
//! use critical_section::Mutex;
//! use esp_hal::{
//!     mcpwm::{
//!         McPwm,
//!         PeripheralClockConfig,
//!         capture::{CaptureChannelConfig, CaptureMode, CaptureTimerConfig},
//!         mcpwm0,
//!     },
//!     time::Rate,
//! };
//!
//! static CAP0: Mutex<RefCell<Option<mcpwm0::CaptureChannel<'static, 0>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! // initialize peripheral
//! let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(__mcpwm_freq__))?;
//! let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
//!
//! // capture rising edges
//! let capture_cfg = CaptureChannelConfig::default().with_capture_mode(CaptureMode::RisingEdge);
//!
//! // initalize capture timer
//! let cap_timer_cfg = CaptureTimerConfig::default();
//! mcpwm.capture_timer.set_config(cap_timer_cfg);
//! mcpwm.capture_timer.start();
//!
//! // create capture channel with given config and `pin`
//! let mut capture = mcpwm.capture0.configure(capture_cfg).with_signal_input(pin);
//! capture.set_enable(true);
//! capture.listen();
//!
//! critical_section::with(|cs| CAP0.borrow_ref_mut(cs).replace(capture));
//!
//! #[esp_hal::handler]
//! fn interrupt_handler() {
//!     critical_section::with(|cs| {
//!         let mut capture = CAP0.borrow_ref_mut(cs);
//!         if let Some(capture) = capture.as_mut() {
//!             if capture.interrupt_is_set() {
//!                 let event = capture.events();
//!                 let time = event.time();
//!                 let edge = event.edge();
//!                 // do something with edge and time
//!                 capture.reset_interrupt();
//!             }
//!         }
//!     });
//! }
//! ```
use core::marker::PhantomData;

use enumset::EnumSet;

use super::{Event, PeripheralGuard};
pub use crate::pac::mcpwm0::{
    cap_ch_cfg::CAP0_MODE as CaptureMode,
    cap_status::CAP0_EDGE as CaptureEdge,
};
use crate::{
    gpio::interconnect::PeripheralInput,
    mcpwm::{
        Instance,
        PwmClockGuard,
        sync::{SyncInSelect, SyncSource},
    },
    pac,
};

/// Configuration for capture timer
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureTimerConfig {
    sync_phase: u32,
}

impl CaptureTimerConfig {
    /// Sets the sync phase for the capture timer
    pub fn with_sync_phase(self, sync_phase: u32) -> Self {
        Self { sync_phase, ..self }
    }
}

impl Default for CaptureTimerConfig {
    fn default() -> Self {
        Self { sync_phase: 0 }
    }
}

/// The MCPWM Capture Timer
///
/// ## Overview
/// This timer is used for all [`CaptureChannel`]'s for a given MCPWM instance.
/// * This timer can be configured with a sync source with [`CaptureTimerConfig`].
///
/// ### Sync Events
/// When this timer receives a sync event the counter of the timer is reset
/// to the phase value set in [`CaptureTimerConfig`].
///
/// **Note:** This timer always counts up towards a the positive direction.
pub struct CaptureTimer<'d, PWM: Instance> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, PWM: Instance> CaptureTimer<'d, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Self {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

    /// Start the capture timer to being counting and capturing events
    pub fn start(&mut self) {
        self.set_enable(true);
    }

    /// Pauses the capture timer
    pub fn pause(&mut self) {
        self.set_enable(false);
    }

    /// Stops the timer and resets the timers counter to 0
    /// **Warning**: This sets the timers sync phase to 0
    pub fn reset(&mut self) {
        self.set_enable(false);
        self.set_sync_phase(0);
        self.trigger_sync();
    }

    /// Configure the capture timer with the provided config
    pub fn set_config(&mut self, config: CaptureTimerConfig) {
        self.set_sync_phase(config.sync_phase);
    }

    /// Triggers a software sync event on the capture timer.
    /// Refer to how sync events are handled in the [`CaptureTimer`] documentation.
    pub fn trigger_sync(&mut self) {
        Self::cfg().modify(|_r, w| w.cap_sync_sw().set_bit());
    }

    /// Sets the capture timers sync source. Refer to how sync events are
    /// handled in the [`CaptureTimer`] documentation.
    pub fn set_sync_in(&mut self, sync_source: &impl SyncSource<PWM>) {
        let sync_in = sync_source.get_kind().into();
        self.set_sync_in_select(sync_in);
    }

    /// Clears the sync in
    pub fn clear_sync_in(&mut self) {
        self.set_sync_in_select(SyncInSelect::None);
    }

    fn set_enable(&mut self, enable: bool) {
        Self::cfg().modify(|_r, w| w.cap_timer_en().bit(enable));
    }

    fn set_sync_phase(&mut self, sync_phase: u32) {
        Self::phase().write(|w| unsafe { w.cap_phase().bits(sync_phase) });
    }

    fn set_sync_in_select(&mut self, sync_sel: SyncInSelect) {
        // SAFETY: Only CAP_TIMER_CFG accessed; unique per PWM instance
        Self::cfg().modify(|_r, w| {
            w.cap_synci_en().bit(sync_sel != SyncInSelect::None);
            unsafe { w.cap_synci_sel().bits(sync_sel as u8) }
        });
    }

    fn cfg() -> &'static crate::Reg<pac::mcpwm0::cap_timer_cfg::CAP_TIMER_CFG_SPEC> {
        let (info, _) = PWM::split();
        info.regs().cap_timer_cfg()
    }

    fn phase() -> &'static crate::Reg<pac::mcpwm0::cap_timer_phase::CAP_TIMER_PHASE_SPEC> {
        let (info, _) = PWM::split();
        info.regs().cap_timer_phase()
    }
}

/// Represents the capture event
/// Contains the capture time and the captured edge
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureEvent {
    time: u32,
    edge: CaptureEdge,
}

impl CaptureEvent {
    /// Gets the captured time
    pub const fn time(&self) -> u32 {
        self.time
    }

    /// Gets the captured edge
    pub const fn edge(&self) -> CaptureEdge {
        self.edge
    }
}

/// Configuration for capture channel
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
// Hash cannot be implemented for CaptureMode
pub struct CaptureChannelConfig {
    invert: bool,
    capture_mode: CaptureMode,
    prescaler: u8,
}

impl CaptureChannelConfig {
    /// Sets the invert value for the config
    pub fn with_invert(self, invert: bool) -> Self {
        Self { invert, ..self }
    }
    /// Sets the capture mode for the config
    pub fn with_capture_mode(self, capture_mode: CaptureMode) -> Self {
        Self {
            capture_mode,
            ..self
        }
    }
    /// Sets the prescaler for the config
    pub fn with_prescaler(self, prescaler: u8) -> Self {
        Self { prescaler, ..self }
    }
}

impl Default for CaptureChannelConfig {
    fn default() -> Self {
        Self {
            invert: false,
            capture_mode: CaptureMode::None,
            prescaler: 0,
        }
    }
}

/// Capture channel creator
pub struct CaptureChannelCreator<'d, const CHAN: u8, PWM: Instance> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
}

impl<'d, const CHAN: u8, PWM: Instance> CaptureChannelCreator<'d, CHAN, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Self {
            phantom: PhantomData,
            _guard: guard,
        }
    }

    /// Configure a new capture channel from a config
    pub fn configure(self, config: CaptureChannelConfig) -> CaptureChannel<'d, CHAN, PWM> {
        CaptureChannel::new(self._guard, config)
    }
}

/// A MCPWM Capture Channel
///
/// The MCPWM Capture Channel has the following functions:
/// * Enable/Disable capturing on this channel
/// * Setting the capture input GPIO pin
/// * Whether to trigger capture events on rising and/or falling edges
/// * Read the last capture edge, and the last capture time
pub struct CaptureChannel<'d, const CHAN: u8, PWM: Instance> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, const CHAN: u8, PWM: Instance> CaptureChannel<'d, CHAN, PWM> {
    pub(super) fn new(guard: PeripheralGuard, config: CaptureChannelConfig) -> Self {
        let mut channel = Self {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        };
        channel.configure(config);
        channel
    }

    pub(super) fn configure(&mut self, config: CaptureChannelConfig) {
        Self::cfg().modify(|_, w| {
            w.mode().variant(config.capture_mode);
            w.in_invert().variant(config.invert);
            unsafe { w.prescale().bits(config.prescaler) }
        });
    }

    /// Assign the input signal for the capture
    pub fn with_signal_input<'a>(self, input: impl PeripheralInput<'a>) -> Self {
        let (info, _) = PWM::split();
        let input_signal = info.capture_input_signal::<CHAN>();

        if input_signal as usize <= property!("gpio.input_signal_max") {
            let source = input.into();
            source.set_input_enable(true);
            input_signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", input_signal);
        }

        self
    }

    /// Enable or disables this capture channel
    pub fn set_enable(&mut self, enable: bool) {
        Self::cfg().modify(|_, w| w.en().variant(enable));
    }

    /// Set the config
    pub fn set_config(&mut self, config: CaptureChannelConfig) {
        self.configure(config);
    }

    /// Triggers a software capture on the current capture channel
    pub fn trigger_capture(&mut self) {
        Self::cfg().modify(|_, w| w.sw().set_bit());
    }

    /// Sets the capture channel to listen to captures on specific edge events
    /// This channel can listen to Falling, and/or Rising edges on any of the GPIO pins.
    #[instability::unstable]
    pub fn listen(&mut self) {
        let (info, state) = PWM::split();
        state.enable_listen::<CHAN>(info, EnumSet::only(Event::Capture), true);
    }

    /// Stops listening to events on this channel
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        let (info, state) = PWM::split();
        state.enable_listen::<CHAN>(info, EnumSet::only(Event::Capture), false);
    }

    /// If the interrupt was set for this channel
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        let (info, state) = PWM::split();
        state.interrupt_set::<CHAN>(info, Event::Capture)
    }

    /// Clear the interrupt
    #[instability::unstable]
    pub fn clear_interrupt(&self) {
        let (info, state) = PWM::split();
        state.clear_interrupt::<CHAN>(info, Event::Capture);
    }

    /// Gets the last captured event
    #[instability::unstable]
    pub fn events(&self) -> CaptureEvent {
        let (info, _) = PWM::split();
        let time = info.regs().cap_ch(CHAN as usize).read().value().bits();
        let edge = info.regs().cap_status().read().cap_edge(CHAN).variant();
        CaptureEvent { time, edge }
    }

    fn cfg() -> &'static crate::Reg<pac::mcpwm0::cap_ch_cfg::CAP_CH_CFG_SPEC> {
        let (info, _) = PWM::split();
        info.regs().cap_ch_cfg(CHAN as usize)
    }
}
