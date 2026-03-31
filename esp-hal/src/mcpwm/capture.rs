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
    }
))]
//! # MCPWM Capture Module
//!
//! ## Overview
//! The `Capture` is resposible for managing the recording of the
//! capture timer during specific software or hardware triggered
//! 'capture' events.
//!
//! Capture events can be triggered in 2 ways:
//! During a falling and/or Rising edge of a preconfigured GPIO pin.
//! Software triggered captures though the CaptureChannel::trigger() function.
//!
//! ## Configuration
//! This module provides the flexiability of configuring any GPIO pin as an input
//! for capturing the rising and/or falling edge of a signal. This module allows
//! for the ability to trigger software captures.
//!
//! ### Capture Timer
//! Capture timer can be configured to sync with a PWM timer during specific events
//! either when the PWM timer sync out event, or a sync in from an external GPIO pin.
//! Note: Capture timer has a default source of __clock_src__
//!
//! ## Example
//!
//! This example shows configuring MCPWM for receiving
//! rising and falling edges from a GPIO pin.
//!
//! TODO Write code for this
use core::marker::PhantomData;

use super::PeripheralGuard;
pub use crate::pac::mcpwm0::{
    cap_ch_cfg::CAP0_MODE as CaptureMode,
    cap_status::CAP0_EDGE as CaptureEdge,
};
use crate::{
    gpio::interconnect::PeripheralInput,
    mcpwm::{
        Instance,
        PwmClockGuard,
        sync::{SyncSelection, SyncSource},
    },
};

/// The MCPWM Capture Timer
///
/// ## Overview
///  - This timer is shared by all instances of [`CaptureChannel`] for a MCPWM.
/// During capture events on any of the channels the time stored in this timer will be
/// loaded into the [`CaptureEvent`]'s time.
/// - A sync source can be set by calling [`CaptureTimer::set_sync_in`] with a sync source.
/// - A sync source can be removed by calling [`CaptureTimer::clear_sync_in`].
///
/// When this timer receives a capture event the counter of the timer is reset.
/// The value that the timer is set to is dependent on [`CaptureTimer::set_sync_phase`]
/// This timer always counts up towards a the positive direction
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

    /// Start the capture timer counting at APB_CLK
    pub fn start(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = PWM::info().regs();

        block
            .cap_timer_cfg()
            .modify(|_r, w| w.cap_timer_en().set_bit());
    }

    /// Pauses the capture timer
    pub fn pause(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = PWM::info().regs();

        block
            .cap_timer_cfg()
            .modify(|_r, w| w.cap_timer_en().clear_bit());
    }

    /// Stops the timer and resets the timers counter to 0
    /// Warning: This sets the timers sync phase to 0
    pub fn reset(&mut self) {
        self.pause();
        self.set_sync_phase(0);
        self.trigger_sync();
    }

    /// Set the sync phase
    pub fn set_sync_phase(&mut self, phase: u32) {
        // SAFETY:
        // We write to our MCPWM_CAP_TIMER_PHASE register
        let block = PWM::info().regs();

        block
            .cap_timer_phase()
            .write(|w| unsafe { w.cap_phase().bits(phase) });
    }

    /// Get the sync phase
    pub fn get_sync_phase(&mut self) -> u32 {
        // SAFETY:
        // We read from our MCPWM_CAP_TIMER_PHASE register
        let block = PWM::info().regs();

        block.cap_timer_phase().read().cap_phase().bits()
    }

    /// Clears the captures timer sync source
    pub fn clear_sync_in(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = PWM::info().regs();

        unsafe {
            block.cap_timer_cfg().modify(|_r, w| {
                w.cap_synci_en().clear_bit(); // Disable sync inputs
                w.cap_synci_sel().bits(SyncSelection::None as u8) // No sync input
            })
        };
    }

    /// ## Overview
    /// Sets the capture timers sync source. Refer to how sync events are
    /// handled in the [`CaptureTimer`] documentation.
    pub fn set_sync_in(&mut self, sync_source: impl SyncSource<PWM>) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = PWM::info().regs();

        let sync_kind = sync_source.get_kind();
        let sync_selection: SyncSelection = sync_kind.into();
        unsafe {
            block.cap_timer_cfg().modify(|_r, w| {
                w.cap_synci_en().set_bit(); // Enable sync input
                w.cap_synci_sel().bits(sync_selection as u8) // Set the sync selection
            })
        };
    }

    /// ## Overview
    /// This triggers a software sync event on the capture timer
    /// Refer to how sync events are handled in the [`CaptureTimer`] documentation.
    pub fn trigger_sync(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = PWM::info().regs();

        // Trigger a software sync
        block
            .cap_timer_cfg()
            .modify(|_r, w| w.cap_sync_sw().set_bit());
    }
}

/// Repersents the capture event
/// Contains the capture time and the captured edge
pub struct CaptureEvent {
    time: u32,
    edge: CaptureEdge,
}

impl CaptureEvent {
    /// Gets the captured time
    pub fn get_time(&self) -> u32 {
        self.time
    }

    /// Gets the captured edge
    pub fn get_edge(&self) -> CaptureEdge {
        self.edge
    }
}

/// Configuration for a capture channel
///
/// ## Overview
///
/// Capture channel can be configured with a signal input,
/// a invert signal boolean, a capture mode, and a prescaler.
///
/// The signal input is what the capture channel listens too for
/// rising and or falling edge events, which is configured in capture mode.
///
/// The invert describes if the input signal is inverted before
/// being sent to the capture channel. If your signal originally had
/// rising edges they are replaced with falling edges and vise versa.
/// Invert signal is useful if the polarity of your signal matters for your code.
///
/// The prescaler determines how often a capture event is generated
/// from incoming edges. Instead of capturing every valid edge,
/// the hardware will generate a capture event once every `prescale`
/// edges.
/// Note: passing in a value of prescaler = 0 will set a prescaler value of 1.
///
/// Prescaler rather then scaling the clock speed, can be thought of as a 'capture rate.'
/// The edges counted are those selected by the channel's edge configuration.
/// (rising, falling, or both). For example:
/// - If configured for rising edges only, captures occur every Nth rising edge
/// - If configured for both edges, captures occur every Nth edge (rising + falling combined)
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
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
        let mut new_channel = CaptureChannel {
            phantom: PhantomData,
            _guard: self._guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        };
        new_channel.configure(config);

        new_channel
    }
}

/// A MCPWM Capture Channel
///
/// The MCPWM Capture Channel has the following functions:
/// * Enable/Disable capturing on this channel
/// * Setting the capture input GPIO pin
/// * Weather to trigger capture events on rising and/or falling edges
/// * Read the last capture edge, and the last capture time
pub struct CaptureChannel<'d, const CHAN: u8, PWM: Instance> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, const CHAN: u8, PWM: Instance> CaptureChannel<'d, CHAN, PWM> {
    pub(super) fn configure(&mut self, config: CaptureChannelConfig) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = PWM::info().regs();

        let enabled = block.cap_ch_cfg(CHAN as usize).read().en().bit();
        block.cap_ch_cfg(CHAN as usize).write(|w| {
            w.mode().variant(config.capture_mode);
            w.in_invert().variant(config.invert);
            unsafe {
                w.prescale().bits(config.prescaler);
            }
            w.en().variant(enabled)
        });
    }

    /// Assign the input signal for the capture
    pub fn with_signal_input<'a>(self, input: impl PeripheralInput<'a>) -> Self {
        let input_signal = PWM::info().capture_input_signal::<CHAN>();

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
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = PWM::info().regs();
        block
            .cap_ch_cfg(CHAN as usize)
            .modify(|_, w| w.en().variant(enable));
    }

    /// Set the config
    pub fn set_config(&mut self, config: CaptureChannelConfig) {
        self.configure(config);
    }

    /// Sets the capture channel to listen to captures on specific edge events
    /// This channel can listen to Falling, and/or Rising edges on any of the GPIO pins.
    #[instability::unstable]
    pub fn listen(&mut self, capture_mode: CaptureMode) {
        // SAFETY:
        // We write to our MCPWM_CAP_CHx_CFG_REG register
        // We are modifying a shared peripheral interrupt enable register
        let block = PWM::info().regs();
        block
            .cap_ch_cfg(CHAN as usize)
            .write(|w| w.mode().variant(capture_mode));

        // Enable the capture interrupt
        block.int_ena().modify(|_, w| w.cap(CHAN).set_bit());
    }

    /// Stops listening to events on this channel
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        // SAFTEY:
        // We are writing to a SHARED peripheral interrupt enable register
        // Tldr lowk don't know if I should use a mutex or not??
        let block = PWM::info().regs();

        // We can unlisten from events simply by disabling the interrupt
        block.int_ena().modify(|_, w| w.cap(CHAN).clear_bit());
    }

    /// If the interrupt was set for this channel
    #[instability::unstable]
    pub fn is_interupt_set(&mut self) -> bool {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG register
        let block = PWM::info().regs();

        block.int_st().read().cap(CHAN).bit()
    }

    /// Clear the interrupt
    #[instability::unstable]
    pub fn clear_interrupt(&mut self) {
        // SAFTEY:
        // We only read from our MCPWM_INT_CLR_REG register
        let block = PWM::info().regs();

        block.int_clr().write(|w| w.cap(CHAN).bit(true));
    }

    /// Gets the last captured event
    #[instability::unstable]
    pub fn get_event(&mut self) -> CaptureEvent {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG & MCPWM_CAP_STATUS_REG register
        let block = PWM::info().regs();

        let time = block.cap_ch(CHAN as usize).read().value().bits();
        let edge = block.cap_status().read().cap_edge(CHAN).variant();
        CaptureEvent { time, edge }
    }
}
