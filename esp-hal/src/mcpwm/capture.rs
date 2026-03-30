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
//! Capture timer can be configured to sync with a PWM timer during specific events
//! either when the PWM timer sync out event, or a sync in from an external GPIO pin.
use core::marker::PhantomData;

use super::PeripheralGuard;
pub use crate::pac::mcpwm0::{
    cap_ch_cfg::CAP0_MODE as CaptureMode,
    cap_status::CAP0_EDGE as CaptureEdge,
};
use crate::{
    gpio::interconnect::PeripheralInput,
    mcpwm::{
        PwmClockGuard,
        PwmPeripheral,
        sync::{SyncSelection, SyncSource},
    },
};

/// The MCPWM Capture Timer
///
/// ## Overview
///  - This timer is shared by all instances of [`CaptureChannel`] for a MCPWM.
/// During capture events on any of the channels the time stored in this timer will be
/// loaded into the [`CaptureEvent`]'s time.
/// - This is a timer has the clock source of APB_CLK.
/// - A sync source can be selected by calling [`CaptureTimer::set_sync`] with a sync source.
/// - A sync source can be removed by calling [`CaptureTimer::clear_sync`].
///
/// During a sync event on this capture timer the counter of the timer is reset.
/// The value that the timer reset to depends on the sync source:
/// - If the sync source came from a timer with [`super::Timer::get_sync_out`]
/// the value that it is reset to the phase value for the given timer.
///
/// - If the sync sorce came from a sync line in [`super::McPwm`] then the timer
/// is reset with a value of 0.
pub struct CaptureTimer<'d, PWM> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, PWM: PwmPeripheral> CaptureTimer<'d, PWM> {
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
        let block = unsafe { &*PWM::block() };

        block
            .cap_timer_cfg()
            .modify(|_r, w| w.cap_timer_en().set_bit());
    }

    /// Pauses the capture timer
    pub fn pause(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = unsafe { &*PWM::block() };

        block
            .cap_timer_cfg()
            .modify(|_r, w| w.cap_timer_en().clear_bit());
    }

    /// Clears the captures timer sync source
    pub fn clear_sync_in(&mut self) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = unsafe { &*PWM::block() };
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
    pub fn set_sync_in(&mut self, sync_source: impl SyncSource) {
        // SAFETY:
        // We modify our MCPWM_CAP_TIMER_CFG_REG register
        let block = unsafe { &*PWM::block() };

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
        let block = unsafe { &*PWM::block() };
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

/// A MCPWM Capture Channel
///
/// The MCPWM Capture Channel has the following functions:
/// * Enable/Disable capturing on this channel
/// * Setting the capture input GPIO pin
/// * Weather to trigger capture events on rising and/or falling edges
/// * Read the last capture edge, and the last capture time
pub struct CaptureChannel<'d, const CHAN: u8, PWM> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, const CHAN: u8, PWM: PwmPeripheral> CaptureChannel<'d, CHAN, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Self {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

    /// Enables this capture channel
    pub fn set_enable(&mut self, enable: bool) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };
        block
            .cap_ch_cfg(CHAN as usize)
            .modify(|_, w| w.en().variant(enable));
    }

    /// Set the capture signal (pin/high/low) for this channel
    pub fn set_signal_capture(&mut self, source: impl PeripheralInput<'d>) {
        let signal = PWM::capture_input_signal::<CHAN>();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
    }

    /// Sets the capture prescaler for the channel.
    /// Although the function of the prescaler on capture channels
    ///
    /// The prescaler determines how often a capture event is generated
    /// from incoming edges. Instead of capturing every valid edge,
    /// the hardware will generate a capture event once every `capture_rate`
    /// edges.
    ///
    /// The edges counted are those selected by the channel's edge configuration
    /// (rising, falling, or both). For example:
    ///
    /// - If configured for rising edges only, captures occur every Nth rising edge
    /// - If configured for both edges, captures occur every Nth edge (rising + falling combined)
    ///
    /// Passing in a value of prescaler = 0 into this function will set the prescaler value of 1
    pub fn set_prescaler(&mut self, prescaler: u8) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };
        unsafe {
            block
                .cap_ch_cfg(CHAN as usize)
                .modify(|_, w| w.prescale().bits(prescaler));
        }
    }

    /// Sets the capture channel to listen to captures on specific edge events
    /// This channel can listen to Falling, and/or Rising edges on any of the GPIO pins.
    pub fn listen(&mut self, capture_mode: CaptureMode) {
        // SAFETY:
        // We write to our MCPWM_CAP_CHx_CFG_REG register
        // We are modifying a shared peripheral interrupt enable register
        let block = unsafe { &*PWM::block() };
        block
            .cap_ch_cfg(CHAN as usize)
            .write(|w| w.mode().variant(capture_mode));

        // Enable the capture interrupt
        block.int_ena().modify(|_, w| w.cap(CHAN as u8).set_bit());
    }

    /// Stops listening to events on this channel
    pub fn unlisten(&mut self) {
        // SAFTEY:
        // We are writing to a SHARED peripheral interrupt enable register
        // Tldr lowk don't know if I should use a mutex or not??
        let block = unsafe { &*PWM::block() };

        // We can unlisten from events simply by disabling the interrupt
        block.int_ena().modify(|_, w| w.cap(CHAN as u8).clear_bit());
    }

    /// If the interrupt was set for this channel
    pub fn is_interupt_set(&mut self) -> bool {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG register
        let block = unsafe { &*PWM::block() };
        block.int_st().read().cap(CHAN as u8).bit()
    }

    pub fn clear_interrupt(&mut self) {
        // SAFTEY:
        // We only read from our MCPWM_INT_CLR_REG register
        let block = unsafe { &*PWM::block() };
        block.int_clr().write(|w| w.cap(CHAN).bit(true));
    }

    /// Gets the last captured event
    pub fn get_event(&mut self) -> CaptureEvent {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG & MCPWM_CAP_STATUS_REG register
        let block = unsafe { &*PWM::block() };
        let time = block.cap_ch(CHAN as usize).read().value().bits();
        let edge = block.cap_status().read().cap_edge(CHAN as u8).variant();
        CaptureEvent { time, edge }
    }
}
