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
//! 
use core::{marker::PhantomData, panic};

use esp_sync::RawMutex;

use super::PeripheralGuard;
pub use crate::pac::mcpwm0::{
    cap_ch_cfg::CAP0_MODE as CaptureMode,
    cap_status::CAP0_EDGE as CaptureEdge,
};
use crate::{
    gpio::{InputSignal, interconnect::PeripheralInput},
    mcpwm::{PwmClockGuard, PwmPeripheral},
};

static MUTEX: RawMutex = RawMutex::new();

/// The MCPWM Capture Timer
/// This timer is specific to the 3 channels
pub struct CaptureTimer<'d, PWM> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

//! Capture timer inside MCPWM
//! 
//! ## Overview
//! 
//! This is a capture timer with the clock source of APB_CLK.
impl<PWM: PwmPeripheral> CaptureTimer<'d, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Timer {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

    /// Start the capture timer counting at APB_CLK
    pub fn start(&mut self) {
        // SAFETY:
        // We write to our MCPWM_CAP_TIMER_CFG_REG register
        let block = unsafe { &*PWM::block() };

        block.cap_timer_cfg().write(|w| w.cap_timer_en().set_bit());
    }

    pub fn pause(&mut self) {
        // SAFETY:
        // We write to our MCPWM_CAP_TIMER_CFG_REG register
        let block = unsafe { &*PWM::block() };

        block
            .cap_timer_cfg()
            .write(|w| w.cap_timer_en().clear_bit());
    }
}

/// A MCPWM Capture Channel
///
/// The MCPWM Capture Channel has the following functions:
/// * Enable/Disable capturing on this channel
/// * Setting the capture input GPIO pin
/// * Weather to trigger capture events on rising and/or falling edges
/// * Read the last capture edge, and the last capture time 
pub struct CaptureChannel<'d, const CHAN: usize, PWM> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<const CHAN: usize, PWM: PwmPeripheral> CaptureChannel<CHAN, PWM> {
    pub(super) fn new<const CHAN: usize>(guard: PeripheralGuard) -> Self {
        Timer {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

    /// Enables this capture channel
    pub fn enable(&mut self) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };
        block.cap_ch_cfg(CHAN).modify(|_, w| w.en().set_bit());
    }

    /// Disable this capture channel
    pub fn disable(&mut self) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };
        block.cap_ch_cfg(CHAN).modify(|_, w| w.en().clear_bit());
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
    /// is better though of as a capture rate.
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
    /// This is useful for reducing interrupt load or effectively averaging
    /// measurements over multiple signal periods.
    /// 
    /// Passing in a value of capture_rate = 0 into this function will set the capture_rate value of 1
    pub fn set_capture_rate(&mut self, capture_rate : u8) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };
        unsafe {
            block.cap_ch_cfg(CHAN).modify(|_, w| w.prescale().bits(capture_rate));
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
            .cap_ch_cfg(CHAN)
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

    pub fn is_interupt_set<const CHAN: usize>(&mut self) -> bool {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG register
        let block = unsafe { &*PWM::block() };
        block.int_st().read().cap(CHAN as u8).bit()
    }

    pub fn last_captured_time<const CHAN: usize>(&mut self) -> u32 {
        // SAFTEY:
        // We only read from our MCPWM_INT_ST_REG register
        let block = unsafe { &*PWM::block() };
        block.cap_ch(CHAN).read().value().bits()
    }

    /// Gets the last captured edge on this channel
    pub fn get_captured_edge<const CHAN: usize>(&mut self) -> CaptureEdge {
        // SAFETY:
        // We only read from our MCPWM_CAP_STATUS_REG register
        let block = unsafe { &*PWM::block() };
        block.cap_status().read().cap_edge(CHAN as u8).variant()
    }
}
