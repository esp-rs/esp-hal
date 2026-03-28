//! # MCPWM Capture Module
//!
//! ## Overview
//! The `Capture` is resposible for managing the recording of the
//! capture timer during specific software or hardware triggered
//! 'capture' events.
//!
//! Capture events can be triggered in 2 ways:
//! During a falling and/or Rising edge of a preconfigured GPIO pin.
//! Software triggered captures though the Capture::capture_time() function.
//!
//! ## Configuration
//! This module provides the flexiability of configuring any GPIO pin as an input
//! for capturing the rising and/or falling edge of a signal. This module allows
//! for the ability to trigger software captures, essentially capturing the current
//! time in the capture clock.

use core::marker::PhantomData;
use core::panic;

use esp_sync::RawMutex;

use super::PeripheralGuard;
use crate::gpio::{InputSignal, interconnect::PeripheralInput};
use crate::mcpwm::{PwmClockGuard, PwmPeripheral};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum EdgeCaptureEvent {
    FallingEdge,
    RisingEdge,
    EitherEdge,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum CapturedEdge {
    FallingEdge,
    RisingEdge,
}

static MUTEX: RawMutex = RawMutex::new();

/// The MCPWM Capture Timer
/// This timer is specific to the 3 channels
pub struct CaptureTimer<'d, PWM> {
    phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<PWM: PwmPeripheral> CaptureTimer<'d, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Timer {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

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
/// *
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

    /// Enables capturing on this channel
    pub fn enable(&mut self) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };

        block.cap_ch_cfg(CHAN).write(|w| w.en().set_bit());
    }

    /// Disable capturing on this channel
    pub fn disable(&mut self) {
        // SAFETY:
        // We only write to our MCPWM_CAP_CHx_CFG_REG register
        let block = unsafe { &*PWM::block() };

        block.cap_ch_cfg(CHAN).write(|w| w.en().clear_bit());
    }

    /// Set the capture signal (pin/high/low) for this channel
    pub fn set_capture_signal(&mut self, source: impl PeripheralInput<'d>) {
        let signal = PWM::capture_input_signal::<CHAN>();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
    }

    /// Sets the capture channel to listen to captures on specific edge events
    /// This channel can listen to Falling, and/or Rising edges on any of the GPIO pins.
    pub fn listen(&mut self, edge_capture_event: EdgeCaptureEvent) {
        // SAFETY:
        // We write to our MCPWM_CAP_CHx_CFG_REG register
        // And we are modifying a shared peripheral interrupt enable register
        let block = unsafe { &*PWM::block() };

        // Maps the edge capture event to the proper bits in the capture
        // configuration register.
        //
        // In the MCPWM_CAP_CHx_CFG_REG register within the
        // the MCPWM_CAP0_MODE field of the register Bits[1..=2] discribe
        // the falling and/or rising edge capture
        //
        // The 2 bits within the MCPWM_CAPx_MODE field are as of the following
        // Bits[0] -> Selects weather or not Falling Edges should be captured
        // Bits[1] -> Selects weather or not Rising Edges should be captured
        //
        // Has been checked for the following chips: ESP32s3
        //
        // Either of them can be on or off allowing for the 4 configurations seen below.

        // Note: would be nice to be able to set this via an enum like the PCNT register block :)
        match edge_capture_event {
            EdgeCaptureEvent::FallingEdge => unsafe {
                // Bit0 set -> Falling edges
                block.cap_ch_cfg(CHAN).write(|w| w.mode().bits(0b01));
            },
            EdgeCaptureEvent::RisingEdge => unsafe {
                // Bit1 set -> Rising edges
                block.cap_ch_cfg(CHAN).write(|w| w.mode().bits(0b10));
            },
            EdgeCaptureEvent::EitherEdge => unsafe {
                // Bit0 and Bit1 set -> Either edge
                block.cap_ch_cfg(CHAN).write(|w| w.mode().bits(0b11));
            },
        };

        // Enable the capture interrupt
        MUTEX.lock(|| {
            // Note: would be nice to be able to get interrupts from channel number
            match CHAN {
                0 => block.int_ena().modify(|r, w| w.cap0().set_bit()),
                1 => block.int_ena().modify(|r, w| w.cap1().set_bit()),
                2 => block.int_ena().modify(|r, w| w.cap2().set_bit()),
                _ => unreachable!(),
            };
        });
    }

    /// Stops listening to events on this channel
    pub fn unlisten(&mut self) {
        // SAFTEY:
        // We are accessing a SHARED peripheral interrupt enable register
        let _ = self._guard;
        let block = unsafe { &*PWM::block() };

        // We can unlisten from events simply by disabling the interrupt
        MUTEX.lock(|| {
            // Note: would be nice to be able to get interrupts from channel number
            match CHAN {
                0 => block.int_ena().modify(|r, w| w.cap0().clear_bit()),
                1 => block.int_ena().modify(|r, w| w.cap1().clear_bit()),
                2 => block.int_ena().modify(|r, w| w.cap2().clear_bit()),
                _ => unreachable!(),
            };
        });
    }

    pub fn is_interupt_set<const CHAN: usize>(&mut self) -> bool {
        let block = unsafe { &*PWM::block() };

        // Note: would be nice to be able to get interrupts from channel number
        match CHAN {
            0 => block.int_raw().read().cap0().bit(),
            1 => block.int_raw().read().cap1().bit(),
            2 => block.int_raw().read().cap2().bit(),
            _ => unreachable!(),
        }
    }

    /// Gets the last captured edge on this channel
    pub fn get_captured_edge<const CHAN: usize>(&mut self) -> CapturedEdge {
        // SAFETY:
        // We only read from our MCPWM_CAP_STATUS_REG register
        let block = unsafe { &*PWM::block() };

        // Read the last edge status from the either of the 3 channels
        // the bit that we read will indicate if it was a faling edge.

        // Note: would be nice to be able to get the capture status from channel number
        let is_falling_edge = match CHAN {
            0 => block.cap_status().read().cap0_edge().bit(),
            1 => block.cap_status().read().cap1_edge().bit(),
            2 => block.cap_status().read().cap2_edge().bit(),
            _ => unreachable!(),
        };

        if is_falling_edge {
            CapturedEdge::FallingEdge
        } else {
            CapturedEdge::RisingEdge
        }
    }
}
