//! # Pulse Counter peripheral driver
//!
//! ## Overview
//! The `PCNT (Pulse Counter)` driver for `ESP` chips is a software component
//! that provides an interface for controlling and utilizing the `PCNT`
//! peripheral. The `PCNT` peripheral is a hardware module available in `ESP`
//! chips, which functions as a pulse counter and encoder. It is capable of
//! counting pulses and monitoring changes in signal levels from external
//! sources.
//!
//! The `PCNT` driver is designed to offer convenient and efficient access to
//! the functionalities of the `PCNT` peripheral. It consists of two main
//! modules:
//!    * [channel]
//!    * [unit]
//!
//! The `channel` module allows users to configure and manage individual
//! channels of the `PCNT` peripheral. It provides methods to set various
//! parameters for each channel, such as control modes for signal edges, action
//! on control level, and configurations for positive and negative edge count
//! modes.
//!
//! The `unit` module is responsible for configuring and handling individual
//! units of the `PCNT` peripheral. Each unit represents a separate instance of
//! the `PCNT` module, identified by unit numbers like `Unit0`, `Unit1`, and so
//! on. Users can interact with these units to configure settings such as low
//! and high limits, thresholds, and optional filtering. The unit module also
//! enables users to pause, resume, and clear the counter, as well as enable or
//! disable interrupts for specific events associated with the unit.
//!
//! ⚠️: The examples for PCNT peripheral are quite extensive, so for a more
//! detailed study of how to use this driver please visit [the repository
//! with corresponding example].
//!
//! [channel]: channel/index.html
//! [unit]: unit/index.html
//! [the repository with corresponding example]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/pcnt_encoder.rs

use self::unit::Unit;
use crate::{
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{self, Interrupt},
    system::PeripheralClockControl,
};

pub mod channel;
pub mod unit;

pub struct Pcnt<'d> {
    _instance: PeripheralRef<'d, peripherals::PCNT>,
}

impl<'d> Pcnt<'d> {
    /// Return a new PCNT
    pub fn new(
        _instance: impl Peripheral<P = peripherals::PCNT> + 'd,
        interrupt: Option<InterruptHandler>,
    ) -> Self {
        crate::into_ref!(_instance);
        // Enable the PCNT peripherals clock in the system peripheral
        PeripheralClockControl::enable(crate::system::Peripheral::Pcnt);

        if let Some(interrupt) = interrupt {
            unsafe {
                interrupt::bind_interrupt(Interrupt::PCNT, interrupt.handler());
                interrupt::enable(Interrupt::PCNT, interrupt.priority()).unwrap();
            }
        }

        Pcnt { _instance }
    }

    /// Return a unit
    pub fn get_unit(&self, number: unit::Number) -> Unit {
        Unit::new(number)
    }
}
