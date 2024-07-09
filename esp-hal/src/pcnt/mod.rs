//! # Pulse Counter (PCNT)
//!
//! ## Overview
//! The PCNT module is designed to count the number of rising
//! and/or falling edges of input signals. They may contain multiple pulse
//! counter units in the module. Each unit is in effect an independent counter
//! with multiple channels, where each channel can increment/decrement the
//! counter on a rising/falling edge. Furthermore, each channel can be
//! configured separately.
//!
//! It consists of two main modules:
//!    * [channel]
//!    * [unit]
//!
//! ## Examples
//! ### Decoding a quadrature encoder
//! Visit the [PCNT Encoder] example for an example of using the peripheral.
//!
//! [channel]: channel/index.html
//! [unit]: unit/index.html
//! [PCNT Encoder]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/pcnt_encoder.rs

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
