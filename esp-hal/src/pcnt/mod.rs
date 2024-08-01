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
    InterruptConfigurable,
};

pub mod channel;
pub mod unit;

pub struct Pcnt<'d> {
    _instance: PeripheralRef<'d, peripherals::PCNT>,

    /// Unit 0
    pub unit0: Unit<'d, 0>,
    /// Unit 1
    pub unit1: Unit<'d, 1>,
    /// Unit 2
    pub unit2: Unit<'d, 2>,
    /// Unit 3
    pub unit3: Unit<'d, 3>,
    #[cfg(esp32)]
    /// Unit 4
    pub unit4: Unit<'d, 4>,
    #[cfg(esp32)]
    /// Unit 5
    pub unit5: Unit<'d, 5>,
    #[cfg(esp32)]
    /// Unit 6
    pub unit6: Unit<'d, 6>,
    #[cfg(esp32)]
    /// Unit 7
    pub unit7: Unit<'d, 7>,
}

impl<'d> Pcnt<'d> {
    /// Return a new PCNT
    pub fn new(_instance: impl Peripheral<P = peripherals::PCNT> + 'd) -> Self {
        crate::into_ref!(_instance);

        // Enable the PCNT peripherals clock in the system peripheral
        PeripheralClockControl::reset(crate::system::Peripheral::Pcnt);
        PeripheralClockControl::enable(crate::system::Peripheral::Pcnt);

        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };

        // disable filter, all events, and channel settings
        for unit in pcnt.unit_iter() {
            unit.conf0().write(|w| unsafe {
                // All bits are accounted for in the TRM.
                w.bits(0)
            });
        }

        // Remove reset bit from units.
        pcnt.ctrl().modify(|_, w| {
            #[cfg(not(esp32))]
            let unit_count = 4;
            #[cfg(esp32)]
            let unit_count = 8;

            for i in 0..unit_count {
                w.cnt_rst_u(i).clear_bit();
            }

            w.clk_en().set_bit()
        });

        Pcnt {
            _instance,
            unit0: Unit::new(),
            unit1: Unit::new(),
            unit2: Unit::new(),
            unit3: Unit::new(),
            #[cfg(esp32)]
            unit4: Unit::new(),
            #[cfg(esp32)]
            unit5: Unit::new(),
            #[cfg(esp32)]
            unit6: Unit::new(),
            #[cfg(esp32)]
            unit7: Unit::new(),
        }
    }
}

impl<'d> crate::private::Sealed for Pcnt<'d> {}

impl<'d> InterruptConfigurable for Pcnt<'d> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            interrupt::bind_interrupt(Interrupt::PCNT, handler.handler());
            interrupt::enable(Interrupt::PCNT, handler.priority()).unwrap();
        }
    }
}
