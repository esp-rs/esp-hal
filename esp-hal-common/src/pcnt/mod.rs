//! Pulse Counter peripheral driver
//! 
//! ## Overview
//! The `PCNT (Pulse Control)` driver for `ESP` chips is a software component that provides
//! an interface for controlling and utilizing the `PCNT` peripheral.
//! The `PCNT` peripheral is a hardware module available in `ESP` chips, which
//! functions as a pulse counter and encoder. It is capable of counting pulses and monitoring
//! changes in signal levels from external sources.
//!
//! The `PCNT` driver is designed to offer convenient and efficient access to the functionalities
//! of the `PCNT` peripheral. It consists of two main modules:
//!    * [channel]
//!    * [unit]
//! 
//! Each serving sprecific purposes.
//! 
//! The `channel` module allows users to configure and manage individual channels of the PCNT peripheral.
//! It provides methods to set various parameters for each channel, such as control
//! modes for signal edges, action on control level, and configurations for positive and negative
//! edge count modes.
//! 
//! The `unit` module is responsible for configuring and handling individual units of the PCNT peripheral.
//! Each unit represents a separate instance of the PCNT module, identified by unit numbers
//! like `Unit0`, `Unit1`, and so on. User can interact with these units to configure settings
//! such as low and high limits, thresholds, and optional filtering. The unit module also enables
//! users to pause, resume, and clear the counter, as well as enable or disable interrupts for specific
//! events associated with the unit.
//! 
//! ## Example
//! ```no_run
//! let unit_number = unit::Number::Unit1;
//!
//! // setup a pulse couter
//! println!("setup pulse counter unit 0");
//! let pcnt = PCNT::new(peripherals.PCNT, &mut system.peripheral_clock_control);
//! let mut u0 = pcnt.get_unit(unit_number);
//! u0.configure(unit::Config {
//!     low_limit: -100,
//!     high_limit: 100,
//!     filter: Some(min(10u16 * 80, 1023u16)),
//!     ..Default::default()
//! })
//! .unwrap();
//!
//! println!("setup channel 0");
//! let mut ch0 = u0.get_channel(channel::Number::Channel0);
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut pin_a = io.pins.gpio5.into_pull_up_input();
//! let mut pin_b = io.pins.gpio6.into_pull_up_input();
//!
//! ch0.configure(
//!     PcntSource::from_pin(&mut pin_a),
//!     PcntSource::from_pin(&mut pin_b),
//!     channel::Config {
//!         lctrl_mode: channel::CtrlMode::Reverse,
//!         hctrl_mode: channel::CtrlMode::Keep,
//!         pos_edge: channel::EdgeMode::Decrement,
//!         neg_edge: channel::EdgeMode::Increment,
//!         invert_ctrl: false,
//!         invert_sig: false,
//!     },
//! );
//!
//! println!("setup channel 1");
//! let mut ch1 = u0.get_channel(channel::Number::Channel1);
//! ch1.configure(
//!     PcntSource::from_pin(&mut pin_b),
//!     PcntSource::from_pin(&mut pin_a),
//!     channel::Config {
//!         lctrl_mode: channel::CtrlMode::Reverse,
//!         hctrl_mode: channel::CtrlMode::Keep,
//!         pos_edge: channel::EdgeMode::Increment,
//!         neg_edge: channel::EdgeMode::Decrement,
//!         invert_ctrl: false,
//!         invert_sig: false,
//!     },
//! );
//! println!("subscribing to events");
//! u0.events(unit::Events {
//!     low_limit: true,
//!     high_limit: true,
//!     thresh0: false,
//!     thresh1: false,
//!     zero: false,
//! });
//!
//! println!("enabling interrupts");
//! u0.listen();
//! println!("resume pulse counter unit 0");
//! u0.resume();
//!
//! critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));
//!
//! interrupt::enable(peripherals::Interrupt::PCNT, interrupt::Priority::Priority2).unwrap();
//!
//! let mut last_value: i32 = 0;
//! loop {
//!     critical_section::with(|cs| {
//!         let mut u0 = UNIT0.borrow_ref_mut(cs);
//!         let u0 = u0.as_mut().unwrap();
//!         let value: i32 = u0.get_value() as i32 + VALUE.load(Ordering::SeqCst);
//!         if value != last_value {
//!             println!("value: {value}");
//!             last_value = value;
//!         }
//!     });
//! }
//! ...
//! 
//! #[interrupt]
//! fn PCNT() {
//!     critical_section::with(|cs| {
//!         let mut u0 = UNIT0.borrow_ref_mut(cs);
//!         let u0 = u0.as_mut().unwrap();
//!         if u0.interrupt_set() {
//!             let events = u0.get_events();
//!             if events.high_limit {
//!                 VALUE.fetch_add(100, Ordering::SeqCst);
//!             } else if events.low_limit {
//!                 VALUE.fetch_add(-100, Ordering::SeqCst);
//!             }
//!             u0.reset_interrupt();
//!         }
//!     });
//! }
//! ```
//! 
//! [channel]: channel/index.html
//! [unit]: unit/index.html
//! 

use self::unit::Unit;
use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

pub mod channel;
pub mod unit;

pub struct PCNT<'d> {
    _instance: PeripheralRef<'d, crate::peripherals::PCNT>,
}

impl<'d> PCNT<'d> {
    /// Return a new PCNT
    pub fn new(
        _instance: impl Peripheral<P = crate::peripherals::PCNT> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(_instance);
        // Enable the PCNT peripherals clock in the system peripheral
        peripheral_clock_control.enable(crate::system::Peripheral::Pcnt);
        PCNT { _instance }
    }

    /// Return a unit
    pub fn get_unit(&self, number: unit::Number) -> Unit {
        Unit::new(number)
    }
}
