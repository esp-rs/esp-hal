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
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::{Input, InputConfig, Pull};
//! # use esp_hal::interrupt::Priority;
//! # use esp_hal::pcnt::{channel, unit, Pcnt};
//! # use core::{sync::atomic::Ordering, cell::RefCell, cmp::min};
//! # use critical_section::Mutex;
//! # use portable_atomic::AtomicI32;
//!
//! static UNIT0: Mutex<RefCell<Option<unit::Unit<'static, 1>>>> =
//! Mutex::new(RefCell::new(None)); static VALUE: AtomicI32 = AtomicI32::new(0);
//!
//! // Initialize Pulse Counter (PCNT) unit with limits and filter settings
//! let mut pcnt = Pcnt::new(peripherals.PCNT);
//! pcnt.set_interrupt_handler(interrupt_handler);
//! let u0 = pcnt.unit1;
//! u0.set_low_limit(Some(-100))?;
//! u0.set_high_limit(Some(100))?;
//! u0.set_filter(Some(min(10u16 * 80, 1023u16)))?;
//! u0.clear();
//!
//! // Set up channels with control and edge signals
//! let ch0 = &u0.channel0;
//! let config = InputConfig::default().with_pull(Pull::Up);
//! let pin_a = Input::new(peripherals.GPIO4, config);
//! let pin_b = Input::new(peripherals.GPIO5, config);
//! let (input_a, _) = pin_a.split();
//! let (input_b, _) = pin_b.split();
//! ch0.set_ctrl_signal(input_a.clone());
//! ch0.set_edge_signal(input_b.clone());
//! ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
//! ch0.set_input_mode(channel::EdgeMode::Increment,
//! channel::EdgeMode::Decrement);
//!
//! let ch1 = &u0.channel1;
//! ch1.set_ctrl_signal(input_b);
//! ch1.set_edge_signal(input_a);
//! ch1.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
//! ch1.set_input_mode(channel::EdgeMode::Decrement,
//! channel::EdgeMode::Increment);
//!
//! // Enable interrupts and resume pulse counter unit
//! u0.listen();
//! u0.resume();
//! let counter = u0.counter.clone();
//!
//! critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));
//!
//! // Monitor counter value and print updates
//! let mut last_value: i32 = 0;
//! loop {
//!     let value: i32 = counter.get() as i32 + VALUE.load(Ordering::SeqCst);
//!     if value != last_value {
//!         last_value = value;
//!     }
//! }
//!
//! #[handler(priority = Priority::Priority2)]
//! fn interrupt_handler() {
//!     critical_section::with(|cs| {
//!         let mut u0 = UNIT0.borrow_ref_mut(cs);
//!         if let Some(u0) = u0.as_mut() {
//!             if u0.interrupt_is_set() {
//!                 let events = u0.events();
//!                 if events.high_limit {
//!                     VALUE.fetch_add(100, Ordering::SeqCst);
//!                 } else if events.low_limit {
//!                     VALUE.fetch_add(-100, Ordering::SeqCst);
//!                 }
//!                 u0.reset_interrupt();
//!             }
//!         }
//!     });
//! }
//! # }
//! ```
//! 
//! [channel]: channel/index.html
//! [unit]: unit/index.html

use self::unit::Unit;
use crate::{
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{Interrupt, PCNT},
    system::GenericPeripheralGuard,
};

pub mod channel;
pub mod unit;

/// Pulse Counter (PCNT) peripheral driver.
pub struct Pcnt<'d> {
    _instance: PeripheralRef<'d, PCNT>,

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

    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Pcnt as u8 }>,
}

impl<'d> Pcnt<'d> {
    /// Return a new PCNT
    pub fn new(_instance: impl Peripheral<P = PCNT> + 'd) -> Self {
        crate::into_ref!(_instance);

        let guard = GenericPeripheralGuard::new();
        let pcnt = PCNT::regs();

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
            _guard: guard,
        }
    }

    /// Set the interrupt handler for the PCNT peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::PCNT);
        }
        unsafe { interrupt::bind_interrupt(Interrupt::PCNT, handler.handler()) };
        unwrap!(interrupt::enable(Interrupt::PCNT, handler.priority()));
    }
}

impl crate::private::Sealed for Pcnt<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Pcnt<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}
