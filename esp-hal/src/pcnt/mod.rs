#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Pulse Counter (PCNT)
//!
//! ## Overview
//! The PCNT module counts rising and/or falling edges of input signals. Each
//! unit is an independent counter with two channels. Channel edge and control
//! inputs can be configured separately.
//!
//! Units are peripheral singletons (`PCNT0_UNIT0`, `PCNT0_UNIT1`, …, and
//! `PCNT1_UNIT0`, … when the chip has a second register block). Construct a
//! [`Unit`] from a singleton; there is no hub driver.
//!
//! The hardware raises one interrupt per PCNT register block. [`Unit`] emulates
//! per-unit interrupts by installing a shared dispatcher that calls the handler
//! registered on each unit whose status bit is set.
//!
//! ## Examples
//! ### Decoding a quadrature encoder
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::gpio::{Input, InputConfig, Pull};
//! # use esp_hal::pcnt::{channel, Unit};
//! # use core::{sync::atomic::Ordering, cell::RefCell, cmp::min};
//! # use critical_section::Mutex;
//! # use portable_atomic::AtomicI32;
//!
//! static UNIT0: Mutex<RefCell<Option<Unit<'static>>>> = Mutex::new(RefCell::new(None));
//! static VALUE: AtomicI32 = AtomicI32::new(0);
//!
//! let mut u0 = Unit::new(peripherals.PCNT0_UNIT1);
//! u0.set_interrupt_handler(interrupt_handler);
//! u0.set_low_limit(Some(-100))?;
//! u0.set_high_limit(Some(100))?;
//! u0.set_filter(Some(min(10u16 * 80, 1023u16)))?;
//! u0.clear();
//!
//! let ch0 = &u0.channel0;
//! let config = InputConfig::default().with_pull(Pull::Up);
//! let pin_a = Input::new(peripherals.GPIO4, config);
//! let pin_b = Input::new(peripherals.GPIO5, config);
//! let input_a = pin_a.peripheral_input();
//! let input_b = pin_b.peripheral_input();
//! ch0.set_ctrl_signal(input_a.clone());
//! ch0.set_edge_signal(input_b.clone());
//! ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
//! ch0.set_input_mode(channel::EdgeMode::Increment, channel::EdgeMode::Decrement);
//!
//! let ch1 = &u0.channel1;
//! ch1.set_ctrl_signal(input_b);
//! ch1.set_edge_signal(input_a);
//! ch1.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
//! ch1.set_input_mode(channel::EdgeMode::Decrement, channel::EdgeMode::Increment);
//!
//! u0.listen();
//! u0.resume();
//! let counter = u0.counter.clone();
//!
//! critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));
//!
//! let mut last_value: i32 = 0;
//! loop {
//!     let value: i32 = counter.get() as i32 + VALUE.load(Ordering::SeqCst);
//!     if value != last_value {
//!         last_value = value;
//!     }
//! }
//!
//! #[esp_hal::handler]
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

use esp_sync::RawMutex;

use crate::{
    gpio::InputSignal,
    handler,
    interrupt::{InterruptHandler, Priority},
    pac::pcnt::RegisterBlock,
    peripherals::Interrupt,
    private::CFnPtr,
    system::Peripheral,
};

pub mod channel;
pub mod unit;

pub use unit::Unit;

/// Immutable per-unit metadata owned by each `PCNTn_UNITm` singleton.
#[doc(hidden)]
pub struct Info {
    unit: usize,
    peripheral: Peripheral,
    interrupt: Interrupt,
    dispatcher: InterruptHandler,
    register_block: *const RegisterBlock,
    /// Guards the registers shared by every unit of this register block.
    lock: &'static RawMutex,
    sig_ch: [InputSignal; 2],
    ctrl_ch: [InputSignal; 2],
}

unsafe impl Sync for Info {}

/// Mutable per-unit interrupt state.
#[doc(hidden)]
pub struct State {
    handler: CFnPtr,
}

impl State {
    const fn new() -> Self {
        Self {
            handler: CFnPtr::new(),
        }
    }

    fn store(&self, handler: extern "C" fn()) {
        self.handler.store(handler);
    }

    fn clear(&self) {
        self.handler.clear();
    }

    fn invoke(&self) -> bool {
        self.handler.call()
    }
}

impl Info {
    fn regs(&self) -> &'static RegisterBlock {
        unsafe { &*self.register_block }
    }

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.lock.lock(f)
    }
}

/// A peripheral singleton compatible with the PCNT unit driver.
#[doc(hidden)]
pub trait Instance: crate::private::Sealed + any::Degrade {
    fn parts(&self) -> (&'static Info, &'static State);
}

impl Instance for AnyPcntUnit<'_> {
    fn parts(&self) -> (&'static Info, &'static State) {
        any::delegate!(self, unit => { unit.parts() })
    }
}

impl AnyPcntUnit<'_> {
    fn info(&self) -> &'static Info {
        self.parts().0
    }

    fn state(&self) -> &'static State {
        self.parts().1
    }

    fn register_block(&self) -> &'static RegisterBlock {
        self.info().regs()
    }

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.info().lock(f)
    }
}

fn dispatch(interrupt: Interrupt) {
    for_each_pcnt_unit! {
        ($peri:ident, $variant:ident, $sys:ident, $regs:ident, $unit:literal, $irq:ident, $sig_ch0:ident, $sig_ch1:ident, $ctrl_ch0:ident, $ctrl_ch1:ident) => {
            if Interrupt::$irq == interrupt {
                let (info, state) = Instance::parts(&unsafe { crate::peripherals::$peri::steal() });
                if info
                    .regs()
                    .int_st()
                    .read()
                    .cnt_thr_event_u(info.unit as u8)
                    .bit()
                {
                    if !state.invoke() {
                        // Unhandled by the user, make sure the interrupt doesn't stay pending.
                        info
                            .regs()
                            .int_clr()
                            .write(|w| {
                                w.cnt_thr_event_u(info.unit as u8).set_bit()
                            });
                    }
                }
            }
        };
    }
}

for_each_pcnt_unit! {
    (interrupt $(($interrupt:ident)),*) => {
        $(
            paste::paste! {
                #[handler(priority = Priority::max())]
                fn [<pcnt_irq_ $interrupt>]() {
                    dispatch(Interrupt::$interrupt);
                }
            }
        )*
    };

    (regs $(($regs:ident)),*) => {
        $(
            paste::paste! {
                static [<$regs _LOCK>]: RawMutex = RawMutex::new();
            }
        )*
    };

    ($peri:ident, $variant:ident, $sys:ident, $regs:ident, $unit:literal, $interrupt:ident, $sig_ch0:ident, $sig_ch1:ident, $ctrl_ch0:ident, $ctrl_ch1:ident) => {
        impl crate::pcnt::Instance for crate::peripherals::$peri<'_> {
            fn parts(&self) -> (&'static crate::pcnt::Info, &'static crate::pcnt::State) {
                static INFO: crate::pcnt::Info = crate::pcnt::Info {
                    unit: $unit,
                    peripheral: crate::system::Peripheral::$sys,
                    interrupt: crate::peripherals::Interrupt::$interrupt,
                    dispatcher: paste::paste! { [<pcnt_irq_ $interrupt>] },
                    register_block: crate::peripherals::$regs::ptr(),
                    lock: paste::paste! { &crate::pcnt::[<$regs _LOCK>] },
                    sig_ch: [crate::gpio::InputSignal::$sig_ch0, crate::gpio::InputSignal::$sig_ch1],
                    ctrl_ch: [crate::gpio::InputSignal::$ctrl_ch0, crate::gpio::InputSignal::$ctrl_ch1],
                };
                static STATE: crate::pcnt::State = crate::pcnt::State::new();
                (&INFO, &STATE)
            }
        }
    };

    (all $(($peri:ident, $variant:ident, $sys:ident, $regs:ident, $unit:literal, $interrupt:ident, $sig_ch0:ident, $sig_ch1:ident, $ctrl_ch0:ident, $ctrl_ch1:ident)),*) => {
        crate::any_peripheral! {
            /// Any PCNT unit.
            pub peripheral AnyPcntUnit<'d> {
                $(
                    $variant(crate::peripherals::$peri<'d>),
                )*
            }
        }
    };
}
