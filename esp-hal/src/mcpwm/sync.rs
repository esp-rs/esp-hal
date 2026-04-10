#![cfg_attr(docsrs, procmacros::doc_replace(
    "mcpwm_freq" => {
        cfg(not(esp32h2)) => "40",
        cfg(esp32h2) => "32"
    }
))]
//! # MCPWM Sync Module
//!
//! ## Overview
//! The `Sync` is responsible for managing the different ways
//! MCPWM can listen to sync events. There are 2 different types of
//! sync sources. One is a [`SyncOut`] that comes from [`super::Timer::sync_out`],
//! or from a [`SyncLine`].
//!
//! This module provides the flexibility to map any of the
//! MCPWM's [`SyncLine`] to any GPIO signal.
//!
//! ## Example
//!
//! ### Configuring a SyncLine signal
//! For configuring a [`SyncLine`] input signal, and then connecting
//! it to timer 0's sync in event. This is useful when you need to sync
//! the timers phase from an external signal, such as a zero-cross event
//! for 3-phase PWM.
//!
//! ```rust, no_run
//! use esp_hal::{
//!     mcpwm::{McPwm, PeripheralClockConfig},
//!     time::Rate,
//! };
//!
//! // initialize peripheral
//! let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(__mcpwm_freq__))?;
//! let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
//!
//! // connect sync line 0 to take input from `pin`
//! mcpwm.sync0.set_signal(pin);
//!
//! // connecting sync line 0 for a timer0's sync in
//! mcpwm.timer0.set_sync_in(&mcpwm.sync0);
//! ```
//!
//! ### Chaining 2 or more timers sync events
//! This is useful when many timers require
//! to be phase aligned for proper timing.
//!
//! ```rust, no_run
//! use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
//!
//! // initialize peripheral
//! let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(__mcpwm_freq__))?;
//! let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
//!
//! // set timer1's sync in
//! mcpwm.timer1.set_sync_in(&mcpwm.timer0.sync_out);
//! // set timer2's sync in
//! mcpwm.timer2.set_sync_in(&mcpwm.timer1.sync_out);
//! ```
use core::marker::PhantomData;

use crate::{gpio::interconnect::PeripheralInput, mcpwm::Instance};

#[allow(private_bounds)]
/// Public trait to represent a sync source for a PWM
pub trait SyncSource<PWM: Instance>: InternalSyncSource {}

/// Sync out created by timers
#[derive(Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyncOut<'d, const TIM: u8, PWM: Instance> {
    _phantom: PhantomData<&'d PWM>,
}

impl<'d, const TIM: u8, PWM: Instance> SyncSource<PWM> for SyncOut<'d, TIM, PWM> {}
impl<'d, const TIM: u8, PWM: Instance> SyncOut<'d, TIM, PWM> {
    pub(super) fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

/// There are only a limited number sync lines for the MCPWM unit
#[derive(Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyncLine<'d, const SYNC: u8, PWM: Instance> {
    _phantom: PhantomData<&'d PWM>,
}

impl<'d, const SYNC: u8, PWM: Instance> SyncSource<PWM> for SyncLine<'d, SYNC, PWM> {}
impl<'d, const SYNC: u8, PWM: Instance> SyncLine<'d, SYNC, PWM> {
    pub(super) fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Set the input signal for the sync line
    pub fn set_signal(&mut self, source: impl PeripheralInput<'d>) {
        // configure GPIO matrix → SYNC
        let info = PWM::info();
        let signal = info.sync_input_signal::<SYNC>();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
    }

    /// Inverts the input signal from the supplied input source
    /// If invert is true sync events are triggered on falling edges.
    /// If invert is false sync events are triggered on rising edges.
    pub fn set_invert(&mut self, invert: bool) {
        let info = PWM::info();
        info.regs()
            .timer_synci_cfg()
            .modify(|_, w| w.external_synci_invert(SYNC).variant(invert));
    }
}

/// Represents the different types of sync sources
/// Either from sync lines or from timers sync out.
///
/// Shall only be created from this file
#[derive(Clone, Copy)]
pub(crate) enum SyncKind {
    SyncLine(u8),
    TimerSyncOut(u8),
}

/// Internal trait to represent which pwm unit and
/// sync out it came from. Broadly represents sync lines,
/// and timer sync out.
pub(crate) trait InternalSyncSource: crate::private::Sealed {
    fn get_kind(&self) -> SyncKind;
}

impl<'d, const TIM: u8, PWM: Instance> crate::private::Sealed for SyncOut<'d, TIM, PWM> {}
impl<'d, const TIM: u8, PWM: Instance> InternalSyncSource for SyncOut<'d, TIM, PWM> {
    fn get_kind(&self) -> SyncKind {
        SyncKind::TimerSyncOut(TIM)
    }
}

impl<'d, const SYNC: u8, PWM: Instance> crate::private::Sealed for SyncLine<'d, SYNC, PWM> {}
impl<'d, const SYNC: u8, PWM: Instance> InternalSyncSource for SyncLine<'d, SYNC, PWM> {
    fn get_kind(&self) -> SyncKind {
        SyncKind::SyncLine(SYNC)
    }
}

/// Values for any of the sync selection registers
/// This is the internal value used by capture timer, and timers
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum SyncInSelect {
    /// Select no sync input for the capture timer
    None          = 0,
    /// Select the timers sync source for a timer's sync out,
    Timer0SyncOut = 1,
    Timer1SyncOut = 2,
    Timer2SyncOut = 3,
    /// Select the timers sync source from a sync line
    SyncLine0     = 4,
    SyncLine1     = 5,
    SyncLine2     = 6,
}

/// Provides a simple way of translating the internal SyncKind to
/// the value for sync selection used in SYNCI_SEL register fields
impl From<SyncKind> for SyncInSelect {
    fn from(value: SyncKind) -> Self {
        match value {
            // SAFETY: Const generics ensure TIM and SYNC are valid (0-2)
            // Runtime values for line, and timer are only created
            // from generic constants
            SyncKind::SyncLine(line) => match line {
                0 => SyncInSelect::SyncLine0,
                1 => SyncInSelect::SyncLine1,
                2 => SyncInSelect::SyncLine2,
                _ => panic!("Invalid sync line"),
            },
            SyncKind::TimerSyncOut(timer) => match timer {
                0 => SyncInSelect::Timer0SyncOut,
                1 => SyncInSelect::Timer1SyncOut,
                2 => SyncInSelect::Timer2SyncOut,
                _ => panic!("Invalid timer for sync out"),
            },
        }
    }
}
