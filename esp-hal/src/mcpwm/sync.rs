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

use crate::{gpio::interconnect::PeripheralInput, mcpwm::Info};

/// Sync line for MCPWM
pub struct SyncLine {
    number: u8,
    mcpwm_info: &'static Info,
}

impl SyncLine {
    pub(crate) fn new(number: u8, mcpwm_info: &'static Info) -> Self {
        Self { number, mcpwm_info }
    }

    /// Set the input signal for the sync line
    pub fn set_signal<'d>(&self, source: impl PeripheralInput<'d>) {
        // configure GPIO matrix → SYNC
        let info = self.mcpwm_info;
        let signal = info.sync_input_signal(self.number);

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
    pub fn set_invert(&self, invert: bool) {
        self.mcpwm_info
            .regs()
            .timer_synci_cfg()
            .modify(|_, w| w.external_synci_invert(self.number).variant(invert));
    }

    /// Get the kind of sync line this is
    pub fn kind(&self) -> SyncKind {
        match self.number {
            0 => SyncKind::SyncLine0,
            1 => SyncKind::SyncLine1,
            2 => SyncKind::SyncLine2,
            _ => unreachable!(),
        }
    }
}

/// Values for any of the sync selection fields in the timer configuration
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SyncKind {
    /// Select no sync input for the timer
    None       = 0,
    /// Sync out from timer0
    Timer0Sync = 1,
    /// Sync out from timer1
    Timer1Sync = 2,
    /// Sync out from timer2
    Timer2Sync = 3,
    /// Sync line 0
    SyncLine0  = 4,
    /// Sync line 1
    SyncLine1  = 5,
    /// Sync line 2
    SyncLine2  = 6,
}
