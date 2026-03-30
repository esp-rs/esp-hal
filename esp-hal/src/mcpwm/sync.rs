//! # MCPWM Sync Module
//!
//! ## Overview
//! The `Sync` is resposible for managing the different ways
//! MCPWM can listen to sync events. There are 2 different
//! sync sources one that can come from a Timer's sync_out
//! or from a Sync line that can be triggered by an external signal.
//!
//! This module provides the flexability to map any of the sync line
//! to any GPIO signal. Aswell to repersent any timer's sync_out source.
use core::marker::PhantomData;

use crate::{gpio::interconnect::PeripheralInput, mcpwm::PwmPeripheral};

/// Must hide the get kind from public facing API
/// prevents users from creating any sync kind
mod sealed {
    /// Repersents the different types of sync sources
    /// Either from sync lines or from timers sync out
    #[derive(Clone, Copy)]
    pub enum SyncKind {
        SyncLine(u8),
        TimerSyncOut(u8),
    }

    pub trait InternalSyncSource {
        fn get_kind(&self) -> super::SyncKind;
    }
}
/// Give our crate access to the internal sync source and sync kind
pub(crate) use sealed::{InternalSyncSource, SyncKind};

/// Public trait to repersent a sync source
pub trait SyncSource: InternalSyncSource {}

/// Sync out for timers
#[derive(Clone, Copy)]
pub struct SyncOut<'d> {
    timer: u8,
    _phantom: PhantomData<&'d u8>,
}

impl<'d> SyncOut<'d> {
    pub(crate) fn new<const TIM: u8>() -> Self {
        Self {
            timer: TIM,
            _phantom: PhantomData,
        }
    }
}

impl<'d> sealed::InternalSyncSource for SyncOut<'d> {
    fn get_kind(&self) -> SyncKind {
        SyncKind::TimerSyncOut(self.timer)
    }
}

/// There are only a limited number sync lines for the MCPWM unit
#[derive(Clone, Copy)]
pub struct SyncLine<'d, const SYNC: u8, PWM> {
    _phantom: PhantomData<&'d PWM>,
}

impl<'d, const SYNC: u8, PWM: PwmPeripheral> sealed::InternalSyncSource
    for SyncLine<'d, SYNC, PWM>
{
    fn get_kind(&self) -> SyncKind {
        SyncKind::SyncLine(SYNC)
    }
}

impl<'d, const SYNC: u8, PWM: PwmPeripheral> SyncLine<'d, SYNC, PWM> {
    /// Create a new sync line
    pub(super) fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Set the input signal for the sync line
    pub fn set_signal(&mut self, source: impl PeripheralInput<'d>) {
        // configure GPIO matrix → SYNC
        let signal = PWM::sync_input_signal::<SYNC>();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
    }
}

/// Values for any of the sync selection registers
#[repr(u8)]
#[derive(Copy, Clone)]
pub(crate) enum SyncSelection {
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
impl From<SyncKind> for SyncSelection {
    fn from(value: SyncKind) -> Self {
        match value {
            SyncKind::SyncLine(line) => match line {
                0 => SyncSelection::SyncLine0,
                1 => SyncSelection::SyncLine1,
                2 => SyncSelection::SyncLine2,
                _ => unreachable!(),
            },
            SyncKind::TimerSyncOut(timer) => match timer {
                0 => SyncSelection::Timer0SyncOut,
                1 => SyncSelection::Timer1SyncOut,
                2 => SyncSelection::Timer2SyncOut,
                _ => unreachable!(),
            },
        }
    }
}
