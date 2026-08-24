//! # PCNT - Channel Configuration
//!
//! ## Overview
//! The `channel` module configures the two channels of a PCNT unit: control
//! modes for signal edges, action on control level, and positive/negative edge
//! count modes.

use core::marker::PhantomData;

pub use crate::pac::pcnt::unit::conf0::{CTRL_MODE as CtrlMode, EDGE_MODE as EdgeMode};
use crate::{
    gpio::{
        InputSignal,
        interconnect::{self, PeripheralInput},
    },
    pcnt::AnyPcntUnit,
    system::PeripheralGuard,
};

/// Represents a channel within a pulse counter unit.
pub struct Channel<'d, const NUM: usize> {
    unit: AnyPcntUnit<'d>,
    // Individual channels are not Send, since they share registers.
    _not_send: PhantomData<*const ()>,
    _guard: PeripheralGuard,
}

impl<'d, const NUM: usize> Channel<'d, NUM> {
    pub(super) fn new(unit: AnyPcntUnit<'d>) -> Self {
        const { ::core::assert!(NUM < 2) };
        let guard = PeripheralGuard::new(unit.info().peripheral);

        Self {
            unit,
            _not_send: PhantomData,
            _guard: guard,
        }
    }

    /// Configures how the channel behaves based on the level of the control
    /// signal.
    ///
    /// * `low` - The behaviour of the channel when the control signal is low.
    /// * `high` - The behaviour of the channel when the control signal is high.
    pub fn set_ctrl_mode(&self, low: CtrlMode, high: CtrlMode) {
        let conf0 = self
            .unit
            .register_block()
            .unit(self.unit.info().unit)
            .conf0();

        conf0.modify(|_, w| {
            w.ch_hctrl_mode(NUM as u8).variant(high);
            w.ch_lctrl_mode(NUM as u8).variant(low)
        });
    }

    /// Configures how the channel affects the counter based on the transition
    /// made by the input signal.
    ///
    /// * `neg_edge` - The effect on the counter when the input signal goes 1 -> 0.
    /// * `pos_edge` - The effect on the counter when the input signal goes 0 -> 1.
    pub fn set_input_mode(&self, neg_edge: EdgeMode, pos_edge: EdgeMode) {
        let conf0 = self
            .unit
            .register_block()
            .unit(self.unit.info().unit)
            .conf0();

        conf0.modify(|_, w| {
            w.ch_neg_mode(NUM as u8).variant(neg_edge);
            w.ch_pos_mode(NUM as u8).variant(pos_edge)
        });
    }

    /// Set the control signal (pin/high/low) for this channel
    pub fn set_ctrl_signal(&self, source: impl PeripheralInput<'d>) -> &Self {
        let signal = self.unit.info().ctrl_ch[NUM];
        self.connect_input(signal, source.into())
    }

    /// Set the edge signal (pin/high/low) for this channel
    pub fn set_edge_signal(&self, source: impl PeripheralInput<'d>) -> &Self {
        let signal = self.unit.info().sig_ch[NUM];
        self.connect_input(signal, source.into())
    }

    fn connect_input(&self, signal: InputSignal, source: interconnect::InputSignal<'d>) -> &Self {
        source.set_input_enable(true);
        signal.connect_to(&source);
        self
    }
}
