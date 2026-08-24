//! # PCNT - Channel Configuration
//!
//! ## Overview
//! The `channel` module configures and manages individual channels of the
//! `PCNT` peripheral. It provides methods to set various parameters for each
//! channel, such as control modes for signal edges, action on control level,
//! and configurations for positive and negative edge count modes.

use core::marker::PhantomData;

pub use crate::pac::pcnt::unit::conf0::{CTRL_MODE as CtrlMode, EDGE_MODE as EdgeMode};
use crate::{
    gpio::{InputSignal, interconnect::PeripheralInput},
    pcnt::AnyPcnt,
    system::PeripheralGuard,
};

/// Represents a channel within a pulse counter unit.
pub struct Channel<'d, const UNIT: usize, const NUM: usize> {
    pcnt: AnyPcnt<'d>,
    // Individual channels are not Send, since they share registers.
    _not_send: PhantomData<*const ()>,
    _guard: PeripheralGuard,
}

impl<'d, const UNIT: usize, const NUM: usize> Channel<'d, UNIT, NUM> {
    /// return a new Channel
    pub(super) fn new(pcnt: AnyPcnt<'d>) -> Self {
        let guard = PeripheralGuard::new(pcnt.peripheral());

        Self {
            pcnt,
            _not_send: PhantomData,
            _guard: guard,
        }
    }

    fn ctrl_signal_id(&self) -> InputSignal {
        #[cfg(soc_has_pcnt1)]
        if self.pcnt.is_pcnt1() {
            return match (UNIT, NUM) {
                (0, 0) => InputSignal::PCNT1_U0_CTRL_CH0,
                (0, 1) => InputSignal::PCNT1_U0_CTRL_CH1,
                (1, 0) => InputSignal::PCNT1_U1_CTRL_CH0,
                (1, 1) => InputSignal::PCNT1_U1_CTRL_CH1,
                (2, 0) => InputSignal::PCNT1_U2_CTRL_CH0,
                (2, 1) => InputSignal::PCNT1_U2_CTRL_CH1,
                (3, 0) => InputSignal::PCNT1_U3_CTRL_CH0,
                (3, 1) => InputSignal::PCNT1_U3_CTRL_CH1,
                _ => unreachable!(),
            };
        }

        match UNIT {
            0 => match NUM {
                0 => InputSignal::PCNT0_CTRL_CH0,
                1 => InputSignal::PCNT0_CTRL_CH1,
                _ => unreachable!(),
            },
            1 => match NUM {
                0 => InputSignal::PCNT1_CTRL_CH0,
                1 => InputSignal::PCNT1_CTRL_CH1,
                _ => unreachable!(),
            },
            2 => match NUM {
                0 => InputSignal::PCNT2_CTRL_CH0,
                1 => InputSignal::PCNT2_CTRL_CH1,
                _ => unreachable!(),
            },
            3 => match NUM {
                0 => InputSignal::PCNT3_CTRL_CH0,
                1 => InputSignal::PCNT3_CTRL_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            4 => match NUM {
                0 => InputSignal::PCNT4_CTRL_CH0,
                1 => InputSignal::PCNT4_CTRL_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            5 => match NUM {
                0 => InputSignal::PCNT5_CTRL_CH0,
                1 => InputSignal::PCNT5_CTRL_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            6 => match NUM {
                0 => InputSignal::PCNT6_CTRL_CH0,
                1 => InputSignal::PCNT6_CTRL_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            7 => match NUM {
                0 => InputSignal::PCNT7_CTRL_CH0,
                1 => InputSignal::PCNT7_CTRL_CH1,
                _ => unreachable!(),
            },
            _ => unreachable!(),
        }
    }

    fn edge_signal_id(&self) -> InputSignal {
        #[cfg(soc_has_pcnt1)]
        if self.pcnt.is_pcnt1() {
            return match (UNIT, NUM) {
                (0, 0) => InputSignal::PCNT1_U0_SIG_CH0,
                (0, 1) => InputSignal::PCNT1_U0_SIG_CH1,
                (1, 0) => InputSignal::PCNT1_U1_SIG_CH0,
                (1, 1) => InputSignal::PCNT1_U1_SIG_CH1,
                (2, 0) => InputSignal::PCNT1_U2_SIG_CH0,
                (2, 1) => InputSignal::PCNT1_U2_SIG_CH1,
                (3, 0) => InputSignal::PCNT1_U3_SIG_CH0,
                (3, 1) => InputSignal::PCNT1_U3_SIG_CH1,
                _ => unreachable!(),
            };
        }

        match UNIT {
            0 => match NUM {
                0 => InputSignal::PCNT0_SIG_CH0,
                1 => InputSignal::PCNT0_SIG_CH1,
                _ => unreachable!(),
            },
            1 => match NUM {
                0 => InputSignal::PCNT1_SIG_CH0,
                1 => InputSignal::PCNT1_SIG_CH1,
                _ => unreachable!(),
            },
            2 => match NUM {
                0 => InputSignal::PCNT2_SIG_CH0,
                1 => InputSignal::PCNT2_SIG_CH1,
                _ => unreachable!(),
            },
            3 => match NUM {
                0 => InputSignal::PCNT3_SIG_CH0,
                1 => InputSignal::PCNT3_SIG_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            4 => match NUM {
                0 => InputSignal::PCNT4_SIG_CH0,
                1 => InputSignal::PCNT4_SIG_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            5 => match NUM {
                0 => InputSignal::PCNT5_SIG_CH0,
                1 => InputSignal::PCNT5_SIG_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            6 => match NUM {
                0 => InputSignal::PCNT6_SIG_CH0,
                1 => InputSignal::PCNT6_SIG_CH1,
                _ => unreachable!(),
            },
            #[cfg(esp32)]
            7 => match NUM {
                0 => InputSignal::PCNT7_SIG_CH0,
                1 => InputSignal::PCNT7_SIG_CH1,
                _ => unreachable!(),
            },
            _ => unreachable!(),
        }
    }

    /// Configures how the channel behaves based on the level of the control
    /// signal.
    ///
    /// * `low` - The behaviour of the channel when the control signal is low.
    /// * `high` - The behaviour of the channel when the control signal is high.
    pub fn set_ctrl_mode(&self, low: CtrlMode, high: CtrlMode) {
        let pcnt = self.pcnt.register_block();
        let conf0 = pcnt.unit(UNIT).conf0();

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
        let pcnt = self.pcnt.register_block();
        let conf0 = pcnt.unit(UNIT).conf0();

        conf0.modify(|_, w| {
            w.ch_neg_mode(NUM as u8).variant(neg_edge);
            w.ch_pos_mode(NUM as u8).variant(pos_edge)
        });
    }

    /// Set the control signal (pin/high/low) for this channel
    pub fn set_ctrl_signal<'p>(&self, source: impl PeripheralInput<'p>) -> &Self {
        let signal = self.ctrl_signal_id();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
        self
    }

    /// Set the edge signal (pin/high/low) for this channel
    pub fn set_edge_signal<'p>(&self, source: impl PeripheralInput<'p>) -> &Self {
        let signal = self.edge_signal_id();

        if signal as usize <= property!("gpio.input_signal_max") {
            let source = source.into();
            source.set_input_enable(true);
            signal.connect_to(&source);
        } else {
            warn!("Signal {:?} out of range", signal);
        }
        self
    }
}
