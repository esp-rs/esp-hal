//! # PCNT - channel configuration
//!
//! ## Overview
//! The `channel` module is part of the `PCNT` peripheral driver
//! for `ESP` chips.
//!
//! It provides a convenient and efficient way to configure and use
//! individual channels of the `PCNT` peripheral of pulse counting and signal
//! edge detection on ESP chips.

use super::unit;
use crate::{
    gpio::{InputPin, InputSignal, Pull, ONE_INPUT, ZERO_INPUT},
    peripheral::Peripheral,
    peripherals::GPIO,
};

/// Configuration for an PCNT input pin
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PcntInputConfig {
    /// Configuration for the internal pull-up resistors
    pub pull: Pull,
}

impl Default for PcntInputConfig {
    fn default() -> Self {
        Self { pull: Pull::None }
    }
}

/// Channel number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Channel0 = 0,
    Channel1 = 1,
}

pub use crate::peripherals::pcnt::unit::conf0::{CTRL_MODE as CtrlMode, EDGE_MODE as EdgeMode};

/// Pulse Counter configuration for a single channel
#[derive(Debug, Copy, Clone)]
pub struct Config {
    /// PCNT low control mode
    pub lctrl_mode: CtrlMode,
    /// PCNT high control mode
    pub hctrl_mode: CtrlMode,
    /// PCNT signal positive edge count mode
    pub pos_edge: EdgeMode,
    /// PCNT signal negative edge count mode
    pub neg_edge: EdgeMode,
    pub invert_ctrl: bool,
    pub invert_sig: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            lctrl_mode: CtrlMode::Reverse,
            hctrl_mode: CtrlMode::Reverse,
            pos_edge: EdgeMode::Increment,
            neg_edge: EdgeMode::Increment,
            invert_ctrl: false,
            invert_sig: false,
        }
    }
}

/// PcntPin can be always high, always low, or an actual pin
#[derive(Clone, Copy)]
pub struct PcntSource {
    source: u8,
}

impl PcntSource {
    pub fn from_pin<'a, P: InputPin>(
        pin: impl Peripheral<P = P> + 'a,
        pin_config: PcntInputConfig,
    ) -> Self {
        crate::into_ref!(pin);

        pin.init_input(
            pin_config.pull == Pull::Down,
            pin_config.pull == Pull::Up,
            crate::private::Internal,
        );

        Self {
            source: pin.number(crate::private::Internal),
        }
    }
    pub fn always_high() -> Self {
        Self { source: ONE_INPUT }
    }
    pub fn always_low() -> Self {
        Self { source: ZERO_INPUT }
    }
}

pub struct Channel {
    unit: unit::Number,
    channel: Number,
}

impl Channel {
    /// return a new Channel
    pub(super) fn new(unit: unit::Number, channel: Number) -> Self {
        Self { unit, channel }
    }

    /// Configure the channel
    pub fn configure(&mut self, ctrl_signal: PcntSource, edge_signal: PcntSource, config: Config) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let conf0 = pcnt.unit(self.unit as usize).conf0();

        conf0.modify(|_, w| {
            w.ch_hctrl_mode(self.channel as u8)
                .variant(config.hctrl_mode);
            w.ch_lctrl_mode(self.channel as u8)
                .variant(config.lctrl_mode);
            w.ch_neg_mode(self.channel as u8).variant(config.neg_edge);
            w.ch_pos_mode(self.channel as u8).variant(config.pos_edge)
        });
        self.set_ctrl_signal(ctrl_signal, config.invert_ctrl);
        self.set_edge_signal(edge_signal, config.invert_sig);
    }

    /// Set the control signal (pin/high/low) for this channel
    pub fn set_ctrl_signal(&self, source: PcntSource, invert: bool) -> &Self {
        let signal = match self.unit {
            unit::Number::Unit0 => match self.channel {
                Number::Channel0 => InputSignal::PCNT0_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT0_CTRL_CH1,
            },
            unit::Number::Unit1 => match self.channel {
                Number::Channel0 => InputSignal::PCNT1_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT1_CTRL_CH1,
            },
            unit::Number::Unit2 => match self.channel {
                Number::Channel0 => InputSignal::PCNT2_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT2_CTRL_CH1,
            },
            unit::Number::Unit3 => match self.channel {
                Number::Channel0 => InputSignal::PCNT3_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT3_CTRL_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit4 => match self.channel {
                Number::Channel0 => InputSignal::PCNT4_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT4_CTRL_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit5 => match self.channel {
                Number::Channel0 => InputSignal::PCNT5_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT5_CTRL_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit6 => match self.channel {
                Number::Channel0 => InputSignal::PCNT6_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT6_CTRL_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit7 => match self.channel {
                Number::Channel0 => InputSignal::PCNT7_CTRL_CH0,
                Number::Channel1 => InputSignal::PCNT7_CTRL_CH1,
            },
        };

        if (signal as usize) <= crate::gpio::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }
                .func_in_sel_cfg(signal as usize)
                .modify(|_, w| unsafe {
                    w.sel().set_bit();
                    w.in_inv_sel().bit(invert);
                    w.in_sel().bits(source.source)
                });
        }
        self
    }

    /// Set the edge signal (pin/high/low) for this channel
    pub fn set_edge_signal(&self, source: PcntSource, invert: bool) -> &Self {
        let signal = match self.unit {
            unit::Number::Unit0 => match self.channel {
                Number::Channel0 => InputSignal::PCNT0_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT0_SIG_CH1,
            },
            unit::Number::Unit1 => match self.channel {
                Number::Channel0 => InputSignal::PCNT1_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT1_SIG_CH1,
            },
            unit::Number::Unit2 => match self.channel {
                Number::Channel0 => InputSignal::PCNT2_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT2_SIG_CH1,
            },
            unit::Number::Unit3 => match self.channel {
                Number::Channel0 => InputSignal::PCNT3_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT3_SIG_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit4 => match self.channel {
                Number::Channel0 => InputSignal::PCNT4_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT4_SIG_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit5 => match self.channel {
                Number::Channel0 => InputSignal::PCNT5_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT5_SIG_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit6 => match self.channel {
                Number::Channel0 => InputSignal::PCNT6_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT6_SIG_CH1,
            },
            #[cfg(esp32)]
            unit::Number::Unit7 => match self.channel {
                Number::Channel0 => InputSignal::PCNT7_SIG_CH0,
                Number::Channel1 => InputSignal::PCNT7_SIG_CH1,
            },
        };

        if (signal as usize) <= crate::gpio::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }
                .func_in_sel_cfg(signal as usize)
                .modify(|_, w| unsafe {
                    w.sel().set_bit();
                    w.in_inv_sel().bit(invert);
                    w.in_sel().bits(source.source)
                });
        }
        self
    }
}
