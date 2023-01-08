use super::unit;
use crate::{
    gpio::{
        types::{InputSignal, ONE_INPUT, ZERO_INPUT},
        InputPin,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::GPIO,
};

/// Channel number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Channel0,
    Channel1,
}

/// PCNT channel action on signal edge
#[derive(Debug, Copy, Clone, Default)]
pub enum EdgeMode {
    /// Hold current count value
    Hold      = 0,
    /// Increase count value
    #[default]
    Increment = 1,
    /// Decrease count value
    Decrement = 2,
}

/// PCNT channel action on control level
#[derive(Debug, Copy, Clone, Default)]
pub enum CtrlMode {
    /// Keep current count mode
    Keep    = 0,
    /// Invert current count mode (increase -> decrease, decrease -> increase)
    #[default]
    Reverse = 1,
    /// Hold current count value
    Disable = 2,
}

/// Pulse Counter configuration for a single channel
#[derive(Debug, Copy, Clone, Default)]
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

/// PcntPin can be always high, always low, or an actual pin
pub enum PcntSignal<'a, P: InputPin> {
    Pin(PeripheralRef<'a, P>),
    High,
    Low,
}

impl<'a, P: InputPin> PcntSignal<'a, P> {
    pub fn from_pin(pin: impl Peripheral<P = P> + 'a) -> Self {
        crate::into_ref!(pin);
        PcntSignal::Pin(pin)
    }
}

impl<'a, P: InputPin> From<PcntSignal<'a, P>> for u8 {
    fn from(pin: PcntSignal<'a, P>) -> Self {
        match pin {
            PcntSignal::Pin(pin) => pin.number(),
            PcntSignal::High => ONE_INPUT,
            PcntSignal::Low => ZERO_INPUT,
        }
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
    pub fn configure<'b, CP: InputPin, EP: InputPin>(
        &mut self,
        ctrl_signal: PcntSignal<'b, CP>,
        edge_signal: PcntSignal<'b, EP>,
        config: Config,
    ) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let conf0 = match self.unit {
            unit::Number::Unit0 => &pcnt.u0_conf0,
            unit::Number::Unit1 => &pcnt.u1_conf0,
            unit::Number::Unit2 => &pcnt.u2_conf0,
            unit::Number::Unit3 => &pcnt.u3_conf0,
            #[cfg(esp32)]
            unit::Number::Unit4 => &pcnt.u4_conf0,
            #[cfg(esp32)]
            unit::Number::Unit5 => &pcnt.u5_conf0,
            #[cfg(esp32)]
            unit::Number::Unit6 => &pcnt.u6_conf0,
            #[cfg(esp32)]
            unit::Number::Unit7 => &pcnt.u7_conf0,
        };
        match self.channel {
            Number::Channel0 => {
                conf0.modify(|_, w| unsafe {
                    w.ch0_hctrl_mode()
                        .bits(config.hctrl_mode as u8)
                        .ch0_lctrl_mode()
                        .bits(config.lctrl_mode as u8)
                        .ch0_neg_mode()
                        .bits(config.neg_edge as u8)
                        .ch0_pos_mode()
                        .bits(config.pos_edge as u8)
                });
            }
            Number::Channel1 => {
                conf0.modify(|_, w| unsafe {
                    w.ch1_hctrl_mode()
                        .bits(config.hctrl_mode as u8)
                        .ch1_lctrl_mode()
                        .bits(config.lctrl_mode as u8)
                        .ch1_neg_mode()
                        .bits(config.neg_edge as u8)
                        .ch1_pos_mode()
                        .bits(config.pos_edge as u8)
                });
            }
        }
        self.set_ctrl_signal(ctrl_signal, config.invert_ctrl);
        self.set_edge_signal(edge_signal, config.invert_sig);
    }

    /// Set the control signal (pin/high/low) for this channel
    pub fn set_ctrl_signal<'b, P: InputPin>(&self, source: PcntSignal<'b, P>, invert: bool) -> &Self {
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
        let pin_num: u8 = source.into();

        if (signal as usize) <= crate::types::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(invert)
                    .in_sel()
                    .bits(pin_num)
            });
        }
        self
    }

    /// Set the edge signal (pin/high/low) for this channel
    pub fn set_edge_signal<'b, P: InputPin>(&self, source: PcntSignal<'b, P>, invert: bool) -> &Self {
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
        let pin_num: u8 = source.into();

        if (signal as usize) <= crate::types::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(invert)
                    .in_sel()
                    .bits(pin_num)
            });
        }
        self
    }
}
