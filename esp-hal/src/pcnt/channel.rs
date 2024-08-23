//! # PCNT - Channel Configuration
//!
//! ## Overview
//! The `channel` module allows users to configure and manage individual
//! channels of the `PCNT` peripheral. It provides methods to set various
//! parameters for each channel, such as control modes for signal edges, action
//! on control level, and configurations for positive and negative edge count
//! modes.

use core::marker::PhantomData;

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

pub use crate::peripherals::pcnt::unit::conf0::{CTRL_MODE as CtrlMode, EDGE_MODE as EdgeMode};

/// PcntPin can be always high, always low, or an actual pin
#[derive(Clone, Copy)]
pub struct PcntSource {
    source: u8,
    inverted: bool,
}

impl PcntSource {
    /// Creates a `PcntSource` from an input pin with the specified
    /// configuration.
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
            inverted: false,
        }
    }

    /// Creates a `PcntSource` that is always high.
    pub fn always_high() -> Self {
        Self {
            source: ONE_INPUT,
            inverted: false,
        }
    }

    /// Creates a `PcntSource` that is always low.
    pub fn always_low() -> Self {
        Self {
            source: ZERO_INPUT,
            inverted: false,
        }
    }

    /// Inverts the `PcntSource` signal.
    pub fn invert(self) -> Self {
        Self {
            source: self.source,
            inverted: !self.inverted,
        }
    }
}

/// Represents a channel within a pulse counter unit.
pub struct Channel<'d, const UNIT: usize, const NUM: usize> {
    _phantom: PhantomData<&'d ()>,
    // Individual channels are not Send, since they share registers.
    _not_send: PhantomData<*const ()>,
}

impl<'d, const UNIT: usize, const NUM: usize> Channel<'d, UNIT, NUM> {
    /// return a new Channel
    pub(super) fn new() -> Self {
        Self {
            _phantom: PhantomData,
            _not_send: PhantomData,
        }
    }

    /// Configures how the channel behaves based on the level of the control
    /// signal.
    ///
    /// * `low` - The behaviour of the channel when the control signal is low.
    /// * `high` - The behaviour of the channel when the control signal is high.
    pub fn set_ctrl_mode(&self, low: CtrlMode, high: CtrlMode) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let conf0 = pcnt.unit(UNIT).conf0();

        conf0.modify(|_, w| {
            w.ch_hctrl_mode(NUM as u8)
                .variant(high)
                .ch_lctrl_mode(NUM as u8)
                .variant(low)
        });
    }

    /// Configures how the channel affects the counter based on the transition
    /// made by the input signal.
    ///
    /// * `neg_edge` - The effect on the counter when the input signal goes 1 ->
    ///   0.
    /// * `pos_edge` - The effect on the counter when the input signal goes 0 ->
    ///   1.
    pub fn set_input_mode(&self, neg_edge: EdgeMode, pos_edge: EdgeMode) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let conf0 = pcnt.unit(UNIT).conf0();

        conf0.modify(|_, w| {
            w.ch_neg_mode(NUM as u8).variant(neg_edge);
            w.ch_pos_mode(NUM as u8).variant(pos_edge)
        });
    }

    /// Set the control signal (pin/high/low) for this channel
    pub fn set_ctrl_signal(&self, source: PcntSource) -> &Self {
        let signal = match UNIT {
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
        };

        if (signal as usize) <= crate::gpio::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }
                .func_in_sel_cfg(signal as usize)
                .modify(|_, w| unsafe {
                    w.sel().set_bit();
                    w.in_inv_sel().bit(source.inverted);
                    w.in_sel().bits(source.source)
                });
        }
        self
    }

    /// Set the edge signal (pin/high/low) for this channel
    pub fn set_edge_signal(&self, source: PcntSource) -> &Self {
        let signal = match UNIT {
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
        };

        if (signal as usize) <= crate::gpio::INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }
                .func_in_sel_cfg(signal as usize)
                .modify(|_, w| unsafe {
                    w.sel().set_bit();
                    w.in_inv_sel().bit(source.inverted);
                    w.in_sel().bits(source.source)
                });
        }
        self
    }
}
