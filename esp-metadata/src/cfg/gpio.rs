//! This module contains configuration used in [device.gpio], as well as
//! functions that generate code for esp-hal.

use std::str::FromStr;

use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

use crate::{
    cfg::{GenericProperty, Value},
    generate_for_each_macro,
    number,
};

/// Additional properties (besides those defined in cfg.rs) for [device.gpio].
/// These don't get turned into symbols, but are used to generate code.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct GpioPinsAndSignals {
    /// The list of GPIO pins and their properties.
    pub pins: Vec<PinConfig>,

    /// The list of peripheral input signals.
    pub input_signals: Vec<IoMuxSignal>,

    /// The list of peripheral output signals.
    pub output_signals: Vec<IoMuxSignal>,
}

impl GenericProperty for GpioPinsAndSignals {}

/// Properties of a single GPIO pin.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct PinConfig {
    /// The GPIO pin number.
    pub pin: usize,

    /// Whether the GPIO has an output stage.
    #[serde(default)]
    pub input_only: bool,

    /// Available IO MUX functions for this pin.
    #[serde(default)]
    pub functions: FunctionMap,

    /// Available analog functions for this pin.
    #[serde(default)]
    pub analog: AnalogMap,

    /// Available LP/RTC IO functions for this pin.
    #[serde(default, alias = "rtc")]
    pub lp: LowPowerMap,

    /// GPIO pin is only available in certain cases.
    #[serde(default)]
    pub limited: bool,
}

/// Available alternate functions for a given GPIO pin.
///
/// Alternate functions allow bypassing the GPIO matrix by selecting a different
/// path in the multiplexers controlled by MCU_SEL.
///
/// Values of this struct correspond to rows in the IO MUX Pad List table.
///
/// Used in [device.gpio.pins[X].functions]. The GPIO function is not
/// written here as that is common to all pins. The values are signal names
/// listed in [device.gpio.input_signals] or [device.gpio.output_signals].
/// `None` means the pin does not provide the given alternate function.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct FunctionMap {
    #[serde(rename = "0")]
    af0: Option<String>,
    #[serde(rename = "1")]
    af1: Option<String>,
    #[serde(rename = "2")]
    af2: Option<String>,
    #[serde(rename = "3")]
    af3: Option<String>,
    #[serde(rename = "4")]
    af4: Option<String>,
    #[serde(rename = "5")]
    af5: Option<String>,
}

impl FunctionMap {
    const COUNT: usize = 6;

    /// Returns the signal associated with the nth alternate function.
    ///
    /// Note that not all alternate functions are defined. The number of the
    /// GPIO function is available separately. Not all alternate function have
    /// IO signals.
    pub fn get(&self, af: usize) -> Option<&str> {
        match af {
            0 => self.af0.as_deref(),
            1 => self.af1.as_deref(),
            2 => self.af2.as_deref(),
            3 => self.af3.as_deref(),
            4 => self.af4.as_deref(),
            5 => self.af5.as_deref(),
            _ => None,
        }
    }
}

/// Available analog functions for a given GPIO pin.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct AnalogMap {
    #[serde(rename = "0")]
    af0: Option<String>,
    #[serde(rename = "1")]
    af1: Option<String>,
    #[serde(rename = "2")]
    af2: Option<String>,
    #[serde(rename = "3")]
    af3: Option<String>,
    #[serde(rename = "4")]
    af4: Option<String>,
    #[serde(rename = "5")]
    af5: Option<String>,
}

impl AnalogMap {
    const COUNT: usize = 6;

    /// Returns the signal associated with the nth alternate function.
    pub fn get(&self, af: usize) -> Option<&str> {
        match af {
            0 => self.af0.as_deref(),
            1 => self.af1.as_deref(),
            2 => self.af2.as_deref(),
            3 => self.af3.as_deref(),
            4 => self.af4.as_deref(),
            5 => self.af5.as_deref(),
            _ => None,
        }
    }
}

/// Available RTC/LP functions for a given GPIO pin.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct LowPowerMap {
    #[serde(rename = "0")]
    af0: Option<String>,
    #[serde(rename = "1")]
    af1: Option<String>,
    #[serde(rename = "2")]
    af2: Option<String>,
    #[serde(rename = "3")]
    af3: Option<String>,
    #[serde(rename = "4")]
    af4: Option<String>,
    #[serde(rename = "5")]
    af5: Option<String>,
}

impl LowPowerMap {
    const COUNT: usize = 6;

    /// Returns the signal associated with the nth alternate function.
    pub fn get(&self, af: usize) -> Option<&str> {
        match af {
            0 => self.af0.as_deref(),
            1 => self.af1.as_deref(),
            2 => self.af2.as_deref(),
            3 => self.af3.as_deref(),
            4 => self.af4.as_deref(),
            5 => self.af5.as_deref(),
            _ => None,
        }
    }
}

/// An input or output peripheral signal. The names usually match the signal
/// name in the Peripheral Signal List table, without the `in` or `out` suffix.
/// If the `id` is `None`, the signal cannot be routed through the GPIO matrix.
///
/// If the TRM's signal table says "no" to Direct Input/Output via IO MUX, the
/// signal does not have an Alternate Function and must be routed through the
/// GPIO matrix.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct IoMuxSignal {
    /// The name of the signal.
    pub name: String,

    /// The numeric ID of the signal, if the signal can be routed through the
    /// GPIO matrix.
    #[serde(default)]
    pub id: Option<usize>,
}

impl super::GpioProperties {
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        let input_max = self
            .pins_and_signals
            .input_signals
            .iter()
            .filter_map(|s| s.id)
            .max()
            .unwrap_or(0) as u32;
        let output_max = self
            .pins_and_signals
            .output_signals
            .iter()
            .filter_map(|s| s.id)
            .max()
            .unwrap_or(0) as u32;

        [
            ("gpio.input_signal_max", false, Value::Number(input_max)),
            ("gpio.output_signal_max", false, Value::Number(output_max)),
        ]
        .into_iter()
    }
}

pub(crate) fn generate_gpios(gpio: &super::GpioProperties) -> TokenStream {
    let pin_numbers = gpio
        .pins_and_signals
        .pins
        .iter()
        .map(|pin| number(pin.pin))
        .collect::<Vec<_>>();

    let pin_peris = gpio
        .pins_and_signals
        .pins
        .iter()
        .map(|pin| format_ident!("GPIO{}", pin.pin))
        .collect::<Vec<_>>();

    let pin_attrs = gpio
        .pins_and_signals
        .pins
        .iter()
        .map(|pin| {
            // Input must come first
            if pin.input_only {
                vec![quote! { Input }, quote! {}]
            } else {
                vec![quote! { Input }, quote! { Output }]
            }
        })
        .collect::<Vec<_>>();

    let mut lp_functions = vec![];
    let mut expanded_lp_functions = vec![];
    let mut analog_functions = vec![];
    let mut expanded_analog_functions = vec![];

    let pin_afs = gpio
        .pins_and_signals
        .pins
        .iter()
        .map(|pin| {
            let mut input_afs = vec![];
            let mut output_afs = vec![];

            let pin_peri = format_ident!("GPIO{}", pin.pin);

            for af in 0..FunctionMap::COUNT {
                let Some(signal) = pin.functions.get(af) else {
                    continue;
                };

                let af_variant = format_ident!("_{af}");
                let mut found = false;

                // Is the signal present among the input signals?
                if let Some(signal) = gpio
                    .pins_and_signals
                    .input_signals
                    .iter()
                    .find(|s| s.name == signal)
                {
                    let signal_tokens = TokenStream::from_str(&signal.name).unwrap();
                    input_afs.push(quote! { #af_variant => #signal_tokens });
                    found = true;
                }

                // Is the signal present among the output signals?
                if let Some(signal) = gpio
                    .pins_and_signals
                    .output_signals
                    .iter()
                    .find(|s| s.name == signal)
                {
                    let signal_tokens = TokenStream::from_str(&signal.name).unwrap();
                    output_afs.push(quote! { #af_variant => #signal_tokens });
                    found = true;
                }

                assert!(
                    found,
                    "Signal '{signal}' not found in input signals for GPIO pin {}",
                    pin.pin
                );
            }

            fn create_matchers_for_signal(
                branches: &mut Vec<TokenStream>,
                pin_peri: &Ident,
                signal: &str,
            ) {
                // Split "NAMEnumber" format fragments into the NAME and the number. The function
                // returns `None` if the input string is not in this format. The NAME part can be
                // empty (i.e. this function can return `Some("", number)`).
                fn split_signal_with_number(fragment: &str) -> Option<(&str, usize)> {
                    // Find the first character that is not a letter.
                    let Some(breakpoint) = fragment
                        .char_indices()
                        .filter_map(|(idx, c)| if c.is_alphabetic() { None } else { Some(idx) })
                        .next()
                    else {
                        // fragment only contains letters
                        return None;
                    };

                    let number: usize = fragment[breakpoint..].parse().ok()?;

                    Some((&fragment[..breakpoint], number))
                }

                let signal_name = TokenStream::from_str(signal).unwrap();

                let full_signal = {
                    // The signal name, with numbers replaced with placeholders
                    let mut pattern = String::new();
                    let mut numbers = vec![];

                    let placeholders = ['n', 'm'];

                    let mut separator = "";
                    for fragment in signal.split('_') {
                        if let Some((prefix, n)) = split_signal_with_number(fragment) {
                            let placeholder = placeholders[numbers.len()];
                            numbers.push(number(n));
                            pattern = format!("{pattern}{separator}{prefix}{placeholder}")
                        } else {
                            pattern = format!("{pattern}{separator}{fragment}");
                        };

                        separator = "_";
                    }

                    if pattern == signal {
                        None
                    } else {
                        let pattern = format_ident!("{pattern}");

                        Some(quote! {
                            ( #signal_name, #pattern #(, #numbers)* )
                        })
                    }
                };

                if let Some(full_signal) = full_signal {
                    branches.push(quote! {
                        #full_signal, #pin_peri
                    });
                }
            }

            for af in 0..AnalogMap::COUNT {
                if let Some(signal) = pin.analog.get(af) {
                    let signal_name = TokenStream::from_str(signal).unwrap();
                    analog_functions.push(quote! { #signal_name, #pin_peri });
                    create_matchers_for_signal(&mut expanded_analog_functions, &pin_peri, signal);
                }
            }

            for af in 0..LowPowerMap::COUNT {
                if let Some(signal) = pin.lp.get(af) {
                    let signal_name = TokenStream::from_str(signal).unwrap();
                    lp_functions.push(quote! { #signal_name, #pin_peri });
                    create_matchers_for_signal(&mut expanded_lp_functions, &pin_peri, signal);
                }
            }

            quote! {
                ( #(#input_afs)* ) ( #(#output_afs)* )
            }
        })
        .collect::<Vec<_>>();

    let io_mux_accessor = if gpio.remap_iomux_pin_registers {
        let iomux_pin_regs = gpio.pins_and_signals.pins.iter().map(|pin| {
            let pin = number(pin.pin);
            let accessor = format_ident!("gpio{pin}");

            quote! { #pin => iomux.#accessor(), }
        });

        quote! {
            pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO0 {
                let iomux = crate::peripherals::IO_MUX::regs();
                match gpio_num {
                    #(#iomux_pin_regs)*
                    other => panic!("GPIO {} does not exist", other),
                }
            }

        }
    } else {
        quote! {
            pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO {
                crate::peripherals::IO_MUX::regs().gpio(gpio_num as usize)
            }
        }
    };

    let mut branches = vec![];
    for (((n, p), af), attrs) in pin_numbers
        .iter()
        .zip(pin_peris.iter())
        .zip(pin_afs.iter())
        .zip(pin_attrs.iter())
    {
        branches.push(quote! {
            #n, #p #af (#([#attrs])*)
        })
    }

    let for_each_gpio = generate_for_each_macro("gpio", &[("all", &branches)]);
    let for_each_analog = generate_for_each_macro(
        "analog_function",
        &[
            ("all", &analog_functions),
            ("all_expanded", &expanded_analog_functions),
        ],
    );
    let for_each_lp = generate_for_each_macro(
        "lp_function",
        &[
            ("all", &lp_functions),
            ("all_expanded", &expanded_lp_functions),
        ],
    );
    let input_signals = render_signals("InputSignal", &gpio.pins_and_signals.input_signals);
    let output_signals = render_signals("OutputSignal", &gpio.pins_and_signals.output_signals);

    quote! {
        /// This macro can be used to generate code for each `GPIOn` instance.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($n:literal, $gpio:ident ($($digital_input_function:ident => $digital_input_signal:ident)*) ($($digital_output_function:ident => $digital_output_signal:ident)*) ($([$pin_attribute:ident])*))`
        ///
        /// Macro fragments:
        ///
        /// - `$n`: the number of the GPIO. For `GPIO0`, `$n` is 0.
        /// - `$gpio`: the name of the GPIO.
        /// - `$digital_input_function`: the number of the digital function, as an identifier (i.e. for function 0 this is `_0`).
        /// - `$digital_input_function`: the name of the digital function, as an identifier.
        /// - `$digital_output_function`: the number of the digital function, as an identifier (i.e. for function 0 this is `_0`).
        /// - `$digital_output_function`: the name of the digital function, as an identifier.
        /// - `$pin_attribute`: `Input` and/or `Output`, marks the possible directions of the GPIO. Bracketed so that they can also be matched as optional fragments. Order is always Input first.
        ///
        /// Example data: `(0, GPIO0 (_5 => EMAC_TX_CLK) (_1 => CLK_OUT1 _5 => EMAC_TX_CLK) ([Input] [Output]))`
        #for_each_gpio

        /// This macro can be used to generate code for each analog function of each GPIO.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has two options for its "Individual matcher" case:
        ///
        /// - `all`: `($signal:ident, $gpio:ident)` - simple case where you only need identifiers
        /// - `all_expanded`: `(($signal:ident, $group:ident $(, $number:literal)+), $gpio:ident)` - expanded signal case, where you need the number(s) of a signal, or the general group to which the signal belongs. For example, in case of `ADC2_CH3` the expanded form looks like `(ADC2_CH3, ADCn_CHm, 2, 3)`.
        ///
        /// Macro fragments:
        ///
        /// - `$signal`: the name of the signal.
        /// - `$group`: the name of the signal, with numbers replaced by placeholders. For `ADC2_CH3` this is `ADCn_CHm`.
        /// - `$number`: the numbers extracted from `$signal`.
        /// - `$gpio`: the name of the GPIO.
        ///
        /// Example data:
        /// - `(ADC2_CH5, GPIO12)`
        /// - `((ADC2_CH5, ADCn_CHm, 2, 5), GPIO12)`
        ///
        /// The expanded syntax is only available when the signal has at least one numbered component.
        #for_each_analog

        /// This macro can be used to generate code for each LP/RTC function of each GPIO.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has two options for its "Individual matcher" case:
        ///
        /// - `all`: `($signal:ident, $gpio:ident)` - simple case where you only need identifiers
        /// - `all_expanded`: `(($signal:ident, $group:ident $(, $number:literal)+), $gpio:ident)` - expanded signal case, where you need the number(s) of a signal, or the general group to which the signal belongs. For example, in case of `SAR_I2C_SCL_1` the expanded form looks like `(SAR_I2C_SCL_1, SAR_I2C_SCL_n, 1)`.
        ///
        /// Macro fragments:
        ///
        /// - `$signal`: the name of the signal.
        /// - `$group`: the name of the signal, with numbers replaced by placeholders. For `ADC2_CH3` this is `ADCn_CHm`.
        /// - `$number`: the numbers extracted from `$signal`.
        /// - `$gpio`: the name of the GPIO.
        ///
        /// Example data:
        /// - `(RTC_GPIO15, GPIO12)`
        /// - `((RTC_GPIO15, RTC_GPIOn, 15), GPIO12)`
        ///
        /// The expanded syntax is only available when the signal has at least one numbered component.
        #for_each_lp

        /// Defines the `InputSignal` and `OutputSignal` enums.
        ///
        /// This macro is intended to be called in esp-hal only.
        #[macro_export]
        #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
        macro_rules! define_io_mux_signals {
            () => {
                #input_signals
                #output_signals
            };
        }

        /// Defines and implements the `io_mux_reg` function.
        ///
        /// The generated function has the following signature:
        ///
        /// ```rust,ignore
        /// pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO0 {
        ///     // ...
        /// # unimplemented!()
        /// }
        /// ```
        ///
        /// This macro is intended to be called in esp-hal only.
        #[macro_export]
        #[expect(clippy::crate_in_macro_def)]
        #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
        macro_rules! define_io_mux_reg {
            () => {
                #io_mux_accessor
            };
        }
    }
}

fn render_signals(enum_name: &str, signals: &[IoMuxSignal]) -> TokenStream {
    if signals.is_empty() {
        // If there are no signals, we don't need to generate an enum.
        return quote! {};
    }
    let mut variants = vec![];

    for signal in signals {
        // First, process only signals that have an ID.
        let Some(id) = signal.id else {
            continue;
        };

        let name = format_ident!("{}", signal.name);
        let value = number(id);
        variants.push(quote! {
            #name = #value,
        });
    }

    for signal in signals {
        // Now process signals that do not have an ID.
        if signal.id.is_some() {
            continue;
        };

        let name = format_ident!("{}", signal.name);
        variants.push(quote! {
            #name,
        });
    }

    let enum_name = format_ident!("{enum_name}");

    quote! {
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum #enum_name {
            #(#variants)*
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Deserialize)]
pub struct DedicatedGpioChannels {
    // Cpu -> list of Signals
    channels: Vec<Vec<String>>,
}

impl DedicatedGpioChannels {
    fn channel_count(&self) -> usize {
        assert!(
            self.channels
                .iter()
                .all(|channel| channel.len() == self.channels[0].len()),
            "All cores must have the same number of dedicated GPIO channels"
        );
        self.channels[0].len()
    }
}

impl GenericProperty for DedicatedGpioChannels {
    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let channel_count = self.channel_count();
        let channel_branches = (0..channel_count).map(number).collect::<Vec<_>>();
        let signal_branches = self
            .channels
            .iter()
            .enumerate()
            .flat_map(|(core, channels)| {
                channels.iter().enumerate().map(move |(channel, signal)| {
                    let signal = format_ident!("{signal}");
                    let core = number(core);
                    let channel = number(channel);
                    quote! { #core, #channel, #signal }
                })
            })
            .collect::<Vec<_>>();

        Some(generate_for_each_macro(
            "dedicated_gpio",
            &[
                ("channels", &channel_branches),
                ("signals", &signal_branches),
            ],
        ))
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let channel_count = number(self.channel_count());
        quote::quote! {
            ("dedicated_gpio.channel_count") => {
                #channel_count
            };
            ("dedicated_gpio.channel_count", str) => {
                stringify!(#channel_count)
            };
        }
    }
}
