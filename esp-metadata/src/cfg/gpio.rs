//! This module contains configuration used in [device.gpio], as well as
//! functions that generate code for esp-hal.

use std::str::FromStr;

use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::Value, generate_for_each_macro, number};

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

/// Properties of a single GPIO pin.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct PinConfig {
    /// The GPIO pin number.
    pub pin: usize,

    /// Different capabilities implemented by this pin.
    pub kind: Vec<PinCapability>,

    /// Available IO MUX functions for this pin.
    #[serde(default)]
    pub functions: FunctionMap,

    /// Available analog functions for this pin.
    #[serde(default)]
    pub analog: AnalogMap,

    /// Available LP/RTC IO functions for this pin.
    #[serde(default, alias = "rtc")]
    pub lp: LowPowerMap,
}

/// Pin capabilities. Some of these will cause a trait to be implemented for the
/// given pin singleton. `UsbDevice` currently only changes what happens on GPIO
/// driver initialization.
#[derive(Debug, Clone, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum PinCapability {
    Input,
    Output,
    Analog,
    Rtc,
    Touch,
    UsbDm,
    UsbDp,
    // Pin has USB pullup according to the IO MUX Function list
    UsbDevice,
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
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, Value)> {
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
            ("gpio.input_signal_max", Value::Number(input_max)),
            ("gpio.output_signal_max", Value::Number(output_max)),
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
            let mut attrs = vec![];
            pin.kind.iter().for_each(|kind| match kind {
                PinCapability::Input => attrs.push(quote::quote! { Input }),
                PinCapability::Output => attrs.push(quote::quote! { Output }),
                PinCapability::Analog => attrs.push(quote::quote! { Analog }),
                PinCapability::Rtc => {
                    attrs.push(quote::quote! { RtcIo });
                    if pin.kind.contains(&PinCapability::Output) {
                        attrs.push(quote::quote! { RtcIoOutput });
                    }
                }
                PinCapability::Touch => attrs.push(quote::quote! { Touch }),
                PinCapability::UsbDm => attrs.push(quote::quote! { UsbDm }),
                PinCapability::UsbDp => attrs.push(quote::quote! { UsbDp }),
                PinCapability::UsbDevice => attrs.push(quote::quote! { UsbDevice }),
            });

            attrs
        })
        .collect::<Vec<_>>();

    let pin_afs = gpio
        .pins_and_signals
        .pins
        .iter()
        .map(|pin| {
            let mut input_afs = vec![];
            let mut output_afs = vec![];

            for af in 0..FunctionMap::COUNT {
                let Some(signal) = pin.functions.get(af) else {
                    continue;
                };

                let af_variant = quote::format_ident!("_{af}");
                let mut found = false;

                // Is the signal present among the input signals?
                if let Some(signal) = gpio
                    .pins_and_signals
                    .input_signals
                    .iter()
                    .find(|s| s.name == signal)
                {
                    let signal_tokens = TokenStream::from_str(&signal.name).unwrap();
                    input_afs.push(quote::quote! { #af_variant => #signal_tokens });
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
                    output_afs.push(quote::quote! { #af_variant => #signal_tokens });
                    found = true;
                }

                assert!(
                    found,
                    "Signal '{signal}' not found in input signals for GPIO pin {}",
                    pin.pin
                );
            }

            quote::quote! {
                ( #(#input_afs)* ) ( #(#output_afs)* )
            }
        })
        .collect::<Vec<_>>();

    let io_mux_accessor = if gpio.remap_iomux_pin_registers {
        let iomux_pin_regs = gpio.pins_and_signals.pins.iter().map(|pin| {
            let pin = number(pin.pin);
            let reg = format_ident!("GPIO{pin}");
            let accessor = format_ident!("gpio{pin}");

            quote::quote! { #pin => transmute::<&'static io_mux::#reg, &'static io_mux::GPIO0>(iomux.#accessor()), }
        });

        quote::quote! {
            pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO0 {
                use core::mem::transmute;

                use crate::{pac::io_mux, peripherals::IO_MUX};

                let iomux = IO_MUX::regs();
                unsafe {
                    match gpio_num {
                        #(#iomux_pin_regs)*
                        other => panic!("GPIO {} does not exist", other),
                    }
                }
            }

        }
    } else {
        quote::quote! {
            pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO {
                crate::peripherals::IO_MUX::regs().gpio(gpio_num as usize)
            }
        }
    };

    // Generates a macro that can select between a `then` and an `else` branch based
    // on whether a pin implement a certain attribute.
    //
    // In essence this expands to (in case of pin = GPIO5, attr = Analog):
    // `if typeof(GPIO5) == Analog { then_tokens } else { else_tokens }`
    let if_pin_is_type = {
        let mut branches = vec![];

        for (pin, attr) in pin_peris.iter().zip(pin_attrs.iter()) {
            branches.push(quote::quote! {
                #( (#pin, #attr, $then_tt:tt else $else_tt:tt ) => { $then_tt }; )*
            });

            branches.push(quote::quote! {
                (#pin, $t:tt, $then_tt:tt else $else_tt:tt ) => { $else_tt };
            });
        }

        quote::quote! {
            #[macro_export]
            macro_rules! if_pin_is_type {
                #(#branches)*
            }
        }
    };

    // Delegates AnyPin functions to GPIOn functions when the pin implements a
    // certain attribute.
    //
    // In essence this expands to (in case of attr = Analog):
    // `if typeof(anypin's current value) == Analog { call $code } else { panic }`
    let impl_for_pin_type = {
        let mut impl_branches = vec![];
        for (gpionum, peri) in pin_numbers.iter().zip(pin_peris.iter()) {
            impl_branches.push(quote::quote! {
                #gpionum => if_pin_is_type!(#peri, $on_type, {{
                    #[allow(unused_unsafe, unused_mut)]
                    let mut $inner_ident = unsafe { crate::peripherals::#peri::steal() };
                    #[allow(unused_braces)]
                    $code
                }} else {
                    $otherwise
                }),
            });
        }

        quote::quote! {
            #[macro_export]
            #[expect(clippy::crate_in_macro_def)]
            macro_rules! impl_for_pin_type {
                ($any_pin:ident, $inner_ident:ident, $on_type:tt, $code:tt else $otherwise:tt) => {
                    match $any_pin.number() {
                        #(#impl_branches)*
                        _ => $otherwise,
                    }
                };
                ($any_pin:ident, $inner_ident:ident, $on_type:tt, $code:tt) => {
                    impl_for_pin_type!($any_pin, $inner_ident, $on_type, $code else { panic!("Unsupported") })
                };
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
        branches.push(quote::quote! {
            #n, #p #af (#(#attrs)*)
        })
    }

    let for_each = generate_for_each_macro("gpio", &branches);
    let input_signals = render_signals("InputSignal", &gpio.pins_and_signals.input_signals);
    let output_signals = render_signals("OutputSignal", &gpio.pins_and_signals.output_signals);

    quote::quote! {
        #for_each

        #if_pin_is_type
        #impl_for_pin_type

        #[macro_export]
        macro_rules! define_io_mux_signals {
            () => {
                #input_signals
                #output_signals
            };
        }

        #[macro_export]
        #[expect(clippy::crate_in_macro_def)]
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
        return quote::quote! {};
    }
    let mut variants = vec![];

    for signal in signals {
        // First, process only signals that have an ID.
        let Some(id) = signal.id else {
            continue;
        };

        let name = format_ident!("{}", signal.name);
        let value = number(id);
        variants.push(quote::quote! {
            #name = #value,
        });
    }

    for signal in signals {
        // Now process signals that do not have an ID.
        if signal.id.is_some() {
            continue;
        };

        let name = format_ident!("{}", signal.name);
        variants.push(quote::quote! {
            #name,
        });
    }

    let enum_name = format_ident!("{enum_name}");

    quote::quote! {
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum #enum_name {
            #(#variants)*
        }
    }
}
