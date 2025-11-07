use std::{collections::HashMap, ops::Range, str::FromStr};

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, pattern};
use proc_macro2::TokenStream;
use quote::quote;
use serde::{Deserialize, Serialize};

use crate::{cfg::Value, number_hex};

impl super::SocProperties {
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        let mut properties = vec![];

        if self.xtal_options.len() > 1 {
            // In this case, the HAL can use `for_each_soc_xtal_options` to see all available
            // options.
            properties.push(("soc.has_multiple_xtal_options", false, Value::Boolean(true)));
        } else {
            properties.push((
                "soc.xtal_frequency",
                false,
                Value::Number(self.xtal_options[0]),
            ));
        }

        properties.into_iter()
    }
}

/// Memory region.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct MemoryRange {
    pub name: String,
    #[serde(flatten)]
    pub range: Range<u32>,
}

/// Memory regions.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct MemoryMap {
    pub ranges: Vec<MemoryRange>,
}

impl super::GenericProperty for MemoryMap {
    fn macros(&self) -> Option<TokenStream> {
        let region_branches = self.ranges.iter().map(|region| {
            let name = region.name.to_uppercase();
            let start = number_hex(region.range.start as usize);
            let end = number_hex(region.range.end as usize);
            let size = format!(
                "{}",
                region.range.end as usize - region.range.start as usize
            );

            quote! {
                ( #name ) => {
                    #start .. #end
                };
                ( size as str, #name ) => {
                    #size
                };
            }
        });

        Some(quote! {
            /// Macro to get the address range of the given memory region.
            ///
            /// This macro provides two syntax options for each memory region:
            ///
            /// - `memory_range!("region_name")` returns the address range as a range expression (`start..end`).
            /// - `memory_range!(size as str, "region_name")` returns the size of the region as a string literal.
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
            macro_rules! memory_range {
                #(#region_branches)*
            }
        })
    }

    fn cfgs(&self) -> Option<Vec<String>> {
        Some(
            self.ranges
                .iter()
                .map(|region| format!("has_{}_region", region.name.to_lowercase()))
                .collect(),
        )
    }
}

/// A named template. Can contain `{{placeholder}}` placeholders that will be substituted with
/// actual values.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct Template {
    /// The name of the template. Other templates can substitute this template's value by using the
    /// `{{name}}` placeholder.
    pub name: String,

    /// The value of the template. Can contain `{{placeholder}}` placeholders that will be
    /// substituted with actual values.
    pub value: String,
}

/// A named peripheral clock signal. These are extracted from the SoC's TRM. Each element generates
/// a `Peripheral::Variant`, and code branches to enable/disable the clock signal, as well as to
/// assert the reset signal of the peripheral.
///
/// `template_params` is a map of substitutions, which will overwrite the defaults set in
/// `PeripheralClocks`. This way each peripheral clock signal can either simply use the defaults,
/// or override them with custom values in case they don't fit the scheme for some reason.
#[derive(Debug, Default, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct PeripheralClock {
    /// The name of the peripheral clock signal. Usually specified as CamelCase. Also determines
    /// the value of the `peripheral` template parameter, by converting the name to snake_case.
    pub name: String,

    /// Custom template parameters. These will override the defaults set in `PeripheralClocks`.
    #[serde(default)]
    template_params: HashMap<String, String>,

    /// When true, prevents resetting and disabling the peripheral on startup.
    // TODO: we should do something better, as we keep too many things running. USB/UART depends on
    // esp-println's output option and whether the USB JTAG is connected, TIMG0 is not necessary
    // outside of clock calibrations when the device has Systimer.
    #[serde(default)]
    keep_enabled: bool,
}

#[derive(Debug, Default, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct PeripheralClocks {
    pub(crate) templates: Vec<Template>,
    pub(crate) peripheral_clocks: Vec<PeripheralClock>,
}

impl PeripheralClocks {
    fn generate_macro(&self) -> Result<TokenStream> {
        let mut clocks = self.peripheral_clocks.clone();
        clocks.sort_by(|a, b| a.name.cmp(&b.name));

        let doclines = clocks.iter().map(|clock| {
            format!(
                "{} peripheral clock signal",
                clock
                    .name
                    .from_case(Case::Custom {
                        boundaries: &[Boundary::LOWER_UPPER, Boundary::DIGIT_UPPER],
                        pattern: pattern::capital,
                        delim: "",
                    })
                    .to_case(Case::UpperSnake)
            )
        });
        let clock_names = clocks
            .iter()
            .map(|clock| quote::format_ident!("{}", clock.name))
            .collect::<Vec<_>>();
        let keep_enabled = clocks.iter().filter_map(|clock| {
            clock
                .keep_enabled
                .then_some(quote::format_ident!("{}", clock.name))
        });

        let clk_en_arms = clocks
            .iter()
            .map(|clock| {
                let clock_name = quote::format_ident!("{}", clock.name);
                let clock_en = self.clk_en(clock)?;

                Ok(quote::quote! {
                    Peripheral::#clock_name => {
                        #clock_en
                    }
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let rst_arms = clocks
            .iter()
            .map(|clock| {
                let clock_name = quote::format_ident!("{}", clock.name);
                let rst = self.rst(clock)?;

                Ok(quote::quote! {
                    Peripheral::#clock_name => {
                        #rst
                    }
                })
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(quote! {
            /// Implement the `Peripheral` enum and enable/disable/reset functions.
            ///
            /// This macro is intended to be placed in `esp_hal::system`.
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
            macro_rules! implement_peripheral_clocks {
                () => {
                    #[doc(hidden)]
                    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
                    #[repr(u8)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    pub enum Peripheral {
                        #(
                            #[doc = #doclines]
                            #clock_names,
                        )*
                    }

                    impl Peripheral {
                        const KEEP_ENABLED: &[Peripheral] = &[
                            #(
                                Self::#keep_enabled,
                            )*
                        ];

                        const COUNT: usize = Self::ALL.len();

                        const ALL: &[Self] = &[
                            #(
                                Self::#clock_names,
                            )*
                        ];
                    }

                    unsafe fn enable_internal_racey(peripheral: Peripheral, enable: bool) {
                        match peripheral {
                            #(#clk_en_arms)*
                        }
                    }

                    unsafe fn assert_peri_reset_racey(peripheral: Peripheral, reset: bool) {
                        match peripheral {
                            #(#rst_arms)*
                        }
                    }
                }
            }
        })
    }

    fn substitute_into(
        &self,
        template_name: &str,
        periph: &PeripheralClock,
    ) -> Result<TokenStream> {
        fn placeholder(name: &str) -> String {
            // format! would work but it needs an insane syntax to escape the curly braces.
            let mut output = String::with_capacity(name.len() + 4);
            output.push_str("{{");
            output.push_str(name);
            output.push_str("}}");
            output
        }

        let mut substitutions = HashMap::new();
        for template in &self.templates {
            substitutions.insert(placeholder(&template.name), template.value.clone());
        }
        substitutions.insert(
            placeholder("peripheral"),
            periph
                .name
                .from_case(Case::Custom {
                    boundaries: &[Boundary::LOWER_UPPER, Boundary::DIGIT_UPPER],
                    pattern: pattern::capital,
                    delim: "",
                })
                .to_case(Case::Snake),
        );
        // Peripheral-specific keys overwrite template defaults
        for (key, value) in periph.template_params.iter() {
            substitutions.insert(placeholder(key), value.clone());
        }

        let template_key = placeholder(template_name);
        let mut template = substitutions[&template_key].clone();

        // Replace while there are substitutions left
        loop {
            let mut found = false;
            for (key, value) in substitutions.iter() {
                if template.contains(key) {
                    template = template.replace(key, value);
                    found = true;
                }
            }
            if !found {
                break;
            }
        }

        match proc_macro2::TokenStream::from_str(&template) {
            Ok(tokens) => Ok(tokens),
            Err(err) => anyhow::bail!("Failed to inflate {template_name}: {err}"),
        }
    }

    fn clk_en(&self, periph: &PeripheralClock) -> Result<TokenStream> {
        self.substitute_into("clk_en_template", periph)
            .with_context(|| format!("Failed to generate clock enable code for {}", periph.name))
    }

    fn rst(&self, periph: &PeripheralClock) -> Result<TokenStream> {
        self.substitute_into("rst_template", periph)
            .with_context(|| format!("Failed to generate reset code for {}", periph.name))
    }
}

#[derive(Debug, Default, Clone, Deserialize)]
pub struct DeviceClocks {
    pub(crate) peripheral_clocks: PeripheralClocks,
}

impl super::GenericProperty for DeviceClocks {
    fn macros(&self) -> Option<TokenStream> {
        match self.peripheral_clocks.generate_macro() {
            Ok(tokens) => Some(tokens),
            Err(err) => panic!(
                "{:?}",
                err.context("Failed to generate peripheral clock control macro")
            ),
        }
    }
}
