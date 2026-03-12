//! Represents the clock tree of an MCU.
//!
//! The clock tree consists of:
//! - Clock sources
//!     - May be configurable or fixed.
//!     - If configurable, the parameter is the desired output frequency.
//! - Clock dividers
//! - Clock muxes
//! - Clock gates
//! - "Derived" clock sources, which act as clock sources derived from other clock sources.
//!
//! Some clock sources are fixed, others are configurable. Some multiplexers and dividers are
//! configured by other elements, others are user-configurable. The output, and some other
//! parameters, of the items is encoded as an expression.
//!
//! Code generation:
//! - An input enum for configurable multiplexers
//!     - we need to deduce which multiplexers are configurable and which are automatically
//!       configured.
//! - `configure` functions with some code filled out
//!     - `configure_impl` function calls for every item, that esp-hal must implement.
//!     - calls to upstream clock tree item `configure` functions
//! - Reference counts
//!     - On clock sources only. `esp_hal::init` is the only place where multiplexers are
//!       configured. Peripheral clock sources can freely be configured by the drivers.
//! - A `clock_source_in_use` bitmap
//!     - This is useful for quickly entering/skipping light sleep in auto-lightsleep mode.
//! - request/release functions that update the reference counts and the bitmap
//! - A cached output frequency value.

use std::collections::HashMap;

use anyhow::Result;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use serde::Deserialize;

use crate::{
    cfg::{
        ClockTreeNodeInstance,
        clock_tree::{
            ClockTreeNodeType,
            Expression,
            RejectExpression,
            ValidationContext,
            ValuesExpression,
            human_readable_frequency,
        },
        soc::ProcessedClockData,
    },
    number,
};

#[derive(Debug, Clone, Deserialize)]
pub struct Source {
    /// The unique name of the clock tree item.
    pub name: String,

    #[serde(default)]
    always_on: bool,

    /// If set, this expression will be used to validate the clock configuration.
    ///
    /// The expression may refer to clock sources, or any of the clock tree item's properties (e.g.
    /// `divisor`).
    #[serde(default)]
    reject: Option<RejectExpression>,

    /// Output frequency options. If omitted, the source has a fixed frequency.
    #[serde(default)]
    values: Option<ValuesExpression>,

    output: OutputExpression,
}

impl ClockTreeNodeType for Source {
    fn always_on(&self) -> bool {
        self.always_on
    }

    fn validate_source_data(&self, _ctx: &ValidationContext<'_>) -> Result<()> {
        Ok(())
    }

    fn is_configurable(&self) -> bool {
        self.values.is_some() || !self.output.is_constant()
    }

    fn config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let ty_name = instance.config_type_name();
        let state = tree.properties(instance.name_str()).field_name();
        let apply_fn_name = instance.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);
        let reject_exprs = self.reject.as_ref().map(|reject| {
            let mut variables = HashMap::new();

            let mut config_fields = vec![];

            variables.insert("VALUE", quote! { config.value() });
            reject.0.visit_variables(|var| {
                if var != "VALUE" {
                    config_fields.push((var, tree.properties(var).field_name()));
                }
            });

            reject.to_rust(&config_fields, variables)
        });
        quote! {
            pub fn #apply_fn_name(clocks: &mut ClockTree, config: #ty_name) {
                #reject_exprs
                let old_config = clocks.#state.replace(config);
                #hal_impl(clocks, old_config, config);
            }
        }
    }

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state_field = tree.properties(instance.name_str()).field_name();

        if self.values.is_some() {
            quote! { unwrap!(clocks.#state_field).value() }
        } else {
            self.output.0.to_rust(HashMap::new())
        }
    }

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream {
        self.impl_config_type(instance, None)
    }

    fn request_direct_dependencies(
        &self,
        _instance: &ClockTreeNodeInstance,
        _tree: &ProcessedClockData,
    ) -> TokenStream {
        // Normal sources don't have dependencies
        quote! {}
    }

    fn release_direct_dependencies(
        &self,
        _instance: &ClockTreeNodeInstance,
        _tree: &ProcessedClockData,
    ) -> TokenStream {
        // Normal sources don't have dependencies
        quote! {}
    }
}

impl Source {
    fn list_of_fixed_frequencies(&self) -> Option<Vec<u32>> {
        self.values.as_ref().and_then(|d| d.as_enum_values())
    }

    fn impl_config_type(
        &self,
        instance: &ClockTreeNodeInstance,
        extra_docs: Option<&str>,
    ) -> TokenStream {
        let clock_name = instance.name_str();
        let ty_name = instance.config_type_name();

        let base_docline = if self.values.is_none() {
            None
        } else if self.list_of_fixed_frequencies().is_some() {
            Some(format!("Selects the output frequency of `{clock_name}`."))
        } else {
            Some(format!(
                "The target frequency of the `{clock_name}` clock source."
            ))
        };

        let docline = match (base_docline, extra_docs) {
            (None, None) => None,
            (Some(base), None) => Some(base),
            (None, Some(extra)) => Some(extra.to_string()),
            (Some(base), Some(extra)) => Some(format!("{base} {extra}")),
        }
        .into_iter();

        if let Some(frequencies) = self.list_of_fixed_frequencies() {
            let mut eval_ctx = somni_expr::Context::new();

            let values = frequencies
                .iter()
                .map(|freq| format_ident!("_{}", freq))
                .collect::<Vec<_>>();

            let frequencies = frequencies
                .iter()
                .map(|freq| {
                    eval_ctx.add_variable("VALUE", *freq as u64);
                    eval_ctx
                        .evaluate_parsed::<u64>(&self.output.0.source, &self.output.0.expr)
                        .unwrap()
                })
                .collect::<Vec<_>>();

            let value_doclines = frequencies
                .iter()
                .map(|freq| {
                    let (amount, unit) = human_readable_frequency(*freq);
                    format!(" {amount} {unit}")
                })
                .collect::<Vec<_>>();
            let frequencies = frequencies
                .iter()
                .map(|freq| number(freq))
                .collect::<Vec<_>>();

            quote! {
                #(#[doc = #docline])*
                #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub enum #ty_name {
                    #(
                        #[doc = #value_doclines]
                        #values,
                    )*
                }

                impl #ty_name {
                    pub fn value(&self) -> u32 {
                        match self {
                           #(#ty_name::#values => #frequencies,)*
                       }
                    }
                }
            }
        } else {
            let mut extra_docs = vec![];
            let validate = self.values.as_ref().map(|d| {
                let (min, max) = d.as_range().expect("Invalid frequency range");

                let assert_failed = format!(
                    "`{clock_name}` output frequency value must be between {min} and {max} (inclusive)."
                );

                let (min_readable, min_unit) = human_readable_frequency(min as _);
                let (max_readable, max_unit) = human_readable_frequency(max as _);

                extra_docs = format!(r#"
 # Panics

 Panics if the output frequency value is outside the
 valid range ({min_readable} {min_unit} - {max_readable} {max_unit})."#)
                .lines().map(|l| quote! { #[doc = #l] }).collect();

                // Frequency is unsigned, avoid generating `>= 0`.
                if min == 0 {
                    quote! { ::core::assert!(frequency <= #max, #assert_failed); }
                } else {
                    quote! { ::core::assert!(frequency >= #min && frequency <= #max, #assert_failed); }
                }
            });

            quote! {
                #(#[doc = #docline])*
                #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub struct #ty_name(u32);

                impl #ty_name {
                    /// Creates a new clock source configuration.
                    #(#extra_docs)*
                    pub const fn new(frequency: u32) -> Self {
                        #validate
                        Self(frequency)
                    }
                }

                impl #ty_name {
                    pub fn value(&self) -> u32 {
                        self.0
                    }
                }
            }
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DerivedClockSource {
    #[serde(flatten)]
    pub source_options: Source,

    from: String,
}

impl ClockTreeNodeType for DerivedClockSource {
    fn input_clocks(&self) -> Vec<String> {
        vec![self.from.clone()]
    }

    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        anyhow::ensure!(
            ctx.has_clock(&self.from),
            "Clock `{}` is not defined",
            self.from
        );

        self.source_options.validate_source_data(ctx)
    }

    fn is_configurable(&self) -> bool {
        self.source_options.is_configurable()
    }

    fn config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.source_options.config_apply_function(instance, tree)
    }

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.source_options.node_frequency_impl(instance, tree)
    }

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream {
        let extra_docs = format!("Depends on `{}`.", self.from);
        self.source_options
            .impl_config_type(instance, Some(&extra_docs))
    }

    fn request_direct_dependencies(
        &self,
        _instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let request_fn_name = tree.node(&self.from).request_fn_name();
        quote! {
            #request_fn_name(clocks);
        }
    }

    fn release_direct_dependencies(
        &self,
        _instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let release_fn_name = tree.node(&self.from).release_fn_name();
        quote! {
            #release_fn_name(clocks);
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct OutputExpression(Expression);

impl OutputExpression {
    fn is_constant(&self) -> bool {
        !self.contains_ident()
    }

    fn contains_ident(&self) -> bool {
        let mut contains = false;
        self.0.visit_variables(|_| contains = true);
        contains
    }
}
