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
        clock_tree::{
            ClockTreeNodeType,
            Expression,
            RejectExpression,
            ValidationContext,
            ValuesExpression,
        },
        soc::ProcessedClockData,
    },
    number,
};

#[derive(Debug, Clone, Deserialize)]
pub struct Divider {
    /// The unique name of the clock tree item.
    pub name: String,

    /// If set, this expression will be used to validate the clock configuration.
    ///
    /// The expression may refer to clock sources, or any of the clock tree item's properties (e.g.
    /// `DIVISOR`).
    #[serde(default)]
    reject: Option<RejectExpression>,

    /// Possible divider values. May be a list of numbers or a range. If None, the divider value is
    /// fixed.
    #[serde(default)]
    divisors: Option<ValuesExpression>,

    /// The divider equation. The expression contains which clock is being divided. The expression
    /// may refer to clock sources, and the divider's value via `DIVISOR`.
    output: DividerOutputExpression,
}

impl ClockTreeNodeType for Divider {
    fn name_str<'a>(&'a self) -> &'a String {
        &self.name
    }

    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        let mut result = Ok(());
        self.output.visit_variables(|v| {
            if v == "DIVISOR" {
                return;
            }
            if !ctx.has_clock(v) && result.is_ok() {
                result = Err(anyhow::format_err!("{v} is not a valid clock signal name"));
            }
        });
        result
    }

    fn is_configurable(&self) -> bool {
        if self.divisors.is_some() {
            true
        } else {
            let mut contains_divisor = false;
            self.output.visit_variables(|var| {
                contains_divisor |= var == "DIVISOR";
            });
            contains_divisor
        }
    }

    fn config_apply_function(&self, tree: &ProcessedClockData<'_>) -> TokenStream {
        let ty_name = self.config_type_name();
        let state = self.node_state().field_name();
        let apply_fn_name = self.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);
        let reject_exprs = self.reject.as_ref().map(|reject| {
            let mut variables = HashMap::new();

            let mut config_fields = vec![];

            variables.insert("DIVISOR", quote! { config.value() });
            reject.0.visit_variables(|var| {
                if var != "DIVISOR" {
                    config_fields.push((var, tree.node(var).node_state().field_name()));
                }
            });

            reject.to_rust(&config_fields, variables)
        });
        quote! {
            pub fn #apply_fn_name(clocks: &mut ClockTree, config: #ty_name) {
                #reject_exprs
                clocks.#state = Some(config);
                #hal_impl(clocks, config);
            }
        }
    }

    fn config_apply_impl_function(&self, _tree: &ProcessedClockData<'_>) -> TokenStream {
        let ty_name = self.config_type_name();
        let apply_fn_name = self.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);
        quote! {
            fn #hal_impl(_clocks: &mut ClockTree, _new_config: #ty_name) {}
        }
    }

    fn apply_configuration(&self, expr: &Expression, tree: &ProcessedClockData<'_>) -> TokenStream {
        let config_function = self.config_apply_function_name();

        let state = self.config_type_name().unwrap();

        let cfg_expr_code = expr.to_rust({
            let mut variables = HashMap::new();
            for clock in tree.clock_tree.iter() {
                let clock_name = clock.name_str().as_str();
                let frequency_fn = clock.frequency_function_name();
                variables.insert(clock_name, quote! { #frequency_fn(clocks) });
            }
            variables
        });

        quote! {
            let config_value = #state::new(#cfg_expr_code);
            #config_function(clocks, config_value);
        }
    }

    fn node_frequency_impl(&self, tree: &ProcessedClockData<'_>) -> TokenStream {
        let state = self.node_state().field_name();
        let parent_clock = self.upstream_clock().unwrap();
        let parent_frequency_fn = tree.node(parent_clock).frequency_function_name();
        let divisor = quote! { unwrap!(clocks.#state).value() };

        let cfg_expr_code = self.output.to_rust({
            let mut variables = HashMap::new();
            variables.insert(parent_clock, quote! { #parent_frequency_fn(clocks) });
            variables.insert("DIVISOR", divisor);
            variables
        });

        cfg_expr_code
    }

    fn config_docline(&self) -> Option<String> {
        let clock_name = self.name.as_str();
        let expr = &self.output.source();
        Some(format!(
            r#" Configures the `{clock_name}` clock divider.

 The output is calculated as `OUTPUT = {expr}`."#
        ))
    }

    fn config_type(&self) -> Option<TokenStream> {
        let clock_name = &self.name;
        let ty_name = self.config_type_name()?;

        if let Some(dividers) = self.list_of_fixed_dividers() {
            let value_variants = dividers
                .iter()
                .map(|d| format_ident!("_{}", d))
                .collect::<Vec<_>>();
            let value_doc = dividers
                .iter()
                .map(|d| format!(" Selects `DIVISOR = {d}`."));
            let dividers = dividers.iter().map(number).collect::<Vec<_>>();

            let unknown_value = format!("Invalid {clock_name} divider value");

            Some(quote! {
                #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub enum #ty_name {
                    #(
                        #[doc = #value_doc]
                        #value_variants = #dividers,
                    )*
                }

                impl #ty_name {
                    const fn new(raw: u32) -> Self {
                        match raw {
                            #(#dividers => #ty_name::#value_variants,)*
                            _ => ::core::panic!(#unknown_value),
                        }
                    }

                    fn value(self) -> u32 {
                        match self {
                            #(#ty_name::#value_variants => #dividers,)*
                        }
                    }
                }
            })
        } else {
            let mut extra_docs = vec![];
            let validate = self.divisors.as_ref().map(|d| {
                let (min, max) = d.as_range().expect("Invalid divisor range");

                let assert_failed = format!(
                    "`{clock_name}` divisor value must be between {min} and {max} (inclusive)."
                );

                extra_docs = format!(
                    r#"
 # Panics

 Panics if the output frequency value is outside the
 valid range ({min} ..= {max})."#
                )
                .lines()
                .map(|l| quote! { #[doc = #l] })
                .collect();

                quote! { ::core::assert!(divisor >= #min && divisor <= #max, #assert_failed); }
            });

            Some(quote! {
                #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub struct #ty_name(u32);

                impl #ty_name {
                    /// Creates a new divider configuration.
                    #(#extra_docs)*
                    pub const fn new(divisor: u32) -> Self {
                        #validate
                        Self(divisor)
                    }

                    fn value(self) -> u32 {
                        self.0
                    }
                }
            })
        }
    }

    fn request_direct_dependencies(
        &self,
        _node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData<'_>,
    ) -> TokenStream {
        let request_fn_name = tree.node(self.upstream_clock().unwrap()).request_fn_name();
        quote! {
            #request_fn_name(clocks);
        }
    }

    fn release_direct_dependencies(
        &self,
        _node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData<'_>,
    ) -> TokenStream {
        let release_fn_name = tree.node(self.upstream_clock().unwrap()).release_fn_name();
        quote! {
            #release_fn_name(clocks);
        }
    }
}

impl Divider {
    pub fn upstream_clock(&self) -> Option<&str> {
        self.find_clock_source()
    }

    pub(super) fn find_clock_source(&self) -> Option<&str> {
        let mut result = None;
        self.output.visit_variables(|var| {
            if var != "DIVISOR" {
                if let Some(seen) = result {
                    panic!("A divider cannot combine two clock sources ({seen}, {var})");
                }
                result = Some(var);
            }
        });
        result
    }

    pub(super) fn list_of_fixed_dividers(&self) -> Option<Vec<u32>> {
        self.divisors.as_ref().and_then(|d| d.as_enum_values())
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DividerOutputExpression(Expression);

impl DividerOutputExpression {
    fn visit_variables<'s>(&'s self, f: impl FnMut(&'s str)) {
        self.0.visit_variables(f);
    }

    fn source(&self) -> &str {
        &self.0.source
    }

    fn to_rust(&self, variables: HashMap<&str, TokenStream>) -> TokenStream {
        self.0.to_rust(variables)
    }
}
