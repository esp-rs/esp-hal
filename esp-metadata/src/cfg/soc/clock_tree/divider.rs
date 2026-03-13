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

use std::collections::{HashMap, HashSet};

use anyhow::Result;
use convert_case::{Case, Casing};
use indexmap::IndexMap;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use serde::Deserialize;

use crate::{
    cfg::{
        ClockTreeNodeInstance,
        clock_tree::{
            ClockTreeNodeType,
            ConfiguresExpression,
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
    /// `divisor`).
    #[serde(default)]
    reject: Option<RejectExpression>,

    /// The divider equation. The expression contains which clock is being divided. The expression
    /// may refer to clock sources, and the divider's value via `divisor`.
    output: DividerOutputExpression,

    /// Possible divider parameter values. Elements may be a list of numbers or a range. If empty,
    /// the divider value is fixed.
    #[serde(default)]
    params: IndexMap<String, ValuesExpression>,
}

impl ClockTreeNodeType for Divider {
    fn input_clocks(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Vec<String> {
        vec![
            instance
                .resolve_node(tree, self.source_clock())
                .name_str()
                .to_string(),
        ]
    }

    fn validate_source_data(
        &self,
        instance: &ClockTreeNodeInstance,
        ctx: &ValidationContext<'_>,
    ) -> Result<()> {
        let mut result = None;
        let mut seen = HashSet::new();
        self.output.visit_variables(|v| {
            if self.params.contains_key(v) {
                return;
            }
            if !ctx.has_clock(instance, v) && matches!(result, None | Some(Ok(()))) {
                result = Some(Err(anyhow::format_err!(
                    "{v} is not a valid clock signal name"
                )));
            }

            if seen.insert(v) {
                if result.is_none() {
                    result = Some(Ok(()));
                } else if matches!(result, Some(Ok(()))) {
                    result = Some(Err(anyhow::anyhow!(
                        "Divider nodes cannot have more than one source clock"
                    )));
                }
            }
        });
        result.unwrap_or_else(|| Err(anyhow::anyhow!("Divider node has no source clock")))
    }

    fn is_configurable(&self) -> bool {
        let mut contains_divisor = false;
        self.output.visit_variables(|var| {
            contains_divisor |= self.params.contains_key(var);
        });
        contains_divisor
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

            for var in self.params.keys() {
                let param_fn = format_ident!("{}", var);
                variables.insert(var.as_str(), quote! { config.#param_fn() });
            }

            reject.to_rust(variables, tree)
        });
        quote! {
            pub fn #apply_fn_name(clocks: &mut ClockTree, config: #ty_name) {
                #reject_exprs
                let old_config = clocks.#state.replace(config);
                #hal_impl(clocks, old_config, config);
            }
        }
    }

    fn validate_configures_expr(
        &self,
        instance: &ClockTreeNodeInstance,
        expr: &ConfiguresExpression,
    ) -> Result<()> {
        anyhow::ensure!(
            self.is_configurable(),
            "Divider `{}` is not configurable",
            instance.name_str()
        );
        if let Some(property) = expr.effect().property.as_ref() {
            anyhow::ensure!(
                self.params.contains_key(property),
                "Divider does not have property `{}`",
                property
            );
        }

        Ok(())
    }

    fn apply_configuration(
        &self,
        instance: &ClockTreeNodeInstance,
        expr: &ConfiguresExpression,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        expr.to_numeric_setter(instance, tree)
    }

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state = tree.properties(instance.name_str()).field_name();
        let parent_clock = self.source_clock(); // TODO get from node?
        let parent_frequency_fn = instance
            .resolve_node(tree, parent_clock)
            .frequency_function_name();

        let params = self.params.keys().map(|var| {
            let param_fn = format_ident!("{}", var);
            (var, quote! { unwrap!(clocks.#state).#param_fn() })
        });

        let mut variables = HashMap::new();
        variables.insert(parent_clock, quote! { #parent_frequency_fn(clocks) });
        for (var, param_value_accessor) in params {
            variables.insert(var.as_str(), param_value_accessor);
        }
        let cfg_expr_code = self.output.to_rust(variables, tree);

        cfg_expr_code
    }

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream {
        let clock_name = instance.name_str();
        let ty_name = instance.config_type_name();

        let mut enum_types = vec![];
        let mut field_names = vec![];
        let mut field_types = vec![];
        let mut panic_docs = vec![];
        let mut validators = vec![];
        let mut param_values: Vec<Box<dyn Fn(TokenStream) -> TokenStream>> = vec![];

        let collapse_types =
            self.params.len() == 1 && self.params.values().all(|v| v.as_enum_values().is_some());

        let expr = &self.output.source();
        let docline = format!(
            r#" Configures the `{clock_name}` clock divider.

 The output is calculated as `OUTPUT = {expr}`."#
        );
        let mut doclines = docline.lines().map(|l| quote! { #[doc = #l] });

        // Create a new type for each enum parameter.
        // Define a field and accessor body for each parameter.
        // Generate an assert for numeric parameters.
        for (param_name, values) in self.params.iter() {
            let param_name_ident = format_ident!("{param_name}");
            if let Some(dividers) = values.as_enum_values() {
                let value_variants = dividers
                    .iter()
                    .map(|d| format_ident!("_{d}"))
                    .collect::<Vec<_>>();
                let value_doc = dividers
                    .iter()
                    .map(|d| format!(" Selects `{param_name} = {d}`."));
                let dividers = dividers.iter().map(number).collect::<Vec<_>>();

                let (enum_ty, doclines) = if collapse_types {
                    (ty_name.clone(), Some(&mut doclines))
                } else {
                    // TODO
                    let full_name = format!("{clock_name}_{}", param_name.to_uppercase());

                    let ty_name = format_ident!(
                        "{}Config",
                        full_name.from_case(Case::Ada).to_case(Case::Pascal)
                    );
                    (ty_name, None)
                };
                let unknown_value = format!("Invalid {clock_name} {param_name} value");
                let doclines = doclines.into_iter().flatten();

                enum_types.push(quote! {
                    #(#doclines)*
                    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    pub enum #enum_ty {
                        #(
                            #[doc = #value_doc]
                            #value_variants = #dividers,
                        )*
                    }

                    impl #enum_ty {
                        /// Creates a new divider configuration.
                        pub const fn new(raw: u32) -> Self {
                            match raw {
                                #(#dividers => #enum_ty::#value_variants,)*
                                _ => ::core::panic!(#unknown_value),
                            }
                        }
                    }
                });

                field_names.push(param_name_ident.clone());
                field_types.push(quote! { #enum_ty });

                let accessor_body = move |field_access: TokenStream| -> TokenStream {
                    quote! {
                        match #field_access {
                            #(#enum_ty::#value_variants => #dividers,)*
                        }
                    }
                };
                param_values.push(Box::new(accessor_body));
            } else {
                let (min, max) = values.as_range().expect("Invalid divisor range");

                panic_docs.push(format!(
                    r#"
Panics if the divisor value is outside the
valid range ({min} ..= {max})."#
                ));
                field_names.push(param_name_ident.clone());
                field_types.push(quote! { u32 });

                let assert_failed = format!(
                    "`{clock_name}` {param_name} value must be between {min} and {max} (inclusive)."
                );
                // Divisor is unsigned, avoid generating `>= 0`.
                validators.push(if min == 0 {
                    quote! { ::core::assert!(#param_name_ident <= #max, #assert_failed); }
                } else {
                    quote! { ::core::assert!(#param_name_ident >= #min && #param_name_ident <= #max, #assert_failed); }
                });

                let accessor_body =
                    move |field_access: TokenStream| -> TokenStream { field_access };
                param_values.push(Box::new(accessor_body));
            }
        }

        // Type declaration for the clock node type. If collapsed, the enum type is used directly.
        let node_type_decl = if collapse_types {
            // The whole divider can be collapsed into a single enum type
            quote! {}
        } else {
            // The divisor is a struct of one or more fields.
            quote! {
                #(#doclines)*
                #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub struct #ty_name {
                    #(#field_names: #field_types),*
                }
            }
        };

        // Constructors for the clock node type. If collapsed, the enum type already defines the
        // constructor.
        let node_ctor = if collapse_types {
            quote! {}
        } else {
            // Transform panics docs into individual doc lines and prepend heading
            let panic_docs = if !panic_docs.is_empty() {
                let mut docs = vec![];
                docs.push("## Panics");

                for doc in panic_docs.iter() {
                    docs.extend(doc.lines());
                }
                docs
            } else {
                vec![]
            };

            quote! {
                /// Creates a new divider configuration.
                #(#[doc = #panic_docs])*
                pub const fn new(#(#field_names: #field_types),*) -> Self {
                    #(#validators)*
                    Self {
                        #(#field_names),*
                    }
                }
            }
        };

        let param_accessors = param_values
            .into_iter()
            .zip(field_names.iter())
            .map(|(accessor, param_name)| {
                let field = if collapse_types {
                    quote! { self }
                } else {
                    quote! { self.#param_name }
                };
                let body = accessor(field);
                quote! { fn #param_name(self) -> u32 {
                    #body
                } }
            })
            .collect::<Vec<_>>();

        let single_accessor = if self.params.len() == 1 {
            let param_name = field_names.first().cloned().unwrap();
            quote! { fn value(self) -> u32 {
                self.#param_name()
            } }
        } else {
            quote! {}
        };

        quote! {
            #(#enum_types)*

            #node_type_decl

            impl #ty_name {
                #node_ctor

                #(#param_accessors)*

                #single_accessor
            }
        }
    }

    fn request_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let request_fn_name = instance
            .resolve_node(tree, self.source_clock())
            .request_fn_name();
        quote! {
            #request_fn_name(clocks);
        }
    }

    fn release_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let release_fn_name = instance
            .resolve_node(tree, self.source_clock())
            .release_fn_name();
        quote! {
            #release_fn_name(clocks);
        }
    }
}

impl Divider {
    fn source_clock(&self) -> &str {
        // We've validated that there is exactly one clock source.
        let mut result = None;
        self.output.visit_variables(|var| {
            if !self.params.contains_key(var) {
                result = Some(var);
            }
        });
        result.expect(
            "Clock divider has no source clock. This should have been prevented in validation.",
        )
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

    fn to_rust(
        &self,
        variables: HashMap<&str, TokenStream>,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.0.to_rust(variables, tree)
    }
}
