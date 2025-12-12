//! Clock multiplexer support.

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, pattern};
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::Deserialize;

use super::Expression;
use crate::cfg::{
    clock_tree::{ClockTreeNodeType, ConfiguresExpression, ValidationContext},
    soc::ProcessedClockData,
};

#[derive(Debug, Clone, Deserialize)]
pub struct Multiplexer {
    /// The unique name of the clock tree item.
    name: String,

    #[serde(default)]
    always_on: bool,

    // reject: Option<RejectExpression>,
    #[serde(default)]
    pub default: Option<String>,
    pub variants: Vec<MultiplexerVariant>,
}

impl ClockTreeNodeType for Multiplexer {
    fn name_str<'a>(&'a self) -> &'a String {
        &self.name
    }

    fn always_on(&self) -> bool {
        self.always_on
    }

    fn input_clocks(&self) -> Vec<String> {
        self.upstream_clocks().map(ToString::to_string).collect()
    }

    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        let mut default_exists = false;
        for variant in &self.variants {
            default_exists |= Some(variant.name.as_str()) == self.default.as_deref();
            variant.validate_source_data(ctx).with_context(|| {
                format!("Multiplexer option {} has incorrect data", variant.name)
            })?;
        }
        anyhow::ensure!(
            self.default.is_none() || default_exists,
            "Multiplexer default option {} not found",
            self.default.as_ref().unwrap()
        );
        Ok(())
    }

    fn is_configurable(&self) -> bool {
        self.upstream_clocks().count() > 1
    }

    fn apply_configuration(&self, expr: &Expression, _tree: &ProcessedClockData) -> TokenStream {
        let config_function = self.config_apply_function_name();

        let enum_name = self.config_type_name().unwrap();

        let configured_name = expr.as_name().unwrap();
        let variant = self
            .variants
            .iter()
            .find(|v| v.name == configured_name)
            .unwrap_or_else(|| panic!("Multiplexer option {configured_name} not found"));

        let variant = variant.config_enum_variant_name();

        quote! {
            #config_function(clocks, #enum_name::#variant);
        }
    }

    fn config_apply_function(&self, tree: &ProcessedClockData) -> TokenStream {
        self.impl_config_apply_function(self, tree)
    }

    fn config_apply_impl_function(&self, _tree: &ProcessedClockData) -> TokenStream {
        let ty_name = self.config_type_name();
        let apply_fn_name = self.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);

        quote! {
            fn #hal_impl(_clocks: &mut ClockTree, _old_selector: Option<#ty_name>, _new_selector: #ty_name) {
                todo!()
            }
        }
    }

    fn node_frequency_impl(&self, tree: &ProcessedClockData) -> TokenStream {
        self.node_frequency_impl2(self, tree)
    }

    fn config_docline(&self) -> Option<String> {
        let clock_name = self.name.as_str();
        Some(format!(
            " The list of clock signals that the `{clock_name}` multiplexer can output."
        ))
    }

    fn config_type(&self) -> Option<TokenStream> {
        let ty_name = self.config_type_name()?;

        Some(self.impl_config_type(ty_name))
    }

    fn affected_nodes<'s>(&'s self) -> Vec<&'s str> {
        self.configures().collect()
    }

    fn request_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state_field = tree.properties(node).field_name();
        self.impl_request_upstream(node, tree, quote! { unwrap!(clocks.#state_field) })
    }

    fn release_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state_field = tree.properties(node).field_name();
        self.impl_release_upstream(node, tree, quote! { unwrap!(clocks.#state_field) })
    }
}

impl Multiplexer {
    pub fn upstream_clocks(&self) -> impl Iterator<Item = &str> {
        self.variants.iter().map(|v| v.outputs.as_str())
    }

    pub fn variant_names(&self) -> impl Iterator<Item = &str> {
        self.variants.iter().map(|v| v.name.as_str())
    }

    pub fn configures(&self) -> impl Iterator<Item = &str> {
        self.variants
            .iter()
            .flat_map(|v| v.configures.iter().map(|c| c.target.as_str()))
    }

    pub fn impl_config_type(&self, ty_name: Ident) -> TokenStream {
        let variants = self.variants.iter().map(|v| {
            let variant = v.config_enum_variant();
            if Some(v.name.as_str()) == self.default.as_deref() {
                quote! {
                    #[default]
                    #variant
                }
            } else {
                variant
            }
        });

        let default = self.default.as_ref().map(|_| quote! { Default, });

        quote! {
            #[derive(Debug, #default Clone, Copy, PartialEq, Eq, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub enum #ty_name {
                #(#variants)*
            }
        }
    }

    pub fn impl_config_apply_function(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let ty_name = node.config_type_name();
        let apply_fn_name = node.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);
        let state = tree.properties(node).field_name();
        let refcount_field = tree.properties(node).refcount_field_name();

        let request_upstream = self.impl_request_upstream(node, tree, quote! { new_selector });
        let release_upstream = self.impl_release_upstream(node, tree, quote! { old_selector });

        let cfgs = self
            .variants
            .iter()
            .filter_map(|variant| {
                if variant.configures.is_empty() {
                    return None;
                }

                let mut variant_configures = quote! {};

                for cfg_expr in variant.configures.iter() {
                    let affected_node = tree.node(&cfg_expr.target);
                    let config_expr = affected_node.apply_configuration(&cfg_expr.value, tree);

                    variant_configures = quote! {
                        #variant_configures
                        #config_expr
                    };
                }

                let name = variant.config_enum_variant_name();

                Some(quote! {
                    #ty_name::#name => {
                        #variant_configures
                    }
                })
            })
            .collect::<Vec<_>>();

        let configures = if cfgs.is_empty() {
            quote! {}
        } else {
            quote! {
                match new_selector {
                    #(#cfgs,)*
                }
            }
        };

        let apply_and_switch_input = quote! {
            #request_upstream
            #hal_impl(clocks, old_selector, new_selector);
            if let Some(old_selector) = old_selector {
                #release_upstream
            }
        };
        let apply_impl = if refcount_field.is_some() {
            quote! {
                if clocks.#refcount_field > 0 {
                    #apply_and_switch_input
                } else {
                    #hal_impl(clocks, old_selector, new_selector);
                }
            }
        } else {
            apply_and_switch_input
        };

        quote! {
            pub fn #apply_fn_name(clocks: &mut ClockTree, new_selector: #ty_name) {
                let old_selector = clocks.#state.replace(new_selector);

                #configures

                #apply_impl
            }
        }
    }

    // Allows reusing the same code for peripheral_source nodes
    pub fn node_frequency_impl2(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let ty_name = node.config_type_name();
        let state = tree.properties(node).field_name();
        let variants = self
            .variants
            .iter()
            .map(|variant| {
                let name = variant.config_enum_variant_name();

                quote! { #ty_name::#name }
            })
            .collect::<Vec<_>>();

        let variant_frequencies = self
            .variants
            .iter()
            .map(|variant| {
                let frequency_fn = tree.node(&variant.outputs).frequency_function_name();

                quote! { #frequency_fn(clocks) }
            })
            .collect::<Vec<_>>();

        if variant_frequencies.len() > 1 {
            quote! {
                match unwrap!(clocks.#state) {
                    #(#variants => #variant_frequencies,)*
                }
            }
        } else {
            let variant_frequency = variant_frequencies.first().unwrap();
            quote! {
                #variant_frequency
            }
        }
    }

    fn impl_request_upstream(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        if self.variants.len() > 1 {
            let ty_name = node.config_type_name().unwrap();
            let request_upstream_branches = self.variants.iter().map(|variant| {
                let match_arm = variant.config_enum_variant_name();
                let function = tree.node(&variant.outputs).request_fn_name();
                quote! {
                    #ty_name::#match_arm => #function(clocks)
                }
            });

            quote! {
                match #config_var {
                    #(#request_upstream_branches,)*
                }
            }
        } else {
            let function = self
                .variants
                .first()
                .map(|variant| tree.node(&variant.outputs).request_fn_name())
                .into_iter();

            quote! {
                #(#function(clocks);)*
            }
        }
    }

    fn impl_release_upstream(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        if self.variants.len() > 1 {
            let ty_name = node.config_type_name().unwrap();
            let release_upstream_branches = self.variants.iter().map(|variant| {
                let match_arm = variant.config_enum_variant_name();
                let function = tree.node(&variant.outputs).release_fn_name();
                quote! {
                    #ty_name::#match_arm => #function(clocks)
                }
            });
            quote! {
                match #config_var {
                    #(#release_upstream_branches,)*
                }
            }
        } else {
            let function = self
                .variants
                .first()
                .map(|variant| tree.node(&variant.outputs).release_fn_name())
                .into_iter();

            quote! {
                #(#function(clocks);)*
            }
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct MultiplexerVariant {
    name: String,
    pub outputs: String,
    #[serde(default, deserialize_with = "super::list_from_str")]
    configures: Vec<ConfiguresExpression>,
}
impl MultiplexerVariant {
    pub fn config_enum_variant_name(&self) -> Ident {
        format_ident!(
            "{}",
            self.name
                .from_case(Case::Custom {
                    boundaries: &[Boundary::LOWER_UPPER, Boundary::UNDERSCORE],
                    pattern: pattern::capital,
                    delim: "",
                })
                .to_case(Case::Pascal)
        )
    }

    pub fn config_enum_variant(&self) -> TokenStream {
        let docline = format!(" Selects `{}`.", self.outputs);
        let name = self.config_enum_variant_name();

        quote! {
            #[doc = #docline]
            #name,
        }
    }

    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        anyhow::ensure!(
            ctx.has_clock(&self.outputs),
            "Clock source {} not found",
            self.outputs
        );

        for (index, config) in self.configures.iter().enumerate() {
            config
                .validate_source_data(ctx)
                .with_context(|| format!("Incorrect `configures` expression at index {index}"))?;
        }

        Ok(())
    }
}
