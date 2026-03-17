//! Clock multiplexer support.

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, pattern};
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::Deserialize;
use somni_parser::ast;

use crate::cfg::{
    ClockTreeNodeInstance,
    clock_tree::{ClockTreeNodeType, ConfiguresExpression, ValidationContext},
    soc::ProcessedClockData,
};

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Multiplexer {
    /// The unique name of the clock tree item.
    pub name: String,

    #[serde(default)]
    always_on: bool,

    // reject: Option<RejectExpression>,
    pub variants: Vec<MultiplexerVariant>,
}

impl ClockTreeNodeType for Multiplexer {
    fn name(&self) -> &str {
        &self.name
    }

    fn always_on(&self) -> bool {
        self.always_on
    }

    fn input_clocks(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Vec<String> {
        self.upstream_clocks()
            .map(|c| instance.resolve_node(tree, c).name_str().clone())
            .collect()
    }

    fn validate_source_data(
        &self,
        instance: &ClockTreeNodeInstance,
        ctx: &ValidationContext<'_>,
    ) -> Result<()> {
        let mut default = None;
        for variant in &self.variants {
            if variant.default {
                if default.is_some() {
                    anyhow::bail!("Multiplexer {} has multiple default options", self.name);
                }
                default = Some(variant.name.as_str());
            }
            variant
                .validate_source_data(instance, ctx)
                .with_context(|| {
                    format!(
                        "Multiplexer option {}/{} has incorrect data",
                        self.name, variant.name
                    )
                })?;
        }

        Ok(())
    }

    fn is_configurable(&self) -> bool {
        self.upstream_clocks().count() > 1
    }

    fn validate_configures_expr(
        &self,
        instance: &ClockTreeNodeInstance,
        expr: &ConfiguresExpression,
    ) -> Result<()> {
        anyhow::ensure!(
            expr.effect().property.is_none(),
            "Multiplexer config expression for `{}` must not have a property",
            instance.name_str()
        );
        let ast::RightHandExpression::Variable { variable } = &expr.effect().value else {
            anyhow::bail!(
                "Multiplexer config expression for `{}` must be a name",
                instance.name_str()
            );
        };

        let clock_name = instance.name_str();

        let name = expr.name(*variable);
        if !self.variant_names().any(|v| v == name) {
            anyhow::bail!("Multiplexer `{clock_name}` does not have variant `{name}`");
        }

        Ok(())
    }

    fn apply_configuration(
        &self,
        instance: &ClockTreeNodeInstance,
        expr: &ConfiguresExpression,
        _tree: &ProcessedClockData,
    ) -> TokenStream {
        let ast::RightHandExpression::Variable { variable } = &expr.effect().value else {
            unreachable!()
        };

        let config_function = instance.config_apply_function_name();
        let enum_name = instance.config_type_name();

        let configured_name = expr.name(*variable);
        let variant = self
            .variants
            .iter()
            .find(|variant| variant.name == configured_name)
            .unwrap()
            .config_enum_variant_name();

        quote! {
            #config_function(clocks, #enum_name::#variant);
        }
    }

    fn config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.impl_config_apply_function(instance, tree)
    }

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let ty_name = instance.config_type_name();
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
                let frequency_fn = instance
                    .resolve_node(tree, &variant.outputs)
                    .frequency_function_name();

                quote! { #frequency_fn(clocks) }
            })
            .collect::<Vec<_>>();

        if variant_frequencies.len() > 1 {
            quote! {
                match config {
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

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream {
        let clock_name = instance.name_str();
        let ty_name = instance.config_type_name();

        let derive_default = if self.variants.iter().any(|v| v.default) {
            quote! { Default, }
        } else {
            quote! {}
        };
        let variants = self.variants.iter().map(|v| v.config_enum_variant());

        let docline =
            format!("The list of clock signals that the `{clock_name}` multiplexer can output.");
        quote! {
            #[doc = #docline]
            #[derive(Debug, #derive_default Clone, Copy, PartialEq, Eq, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub enum #ty_name {
                #(#variants)*
            }
        }
    }

    fn affected_nodes<'s>(&'s self) -> Vec<&'s str> {
        self.configures().collect()
    }

    fn request_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let config_field = tree.properties(instance.name_str()).config_accessor();
        self.impl_request_upstream(instance, tree, quote! { unwrap!(#config_field) })
    }

    fn release_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let config_field = tree.properties(instance.name_str()).config_accessor();
        self.impl_release_upstream(instance, tree, quote! { unwrap!(#config_field) })
    }
}

impl Multiplexer {
    fn upstream_clocks(&self) -> impl Iterator<Item = &str> {
        self.variants.iter().map(|v| v.outputs.as_str())
    }

    fn variant_names(&self) -> impl Iterator<Item = &str> {
        self.variants.iter().map(|v| v.name.as_str())
    }

    pub fn configures(&self) -> impl Iterator<Item = &str> {
        self.variants
            .iter()
            .flat_map(|v| v.configures.iter().map(|c| c.effect().node.as_str()))
    }

    pub fn impl_config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let ty_name = instance.config_type_name();
        let apply_fn_name = instance.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);
        let config_field = tree.properties(instance.name_str()).config_accessor();
        let refcount_field = tree.properties(instance.name_str()).refcount_field_name();

        let request_upstream = self.impl_request_upstream(instance, tree, quote! { new_selector });
        let release_upstream = self.impl_release_upstream(instance, tree, quote! { old_selector });

        let cfgs = self
            .variants
            .iter()
            .filter_map(|variant| {
                if variant.configures.is_empty() {
                    return None;
                }

                let mut variant_configures = quote! {};

                for cfg_expr in variant.configures.iter() {
                    let affected_node = instance.resolve_node(tree, &cfg_expr.effect().node);
                    let config_expr = affected_node.apply_configuration(&cfg_expr, tree);

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
                let old_selector = #config_field.replace(new_selector);

                #configures

                #apply_impl
            }
        }
    }

    fn impl_request_upstream(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        if self.variants.len() > 1 {
            let ty_name = instance.config_type_name();
            let request_upstream_branches = self.variants.iter().map(|variant| {
                let match_arm = variant.config_enum_variant_name();
                let function = instance
                    .resolve_node(tree, &variant.outputs)
                    .request_fn_name();
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
                .map(|variant| {
                    instance
                        .resolve_node(tree, &variant.outputs)
                        .request_fn_name()
                })
                .into_iter();

            quote! {
                #(#function(clocks);)*
            }
        }
    }

    fn impl_release_upstream(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        if self.variants.len() > 1 {
            let ty_name = instance.config_type_name();
            let release_upstream_branches = self.variants.iter().map(|variant| {
                let match_arm = variant.config_enum_variant_name();
                let function = instance
                    .resolve_node(tree, &variant.outputs)
                    .release_fn_name();
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
                .map(|variant| {
                    instance
                        .resolve_node(tree, &variant.outputs)
                        .release_fn_name()
                })
                .into_iter();

            quote! {
                #(#function(clocks);)*
            }
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct MultiplexerVariant {
    pub name: String,
    pub outputs: String,
    #[serde(default, deserialize_with = "super::list_from_str")]
    pub configures: Vec<ConfiguresExpression>,
    #[serde(default)]
    pub default: bool,
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

        let default = if self.default {
            quote! {
                #[default]
            }
        } else {
            quote! {}
        };

        quote! {
            #default
            #[doc = #docline]
            #name,
        }
    }

    pub fn validate_source_data(
        &self,
        instance: &ClockTreeNodeInstance,
        ctx: &ValidationContext<'_>,
    ) -> Result<()> {
        anyhow::ensure!(
            ctx.has_clock(instance, &self.outputs),
            "Clock source {} not found",
            self.outputs
        );

        for (index, config) in self.configures.iter().enumerate() {
            let Some(clock) = ctx.clock(instance, &config.effect().node) else {
                anyhow::bail!("Clock source {} not found", config.effect().node);
            };

            clock
                .validate_configures_expr(config)
                .with_context(|| format!("Incorrect `configures` expression at index {index}"))?;
        }

        Ok(())
    }
}
