//! Represents a generic clock node.

use std::{
    collections::{HashMap, HashSet},
    fmt,
    str::FromStr,
};

use anyhow::{Context, Result};
use convert_case::{Case, Casing};
use indexmap::IndexMap;
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::{self, Deserialize, de};
use somni_parser::ast;

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
            expr_compiler::ExprCompiler,
            mux::MultiplexerVariant,
        },
        soc::ProcessedClockData,
    },
    number,
};

/// Configurable parameter kinds.
#[derive(Debug, Clone)]
enum NodeParameter {
    /// A number that will be substituted into the expression. Usually a configurable divider value.
    Value(ValuesExpression),

    /// A configurable source clock, representing a clock multiplexer.
    Source(Vec<MultiplexerVariant>),
}

impl<'de> Deserialize<'de> for NodeParameter {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor;

        impl<'de> de::Visitor<'de> for Visitor {
            type Value = NodeParameter;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("divider spec or list of mux inputs")
            }

            fn visit_str<E>(self, value: &str) -> Result<NodeParameter, E>
            where
                E: de::Error,
            {
                ValuesExpression::from_str(value)
                    .map(NodeParameter::Value)
                    .map_err(|err| E::custom(format!("string is not a valid divider spec: {err}")))
            }

            fn visit_seq<S>(self, seq: S) -> std::result::Result<NodeParameter, S::Error>
            where
                S: de::SeqAccess<'de>,
            {
                Vec::deserialize(de::value::SeqAccessDeserializer::new(seq))
                    .map(NodeParameter::Source)
            }
        }

        deserializer.deserialize_any(Visitor)
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Generic {
    /// The unique name of the clock tree item.
    pub name: String,

    #[serde(default)]
    always_on: bool,

    /// The expression that calculates the clock node's output frequency.
    output: Expression,

    /// If set, this expression will be used to validate the clock configuration.
    ///
    /// The expression may refer to clock sources, or any of the clock tree item's properties (e.g.
    /// `divisor`).
    #[serde(default)]
    reject: Option<RejectExpression>,

    /// Variables in the output expression.
    #[serde(default)]
    params: IndexMap<String, NodeParameter>,
}

impl ClockTreeNodeType for Generic {
    fn input_clocks(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Vec<String> {
        self.upstream_clocks()
            .to_names()
            .into_iter()
            .map(|c| instance.resolve_node(tree, &c).name_str().clone())
            .collect()
    }

    fn always_on(&self) -> bool {
        self.always_on
    }

    fn validate_source_data(
        &self,
        instance: &ClockTreeNodeInstance,
        ctx: &ValidationContext<'_>,
    ) -> Result<()> {
        let mut result = None;
        let mut seen = HashSet::new();
        let mut has_source = false;

        self.output.visit_variables(|v| {
            match self.params.get(v) {
                Some(NodeParameter::Value(_)) => return,
                Some(NodeParameter::Source(variants)) => {
                    if has_source {
                        result = Some(Err(anyhow::anyhow!(
                            "Multiple clock sources are not supported"
                        )));
                        return;
                    }
                    has_source = true;
                    if variants.is_empty() && matches!(result, None | Some(Ok(()))) {
                        result = Some(Err(anyhow::anyhow!(
                            "{v} represents a multiplexer with no inputs"
                        )));
                        return;
                    }
                }
                _ => {
                    if !ctx.has_clock(instance, v) && matches!(result, None | Some(Ok(()))) {
                        result = Some(Err(anyhow::anyhow!("{v} is not a valid clock signal name")));
                    }
                }
            }

            if seen.insert(v) {
                if result.is_none() {
                    result = Some(Ok(()));
                } else if matches!(result, Some(Ok(()))) {
                    result = Some(Err(anyhow::anyhow!(
                        "Clock nodes cannot have more than one source clock"
                    )));
                }
            }
        });

        let mut default = None;
        for (name, param) in &self.params {
            let NodeParameter::Source(variants) = param else {
                continue;
            };

            for variant in variants.iter() {
                if variant.default {
                    if default.is_some() {
                        anyhow::bail!(
                            "Node parameter {}/{} has multiple default options",
                            self.name,
                            name
                        );
                    }
                    default = Some(variant.name.as_str());
                }
                variant
                    .validate_source_data(instance, ctx)
                    .with_context(|| {
                        format!(
                            "Node option {}/{}/{} has incorrect data",
                            self.name, name, variant.name
                        )
                    })?;
            }
        }

        result.unwrap_or_else(|| Err(anyhow::anyhow!("Clock node has no source clock")))
    }

    fn is_configurable(&self) -> bool {
        let mut contains_param = false;
        self.output.visit_variables(|var| {
            contains_param |= self.params.contains_key(var);
        });
        contains_param
    }

    fn config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let num_inputs = self.upstream_clocks().len();

        let ty_name = instance.config_type_name();
        let instance_properties = tree.properties(instance.name_str());
        let apply_fn_name = instance.config_apply_function_name();
        let hal_impl = format_ident!("{apply_fn_name}_impl");

        // TODO: support reject exprs - implement `value(node, property)` in expressions

        let func_body = if num_inputs == 1 {
            quote! {
                #hal_impl(clocks, old_config, config);
            }
        } else {
            let request_upstream = self.impl_request_upstream(instance, tree, quote! { config });
            let release_upstream =
                self.impl_release_upstream(instance, tree, quote! { old_config });

            // Selecting a mux input may affect other nodes' configurations.
            let configures = if let Some((mux_param, NodeParameter::Source(mux_inputs))) = self
                .params
                .iter()
                .find(|(_, param)| matches!(param, NodeParameter::Source(_)))
                && mux_inputs
                    .iter()
                    .any(|variant| !variant.configures.is_empty())
            {
                let mux_param_field = format_ident!("{mux_param}");
                let param_ty_name = self.param_type_name(instance, mux_param);
                let branches = mux_inputs.iter().filter_map(|variant| {
                    let name = variant.config_enum_variant_name();
                    let variant_configures = variant.configures.iter().map(|cfg_expr| {
                        let affected_node = instance.resolve_node(tree, &cfg_expr.effect().node);
                        affected_node.apply_configuration(cfg_expr, tree)
                    });

                    Some(quote! {
                        #param_ty_name::#name => {
                            #(#variant_configures)*
                        }
                    })
                });

                quote! {
                    match config.#mux_param_field {
                        #(#branches,)*
                    }
                }
            } else {
                quote! {}
            };

            let refcount_field = instance_properties.refcount_field_name();
            if refcount_field.is_some() {
                quote! {
                    #configures
                    if clocks.#refcount_field > 0 {
                        #request_upstream
                        #hal_impl(clocks, old_config, config);
                        if let Some(old_config) = old_config {
                            #release_upstream
                        }
                    } else {
                        #hal_impl(clocks, old_config, config);
                    }
                }
            } else {
                quote! {
                    #configures
                    #request_upstream
                    #hal_impl(clocks, old_config, config);
                    if let Some(old_config) = old_config {
                        #release_upstream
                    }
                }
            }
        };

        let reject_exprs = self.reject.as_ref().map(|reject| {
            let mut variables = HashMap::new();

            for var in self.params.keys() {
                let param_fn = format_ident!("{}", var);
                variables.insert(var.as_str(), quote! { config.#param_fn() });
            }

            reject.to_rust(variables, instance, tree)
        });

        let state = instance_properties.field_name();
        quote! {
            pub fn #apply_fn_name(clocks: &mut ClockTree, config: #ty_name) {
                #reject_exprs
                let old_config = clocks.#state.replace(config);

                #func_body
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
            "Clock node `{}` is not configurable",
            instance.name_str()
        );
        let clock_name = instance.name_str();

        let (param, v) = if let Some(property) = expr.effect().property.as_ref() {
            self.params.get_key_value(property).ok_or_else(|| {
                anyhow::anyhow!("Clock node does not have property `{}`", property)
            })?
        } else {
            anyhow::ensure!(
                self.params.len() == 1,
                "Configuring nodes with multiple parameters requires specifying a property",
            );
            self.params.get_index(0).unwrap()
        };

        if let NodeParameter::Source(variants) = v {
            let ast::RightHandExpression::Variable { variable } = &expr.effect().value else {
                anyhow::bail!("Multiplexer parameters can only be changed to names");
            };

            let name = expr.name(*variable);
            if !variants.iter().any(|v| v.name == name) {
                anyhow::bail!("Multiplexer `{clock_name}/{param}` does not have variant `{name}`");
            }
        }

        Ok(())
    }

    fn apply_configuration(
        &self,
        instance: &ClockTreeNodeInstance,
        expr: &ConfiguresExpression,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let effect = expr.effect();

        let (param_name, v) = if let Some(property) = effect.property.as_ref() {
            (property, &self.params[property])
        } else {
            if self.params.len() != 1 {
                panic!(
                    "multi-parameter generic nodes must specify a property in configures expression"
                );
            }
            self.params.get_index(0).unwrap()
        };

        let cfg_value =
            match v {
                NodeParameter::Value(values) => {
                    let mut variables = HashMap::new();
                    for clock in tree.clock_tree.iter() {
                        let clock_name = clock.name_str().as_str();
                        let frequency_fn = clock.frequency_function_name();
                        variables.insert(clock_name, quote! { #frequency_fn(clocks) });
                    }

                    let cfg_expr_code = ExprCompiler::new(&variables)
                        .compile_right_hand_expression(&expr.source, &effect.value, instance, tree);

                    if values.as_enum_values().is_some() {
                        let ty_name = self.param_type_name(instance, param_name);
                        quote! {
                            #ty_name::new(#cfg_expr_code)
                        }
                    } else {
                        cfg_expr_code
                    }
                }
                NodeParameter::Source(variants) => {
                    let ast::RightHandExpression::Variable { variable } = &effect.value else {
                        unreachable!()
                    };

                    let enum_name = self.param_type_name(instance, param_name);

                    let configured_name = expr.name(*variable);
                    let variant = variants
                        .iter()
                        .find(|variant| variant.name == configured_name)
                        .unwrap()
                        .config_enum_variant_name();

                    quote! {
                        #enum_name::#variant
                    }
                }
            };

        let node_name = instance.name_str();
        let config_function = instance.config_apply_function_name();
        let state = instance.config_type_name();
        if let Some(property) = effect.property.as_ref() {
            let read_config_function = instance.current_config_function_name();
            let property = format_ident!("{}", property);
            quote! {
                let mut config_value = unwrap!(
                    #read_config_function(clocks),
                    concat!("Attempted to change ", stringify!(#property), " on ", #node_name, " which has not yet been configured."),
                );

                config_value.#property = #cfg_value;

                #config_function(clocks, config_value);
            }
        } else {
            quote! {
                let config_value = #state::new(#cfg_value);
                #config_function(clocks, config_value);
            }
        }
    }

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let mut variables = HashMap::new();

        // Source clock
        let source_param_name = self.clock_source_parameter();
        let source_frequency_tokens = match self.upstream_clocks() {
            ClockSource::Fixed(input) => {
                let source_node = instance.resolve_node(tree, &input);
                let freq_fn = source_node.frequency_function_name();
                quote! { #freq_fn(clocks) }
            }
            ClockSource::Mux(input) if input.len() == 1 => {
                let source_node = instance.resolve_node(tree, &input[0].outputs);
                let freq_fn = source_node.frequency_function_name();
                quote! { #freq_fn(clocks) }
            }
            ClockSource::Mux(inputs) => {
                let ty_name = self.param_type_name(instance, source_param_name);
                let source_param_name = format_ident!("{source_param_name}");

                let (variants, variant_frequencies) = inputs
                    .iter()
                    .map(|variant| {
                        let name = variant.config_enum_variant_name();
                        let source_node = instance.resolve_node(tree, &variant.outputs);
                        let frequency_fn = source_node.frequency_function_name();

                        (quote! { #ty_name::#name }, quote! { #frequency_fn(clocks) })
                    })
                    .unzip::<_, _, Vec<_>, Vec<_>>();

                quote! {
                    match config.#source_param_name {
                        #(#variants => #variant_frequencies,)*
                    }
                }
            }
        };
        variables.insert(source_param_name, source_frequency_tokens);

        // Numeric parameters
        variables.extend(self.params.iter().flat_map(|(var, p)| {
            if let NodeParameter::Value(_) = p {
                let param_fn = format_ident!("{var}");
                Some((var.as_str(), quote! { config.#param_fn() }))
            } else {
                None
            }
        }));

        self.output.to_rust(variables, instance, tree)
    }

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream {
        let clock_name = instance.name_str();
        let ty_name = instance.config_type_name();

        let enum_types = self
            .params
            .keys()
            .filter_map(|param_name| self.parameter_config_type_impl(instance, param_name))
            .collect::<Vec<_>>();

        let (field_types, field_names) = self
            .params
            .keys()
            .map(|param| {
                (
                    self.param_type_name(instance, param),
                    format_ident!("{param}"),
                )
            })
            .unzip::<_, _, Vec<_>, Vec<_>>();

        let mut panic_docs = vec![];
        let param_asserts = self
            .params
            .iter()
            .filter_map(|(param_name, parameter)| {
                self.param_validator_assert(instance, param_name, parameter, &mut panic_docs)
            })
            .collect::<Vec<_>>();

        let expr = &self.output.source;
        let docs = format!(
            r" Configures the `{clock_name}` clock node.

 The output is calculated as `OUTPUT = {expr}`."
        );
        let docs = docs.lines();

        let param_accessors = self.params.iter().map(|(param_name, param)| {
            let param_ty = self.param_type_name(instance, param_name);
            let param_name = format_ident!("{param_name}");

            match param {
                NodeParameter::Value(_) => {
                    quote! {
                        fn #param_name(self) -> u32 {
                            self.#param_name as u32
                        }
                    }
                }
                NodeParameter::Source(_) => {
                    quote! {
                        fn #param_name(self) -> #param_ty {
                            self.#param_name
                        }
                    }
                }
            }
        });

        // Transform panics docs into individual doc lines and prepend heading
        let mut constructor_docs = vec![];
        constructor_docs.push(format!(
            "Creates a new configuration for the {} clock node.",
            self.name
        ));
        if !panic_docs.is_empty() {
            constructor_docs.push(String::new());
            constructor_docs.push(String::from("## Panics"));

            for doc in panic_docs.iter() {
                constructor_docs.extend(doc.lines().map(String::from));
            }
        }

        quote! {
            #(#enum_types)*

            #(#[doc = #docs])*
            #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct #ty_name {
                #(#field_names: #field_types),*
            }

            impl #ty_name {
                #(#[doc = #constructor_docs])*
                pub const fn new(#(#field_names: #field_types),*) -> Self {
                    #(#param_asserts)*

                    Self {
                        #(#field_names),*
                    }
                }

                #(#param_accessors)*
            }
        }
    }

    fn affected_nodes(&self) -> Vec<&str> {
        self.params
            .iter()
            .filter_map(|(_, v)| {
                if let NodeParameter::Source(v) = v {
                    Some(v) // For each mux param (expect at most 1)
                } else {
                    None
                }
            })
            .flat_map(|v| {
                // List the targets of `configures` expressions
                v.iter()
                    .flat_map(|v| v.configures.iter())
                    .map(|c| c.effect().node.as_str())
            })
            .collect()
    }

    fn request_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state_field = tree.properties(instance.name_str()).field_name();
        self.impl_request_upstream(instance, tree, quote! { unwrap!(clocks.#state_field) })
    }

    fn release_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let state_field = tree.properties(instance.name_str()).field_name();
        self.impl_release_upstream(instance, tree, quote! { unwrap!(clocks.#state_field) })
    }
}

enum ClockSource<'d> {
    Fixed(&'d str),
    Mux(&'d [MultiplexerVariant]),
}

#[allow(clippy::len_without_is_empty, reason = "Cannot be empty")]
impl ClockSource<'_> {
    fn to_names(&self) -> Vec<String> {
        match self {
            ClockSource::Fixed(name) => vec![name.to_string()],
            ClockSource::Mux(variants) => variants.iter().map(|v| v.outputs.clone()).collect(),
        }
    }

    fn len(&self) -> usize {
        match self {
            ClockSource::Fixed(_) => 1,
            ClockSource::Mux(variants) => variants.len(),
        }
    }
}

impl Generic {
    fn upstream_clocks(&self) -> ClockSource<'_> {
        let input_name = self.clock_source_parameter();

        // Either our source clock is a configurable parameter, or the expression references a node.
        match self.params.get(input_name) {
            Some(NodeParameter::Source(mux_inputs)) => ClockSource::Mux(mux_inputs),
            Some(_) => unreachable!(),
            None => ClockSource::Fixed(input_name),
        }
    }

    fn clock_source_parameter(&self) -> &str {
        let mut upstream = None;
        self.output.visit_variables(|var| {
            if matches!(self.params.get(var), None | Some(NodeParameter::Source(_))) {
                upstream = Some(var);
            }
        });

        // We've validated that there is exactly one clock source.
        upstream.unwrap()
    }

    fn impl_source_operation(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
        config_var: TokenStream,
        op: impl Fn(&ClockTreeNodeInstance) -> Ident,
    ) -> TokenStream {
        let single_source = match self.upstream_clocks() {
            ClockSource::Fixed(name) => name,
            ClockSource::Mux(mux) if mux.len() == 1 => &mux.first().unwrap().outputs,
            ClockSource::Mux(mux_inputs) => {
                // Multiple options, generate a match expression
                let param = self.clock_source_parameter();
                let ty_name = self.param_type_name(instance, param);
                let request_upstream_branches = mux_inputs.iter().map(|variant| {
                    let match_arm = variant.config_enum_variant_name();
                    let function = op(instance.resolve_node(tree, &variant.outputs));
                    quote! {
                        #ty_name::#match_arm => #function(clocks)
                    }
                });

                let param = format_ident!("{param}");
                return quote! {
                    match #config_var.#param {
                        #(#request_upstream_branches,)*
                    }
                };
            }
        };

        // Single option, generate a function call
        let function = op(instance.resolve_node(tree, single_source));
        quote! {
            #function(clocks);
        }
    }

    fn impl_request_upstream(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        self.impl_source_operation(instance, tree, config_var, |node| node.request_fn_name())
    }

    fn impl_release_upstream(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
        config_var: TokenStream,
    ) -> TokenStream {
        self.impl_source_operation(instance, tree, config_var, |node| node.release_fn_name())
    }

    fn param_type_name(&self, instance: &ClockTreeNodeInstance, param_name: &str) -> Ident {
        // Numeric parameter
        if let NodeParameter::Value(ve) = &self.params[param_name]
            && ve.as_range().is_some()
        {
            // TODO: use smallest type that fits the range
            return format_ident!("u32");
        }

        // Enum parameter
        let enum_name_prefix = instance
            .template_name
            .from_case(Case::Constant)
            .to_case(Case::Pascal);
        let enum_param_name = param_name.from_case(Case::Snake).to_case(Case::Pascal);
        format_ident!("{enum_name_prefix}{enum_param_name}")
    }

    fn parameter_config_type_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        param_name: &str,
    ) -> Option<TokenStream> {
        let mut derive_default = quote! {};
        let enum_name = self.param_type_name(instance, param_name);
        match &self.params[param_name] {
            NodeParameter::Value(ve) if ve.as_range().is_some() => {
                // Numeric parameter
                return None;
            }
            NodeParameter::Value(ve) => {
                let raw_value = ve
                    .as_enum_values()
                    .unwrap()
                    .into_iter()
                    .map(number)
                    .collect::<Vec<_>>();

                let variant_ident = raw_value
                    .iter()
                    .map(|divisor| format_ident!("_{divisor}"))
                    .collect::<Vec<_>>();

                let variants =
                    raw_value
                        .iter()
                        .zip(variant_ident.iter())
                        .map(|(divisor, variant)| {
                            let value_doc = format!(" Selects `{param_name} = {divisor}`.");
                            let v = number(divisor);
                            quote! {
                                #[doc = #value_doc]
                                #variant = #v,
                            }
                        });

                let unknown_variant_error =
                    format!("Invalid {} {} value", instance.name_str(), param_name);

                Some(quote! {
                    #[derive(Debug, #derive_default Clone, Copy, PartialEq, Eq, Hash)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    pub enum #enum_name {
                        #(#variants)*
                    }

                    impl #enum_name {
                        /// Creates a new parameter value from a raw number.
                        pub const fn new(raw: u32) -> Self {
                            match raw {
                                #(#raw_value => Self::#variant_ident,)*
                                _ => ::core::panic!(#unknown_variant_error),
                            }
                        }
                    }
                })
            }
            NodeParameter::Source(variants) => {
                if variants.iter().any(|v| v.default) {
                    derive_default = quote! { Default, };
                }
                let variants = variants.iter().map(|v| v.config_enum_variant());

                Some(quote! {
                    #[derive(Debug, #derive_default Clone, Copy, PartialEq, Eq, Hash)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    pub enum #enum_name {
                        #(#variants)*
                    }
                })
            }
        }
    }

    fn param_validator_assert(
        &self,
        instance: &ClockTreeNodeInstance,
        param_name: &str,
        parameter: &NodeParameter,
        panic_docs: &mut Vec<String>,
    ) -> Option<TokenStream> {
        let (min, max) = if let NodeParameter::Value(ve) = parameter
            && let Some(range) = ve.as_range()
        {
            // Only numeric parameters are validated
            range
        } else {
            return None;
        };

        panic_docs.push(format!(
            r#"
Panics if the {param_name} value is outside the
valid range ({min} ..= {max})."#
        ));

        let clock_name = instance.name_str();
        let assert_failed =
            format!("`{clock_name}` {param_name} must be between {min} and {max} (inclusive).");

        // Divisor is always unsigned, avoid generating trivial `>= 0`.
        let param_name = format_ident!("{param_name}");
        let max = number(max);

        // TODO: skip if all values of the type are allowed
        Some(if min == 0 {
            quote! { ::core::assert!(#param_name <= #max, #assert_failed); }
        } else {
            let min = number(min);
            quote! { ::core::assert!(#param_name >= #min && #param_name <= #max, #assert_failed); }
        })
    }
}
