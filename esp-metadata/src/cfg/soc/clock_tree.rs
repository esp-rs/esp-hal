//! Represents the clock tree of an MCU.
//!
//! The clock tree consists of:
//! - Clock sources
//!     - May be configurable or fixed.
//!     - If configurable, the parameter is the desired output frequency.
//! - "Derived" clock sources, which act as clock sources derived from other clock sources.
//! - Clock dividers
//! - Clock muxes
//! - Clock gates
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
//!     - `esp_hal::init` is the only place where system multiplexers are configured. Peripheral
//!       clock sources can freely be configured by the drivers. Perhaps low power is an exception?
//! - A `clock_source_in_use` bitmap
//!     - This is useful for quickly entering/skipping light sleep in auto-lightsleep mode.
//! - request/release functions that update the reference counts and the bitmap
//! - A way to access output frequency values. Ideally, cached.

// TODO: some clock sources don't _really_ need refcount. This needs analysis to see which nodes can
//       be simplified. (Probably the ones that only have one user, or those that are always on).
// TODO: optimize out 1-variant clock multiplexers?
// TODO: ClockConfig doc comment should be tailored to the specific chip
// TODO: finalize public API
// TODO: define clock IDs and maintain active clocks as a bitmap
// TODO: classified clocks should be returned in a topological order
// TODO: ClockConfig should configure clocks in topological order

use std::{any::Any, collections::HashMap, str::FromStr};

use anyhow::Result;
use convert_case::{Case, Casing, StateConverter};
use indexmap::IndexMap;
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::{
    Deserialize,
    Deserializer,
    de::{self, SeqAccess, Visitor},
};
use somni_parser::{ast, lexer::Token, parser::DefaultTypeSet};

use crate::cfg::{
    clock_tree::{
        divider::Divider,
        expr_compiler::ExprCompiler,
        mux::Multiplexer,
        source::{DerivedClockSource, Source},
    },
    soc::ProcessedClockData,
};

mod divider;
mod expr_compiler;
mod mux;
mod peripheral_source;
mod source;

pub(crate) use peripheral_source::PeripheralClockSource;

#[derive(Clone)]
pub(crate) struct Function {
    pub _name: String,
    pub implementation: TokenStream,
}

pub(crate) struct ClockNodeFunctions {
    pub request: Function,
    pub release: Function,
    pub apply_config: Function,

    pub frequency: Function,
    pub hal_functions: Vec<TokenStream>,
}

impl ClockNodeFunctions {
    pub fn implement_functions(&self) -> TokenStream {
        let request_impl = &self.request.implementation;
        let release_impl = &self.release.implementation;
        let apply_impl = &self.apply_config.implementation;
        let frequency_impl = &self.frequency.implementation;

        quote! {
            #apply_impl
            #request_impl
            #release_impl
            #frequency_impl
        }
    }
}

/// Represents the clock input options for a peripheral.
#[derive(Debug, Clone, Deserialize)]
pub enum PeripheralClockTreeEntry {
    /// Defines clock tree items relevant for the current peripheral.
    Definition(Vec<ClockTreeItem>),

    /// References a clock tree defined in another peripheral. This peripheral will inherit the
    /// clock tree from the referenced peripheral.
    Reference(String),
}

impl Default for PeripheralClockTreeEntry {
    fn default() -> Self {
        PeripheralClockTreeEntry::Definition(Vec::new())
    }
}

// Based on https://serde.rs/string-or-struct.html
pub(super) fn ref_or_def<'de, D>(deserializer: D) -> Result<PeripheralClockTreeEntry, D::Error>
where
    D: Deserializer<'de>,
{
    struct PeripheralClockTreeEntryVisitor;

    impl<'de> Visitor<'de> for PeripheralClockTreeEntryVisitor {
        type Value = PeripheralClockTreeEntry;

        fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
            formatter.write_str("string or list")
        }

        fn visit_str<E>(self, value: &str) -> Result<PeripheralClockTreeEntry, E>
        where
            E: de::Error,
        {
            Ok(PeripheralClockTreeEntry::Reference(value.to_string()))
        }

        fn visit_seq<S>(self, list: S) -> Result<PeripheralClockTreeEntry, S::Error>
        where
            S: SeqAccess<'de>,
        {
            // `SeqAccessDeserializer` is a wrapper that turns a `SeqAccess`
            // into a `Deserializer`, allowing it to be used as the input to T's
            // `Deserialize` implementation. T then deserializes itself using
            // the entries from the map visitor.
            Deserialize::deserialize(de::value::SeqAccessDeserializer::new(list))
                .map(PeripheralClockTreeEntry::Definition)
        }
    }

    deserializer.deserialize_any(PeripheralClockTreeEntryVisitor)
}

pub struct ValidationContext<'c> {
    pub tree: &'c [ClockTreeItem],
}

impl ValidationContext<'_> {
    pub fn has_clock(&self, clk: &str) -> bool {
        self.clock(clk).is_some()
    }

    fn clock(&self, clk: &str) -> Option<&ClockTreeItem> {
        self.tree
            .iter()
            .find(|item| item.as_dyn_ref().name_str() == clk)
    }
}

pub(crate) struct DependencyGraph {
    /// Direct upstream -> downstream clock relationships
    graph: IndexMap<String, Vec<String>>,
    /// Direct downstream -> upstream clock relationships
    reverse_graph: IndexMap<String, Vec<String>>,
}

impl DependencyGraph {
    pub fn build_from(clock_tree: &[Box<dyn ClockTreeNodeType>]) -> Self {
        let mut dependency_graph = IndexMap::new();
        let mut reverse_dependency_graph = IndexMap::new();

        for node in clock_tree.iter() {
            for input in node.input_clocks() {
                let node_name = node.name_str();

                let graph_node = dependency_graph
                    .entry(input.clone())
                    .or_insert_with(Vec::new);

                if !graph_node.contains(node_name) {
                    graph_node.push(node_name.clone());
                }
                reverse_dependency_graph
                    .entry(node_name.clone())
                    .or_insert_with(Vec::new)
                    .push(input);
            }
        }

        DependencyGraph {
            graph: dependency_graph,
            reverse_graph: reverse_dependency_graph,
        }
    }

    pub fn inputs(&self, clk: &str) -> &[String] {
        self.reverse_graph
            .get(clk)
            .map(|v| v.as_slice())
            .unwrap_or_default()
    }

    pub fn users(&self, clk: &str) -> &[String] {
        self.graph
            .get(clk)
            .map(|v| v.as_slice())
            .unwrap_or_default()
    }
}

pub(crate) struct ManagementProperties {
    pub name: Ident,
    pub state_ty: Option<Ident>,
    pub refcounted: bool,
    pub has_enable: bool,

    /// This clock node is considered always running.
    pub always_on: bool,
}

impl ManagementProperties {
    pub fn field_name(&self) -> Ident {
        self.name.clone()
    }

    pub fn type_name(&self) -> Option<Ident> {
        self.state_ty.clone()
    }

    pub fn refcount_field_name(&self) -> Option<Ident> {
        if self.refcounted {
            Some(format_ident!("{}_refcount", self.name))
        } else {
            None
        }
    }

    pub fn has_enable(&self) -> bool {
        self.has_enable
    }

    pub fn always_on(&self) -> bool {
        self.always_on
    }
}

pub(crate) trait ClockTreeNodeType: Any {
    /// Returns which clock nodes' configurations are affected when this node is configured.
    fn affected_nodes<'s>(&'s self) -> Vec<&'s str> {
        vec![]
    }

    fn input_clocks(&self) -> Vec<String> {
        vec![]
    }

    fn always_on(&self) -> bool {
        false
    }
    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()>;
    fn is_configurable(&self) -> bool;
    fn config_apply_function(&self, tree: &ProcessedClockData) -> TokenStream;
    fn config_apply_impl_function(&self, _tree: &ProcessedClockData) -> TokenStream {
        quote! {}
    }

    fn node_frequency_impl(&self, _tree: &ProcessedClockData) -> TokenStream;

    fn name_str<'a>(&'a self) -> &'a String;
    fn name<'a>(&'a self) -> StateConverter<'a, String> {
        self.name_str().from_case(Case::Ada)
    }

    /// Returns the name of the clock configuration type. The corresponding field in the
    /// `ClockConfig` struct will have this type.
    fn config_type_name(&self) -> Option<Ident> {
        if self.is_configurable() {
            let item = self.name().to_case(Case::Pascal);
            Some(quote::format_ident!("{}Config", item))
        } else {
            None
        }
    }

    /// Returns the documentation for the clock configuration, which will be placed on the
    /// `ClockConfig` field.
    fn config_documentation(&self) -> Option<String> {
        Some(format!(" `{}` configuration.", self.name_str()))
    }

    fn apply_configuration(&self, _expr: &Expression, _tree: &ProcessedClockData) -> TokenStream {
        if self.is_configurable() {
            unimplemented!();
        } else {
            quote! {}
        }
    }

    fn config_type(&self) -> Option<TokenStream>;
    fn config_docline(&self) -> Option<String>;

    fn config_apply_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("configure_{}", name)
    }

    fn frequency_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("{}_frequency", name)
    }

    fn request_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("request_{}", name)
    }

    fn release_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("release_{}", name)
    }

    fn enable_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("enable_{}", name)
    }

    fn request_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream;
    fn release_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream;
}

/// Represents a clock tree item.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
pub enum ClockTreeItem {
    #[serde(rename = "mux")]
    Multiplexer(Multiplexer),

    #[serde(rename = "source")]
    Source(Source),

    #[serde(rename = "divider")]
    Divider(Divider),

    #[serde(rename = "derived")]
    Derived(DerivedClockSource),
}

impl ClockTreeItem {
    pub(crate) fn as_dyn_ref(&self) -> &dyn ClockTreeNodeType {
        match self {
            ClockTreeItem::Multiplexer(mux) => mux,
            ClockTreeItem::Source(src) => src,
            ClockTreeItem::Divider(div) => div,
            ClockTreeItem::Derived(drv) => drv,
        }
    }

    pub(crate) fn node_functions(
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> ClockNodeFunctions {
        let ty_name = node.config_type_name();

        let request_fn_name = node.request_fn_name();
        let release_fn_name = node.release_fn_name();
        let properties = tree.properties(node);
        let refcount_name = properties.refcount_field_name();
        let enable_fn_name = node.enable_fn_name();
        let enable_fn_impl_name = format_ident!("{}_impl", enable_fn_name);
        let always_on = properties.always_on();

        let request_direct_dependencies = node.request_direct_dependencies(node, tree);
        let release_direct_dependencies = node.release_direct_dependencies(node, tree);

        // Only configurables have an apply fn
        let apply_fn = ty_name.as_ref().map(|_| node.config_apply_function(tree));
        let apply_fn_impl = ty_name
            .as_ref()
            .map(|_| node.config_apply_impl_function(tree))
            .unwrap_or_default();
        let frequency_function_impl = node.node_frequency_impl(tree);
        let frequency_function_name = node.frequency_function_name();

        ClockNodeFunctions {
            request: Function {
                _name: request_fn_name.to_string(),
                implementation: if always_on {
                    quote! {
                        fn #request_fn_name(_clocks: &mut ClockTree) { }
                    }
                } else if refcount_name.is_some() {
                    quote! {
                        pub fn #request_fn_name(clocks: &mut ClockTree) {
                            if increment_reference_count(&mut clocks.#refcount_name) {
                                #request_direct_dependencies
                                #enable_fn_impl_name(clocks, true);
                            }
                        }
                    }
                } else if properties.has_enable() {
                    quote! {
                        pub fn #request_fn_name(clocks: &mut ClockTree) {
                            #request_direct_dependencies
                            #enable_fn_impl_name(clocks, true);
                        }
                    }
                } else {
                    quote! {
                        pub fn #request_fn_name(clocks: &mut ClockTree) {
                            #request_direct_dependencies
                        }
                    }
                },
            },
            release: Function {
                _name: release_fn_name.to_string(),
                implementation: if always_on {
                    quote! {
                        fn #release_fn_name(_clocks: &mut ClockTree) { }
                    }
                } else if refcount_name.is_some() {
                    quote! {
                        pub fn #release_fn_name(clocks: &mut ClockTree) {
                            if decrement_reference_count(&mut clocks.#refcount_name) {
                                #enable_fn_impl_name(clocks, false);
                                #release_direct_dependencies
                            }
                        }
                    }
                } else if properties.has_enable() {
                    quote! {
                        pub fn #release_fn_name(clocks: &mut ClockTree) {
                            #enable_fn_impl_name(clocks, false);
                            #release_direct_dependencies
                        }
                    }
                } else {
                    quote! {
                        pub fn #release_fn_name(clocks: &mut ClockTree) {
                            #release_direct_dependencies
                        }
                    }
                },
            },

            apply_config: Function {
                _name: node.config_apply_function_name().to_string(),
                implementation: quote! { #apply_fn },
            },

            frequency: Function {
                _name: frequency_function_name.to_string(),
                implementation: quote! {
                    pub fn #frequency_function_name(clocks: &mut ClockTree) -> u32 {
                        #frequency_function_impl
                    }
                },
            },

            hal_functions: vec![
                if !always_on && (refcount_name.is_some() || properties.has_enable()) {
                    quote! {
                        fn #enable_fn_impl_name(_clocks: &mut ClockTree, _en: bool) {}
                    }
                } else {
                    quote! {}
                },
                apply_fn_impl,
            ],
        }
    }
}

// Based on https://serde.rs/string-or-struct.html
pub(super) fn list_from_str<'de, D>(deserializer: D) -> Result<Vec<ConfiguresExpression>, D::Error>
where
    D: Deserializer<'de>,
{
    struct ConfigVisitor;

    impl<'de> Visitor<'de> for ConfigVisitor {
        type Value = Vec<ConfiguresExpression>;

        fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
            formatter.write_str("list of strings")
        }

        fn visit_seq<S>(self, list: S) -> Result<Vec<ConfiguresExpression>, S::Error>
        where
            S: SeqAccess<'de>,
        {
            // `SeqAccessDeserializer` is a wrapper that turns a `SeqAccess`
            // into a `Deserializer`, allowing it to be used as the input to T's
            // `Deserialize` implementation. T then deserializes itself using
            // the entries from the map visitor.
            Deserialize::deserialize(de::value::SeqAccessDeserializer::new(list))
        }
    }

    deserializer.deserialize_any(ConfigVisitor)
}

/// Two options:
/// - Multiplexer = variant
/// - Divider = value
#[derive(Debug, Clone)]
pub struct ConfiguresExpression {
    target: String,
    value: Expression,
}

impl ConfiguresExpression {
    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        let Some(clock) = ctx.clock(&self.target) else {
            anyhow::bail!("Clock source {} not found", self.target);
        };

        let clock_name = clock.as_dyn_ref().name().to_case(Case::Ada);
        match clock {
            ClockTreeItem::Multiplexer(multiplexer) => {
                if let Some(name) = self.value.as_name() {
                    if !multiplexer.variant_names().any(|v| v == name) {
                        anyhow::bail!("Multiplexer `{clock_name}` does not have variant `{name}`");
                    }
                } else {
                    anyhow::bail!(
                        "Multiplexer config expression for `{clock_name}` must be a name"
                    );
                }
            }
            ClockTreeItem::Divider(divider) => {
                anyhow::ensure!(
                    divider.is_configurable(),
                    "Divider `{clock_name}` is not configurable",
                )
            }
            _ => anyhow::bail!("Cannot configure source clock {}", self.target),
        }

        Ok(())
    }
}

impl<'de> Deserialize<'de> for ConfiguresExpression {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Ok(Self::from_str(&s).unwrap())
    }
}

impl FromStr for ConfiguresExpression {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (target, value_str) = s.split_once('=').ok_or_else(|| {
            format!("The config expression must be in the format 'target = value'")
        })?;
        let value = somni_parser::parser::parse_expression(value_str)
            .map_err(|e| format!("Failed to parse expression: {}", e))?;
        Ok(ConfiguresExpression {
            target: target.trim().to_string(),
            value: Expression {
                source: value_str.to_string(),
                expr: value,
            },
        })
    }
}

fn human_readable_frequency(mut output: u64) -> (u64, &'static str) {
    let units = ["Hz", "kHz", "MHz", "GHz"];

    let mut index = 0;
    while output >= 1000 && index < units.len() {
        output /= 1000;
        index += 1;
    }

    (output, units[index])
}

#[derive(Debug, Default, Clone)]
pub struct ValuesExpression(Vec<ValueFragment>);

impl ValuesExpression {
    fn as_enum_values(&self) -> Option<Vec<u32>> {
        let frequencies = self
            .0
            .iter()
            .filter_map(|v| match v {
                ValueFragment::FixedFrequency(freq) => Some(*freq),
                _ => None,
            })
            .collect::<Vec<_>>();

        if frequencies.len() == self.0.len() {
            Some(frequencies)
        } else {
            None
        }
    }

    fn as_range(&self) -> Option<(u32, u32)> {
        if self.0.len() != 1 {
            None
        } else {
            let ValueFragment::Range(min, max) = self.0[0] else {
                return None;
            };

            Some((min, max))
        }
    }
}

impl FromStr for ValuesExpression {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let fragments = s
            .split(',')
            .map(|s| s.trim().parse())
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ValuesExpression(fragments))
    }
}

impl<'de> Deserialize<'de> for ValuesExpression {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        FromStr::from_str(&s).map_err(de::Error::custom)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub enum ValueFragment {
    FixedFrequency(u32),
    Range(u32, u32),
}

impl FromStr for ValueFragment {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        fn parse_number(s: &str) -> Result<u32, String> {
            s.replace('_', "")
                .trim()
                .parse()
                .map_err(|e| format!("Invalid number: {}", e))
        }

        if let Some((start, end_incl)) = s.split_once("..=") {
            let start = parse_number(start)?;
            let end = parse_number(end_incl)?;
            Ok(ValueFragment::Range(start, end))
        } else if let Some((start, end_excl)) = s.split_once("..") {
            let start = parse_number(start)?;
            let end = parse_number(end_excl)?;
            Ok(ValueFragment::Range(start, end - 1))
        } else {
            let value = parse_number(s)?;
            Ok(ValueFragment::FixedFrequency(value))
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct RejectExpression(Expression);
impl RejectExpression {
    fn to_rust<'a>(
        &self,
        config_fields: &[(&'a str, Ident)],
        mut variables: HashMap<&'a str, TokenStream>,
    ) -> TokenStream {
        let mut patterns = vec![];

        for (var, config_field) in config_fields {
            patterns.push(quote! { let Some(#config_field) = clocks.#config_field });
            variables.insert(var, quote! { #config_field.value() });
        }

        let reject_expr = self.0.to_rust(variables);

        let assert_reject = quote! {
            assert!(!(#reject_expr));
        };

        if patterns.is_empty() {
            assert_reject
        } else {
            quote! {
                if #(#patterns)&&* {
                    #assert_reject
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Expression {
    source: String,
    expr: ast::Expression<DefaultTypeSet>,
}

impl Expression {
    fn lookup(&self, token: Token) -> &str {
        token.source(&self.source)
    }

    fn expr(&self) -> &ast::RightHandExpression<DefaultTypeSet> {
        match &self.expr {
            ast::Expression::Assignment { .. } => unimplemented!("Assignments are not supported"),
            ast::Expression::Expression { expression } => expression,
        }
    }

    fn visit_variables<'s>(&'s self, mut f: impl FnMut(&'s str)) {
        fn visit_variables(
            expr: &ast::RightHandExpression<DefaultTypeSet>,
            f: &mut impl FnMut(Token),
        ) {
            match expr {
                ast::RightHandExpression::Variable { variable } => f(*variable),
                ast::RightHandExpression::Literal { .. } => {}
                ast::RightHandExpression::UnaryOperator { operand, .. } => {
                    visit_variables(operand, f);
                }
                ast::RightHandExpression::BinaryOperator { operands, .. } => {
                    visit_variables(&operands[0], f);
                    visit_variables(&operands[1], f);
                }
                ast::RightHandExpression::FunctionCall { .. } => {
                    panic!("Function calls are not supported")
                }
            }
        }

        visit_variables(self.expr(), &mut |token| f(self.lookup(token)))
    }

    fn as_name(&self) -> Option<&str> {
        if let ast::RightHandExpression::Variable { variable } = self.expr() {
            Some(self.lookup(*variable))
        } else {
            None
        }
    }

    fn to_rust(&self, variables: HashMap<&str, TokenStream>) -> TokenStream {
        ExprCompiler::new(&variables).compile_expression(self)
    }
}

impl<'de> Deserialize<'de> for Expression {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        let expr = somni_parser::parser::parse_expression::<DefaultTypeSet>(&s).unwrap();
        Ok(Expression { source: s, expr })
    }
}
