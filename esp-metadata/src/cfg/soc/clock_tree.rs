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

use anyhow::{Context, Result};
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
        expr_compiler::ExprCompiler,
        generic::Generic,
        mux::Multiplexer,
        source::{DerivedClockSource, Source},
    },
    soc::{ClockTreeNodeInstance, ProcessedClockData},
};

mod expr_compiler;
mod generic;
mod mux;
mod source;

#[derive(Clone)]
pub(crate) struct Function {
    pub _name: String,
    pub implementation: TokenStream,
}

pub(crate) struct ClockNodeFunctions {
    pub request: Function,
    pub release: Function,
    pub apply_config: Function,
    pub current_config: Function,

    pub frequency: Function,
    pub hal_functions: Vec<TokenStream>,
}

impl ClockNodeFunctions {
    pub fn implement_functions(&self) -> TokenStream {
        let request_impl = &self.request.implementation;
        let release_impl = &self.release.implementation;
        let apply_impl = &self.apply_config.implementation;
        let current_config_impl = &self.current_config.implementation;
        let frequency_impl = &self.frequency.implementation;

        quote! {
            #apply_impl
            #current_config_impl
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
    pub tree: &'c [&'c ClockTreeNodeInstance],
}

impl<'c> ValidationContext<'c> {
    pub fn has_clock(&self, instance: &ClockTreeNodeInstance, clk: &str) -> bool {
        self.clock(instance, clk).is_some()
    }

    fn clock(&self, instance: &ClockTreeNodeInstance, clk: &str) -> Option<&ClockTreeNodeInstance> {
        let local_clk = format!("{}_{}", instance.group, clk);
        self.tree
            .iter()
            .cloned()
            .find(|item| [&local_clk, clk].contains(&item.name_str().as_str()))
    }

    pub(crate) fn run_validation(&self) -> Result<()> {
        for node in self.tree {
            node.validate_source_data(self)
                .with_context(|| format!("Invalid clock tree item: {}", node.name_str()))?;
        }

        Ok(())
    }
}

pub(crate) struct DependencyGraph {
    /// Direct upstream -> downstream clock relationships
    graph: IndexMap<String, Vec<String>>,
    /// Direct downstream -> upstream clock relationships
    reverse_graph: IndexMap<String, Vec<String>>,
}

impl DependencyGraph {
    pub fn empty() -> Self {
        Self {
            graph: IndexMap::new(),
            reverse_graph: IndexMap::new(),
        }
    }

    pub(super) fn build_from(clock_tree: &ProcessedClockData) -> Self {
        let mut dependency_graph = IndexMap::new();
        let mut reverse_dependency_graph = IndexMap::new();

        for node in clock_tree.clock_tree.iter() {
            dependency_graph.insert(node.name.clone(), Vec::new());
            reverse_dependency_graph.insert(node.name.clone(), Vec::new());
        }

        for node in clock_tree.clock_tree.iter() {
            let node_name = node.name_str();

            for input in node.input_clocks(clock_tree) {
                let graph_node = &mut dependency_graph[&input];
                if !graph_node.contains(node_name) {
                    graph_node.push(node_name.clone());
                }

                let reverse_node = &mut reverse_dependency_graph[node_name];
                if !reverse_node.contains(&input) {
                    reverse_node.push(input);
                }
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

    /// Returns the names of nodes in a topological order.
    pub fn iter(&self) -> impl Iterator<Item = String> {
        topological_sort(&self.reverse_graph)
    }
}

fn topological_sort(dep_graph: &IndexMap<String, Vec<String>>) -> impl Iterator<Item = String> {
    let mut sorted = Vec::new();
    let mut dep_graph = dep_graph.clone();
    while !dep_graph.is_empty() {
        dep_graph.retain(|node, deps| {
            deps.retain(|dep| !sorted.contains(dep));

            if deps.is_empty() {
                sorted.push(node.clone());
                false
            } else {
                true
            }
        });
    }

    sorted.into_iter()
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
    /// Name of the node's current configuration field in the ClockTree struct.
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

/// Common interface for clock node types.
pub(crate) trait ClockTreeNodeType: Any {
    /// Returns which clock nodes' configurations are affected when this node is configured.
    // TODO: pass instance to apply template naming scheme to returned clocks
    // (e.g. FUNCTION_CLOCK -> UART0_FUNCTION_CLOCK)
    fn affected_nodes<'s>(&'s self) -> Vec<&'s str> {
        vec![]
    }

    fn input_clocks(
        &self,
        _instance: &ClockTreeNodeInstance,
        _tree: &ProcessedClockData,
    ) -> Vec<String> {
        vec![]
    }

    fn always_on(&self) -> bool {
        false
    }
    fn validate_source_data(
        &self,
        instance: &ClockTreeNodeInstance,
        ctx: &ValidationContext<'_>,
    ) -> Result<()>;
    fn is_configurable(&self) -> bool;
    fn config_apply_function(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream;

    fn node_frequency_impl(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream;

    /// Returns the documentation for the clock configuration, which will be placed on the
    /// `ClockConfig` field.
    fn config_documentation(&self, instance: &ClockTreeNodeInstance) -> Option<String> {
        Some(format!(" `{}` configuration.", instance.name_str()))
    }

    fn validate_configures_expr(
        &self,
        _instance: &ClockTreeNodeInstance,
        _expr: &ConfiguresExpression,
    ) -> Result<()> {
        anyhow::bail!("Configures expressions are not supported")
    }

    fn apply_configuration(
        &self,
        instance: &ClockTreeNodeInstance,
        _expr: &ConfiguresExpression,
        _tree: &ProcessedClockData,
    ) -> TokenStream {
        if instance.is_configurable() {
            unimplemented!();
        } else {
            quote! {}
        }
    }

    fn config_type(&self, instance: &ClockTreeNodeInstance) -> TokenStream;

    fn request_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream;
    fn release_direct_dependencies(
        &self,
        instance: &ClockTreeNodeInstance,
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

    #[serde(rename = "generic")]
    Generic(Generic),

    #[serde(rename = "derived")]
    Derived(DerivedClockSource),
}

impl ClockTreeItem {
    pub(crate) fn name(&self) -> &str {
        match self {
            ClockTreeItem::Multiplexer(mux) => mux.name.as_str(),
            ClockTreeItem::Source(src) => src.name.as_str(),
            ClockTreeItem::Generic(div) => div.name.as_str(),
            ClockTreeItem::Derived(drv) => drv.source_options.name.as_str(),
        }
    }

    pub(crate) fn boxed(&self) -> Box<dyn ClockTreeNodeType> {
        match self {
            ClockTreeItem::Multiplexer(mux) => Box::new(mux.clone()),
            ClockTreeItem::Source(src) => Box::new(src.clone()),
            ClockTreeItem::Generic(div) => Box::new(div.clone()),
            ClockTreeItem::Derived(drv) => Box::new(drv.clone()),
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
    effect: ConfiguresEffect,
    source: String,
}

#[derive(Debug, Clone)]
pub struct ConfiguresEffect {
    node: String,
    property: Option<String>,
    value: ast::RightHandExpression<DefaultTypeSet>,
}

impl ConfiguresExpression {
    pub(crate) fn effect(&self) -> &ConfiguresEffect {
        &self.effect
    }

    pub(crate) fn name(&self, token: Token) -> &str {
        token.source(&self.source)
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
        let value = somni_parser::parser::parse_expression(s)
            .map_err(|e| format!("Failed to parse expression: {}", e))?;

        let effect = match &value {
            ast::Expression::Assignment {
                left_expr,
                operator: _,
                right_expr,
            } => ConfiguresEffect {
                // NODE = VALUE
                node: if let ast::LeftHandExpression::Name { variable } = left_expr {
                    variable.source(s).to_string()
                } else {
                    return Err(format!("Invalid config expression: {}", s));
                },
                property: None,
                value: right_expr.clone(),
            },
            ast::Expression::Expression {
                expression: ast::RightHandExpression::FunctionCall { name, arguments },
            } if name.source(s) == "configure" => {
                // configure(NODE, property, VALUE)
                if arguments.len() != 3 {
                    return Err(format!("Invalid config expression: {}", s));
                }
                let ast::RightHandExpression::Variable { variable } = &arguments[0] else {
                    return Err(format!(
                        "Node name in configure expression must be an identifier: {s}"
                    ));
                };
                let node = variable.source(s).to_string();

                let ast::RightHandExpression::Variable { variable } = &arguments[1] else {
                    return Err(format!(
                        "Property name in configure expression must be an identifier: {s}"
                    ));
                };
                let property = Some(variable.source(s).to_string());

                ConfiguresEffect {
                    node,
                    property,
                    value: arguments[2].clone(),
                }
            }
            _ => return Err(format!("Invalid config expression: {}", s)),
        };

        Ok(ConfiguresExpression {
            effect,
            source: s.to_string(),
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
        if let [ValueFragment::Range(min, max)] = self.0.as_slice() {
            Some((*min, *max))
        } else {
            None
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
            let s = s.replace('_', "");
            let s = s.trim();

            let result = if s.starts_with("0x") {
                u32::from_str_radix(&s[2..], 16)
            } else if s.starts_with("0o") {
                u32::from_str_radix(&s[2..], 8)
            } else if s.starts_with("0b") {
                u32::from_str_radix(&s[2..], 2)
            } else {
                s.parse()
            };

            result.map_err(|e| format!("Failed to parse {s}: {e}"))
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

// Asserts that will be run before configuring a node.
//
// Referring to a node by name resolves to its output frequency. `property(NODE)` resolves to the
// node's property value.
#[derive(Debug, Clone, Deserialize)]
pub struct RejectExpression(Expression);
impl RejectExpression {
    fn to_rust<'a>(
        &'a self,
        mut variables: HashMap<&'a str, TokenStream>,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let mut patterns = vec![];

        self.0.visit_variables(|var| {
            if !variables.contains_key(var) {
                // Referring to a node by name resolves to its output frequency.
                let properties = tree.properties(var);
                let node = tree.node(var);
                let freq_fn = node.frequency_function_name();
                variables.insert(var, quote! { #freq_fn(clocks) });

                // Only run the assert if the referenced nodes have been configured
                let node_field = properties.field_name();
                patterns.push(quote! { clocks.#node_field.is_some() });
            }
        });

        let reject_expr = self.0.to_rust(variables, tree);

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
    expr: ast::RightHandExpression<DefaultTypeSet>,
}

impl Expression {
    fn lookup(&self, token: Token) -> &str {
        token.source(&self.source)
    }

    fn expr(&self) -> &ast::RightHandExpression<DefaultTypeSet> {
        &self.expr
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
                ast::RightHandExpression::FunctionCall { name: _, arguments } => {
                    // This will ensure the user knows the node is referenced in the expression. A
                    // bit of a convention over specification, though.
                    visit_variables(&arguments[0], f);
                }
            }
        }

        visit_variables(self.expr(), &mut |token| f(self.lookup(token)))
    }

    fn to_rust(
        &self,
        variables: HashMap<&str, TokenStream>,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        ExprCompiler::new(&variables).compile_expression(self, tree)
    }
}

impl<'de> Deserialize<'de> for Expression {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        if let ast::Expression::Expression { expression } =
            somni_parser::parser::parse_expression::<DefaultTypeSet>(&s).unwrap()
        {
            Ok(Expression {
                source: s,
                expr: expression,
            })
        } else {
            Err(serde::de::Error::custom("Assignments are not supported"))
        }
    }
}
