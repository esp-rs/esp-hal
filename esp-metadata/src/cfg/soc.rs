use std::{cell::RefCell, collections::HashMap, ops::Range, str::FromStr};

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, StateConverter, pattern};
use indexmap::{IndexMap, IndexSet};
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{
    cfg::{
        clock_tree::{
            ClockNodeFunctions,
            ClockTreeItem,
            ClockTreeNodeType,
            ConfiguresExpression,
            DependencyGraph,
            Function,
            ManagementProperties,
            ValidationContext,
        },
        soc::clock_tree::PeripheralClockTreeEntry,
    },
    number_hex,
};

pub mod clock_tree;

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

/// Represents the clock sources and clock distribution tree in the SoC.
#[derive(Debug, Default, Clone, Deserialize)]
pub struct SystemClocks {
    clock_tree: Vec<ClockTreeItem>,
}

pub(crate) struct ProcessedClockData {
    /// All instantiated clock tree nodes.
    clock_tree: Vec<ClockTreeNodeInstance>,

    /// Refcount/config type properties of the clock tree nodes.
    management_properties: HashMap<String, ManagementProperties>,

    /// System clock graph.
    dependency_graph: DependencyGraph,
}

pub(crate) struct ClockTreeNodeInstance {
    node: Box<dyn ClockTreeNodeType>,

    include_in_global_config: bool,
    is_first_instance: bool,
    force_configurable: bool,

    /// Name of the instantiated clock tree node.
    ///
    /// Must be in CONSTANT_CASE, e.g. FUNCTION_CLOCK.
    name: String,

    /// Name of the group (e.g. peripheral) this tree node belongs to.
    ///
    /// Must be in CONSTANT_CASE, e.g. UART0.
    group: String,

    /// Name of the template used to instantiate this clock tree node.
    ///
    /// Must be in CONSTANT_CASE, e.g. UART0_FUNCTION_CLOCK.
    // TODO: we should have methods to modify names in implementation code
    // (e.g. to make them able to first refer to a peripheral-local clock node, then fall back to a
    // system node) - fn resolve_clock_node(&self, tree: &ProcessedClockData, name: &str) -> &str
    template_name: String,
}

impl ClockTreeNodeInstance {
    /// Returns which clock nodes' configurations are affected when this node is configured.
    // TODO: pass instance to apply template naming scheme to returned clocks
    // (e.g. FUNCTION_CLOCK -> UART0_FUNCTION_CLOCK)
    fn affected_nodes(&self) -> Vec<&str> {
        self.node.affected_nodes()
    }

    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> Result<()> {
        self.node.validate_source_data(self, ctx)
    }

    fn is_configurable(&self) -> bool {
        self.node.is_configurable() || self.force_configurable
    }

    fn config_type(&self) -> Option<TokenStream> {
        if !self.is_first_instance || !self.is_configurable() {
            return None;
        }
        Some(self.node.config_type(self))
    }

    fn config_documentation(&self) -> Option<String> {
        self.node.config_documentation(self)
    }

    fn node_frequency_impl(&self, tree: &ProcessedClockData) -> TokenStream {
        self.node.node_frequency_impl(self, tree)
    }

    fn always_on(&self) -> bool {
        self.node.always_on()
    }

    fn input_clocks(&self, tree: &ProcessedClockData) -> Vec<String> {
        self.node.input_clocks(self, tree)
    }

    /// Returns the name of the clock configuration type. The corresponding field in the
    /// `ClockConfig` struct will have this type.
    fn config_type_name(&self) -> Ident {
        let item = self
            .template_name
            .from_case(Case::Constant)
            .to_case(Case::Pascal);
        quote::format_ident!("{}Config", item)
    }

    fn apply_configuration(
        &self,
        expr: &ConfiguresExpression,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.node.apply_configuration(self, expr, tree)
    }

    fn name(&self) -> StateConverter<'_, String> {
        self.name_str().from_case(Case::Constant)
    }

    fn name_str<'a>(&'a self) -> &'a String {
        &self.name
    }

    fn enable_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("enable_{}", name)
    }

    fn request_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("request_{}", name)
    }

    fn release_fn_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("release_{}", name)
    }

    fn frequency_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("{}_frequency", name)
    }

    fn config_frequency_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("{}_config_frequency", name)
    }

    fn config_apply_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("configure_{}", name)
    }

    fn current_config_function_name(&self) -> Ident {
        let name = self.name().to_case(Case::Snake);
        format_ident!("{}_config", name)
    }

    fn config_current_function(&self, tree: &ProcessedClockData) -> TokenStream {
        if self.is_configurable() {
            let ty_name = self.config_type_name();
            let state = tree.properties(self.name_str()).field_name();
            let fn_name = self.current_config_function_name();
            quote! {
                pub fn #fn_name(clocks: &mut ClockTree) -> Option<#ty_name> {
                    clocks.#state
                }
            }
        } else {
            quote! {}
        }
    }

    fn node_functions(&self, tree: &ProcessedClockData) -> ClockNodeFunctions {
        let ty_name = if self.is_configurable() {
            Some(self.config_type_name())
        } else {
            None
        };

        let request_fn_name = self.request_fn_name();
        let release_fn_name = self.release_fn_name();
        let properties = tree.properties(self.name_str());
        let refcount_name = properties.refcount_field_name();
        let enable_fn_name = self.enable_fn_name();
        let enable_fn_impl_name = format_ident!("{}_impl", enable_fn_name);
        let always_on = properties.always_on();

        let request_direct_dependencies = self.node.request_direct_dependencies(self, tree);
        let release_direct_dependencies = self.node.release_direct_dependencies(self, tree);

        // Only configurables have an apply fn
        let apply_fn = ty_name
            .as_ref()
            .map(|_| self.node.config_apply_function(self, tree));
        let current_config_fn = self.config_current_function(tree);
        let apply_fn_impl = ty_name
            .as_ref()
            .map(|_| self.config_apply_impl_function())
            .unwrap_or_default();
        let frequency_function_impl = self.node_frequency_impl(tree);
        let frequency_function_name = self.frequency_function_name();
        let config_frequency_function_name = self.config_frequency_function_name();

        let enable_trace = format!("Enabling {}", self.name_str());
        let disable_trace = format!("Disabling {}", self.name_str());

        let request_trace = format!("Requesting {}", self.name_str());
        let release_trace = format!("Releasing {}", self.name_str());

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
                            trace!(#request_trace);
                            if increment_reference_count(&mut clocks.#refcount_name) {
                                trace!(#enable_trace);
                                #request_direct_dependencies
                                #enable_fn_impl_name(clocks, true);
                            }
                        }
                    }
                } else if properties.has_enable() {
                    quote! {
                        pub fn #request_fn_name(clocks: &mut ClockTree) {
                            trace!(#request_trace);
                            trace!(#enable_trace);
                            #request_direct_dependencies
                            #enable_fn_impl_name(clocks, true);
                        }
                    }
                } else {
                    quote! {
                        pub fn #request_fn_name(clocks: &mut ClockTree) {
                            trace!(#request_trace);
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
                            trace!(#release_trace);
                            if decrement_reference_count(&mut clocks.#refcount_name) {
                                trace!(#disable_trace);
                                #enable_fn_impl_name(clocks, false);
                                #release_direct_dependencies
                            }
                        }
                    }
                } else if properties.has_enable() {
                    quote! {
                        pub fn #release_fn_name(clocks: &mut ClockTree) {
                            trace!(#release_trace);
                            trace!(#disable_trace);
                            #enable_fn_impl_name(clocks, false);
                            #release_direct_dependencies
                        }
                    }
                } else {
                    quote! {
                        pub fn #release_fn_name(clocks: &mut ClockTree) {
                            trace!(#release_trace);
                            #release_direct_dependencies
                        }
                    }
                },
            },

            apply_config: Function {
                _name: self.config_apply_function_name().to_string(),
                implementation: quote! { #apply_fn },
            },

            current_config: Function {
                _name: self.current_config_function_name().to_string(),
                implementation: quote! { #current_config_fn },
            },

            frequency: Function {
                _name: frequency_function_name.to_string(),
                implementation: if self.is_configurable() {
                    let node_field = properties.field_name();
                    quote! {
                        #[allow(unused_variables)]
                        pub fn #config_frequency_function_name(clocks: &mut ClockTree, config: #ty_name) -> u32 {
                            #frequency_function_impl
                        }
                        pub fn #frequency_function_name(clocks: &mut ClockTree) -> u32 {
                            if let Some(config) = clocks.#node_field {
                                #config_frequency_function_name(clocks, config)
                            } else {
                                0
                            }
                        }
                    }
                } else {
                    quote! {
                        pub fn #frequency_function_name(clocks: &mut ClockTree) -> u32 {
                            #frequency_function_impl
                        }
                    }
                },
            },

            hal_functions: vec![
                if !always_on && (refcount_name.is_some() || properties.has_enable()) {
                    quote! {
                        fn #enable_fn_impl_name(_clocks: &mut ClockTree, _en: bool) {
                            todo!()
                        }
                    }
                } else {
                    quote! {}
                },
                apply_fn_impl,
            ],
        }
    }

    /// Generates a stub implementation that the HAL needs to provide.
    fn config_apply_impl_function(&self) -> TokenStream {
        let ty_name = self.config_type_name();
        let apply_fn_name = self.config_apply_function_name();
        let hal_impl = format_ident!("{}_impl", apply_fn_name);

        quote! {
            fn #hal_impl(_clocks: &mut ClockTree, _old_config: Option<#ty_name>, _new_config: #ty_name) {
                todo!()
            }
        }
    }

    fn resolve_node<'tree>(
        &self,
        tree: &'tree ProcessedClockData,
        clock: &str,
    ) -> &'tree ClockTreeNodeInstance {
        let local_node = format!("{}_{}", self.group, clock);
        if let Some(node) = tree.try_get_node(&local_node) {
            node
        } else {
            tree.node(clock)
        }
    }

    fn validate_configures_expr(&self, expr: &ConfiguresExpression) -> Result<()> {
        self.node.validate_configures_expr(self, expr)
    }
}

impl ProcessedClockData {
    /// Returns a node by its name (e.g. `XTAL_CLK`).
    ///
    /// As the clock tree is stored as a vector, this method performs a linear search.
    fn node(&self, name: &str) -> &ClockTreeNodeInstance {
        self.try_get_node(name)
            .unwrap_or_else(|| panic!("Clock node {name} not found"))
    }

    /// Returns a node by its name (e.g. `XTAL_CLK`), or None if not found.
    ///
    /// As the clock tree is stored as a vector, this method performs a linear search.
    fn try_get_node(&self, name: &str) -> Option<&ClockTreeNodeInstance> {
        self.clock_tree.iter().find(|item| item.name_str() == name)
    }

    fn properties(&self, node_name: &str) -> &ManagementProperties {
        self.management_properties
            .get(node_name)
            .unwrap_or_else(|| panic!("Management properties for {node_name} not found"))
    }
}

impl SystemClocks {
    fn generate_macro(&self, tree: &ProcessedClockData) -> Result<TokenStream> {
        let mut clock_tree_node_defs = vec![];
        let mut clock_tree_node_impls = vec![];
        let mut clock_tree_node_state_getter_doclines = vec![];
        let mut clock_tree_state_fields = vec![];
        let mut clock_tree_state_field_types = vec![];
        let mut clock_tree_refcount_fields = vec![];
        let mut configurables = vec![];
        let mut system_config_steps = HashMap::new();
        let mut provided_function_doclines = vec![];
        for clock_item in tree.clock_tree.iter() {
            // Generate code for all clock tree nodes
            if let Some(config_type) = clock_item.config_type() {
                clock_tree_node_defs.push(config_type);
            }
            let node_state = tree.properties(clock_item.name_str());
            if let Some(type_name) = node_state.type_name() {
                clock_tree_state_fields.push(node_state.field_name());
                clock_tree_state_field_types.push(type_name);
                clock_tree_node_state_getter_doclines.push(format!(
                    "Returns the current configuration of the {} clock tree node",
                    clock_item.name_str(),
                ));
            }
            if let Some(refcount_field) = node_state.refcount_field_name() {
                clock_tree_refcount_fields.push(refcount_field);
            }

            let node_funcs = clock_item.node_functions(tree);
            clock_tree_node_impls.push(node_funcs.implement_functions());
            if !node_funcs.hal_functions.is_empty() {
                let header = format!(" // {}", clock_item.name_str());
                provided_function_doclines.push(quote! {
                    #[doc = ""]
                    #[doc = #header]
                    #[doc = ""]
                });
                for func in node_funcs.hal_functions.iter() {
                    if func.is_empty() {
                        continue;
                    }
                    let func = func.to_string();
                    provided_function_doclines.push(quote! {
                        #[doc = #func]
                        #[doc = ""] // empty line between functions
                    });
                }
            }

            if clock_item.is_configurable() && clock_item.include_in_global_config {
                // Generate code for the global clock configuration
                let config_type_name = clock_item.config_type_name();
                let config_apply_function_name = clock_item.config_apply_function_name();

                let item = clock_item.name().to_case(Case::Snake);

                let name = format_ident!("{}", item);

                let docline = clock_item.config_documentation().map(|doc| {
                    // TODO: add explanation what happens if the field is left `None`.
                    let doc = doc.lines();
                    quote! { #(#[doc = #doc])* }
                });

                configurables.push(quote! {
                    #docline
                    pub #name: Option<#config_type_name>,
                });

                system_config_steps.insert(
                    clock_item.name_str(),
                    quote! {
                        if let Some(config) = self.#name {
                            #config_apply_function_name(clocks, config);
                        }
                    },
                );
            }
        }

        let system_config_steps = tree
            .dependency_graph
            .iter()
            .map(|node| system_config_steps.get(&node));

        Ok(quote! {
            #[macro_export]
            /// ESP-HAL must provide implementation for the following functions:
            /// ```rust, no_run
            #(#provided_function_doclines)*
            /// ```
            macro_rules! define_clock_tree_types {
                () => {
                    #(#clock_tree_node_defs)*

                    /// Represents the device's clock tree.
                    pub struct ClockTree {
                        #(#clock_tree_state_fields: Option<#clock_tree_state_field_types>,)*
                        #(#clock_tree_refcount_fields: u32,)*
                    }
                    impl ClockTree {
                        /// Locks the clock tree for exclusive access.
                        pub fn with<R>(f: impl FnOnce(&mut ClockTree) -> R) -> R {
                            CLOCK_TREE.with(f)
                        }

                        #(
                            #[doc = #clock_tree_node_state_getter_doclines]
                            pub fn #clock_tree_state_fields(&self) -> Option<#clock_tree_state_field_types> {
                                self.#clock_tree_state_fields
                            }
                        )*
                    }

                    static CLOCK_TREE: ::esp_sync::NonReentrantMutex<ClockTree> =
                        ::esp_sync::NonReentrantMutex::new(ClockTree {
                            #(#clock_tree_state_fields: None,)*
                            #(#clock_tree_refcount_fields: 0,)*
                        });

                    #(#clock_tree_node_impls)*

                    /// Clock tree configuration.
                    ///
                    /// The fields of this struct are optional, with the following caveats:
                    /// - If `XTAL_CLK` is not specified, the crystal frequency will be automatically detected
                    ///   if possible.
                    /// - The CPU and its upstream clock nodes will be set to a default configuration.
                    /// - Other unspecified clock sources will not be useable by peripherals.
                    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    #[instability::unstable]
                    pub struct ClockConfig {
                        #(#configurables)*
                    }

                    impl ClockConfig {
                        fn apply(&self) {
                            ClockTree::with(|clocks| {
                                #(#system_config_steps)*
                            });
                        }
                    }

                    // static CLOCK_BITMAP: ::portable_atomic::AtomicU32 = ::portable_atomic::AtomicU32::new(0);
                    fn increment_reference_count(refcount: &mut u32) -> bool {
                        let first = *refcount == 0;
                        // CLOCK_BITMAP.fetch_or(!clock_id, Ordering::Relaxed);
                        *refcount = unwrap!(refcount.checked_add(1), "Reference count overflow");
                        first
                    }
                    fn decrement_reference_count(refcount: &mut u32) -> bool {
                        *refcount = refcount.saturating_sub(1);
                        let last = *refcount == 0;
                        // CLOCK_BITMAP.fetch_and(!clock_id, Ordering::Relaxed);
                        last
                    }
                };
            }
        })
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
#[derive(Debug, Default, Clone, Deserialize)]
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

    /// Peripheral clock source data.
    ///
    /// These can be new definitions, or they can reference another `peripheral_clocks` entry, in
    /// which case the peripherals will use the same data types, and the same API will be
    /// generated for them.
    #[serde(default)]
    #[serde(deserialize_with = "clock_tree::ref_or_def")]
    clocks: PeripheralClockTreeEntry,
}
impl PeripheralClock {
    fn clock_signals<'a>(&'a self, peripheral_clocks: &'a PeripheralClocks) -> &'a [ClockTreeItem] {
        match &self.clocks {
            PeripheralClockTreeEntry::Definition(items) => items.as_slice(),
            PeripheralClockTreeEntry::Reference(template) => peripheral_clocks
                .peripheral_clocks
                .iter()
                .filter_map(|c| {
                    if c.name.as_str() != template {
                        return None;
                    }
                    match &c.clocks {
                        PeripheralClockTreeEntry::Definition(items) => Some(items.as_slice()),
                        PeripheralClockTreeEntry::Reference(_) => {
                            panic!("Referenced peripheral must have clock node definitions");
                        }
                    }
                })
                .next()
                .unwrap(),
        }
    }

    fn template_peripheral_name(&self) -> &str {
        match &self.clocks {
            PeripheralClockTreeEntry::Definition(_) => self.name.as_str(),
            PeripheralClockTreeEntry::Reference(inherit_from) => inherit_from.as_str(),
        }
    }
}

#[derive(Debug, Default, Clone, Deserialize)]
pub struct PeripheralClocks {
    pub(crate) templates: Vec<Template>,
    pub(crate) peripheral_clocks: Vec<PeripheralClock>,
}

impl PeripheralClocks {
    fn generate_macro(&self, _tree: &ProcessedClockData) -> Result<TokenStream> {
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
            .map(|clock| format_ident!("{}", clock.name))
            .collect::<Vec<_>>();
        let keep_enabled = clocks.iter().filter_map(|clock| {
            clock
                .keep_enabled
                .then_some(format_ident!("{}", clock.name))
        });

        let clk_en_arms = clocks
            .iter()
            .map(|clock| {
                let clock_name = format_ident!("{}", clock.name);
                let clock_en = self.clk_en(clock)?;

                Ok(quote! {
                    Peripheral::#clock_name => {
                        #clock_en
                    }
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let rst_arms = clocks
            .iter()
            .map(|clock| {
                let clock_name = format_ident!("{}", clock.name);
                let rst = self.rst(clock)?;

                Ok(quote! {
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
    #[serde(default)]
    pub(crate) system_clocks: SystemClocks,

    #[serde(default)]
    pub(crate) peripheral_clocks: PeripheralClocks,
}

impl DeviceClocks {
    /// Returns an iterator over the clock tree items.
    fn process(&self) -> Result<ProcessedClockData> {
        let mut clock_tree = self
            .system_clocks
            .clock_tree
            .iter()
            .map(|node| {
                let node_name = node.name();
                let node = ClockTreeNodeInstance {
                    node: node.boxed(),
                    include_in_global_config: true,
                    is_first_instance: true,
                    force_configurable: false,
                    name: node.name().to_string(),
                    group: String::new(),
                    template_name: node.name().to_string(),
                };

                (node_name.to_string(), RefCell::new(node))
            })
            .collect::<IndexMap<_, _>>();

        // Merge peripheral clock sources into the clock tree
        let mut peri_clocks = self.peripheral_clocks.peripheral_clocks.clone();
        peri_clocks.sort_by(|a, b| a.name.cmp(&b.name));

        for peri in peri_clocks {
            // A peripheral can have any number of clock sources. We'll turn them into clock tree
            // nodes here.
            let template_peripheral = peri.template_peripheral_name();
            for def in peri.clock_signals(&self.peripheral_clocks) {
                let node = ClockTreeNodeInstance {
                    node: def.boxed(),
                    include_in_global_config: false,
                    is_first_instance: matches!(
                        peri.clocks,
                        PeripheralClockTreeEntry::Definition(_)
                    ),
                    // FIXME peripherals force configurability because we don't have a way to
                    // cfg them out. Decide if we can do better.
                    force_configurable: true,
                    name: format!(
                        "{}_{}",
                        peri.name.from_case(Case::Ada).to_case(Case::Constant),
                        def.name()
                    ),
                    group: template_peripheral
                        .from_case(Case::Ada)
                        .to_case(Case::Constant),
                    template_name: format!(
                        "{}_{}",
                        template_peripheral
                            .from_case(Case::Ada)
                            .to_case(Case::Constant),
                        def.name()
                    ),
                };

                clock_tree.insert(node.name.clone(), RefCell::new(node));
            }
        }

        ValidationContext {
            tree: clock_tree
                .values()
                .map(|item| unsafe {
                    // Safety: we're not mutating the item, so immutable borrow should be ok without
                    // the guard wrapper.
                    item.try_borrow_unguarded().unwrap()
                })
                .collect::<Vec<_>>()
                .as_slice(),
        }
        .run_validation()?;

        // Apply legacy sorting to avoid reordering generated code.
        let mut sorted_clocks = IndexSet::new();

        for idx in 0..clock_tree.len() {
            let node = &clock_tree[idx].borrow();

            // If item A configures item B, then item B is a dependent clock tree item. Sort the
            // dependent clocks to be before their configuring nodes.
            for clock in node.affected_nodes() {
                let (aff_idx, _, affected) = clock_tree.get_full(clock).unwrap();
                affected.borrow_mut().include_in_global_config = false;

                sorted_clocks.insert(aff_idx);
            }

            // Each tree item tells its own clock type, except for dependent items. If something has
            // already been classified, we can only change it from its kind to dependent.
            sorted_clocks.insert(idx);
        }

        // Now sort the nodes according to the order we've established.
        let clock_tree = {
            // ClockTreeNodeInstance is not Clone, so let's turn the vector into a vector of
            // optionals that we can take from.
            let mut clock_tree = clock_tree
                .into_iter()
                .map(|(_, n)| Some(n.into_inner()))
                .collect::<Vec<_>>();
            let mut sorted = Vec::with_capacity(clock_tree.len());
            for idx in sorted_clocks {
                sorted.push(clock_tree[idx].take().unwrap());
            }
            sorted
        };

        let mut tree = ProcessedClockData {
            clock_tree,
            management_properties: HashMap::new(),
            dependency_graph: DependencyGraph::empty(),
        };

        // To compute refcount requirement and correct initialization order, we need to be able to
        // access direct dependencies (downstream clocks). As it is simpler to define
        // dependents (inputs), we have to do a bit of maths.
        tree.dependency_graph = DependencyGraph::build_from(&tree);

        for node in tree.clock_tree.iter() {
            // If there's a single dependent clock, we can piggyback on its refcount.
            // Note that this is only valid if the clock node is not expected to be manually
            // managed. In the current model, manually managed clocks have 0 consumers.
            let has_one_direct_dependent = tree.dependency_graph.users(node.name_str()).len() == 1;

            let refcounted = !(node.always_on() || has_one_direct_dependent);

            tree.management_properties.insert(
                node.name_str().clone(),
                ManagementProperties {
                    name: format_ident!("{}", node.name().to_case(Case::Snake)),
                    refcounted,
                    // Always-on clock sources don't need enable functions.
                    has_enable: !(node.always_on()
                        && tree.dependency_graph.inputs(node.name_str()).is_empty()),
                    state_ty: if node.is_configurable() {
                        Some(node.config_type_name())
                    } else {
                        None
                    },
                    always_on: node.always_on(),
                },
            );
        }

        Ok(tree)
    }
}

impl super::GenericProperty for DeviceClocks {
    fn cfgs(&self) -> Option<Vec<String>> {
        let processed_clocks = self.process().unwrap();

        let cfgs = processed_clocks
            .clock_tree
            .iter()
            .map(|node| node.name().to_case(Case::Snake))
            .map(|name| format!("soc_has_clock_node_{name}"))
            .collect::<Vec<_>>();

        Some(cfgs)
    }

    fn macros(&self) -> Option<TokenStream> {
        let processed_clocks = self.process().unwrap();

        let system_clocks = match self.system_clocks.generate_macro(&processed_clocks) {
            Ok(tokens) => tokens,
            Err(err) => panic!(
                "{:?}",
                err.context("Failed to generate system clock control code")
            ),
        };
        let peripheral_clocks = match self.peripheral_clocks.generate_macro(&processed_clocks) {
            Ok(tokens) => tokens,
            Err(err) => panic!(
                "{:?}",
                err.context("Failed to generate peripheral clock control macro")
            ),
        };

        Some(quote! {
            #system_clocks
            #peripheral_clocks
        })
    }
}
