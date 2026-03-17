use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
    ops::Range,
    str::FromStr,
};

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, StateConverter, pattern};
use indexmap::{IndexMap, IndexSet};
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{
    PeripheralDef,
    cfg::clock_tree::{
        ClockNodeFunctions,
        ClockTreeItem,
        ClockTreeNodeType,
        ConfiguresExpression,
        DependencyGraph,
        Function,
        ManagementProperties,
        ValidationContext,
    },
    number,
    number_hex,
};

pub mod clock_tree;

#[derive(Debug, Clone, Deserialize)]
pub struct SocConfig {
    #[serde(default)]
    pub peripherals: Vec<PeripheralDef>,
    #[serde(default)]
    pub clocks: DeviceClocks,
    pub memory_map: MemoryMap,
}

impl super::GenericProperty for SocConfig {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut cfgs = vec![];

        cfgs.extend(self.clocks.cfgs(self));
        cfgs.extend(self.memory_map.cfgs());

        Some(cfgs)
    }

    fn macros(&self) -> Option<TokenStream> {
        let mut tokens = quote! {};

        tokens.extend(self.clocks.macros(self));
        tokens.extend(self.memory_map.macros());

        Some(tokens)
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

impl MemoryMap {
    fn macros(&self) -> TokenStream {
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

        quote! {
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
        }
    }

    fn cfgs(&self) -> Vec<String> {
        self.ranges
            .iter()
            .map(|region| format!("has_{}_region", region.name.to_lowercase()))
            .collect()
    }
}

/// Represents the clock sources and clock distribution tree in the SoC.
#[derive(Debug, Default, Clone, Deserialize)]
pub struct SystemClocks {
    clock_tree: Vec<ClockTreeItem>,

    /// Clock template groups.
    #[serde(default)]
    template_groups: Vec<ClockGroup>,
}

#[derive(Debug, Default, Clone, Deserialize)]
struct ClockGroup {
    group: String,
    clocks: Vec<ClockTreeItem>,
}

pub(crate) struct ProcessedClockData {
    /// All instantiated clock tree nodes.
    clock_tree: IndexMap<String, ClockTreeNodeInstance>,

    /// Clock template groups and the instances of those groups.
    ///
    /// For each group, we generate an enum that lists the instances.
    group_instances: IndexMap<String, IndexSet<String>>,

    /// System clock graph.
    dependency_graph: DependencyGraph,
}

pub(crate) struct ClockTreeNodeInstance {
    node: Box<dyn ClockTreeNodeType>,

    include_in_global_config: bool,
    force_configurable: bool,

    /// Name of the instantiated clock tree node.
    ///
    /// Must be in CONSTANT_CASE, e.g. UART0_FUNCTION_CLOCK.
    name: String,

    /// Name of the group (e.g. peripheral) this tree node belongs to.
    ///
    /// Must be in CONSTANT_CASE, e.g. UART0.
    group_instance: String,

    /// Name of the template group used to instantiate this clock tree node.
    ///
    /// Must be in CONSTANT_CASE, e.g. UART.
    group_template: String,

    properties: ManagementProperties,
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

    fn config_type(&self) -> TokenStream {
        self.node.config_type(self)
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
        let type_name = if self.group_template.is_empty() {
            self.node.name().to_string()
        } else {
            format!("{}_{}", self.group_template, self.node.name())
        }
        .from_case(Case::Constant)
        .to_case(Case::Pascal);
        quote::format_ident!("{type_name}Config")
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

    fn prefix_function(&self, prefix: &str) -> Ident {
        if self.properties.receiver.is_some() {
            format_ident!(
                "{prefix}_{}",
                self.node
                    .name()
                    .from_case(Case::Constant)
                    .to_case(Case::Snake)
            )
        } else {
            let name = self.name().to_case(Case::Snake);
            format_ident!("{prefix}_{name}")
        }
    }

    fn suffix_function(&self, suffix: &str) -> Ident {
        if self.properties.receiver.is_some() {
            format_ident!(
                "{}_{suffix}",
                self.node
                    .name()
                    .from_case(Case::Constant)
                    .to_case(Case::Snake)
            )
        } else {
            let name = self.name().to_case(Case::Snake);
            format_ident!("{name}_{suffix}")
        }
    }

    fn enable_fn_name(&self) -> Ident {
        self.prefix_function("enable")
    }

    fn request_fn_name(&self) -> Ident {
        self.prefix_function("request")
    }

    fn release_fn_name(&self) -> Ident {
        self.prefix_function("release")
    }

    fn frequency_function_name(&self) -> Ident {
        self.suffix_function("frequency")
    }

    fn config_frequency_function_name(&self) -> Ident {
        self.suffix_function("config_frequency")
    }

    fn config_apply_function_name(&self) -> Ident {
        self.prefix_function("configure")
    }

    fn current_config_function_name(&self) -> Ident {
        self.suffix_function("config")
    }

    fn config_current_function(&self, _tree: &ProcessedClockData) -> TokenStream {
        if self.is_configurable() {
            let ty_name = self.config_type_name();
            let config_field = self.properties.config_accessor();
            let fn_name = self.current_config_function_name();
            let receiver = self.properties.receiver();
            quote! {
                pub fn #fn_name(#(#receiver,)* clocks: &mut ClockTree) -> Option<#ty_name> {
                    #config_field
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
        let enable_fn_name = self.enable_fn_name();
        let enable_fn_impl_name = format_ident!("{}_impl", enable_fn_name);
        let always_on = self.properties.always_on();

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

        let refcount_accessor = self.properties.refcount_accessor();
        let receiver = self.properties.receiver();
        ClockNodeFunctions {
            impl_type: if self.properties.receiver.is_some() {
                Some(format_ident!(
                    "{}Instance",
                    self.group_template
                        .from_case(Case::Constant)
                        .to_case(Case::Pascal)
                ))
            } else {
                None
            },
            request: Function {
                _name: request_fn_name.to_string(),
                implementation: if always_on {
                    quote! {
                        fn #request_fn_name(#(#receiver,)* _clocks: &mut ClockTree) { }
                    }
                } else if let Some(refcount_accessor) = &refcount_accessor {
                    quote! {
                        pub fn #request_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
                            trace!(#request_trace);
                            if increment_reference_count(&mut #refcount_accessor) {
                                trace!(#enable_trace);
                                #request_direct_dependencies
                                #(#receiver.)* #enable_fn_impl_name(clocks, true);
                            }
                        }
                    }
                } else if self.properties.has_enable() {
                    quote! {
                        pub fn #request_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
                            trace!(#request_trace);
                            trace!(#enable_trace);
                            #request_direct_dependencies
                            #(#receiver.)* #enable_fn_impl_name(clocks, true);
                        }
                    }
                } else {
                    quote! {
                        pub fn #request_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
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
                        fn #release_fn_name(#(#receiver,)* _clocks: &mut ClockTree) { }
                    }
                } else if let Some(refcount_accessor) = &refcount_accessor {
                    quote! {
                        pub fn #release_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
                            trace!(#release_trace);
                            if decrement_reference_count(&mut #refcount_accessor) {
                                trace!(#disable_trace);
                                #(#receiver.)* #enable_fn_impl_name(clocks, false);
                                #release_direct_dependencies
                            }
                        }
                    }
                } else if self.properties.has_enable() {
                    quote! {
                        pub fn #release_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
                            trace!(#release_trace);
                            trace!(#disable_trace);
                            #(#receiver.)* #enable_fn_impl_name(clocks, false);
                            #release_direct_dependencies
                        }
                    }
                } else {
                    quote! {
                        pub fn #release_fn_name(#(#receiver,)* clocks: &mut ClockTree) {
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
                    let config_field = self.properties.config_accessor();
                    quote! {
                        #[allow(unused_variables)]
                        pub fn #config_frequency_function_name(#(#receiver,)* clocks: &mut ClockTree, config: #ty_name) -> u32 {
                            #frequency_function_impl
                        }
                        pub fn #frequency_function_name(#(#receiver,)* clocks: &mut ClockTree) -> u32 {
                            if let Some(config) = #config_field {
                                #(#receiver.)* #config_frequency_function_name(clocks, config)
                            } else {
                                0
                            }
                        }
                    }
                } else {
                    quote! {
                        pub fn #frequency_function_name(#(#receiver,)* clocks: &mut ClockTree) -> u32 {
                            #frequency_function_impl
                        }
                    }
                },
            },

            hal_functions: vec![
                if !always_on && (refcount_accessor.is_some() || self.properties.has_enable()) {
                    quote! {
                        fn #enable_fn_impl_name(#(#receiver,)* _clocks: &mut ClockTree, _en: bool) {
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
        let receiver = self.properties.receiver();

        quote! {
            fn #hal_impl(#(#receiver,)* _clocks: &mut ClockTree, _old_config: Option<#ty_name>, _new_config: #ty_name) {
                todo!()
            }
        }
    }

    fn resolve_node<'tree>(
        &self,
        tree: &'tree ProcessedClockData,
        clock: &str,
    ) -> &'tree ClockTreeNodeInstance {
        let local_node = format!("{}_{}", self.group_instance, clock);
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
    fn node(&self, name: &str) -> &ClockTreeNodeInstance {
        self.try_get_node(name)
            .unwrap_or_else(|| panic!("Clock node {name} not found"))
    }

    /// Returns a node by its name (e.g. `XTAL_CLK`), or None if not found.
    fn try_get_node(&self, name: &str) -> Option<&ClockTreeNodeInstance> {
        self.clock_tree.get(name)
    }
}

impl SystemClocks {
    fn generate_macro(&self, tree: &ProcessedClockData) -> Result<TokenStream> {
        let mut clock_tree_node_defs = vec![];
        let mut clock_tree_node_impls = IndexMap::<_, Vec<_>>::new();
        let mut provided_function_doclines = IndexMap::<_, Vec<_>>::new();
        let mut clock_tree_node_state_getter_doclines = vec![];
        let mut clock_tree_state_funcs = vec![];
        let mut clock_tree_state_func_return_types = vec![];
        let mut clock_tree_state_field_names = vec![];
        let mut clock_tree_state_accessors = vec![];
        let mut clock_tree_state_field_types = vec![];
        let mut clock_tree_state_field_initializers = vec![];
        let mut clock_tree_refcount_field_decls = vec![];
        let mut clock_tree_refcount_field_inits = vec![];
        let mut configurables = vec![];
        let mut system_config_steps = HashMap::new();

        let mut first_instances = HashSet::new();
        let mut instance_enums = Vec::new();

        for (group_template, instances) in tree.group_instances.iter() {
            let enum_name = format_ident!(
                "{}Instance",
                group_template
                    .from_case(Case::Constant)
                    .to_case(Case::Pascal)
            );
            let variants = instances
                .iter()
                .map(|v| format_ident!("{}", v.from_case(Case::Constant).to_case(Case::Pascal)));
            let idxs = (0..).map(number);
            instance_enums.push(quote! {
                #[derive(Clone, Copy, PartialEq, Eq, Debug)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub enum #enum_name {
                    #(#variants = #idxs,)*
                }
            });
        }

        for clock_item in tree.clock_tree.values() {
            // Generate code for all clock tree nodes
            let is_first_instance = first_instances.insert(format!(
                "{}_{}",
                clock_item.group_template,
                clock_item.node.name()
            ));
            if is_first_instance {
                if clock_item.is_configurable() {
                    clock_tree_node_defs.push(clock_item.config_type());
                }

                let instance_count =
                    if let Some(instances) = tree.group_instances.get(&clock_item.group_template) {
                        Some(number(instances.len()))
                    } else {
                        None
                    };

                if let Some(refcount_field) = clock_item.properties.refcount_field() {
                    if let Some(instance_count) = instance_count.as_ref() {
                        clock_tree_refcount_field_decls
                            .push(quote! { #refcount_field: [u32; #instance_count] });
                        clock_tree_refcount_field_inits
                            .push(quote! { #refcount_field: [0; #instance_count] });
                    } else {
                        clock_tree_refcount_field_decls.push(quote! { #refcount_field: u32 });
                        clock_tree_refcount_field_inits.push(quote! { #refcount_field: 0 });
                    }
                }

                if let Some(type_name) = clock_item.properties.type_name() {
                    clock_tree_state_field_names.push(clock_item.properties.field_name());

                    if let Some(instance_count) = instance_count.as_ref() {
                        clock_tree_state_field_types.push(quote! { [#type_name; #instance_count] });
                        clock_tree_state_field_initializers
                            .push(quote! { [None; #instance_count] });
                    } else {
                        clock_tree_state_field_types.push(type_name);
                        clock_tree_state_field_initializers.push(quote! {None});
                    }
                }
            }

            if let Some(type_name) = clock_item.properties.type_name() {
                clock_tree_state_funcs.push(format_ident!(
                    "{}",
                    clock_item
                        .name
                        .from_case(Case::Constant)
                        .to_case(Case::Snake)
                ));
                clock_tree_state_func_return_types.push(type_name.clone());
                clock_tree_state_accessors.push(clock_item.properties.config_accessor_from("self"));

                clock_tree_node_state_getter_doclines.push(format!(
                    "Returns the current configuration of the {} clock tree node",
                    clock_item.name_str(),
                ));
            }

            let node_funcs = clock_item.node_functions(tree);
            if !node_funcs.hal_functions.is_empty() && is_first_instance {
                let mut doclines = vec![];
                let header = if clock_item.group_template.is_empty() {
                    format!(" // {}", clock_item.name_str())
                } else {
                    format!(
                        " // {}_{}",
                        clock_item.group_template,
                        clock_item.node.name()
                    )
                };
                doclines.push(quote! {
                    #[doc = ""]
                    #[doc = #header]
                    #[doc = ""]
                });
                for func in node_funcs.hal_functions.iter() {
                    if func.is_empty() {
                        continue;
                    }
                    let func = func.to_string();
                    doclines.push(quote! {
                        #[doc = #func]
                        #[doc = ""] // empty line between functions
                    });
                }
                provided_function_doclines
                    .entry(node_funcs.impl_type.clone())
                    .or_default()
                    .extend_from_slice(&doclines);
            }

            if is_first_instance {
                clock_tree_node_impls
                    .entry(node_funcs.impl_type.clone())
                    .or_default()
                    .push(node_funcs);
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

        let clock_tree_node_impls = clock_tree_node_impls
            .into_iter()
            .map(|(impl_type, node_funcs)| {
                let impl_block = node_funcs.iter().map(|func| func.implement_functions());
                if let Some(impl_type) = impl_type {
                    quote! {
                        impl #impl_type {
                            #(#impl_block)*
                        }
                    }
                } else {
                    quote! {
                        #(#impl_block)*
                    }
                }
            })
            .collect::<Vec<_>>();

        let provided_function_doclines = provided_function_doclines
            .into_iter()
            .map(|(impl_type, doclines)| {
                if let Some(impl_type) = impl_type {
                    let impl_type_line = format!("impl {impl_type} {{");
                    quote! {
                        #[doc = #impl_type_line]
                            #(#doclines)*
                        #[doc = "}"]
                    }
                } else {
                    quote! {
                        #(#doclines)*
                    }
                }
            })
            .collect::<Vec<_>>();

        Ok(quote! {
            #[macro_export]
            /// ESP-HAL must provide implementation for the following functions:
            /// ```rust, no_run
            #(#provided_function_doclines)*
            /// ```
            macro_rules! define_clock_tree_types {
                () => {
                    #(#instance_enums)*
                    #(#clock_tree_node_defs)*

                    /// Represents the device's clock tree.
                    pub struct ClockTree {
                        #(#clock_tree_state_field_names: #clock_tree_state_field_types,)*
                        #(#clock_tree_refcount_field_decls,)*
                    }
                    impl ClockTree {
                        /// Locks the clock tree for exclusive access.
                        pub fn with<R>(f: impl FnOnce(&mut ClockTree) -> R) -> R {
                            CLOCK_TREE.with(f)
                        }

                        #(
                            #[doc = #clock_tree_node_state_getter_doclines]
                            pub fn #clock_tree_state_funcs(&self) -> #clock_tree_state_func_return_types {
                                #clock_tree_state_accessors
                            }
                        )*
                    }

                    static CLOCK_TREE: ::esp_sync::NonReentrantMutex<ClockTree> =
                        ::esp_sync::NonReentrantMutex::new(ClockTree {
                            #(#clock_tree_state_field_names: #clock_tree_state_field_initializers,)*
                            #(#clock_tree_refcount_field_inits,)*
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

        match TokenStream::from_str(&template) {
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
#[serde(deny_unknown_fields)]
pub struct DeviceClocks {
    #[serde(default)]
    pub(crate) system_clocks: SystemClocks,

    #[serde(default)]
    pub(crate) peripheral_clocks: PeripheralClocks,
}

impl DeviceClocks {
    /// Returns an iterator over the clock tree items.
    fn process(&self, config: &SocConfig) -> Result<ProcessedClockData> {
        let mut clock_tree = self
            .system_clocks
            .clock_tree
            .iter()
            .map(|node| {
                let node_name = node.name();
                let name = node.name().to_string();
                let node = ClockTreeNodeInstance {
                    node: node.boxed(),
                    include_in_global_config: true,
                    force_configurable: false,
                    name: name.clone(),
                    group_instance: String::new(),
                    group_template: String::new(),
                    properties: ManagementProperties {
                        name: format_ident!(
                            "{}",
                            name.from_case(Case::Constant).to_case(Case::Snake)
                        ),
                        state_ty: None,
                        refcounted: false,
                        has_enable: false,
                        always_on: false,
                        receiver: None,
                        accessor: None, // System clocks are never collected in an array
                    },
                };

                (node_name.to_string(), RefCell::new(node))
            })
            .collect::<IndexMap<_, _>>();

        for peri in config.peripherals.iter() {
            let Some(group_name) = peri.clock_group.as_ref() else {
                // Peripheral doesn't have a clock tree entry.
                continue;
            };

            // Instantiate template group
            let peri_name = peri.name.from_case(Case::Ada).to_case(Case::Constant);

            // A peripheral can have any number of clock sources. We'll turn them into clock tree
            // nodes here.
            for def in self
                .system_clocks
                .template_groups
                .iter()
                .find(|g| g.group == *group_name)
                .ok_or_else(|| anyhow::anyhow!("Clock group {group_name} not found"))?
                .clocks
                .iter()
            {
                let name = format!("{peri_name}_{}", def.name());
                let instance_ty = format_ident!(
                    "{}Instance",
                    group_name.from_case(Case::Constant).to_case(Case::Pascal)
                );
                let instance_variant = format_ident!(
                    "{}",
                    peri_name.from_case(Case::Constant).to_case(Case::Pascal)
                );
                let node = ClockTreeNodeInstance {
                    node: def.boxed(),
                    include_in_global_config: false,
                    // FIXME peripherals force configurability because we don't have a way to
                    // cfg them out. Decide if we can do better.
                    force_configurable: true,
                    name: name.clone(),
                    group_instance: peri_name.clone(),
                    group_template: group_name.clone(),
                    properties: ManagementProperties {
                        name: format_ident!(
                            "{}",
                            format!("{group_name}_{}", def.name())
                                .from_case(Case::Constant)
                                .to_case(Case::Snake)
                        ),
                        state_ty: None,
                        refcounted: false,
                        has_enable: false,
                        always_on: false,
                        receiver: Some(quote! { self }),
                        accessor: Some(quote! { #instance_ty::#instance_variant as usize }),
                    },
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
            let mut sorted = IndexMap::with_capacity(clock_tree.len());
            for idx in sorted_clocks {
                let node = clock_tree[idx].take().unwrap();
                sorted.insert(node.name.clone(), node);
            }
            sorted
        };

        let mut tree = ProcessedClockData {
            clock_tree,
            group_instances: IndexMap::new(),
            dependency_graph: DependencyGraph::empty(),
        };

        // To compute refcount requirement and correct initialization order, we need to be able to
        // access direct dependencies (downstream clocks). As it is simpler to define
        // dependents (inputs), we have to do a bit of maths.
        tree.dependency_graph = DependencyGraph::build_from(&tree);

        for node in tree.clock_tree.values_mut() {
            // If there's a single dependent clock, we can piggyback on its refcount.
            // Note that this is only valid if the clock node is not expected to be manually
            // managed. In the current model, manually managed clocks have 0 consumers.
            let has_one_direct_dependent = tree.dependency_graph.users(node.name_str()).len() == 1;

            // Peripheral nodes are always refcounted, because we can't assume which node is
            // referred to by the drivers.
            let is_peripheral_node = !node.group_instance.is_empty();

            node.properties.refcounted =
                is_peripheral_node || !(node.always_on() || has_one_direct_dependent);

            // Always-on clock sources don't need enable functions.
            node.properties.has_enable =
                !(node.always_on() && tree.dependency_graph.inputs(node.name_str()).is_empty());
            node.properties.state_ty = if node.is_configurable() {
                let type_name = node.config_type_name();
                Some(quote! { Option<#type_name> })
            } else {
                None
            };
            node.properties.always_on = node.always_on();

            if !node.group_template.is_empty() {
                tree.group_instances
                    .entry(node.group_template.clone())
                    .or_default()
                    .insert(node.group_instance.clone());
            }
        }

        Ok(tree)
    }

    fn cfgs(&self, config: &SocConfig) -> Vec<String> {
        let processed_clocks = self.process(config).unwrap();

        // TODO: output per-template cfgs once we have a single API per template group instead of
        // per instance
        processed_clocks
            .clock_tree
            .values()
            .map(|node| node.name().to_case(Case::Snake))
            .map(|name| format!("soc_has_clock_node_{name}"))
            .collect::<Vec<_>>()
    }

    fn macros(&self, config: &SocConfig) -> TokenStream {
        let processed_clocks = self.process(config).unwrap();

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

        quote! {
            #system_clocks
            #peripheral_clocks
        }
    }
}
