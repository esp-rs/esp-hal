use std::{collections::HashMap, ops::Range, str::FromStr};

use anyhow::{Context, Result};
use convert_case::{Boundary, Case, Casing, pattern};
use indexmap::IndexMap;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{
    cfg::{
        clock_tree::{
            ClockTreeItem,
            ClockTreeNodeType,
            DependencyGraph,
            ManagementProperties,
            PeripheralClockSource,
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
    /// For each clock tree node, this map stores which of them are fixed, configurable, or
    /// dependent on other nodes.
    classified_clocks: IndexMap<String, ClockType>,

    /// The device's `system_clocks` data.
    clock_tree: Vec<Box<dyn ClockTreeNodeType>>,

    /// Refcount/config type properties of the clock tree nodes.
    management_properties: HashMap<String, ManagementProperties>,

    /// System clock graph.
    dependency_graph: DependencyGraph,
}

impl ProcessedClockData {
    /// Returns a node by its name (e.g. `XTAL_CLK`).
    ///
    /// As the clock tree is stored as a vector, this method performs a linear search.
    fn node(&self, name: &str) -> &dyn ClockTreeNodeType {
        self.clock_tree
            .iter()
            .find(|item| item.name_str() == name)
            .map(|b| b.as_ref())
            .unwrap_or_else(|| panic!("Clock node {} not found", name))
    }

    fn properties(&self, node: &dyn ClockTreeNodeType) -> &ManagementProperties {
        self.management_properties
            .get(node.name_str())
            .unwrap_or_else(|| panic!("Management properties for {} not found", node.name_str()))
    }
}

#[derive(Clone, Debug, PartialEq)]
enum ClockType {
    /// The clock tree item is not configurable.
    Fixed,

    /// The clock tree item is configurable.
    Configurable,

    /// The clock tree item is configured by some other item.
    Dependent(String),

    /// Peripheral clock source - may be Configurable, but shouldn't be exposed via the global clock
    /// tree config.
    Peripheral,
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
        for (item, kind) in tree
            .classified_clocks
            .iter()
            .map(|(item, kind)| (item, kind.clone()))
        {
            let clock_item = tree.node(item);

            // Generate code for all clock tree nodes
            if let Some(config_type) = clock_item.config_type() {
                let doclines = clock_item.config_docline().unwrap();
                let doclines = doclines.lines();

                clock_tree_node_defs.push(quote! {
                    #(#[doc = #doclines])*
                    #config_type
                });
            }
            let node_state = tree.properties(clock_item);
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

            let node_funcs = ClockTreeItem::node_functions(clock_item, tree);
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

            if kind == ClockType::Configurable {
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
                let boxed: Box<dyn ClockTreeNodeType> = match node {
                    ClockTreeItem::Multiplexer(inner) => Box::new(inner.clone()),
                    ClockTreeItem::Source(inner) => Box::new(inner.clone()),
                    ClockTreeItem::Divider(inner) => Box::new(inner.clone()),
                    ClockTreeItem::Derived(inner) => Box::new(inner.clone()),
                };
                boxed
            })
            .collect::<Vec<_>>();

        let validation_context = ValidationContext {
            tree: self.system_clocks.clock_tree.as_slice(),
        };
        for node in clock_tree.iter() {
            node.validate_source_data(&validation_context)
                .with_context(|| format!("Invalid clock tree item: {}", node.name_str()))?;
        }

        let mut classified_clocks = IndexMap::new();
        let mut management_properties = HashMap::new();

        // Merge peripheral clock sources into the clock tree
        let mut peri_clocks = self.peripheral_clocks.peripheral_clocks.clone();
        peri_clocks.sort_by(|a, b| a.name.cmp(&b.name));

        for peri in peri_clocks {
            // A peripheral can have any number of clock sources. We'll turn them into clock tree
            // nodes here.
            let template_peripheral = peri.template_peripheral_name();
            for def in peri.clock_signals(&self.peripheral_clocks) {
                // A fake node because we need to prefix the peripheral name, and the
                // behaviour differs slightly from the standard mutex (we generate a type
                // for all variant counts for now).
                let node = match def {
                    ClockTreeItem::Multiplexer(mux) => PeripheralClockSource {
                        peripheral: format!(
                            "{}_{}",
                            peri.name.from_case(Case::Ada).to_case(Case::Constant),
                            mux.name_str()
                        ),
                        is_definition: matches!(
                            peri.clocks,
                            PeripheralClockTreeEntry::Definition(_)
                        ),
                        template: format!(
                            "{}_{}",
                            template_peripheral
                                .from_case(Case::Ada)
                                .to_case(Case::Constant),
                            mux.name_str()
                        ),
                        mux: mux.clone(),
                    },
                    _ => anyhow::bail!("only muxes are supported as clock source data"),
                };

                node.validate_source_data(&validation_context)
                    .with_context(|| format!("Invalid clock tree item: {}", node.name_str()))?;

                clock_tree.push(Box::new(node));
            }
        }

        // To compute refcount requirement and correct initialization order, we need to be able to
        // access direct dependencies (downstream clocks). As it is simpler to define
        // dependents (inputs), we have to do a bit of maths.
        let dependency_graph = DependencyGraph::build_from(&clock_tree);

        // Classify clock tree items
        for node in clock_tree.iter() {
            // If item A configures item B, then item B is a dependent clock tree item.
            for clock in node.affected_nodes() {
                classified_clocks.insert(
                    clock.to_string(),
                    ClockType::Dependent(node.name_str().clone()),
                );
            }

            // Each tree item tells its own clock type, except for dependent items. If something has
            // already been classified, we can only change it from its kind to dependent.
            if classified_clocks.get(node.name_str().as_str()).is_none() {
                classified_clocks.insert(
                    node.name_str().clone(),
                    if (node.as_ref() as &dyn std::any::Any).is::<PeripheralClockSource>() {
                        ClockType::Peripheral
                    } else if node.is_configurable() {
                        ClockType::Configurable
                    } else {
                        ClockType::Fixed
                    },
                );
            }

            // If there's a single dependent clock, we can piggyback on its refcount.
            // Note that this is only valid if the clock node is not expected to be manually
            // managed. In the current model, manually managed clocks have 0 consumers.
            let has_one_direct_dependent = dependency_graph.users(node.name_str()).len() == 1;

            let refcounted = !(node.always_on() || has_one_direct_dependent);

            management_properties.insert(
                node.name_str().clone(),
                ManagementProperties {
                    name: format_ident!("{}", node.name().to_case(Case::Snake)),
                    refcounted,
                    // Always-on clock sources don't need enable functions.
                    has_enable: !(node.always_on()
                        && dependency_graph.inputs(node.name_str()).is_empty()),
                    state_ty: node.config_type_name(),
                    always_on: node.always_on(),
                },
            );
        }

        Ok(ProcessedClockData {
            clock_tree,
            classified_clocks,
            management_properties,
            dependency_graph,
        })
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
