use proc_macro2::TokenStream;

use crate::cfg::{
    ClockTreeNodeInstance,
    clock_tree::{ClockTreeNodeType, ValidationContext, mux::Multiplexer},
    soc::ProcessedClockData,
};

#[derive(Debug, Clone)]
pub(crate) struct PeripheralClockSource {
    pub peripheral: String,
    pub template: String,
    pub mux: Multiplexer,
}

impl ClockTreeNodeType for PeripheralClockSource {
    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> anyhow::Result<()> {
        self.mux.validate_source_data(ctx)
    }

    fn input_clocks(&self) -> Vec<String> {
        self.mux.input_clocks()
    }

    fn is_configurable(&self) -> bool {
        self.mux.is_configurable()
    }

    fn config_apply_function(
        &self,
        node: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.impl_config_apply_function(node, tree)
    }

    fn config_apply_impl_function(
        &self,
        node: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.config_apply_impl_function(node, tree)
    }

    fn node_frequency_impl(
        &self,
        node: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.node_frequency_impl2(node, tree)
    }

    fn config_type(&self, node: &ClockTreeNodeInstance) -> TokenStream {
        self.mux.config_type(node)
    }

    fn config_docline(&self, node: &ClockTreeNodeInstance) -> Option<String> {
        self.mux.config_docline(node)
        //        let clock_name = self.peripheral.as_str();
        //        Some(format!(
        //            " The list of clock signals that the `{clock_name}` multiplexer can output."
        //        ))
    }

    fn request_direct_dependencies(
        &self,
        node: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.request_direct_dependencies(node, tree)
    }

    fn release_direct_dependencies(
        &self,
        node: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.release_direct_dependencies(node, tree)
    }
}
