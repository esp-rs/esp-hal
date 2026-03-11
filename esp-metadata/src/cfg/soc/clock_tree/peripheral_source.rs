use convert_case::{Case, Casing};
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

use crate::cfg::{
    clock_tree::{ClockTreeNodeType, ValidationContext, mux::Multiplexer},
    soc::ProcessedClockData,
};

#[derive(Debug, Clone)]
pub(crate) struct PeripheralClockSource {
    pub peripheral: String,
    pub template: String,
    pub is_definition: bool,
    pub mux: Multiplexer,
}

impl ClockTreeNodeType for PeripheralClockSource {
    fn validate_source_data(&self, ctx: &ValidationContext<'_>) -> anyhow::Result<()> {
        self.mux.validate_source_data(ctx)
    }

    fn input_clocks(&self) -> Vec<String> {
        self.mux.input_clocks()
    }

    fn name_str<'a>(&'a self) -> &'a String {
        &self.peripheral
    }

    fn config_type_name(&self) -> Option<Ident> {
        if self.is_configurable() {
            let item = self
                .template
                .from_case(Case::Constant)
                .to_case(Case::Pascal);
            Some(quote::format_ident!("{}Config", item))
        } else {
            None
        }
    }

    fn is_configurable(&self) -> bool {
        true
    }

    fn config_apply_function(&self, tree: &ProcessedClockData) -> TokenStream {
        self.mux.impl_config_apply_function(self, tree)
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
        self.mux.node_frequency_impl2(self, tree)
    }

    fn config_type(&self) -> Option<TokenStream> {
        if !self.is_definition {
            return None;
        }
        let ty_name = self.config_type_name()?;

        Some(self.mux.impl_config_type(ty_name))
    }

    fn config_docline(&self) -> Option<String> {
        let clock_name = self.peripheral.as_str();
        Some(format!(
            " The list of clock signals that the `{clock_name}` multiplexer can output."
        ))
    }

    fn request_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.request_direct_dependencies(node, tree)
    }

    fn release_direct_dependencies(
        &self,
        node: &dyn ClockTreeNodeType,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.mux.release_direct_dependencies(node, tree)
    }
}
