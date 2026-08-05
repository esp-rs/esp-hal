use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::McpwmProperties, generate_for_each_macro};

/// Instance configuration, used in [device.mcpwm.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct McpwmInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,
}

/// Generates `for_each_mcpwm!` which can be used to implement the MCPWM
/// Instance trait for the relevant peripherals. The macro generates code
/// for each [device.mcpwm.instances[X]] instance.
pub(crate) fn generate_mcpwm_peripherals(mcpwm: &McpwmProperties) -> TokenStream {
    let mcpwm_instance_cfgs = mcpwm
        .instances
        .iter()
        .enumerate()
        .map(|(index, instance)| {
            let instance_config = &instance.instance_config;

            let id = crate::number(index);

            let instance = format_ident!("{}", instance.name.to_uppercase());
            let sys = format_ident!("{}", instance_config.sys_instance);

            // The order and meaning of these tokens must match their use in the
            // `for_each_mcpwm!` call.
            quote! {
                #id, #instance, #sys
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("mcpwm", &[("all", &mcpwm_instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the MCPWM driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($id:literal, $instance:ident, $sys:ident)`
        ///
        /// Macro fragments:
        ///
        /// - `$id`: the index of the MCPWM instance
        /// - `$instance`: the name of the MCPWM instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        ///
        /// Example data: `(0, MCPWM0, Mcpwm0)`
        #for_each
    }
}
