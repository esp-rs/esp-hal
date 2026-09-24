use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::CanFdProperties, generate_for_each_macro};

/// Instance configuration, used in [device.canfd.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CanFdInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,

    /// IOMUX signal name of the instance's RX signal.
    pub rx: String,

    /// IOMUX signal name of the instance's TX signal.
    pub tx: String,
}

/// Generates `for_each_canfd!` which can be used to implement the CAN FD
/// Instance trait for the relevant peripherals. The macro generates code
/// for each [device.canfd.instances[X]] instance.
pub(crate) fn generate_canfd_peripherals(canfd: &CanFdProperties) -> TokenStream {
    let instance_cfgs = canfd
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let rx = format_ident!("{}", instance_config.rx);
            let tx = format_ident!("{}", instance_config.tx);

            // The order and meaning of these tokens must match their use in the
            // `for_each_canfd!` call.
            quote! {
                #instance, #sys, #rx, #tx
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("canfd", &[("all", &instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the CAN FD driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $rx:ident, $tx:ident)`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the CAN FD instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$rx`, `$tx`: signal names.
        ///
        /// Example data: `(TWAI0, Twai0, TWAI0_RX, TWAI0_TX)`
        #for_each
    }
}
