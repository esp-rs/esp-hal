use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::SpiMasterProperties, generate_for_each_macro};

/// Instance configuration, used in [device.spi_slave.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct SpiMasterInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,
    // IOMUX signal names of the instance's signals:
    pub sclk: String,
    pub sio: Vec<String>,
    pub cs: Vec<String>,
}

/// Generates `for_each_spi_slave!` which can be used to implement the SPI
/// master Instance trait for the relevant peripherals. The macro generates code
/// for each [device.spi_master.instances[X]] instance.
pub(crate) fn generate_spi_master_peripherals(spi_slave: &SpiMasterProperties) -> TokenStream {
    let instance_cfgs = spi_slave
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let sclk = format_ident!("{}", instance_config.sclk);
            let cs = instance_config.cs.iter().map(|cs| format_ident!("{cs}"));
            let sio = instance_config.sio.iter().map(|cs| format_ident!("{cs}"));

            let is_qspi = if instance_config.sio.len() > 2 {
                quote::quote! { , true }
            } else {
                quote::quote! {}
            };

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2c_master!` call.
            quote::quote! {
                #instance, #sys, #sclk [#(#cs),*] [#(#sio),*] #is_qspi
            }
        })
        .collect::<Vec<_>>();

    generate_for_each_macro("spi_master", &instance_cfgs)
}
