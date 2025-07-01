use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::SpiSlaveProperties, generate_for_each_macro};

/// Instance configuration, used in [device.spi_slave.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct SpiSlaveInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,
    // IOMUX signal names of the instance's signals:
    pub sclk: String,
    pub mosi: String,
    pub miso: String,
    pub cs: String,
}

/// Generates `for_each_spi_slave!` which can be used to implement the SPI
/// slave Instance trait for the relevant peripherals. The macro generates code
/// for each [device.spi_slave.instances[X]] instance.
pub(crate) fn generate_spi_slave_peripherals(spi_slave: &SpiSlaveProperties) -> TokenStream {
    let instance_cfgs = spi_slave
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let sclk = format_ident!("{}", instance_config.sclk);
            let mosi = format_ident!("{}", instance_config.mosi);
            let miso = format_ident!("{}", instance_config.miso);
            let cs = format_ident!("{}", instance_config.cs);

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2c_master!` call.
            quote::quote! {
                #instance, #sys, #sclk, #mosi, #miso, #cs
            }
        })
        .collect::<Vec<_>>();

    generate_for_each_macro("spi_slave", &instance_cfgs)
}
