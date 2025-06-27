use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::I2cMasterProperties, generate_for_each_macro};

/// Instance configuration, used in [device.i2c_master.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct I2cMasterInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,

    /// IOMUX signal name of the instane's SCL signal.
    pub scl: String,

    /// IOMUX signal name of the instane's SDA signal.
    pub sda: String,

    /// The name of the instance's interrupt handler.
    pub interrupt: String,
}

/// Generates `for_each_i2c_master!` which can be used to implement the I2C
/// master Instance trait for the relevant peripherals. The macro generates code
/// for each [device.i2c_master.instances[X]] instance.
pub(crate) fn generate_i2c_master_peripehrals(i2c: &I2cMasterProperties) -> TokenStream {
    let i2c_master_instance_cfgs = i2c
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let sda = format_ident!("{}", instance_config.sda);
            let scl = format_ident!("{}", instance_config.scl);
            let int = format_ident!("{}", instance_config.interrupt);

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2c_master!` call.
            quote::quote! {
                #instance, #sys, #scl, #sda, #int
            }
        })
        .collect::<Vec<_>>();

    generate_for_each_macro("i2c_master", &i2c_master_instance_cfgs)
}
