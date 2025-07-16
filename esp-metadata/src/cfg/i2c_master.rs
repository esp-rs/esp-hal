use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::I2cMasterProperties, generate_for_each_macro};

/// Instance configuration, used in [device.i2c_master.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct I2cMasterInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,

    /// IOMUX signal name of the instance's SCL signal.
    pub scl: String,

    /// IOMUX signal name of the instance's SDA signal.
    pub sda: String,
}

/// Generates `for_each_i2c_master!` which can be used to implement the I2C
/// master Instance trait for the relevant peripherals. The macro generates code
/// for each [device.i2c_master.instances[X]] instance.
pub(crate) fn generate_i2c_master_peripherals(i2c: &I2cMasterProperties) -> TokenStream {
    let i2c_master_instance_cfgs = i2c
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let sda = format_ident!("{}", instance_config.sda);
            let scl = format_ident!("{}", instance_config.scl);

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2c_master!` call.
            quote! {
                #instance, #sys, #scl, #sda
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("i2c_master", &[("all", &i2c_master_instance_cfgs)]);

    quote! {
        /// This macro can be used to generate code for each peripheral instance of the I2C master driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $scl:ident, $sda:ident)`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the I2C instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$scl`, `$sda`: peripheral signal names.
        ///
        /// Example data: `(I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA)`
        #for_each
    }
}
