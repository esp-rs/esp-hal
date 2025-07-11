use proc_macro2::TokenStream;
use quote::{format_ident, quote};

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
            quote! {
                #instance, #sys, #sclk, #mosi, #miso, #cs
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("spi_slave", &[("all", &instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the SPI slave driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $sclk:ident, $mosi:ident, $miso:ident, $cs:ident)`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the I2C instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$mosi`, `$miso`, `$cs`: signal names.
        ///
        /// Example data: `(SPI2, Spi2, FSPICLK, FSPID, FSPIQ, FSPICS0)`
        #for_each
    }
}
