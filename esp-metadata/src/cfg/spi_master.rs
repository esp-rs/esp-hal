use proc_macro2::TokenStream;
use quote::{format_ident, quote};

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
                quote! { , true }
            } else {
                quote! {}
            };

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2c_master!` call.
            quote! {
                #instance, #sys, #sclk [#(#cs),*] [#(#sio),*] #is_qspi
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("spi_master", &[("all", &instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the SPI master driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $sclk:ident, [$($cs:ident),*] [$($sio:ident),* $($is_qspi:iteral)?])`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the SPI instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$cs`, `$sio`: chip select and SIO signal names.
        /// - `$is_qspi`: a `true` literal present if the SPI instance supports QSPI.
        ///
        /// Example data:
        /// - `(SPI2, Spi2, FSPICLK [FSPICS0, FSPICS1, FSPICS2, FSPICS3, FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD, FSPIIO4, FSPIIO5, FSPIIO6, FSPIIO7], true)`
        /// - `(SPI3, Spi3, SPI3_CLK [SPI3_CS0, SPI3_CS1, SPI3_CS2] [SPI3_D, SPI3_Q])`
        #for_each
    }
}
