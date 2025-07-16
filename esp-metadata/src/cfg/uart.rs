use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::UartProperties, generate_for_each_macro};

/// Instance configuration, used in [device.uart.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct UartInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,

    /// IOMUX signal name of the instance's RX signal.
    pub rx: String,

    /// IOMUX signal name of the instance's TX signal.
    pub tx: String,

    /// IOMUX signal name of the instance's CTS signal.
    pub cts: String,

    /// IOMUX signal name of the instance's RTS signal.
    pub rts: String,
}

/// Generates `for_each_uart!` which can be used to implement the UART
/// Instance trait for the relevant peripherals. The macro generates code
/// for each [device.uart.instances[X]] instance.
pub(crate) fn generate_uart_peripherals(uart: &UartProperties) -> TokenStream {
    let uart_instance_cfgs = uart
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            let rx = format_ident!("{}", instance_config.rx);
            let tx = format_ident!("{}", instance_config.tx);
            let cts = format_ident!("{}", instance_config.cts);
            let rts = format_ident!("{}", instance_config.rts);

            // The order and meaning of these tokens must match their use in the
            // `for_each_uart!` call.
            quote! {
                #instance, #sys, #rx, #tx, #cts, #rts
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("uart", &[("all", &uart_instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the UART driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $rx:ident, $tx:ident, $cts:ident, $rts:ident)`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the UART instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$rx`, `$tx`, `$cts`, `$rts`: signal names.
        ///
        /// Example data: `(UART0, Uart0, U0RXD, U0TXD, U0CTS, U0RTS)`
        #for_each
    }
}
