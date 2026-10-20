use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{
    cfg::{I2sProperties, Value},
    generate_for_each_macro,
};

impl I2sProperties {
    /// Derives the chip-level PDM support flags from the per-instance
    /// configuration. A chip supports a given PDM capability if any of its
    /// instances does.
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        let supports_pdm_tx = self.instances.iter().any(|i| i.instance_config.pdm_tx);
        let supports_pdm_rx = self.instances.iter().any(|i| i.instance_config.pdm_rx);
        let supports_pcm2pdm = self.instances.iter().any(|i| i.instance_config.pcm2pdm);
        let supports_pdm2pcm = self.instances.iter().any(|i| i.instance_config.pdm2pcm);

        [
            (
                "i2s.supports_pdm_tx",
                false,
                Value::Boolean(supports_pdm_tx),
            ),
            (
                "i2s.supports_pdm_rx",
                false,
                Value::Boolean(supports_pdm_rx),
            ),
            (
                "i2s.supports_pcm2pdm",
                false,
                Value::Boolean(supports_pcm2pdm),
            ),
            (
                "i2s.supports_pdm2pcm",
                false,
                Value::Boolean(supports_pdm2pcm),
            ),
        ]
        .into_iter()
    }
}

/// Instance configuration, used in [device.i2s.instances]
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct I2sInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum
    pub sys_instance: String,
    /// IOMUX signal name of the instance's MCLK output.
    ///
    /// Not set on chips where the master clock is not routed through the GPIO
    /// matrix (ESP32).
    #[serde(default)]
    pub mclk: Option<String>,
    // IOMUX signal names of the instance's signals:
    pub bclk: String,
    pub ws: String,
    pub bclk_rx: String,
    pub ws_rx: String,
    pub dout: Vec<String>,
    pub din: Vec<String>,
    /// Whether this instance supports PDM TX mode.
    #[serde(default)]
    pub pdm_tx: bool,
    /// Whether this instance supports PDM RX mode.
    #[serde(default)]
    pub pdm_rx: bool,
    /// Whether this instance supports the hardware PCM-to-PDM filter on TX.
    #[serde(default)]
    pub pcm2pdm: bool,
    /// Whether this instance supports the hardware PDM-to-PCM filter on RX.
    #[serde(default)]
    pub pdm2pcm: bool,
}

/// Generates `for_each_i2s!` which can be used to implement the I2S Instance
/// trait for the relevant peripherals. The macro generates code for each
/// [device.i2s.instances[X]] instance.
pub(crate) fn generate_i2s_peripherals(i2s: &I2sProperties) -> TokenStream {
    let names = i2s
        .instances
        .iter()
        .map(|instance| {
            let instance = format_ident!("{}", instance.name.to_uppercase());
            quote! { #instance }
        })
        .collect::<Vec<_>>();

    let instance_cfgs = i2s
        .instances
        .iter()
        .map(|instance| {
            let instance_config = &instance.instance_config;

            let instance = format_ident!("{}", instance.name.to_uppercase());

            let sys = format_ident!("{}", instance_config.sys_instance);
            // On chips without a GPIO-matrix MCLK signal (ESP32) the driver conditionally
            // compiles out the field that uses this token, so the placeholder is never
            // resolved to an actual signal.
            let mclk = format_ident!("{}", instance_config.mclk.as_deref().unwrap_or("__NO_MCLK"));
            let bclk = format_ident!("{}", instance_config.bclk);
            let ws = format_ident!("{}", instance_config.ws);
            let bclk_rx = format_ident!("{}", instance_config.bclk_rx);
            let ws_rx = format_ident!("{}", instance_config.ws_rx);
            let dout = instance_config.dout.iter().map(|s| format_ident!("{s}"));
            let din = instance_config.din.iter().map(|s| format_ident!("{s}"));
            let pdm_tx = instance_config.pdm_tx;
            let pdm_rx = instance_config.pdm_rx;
            let pcm2pdm = instance_config.pcm2pdm;
            let pdm2pcm = instance_config.pdm2pcm;

            // The order and meaning of these tokens must match their use in the
            // `for_each_i2s!` call.
            quote! {
                #instance, #sys, #mclk, #bclk, #ws, #bclk_rx, #ws_rx, [#(#dout),*], [#(#din),*], #pdm_tx, #pdm_rx, #pcm2pdm, #pdm2pcm
            }
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro("i2s", &[("names", &names), ("all", &instance_cfgs)]);
    quote! {
        /// This macro can be used to generate code for each peripheral instance of the I2S driver.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has one option for its "Individual matcher" case:
        ///
        /// Syntax: `($instance:ident, $sys:ident, $mclk:ident, $bclk:ident, $ws:ident, $bclk_rx:ident, $ws_rx:ident, [$($dout:ident),*], [$($din:ident),*], $pdm_tx:literal, $pdm_rx:literal, $pcm2pdm:literal, $pdm2pcm:literal)`
        ///
        /// Macro fragments:
        ///
        /// - `$instance`: the name of the I2S instance
        /// - `$sys`: the name of the instance as it is in the `esp_hal::system::Peripheral` enum.
        /// - `$mclk`: the MCLK output signal (a placeholder on chips without a GPIO-matrix MCLK).
        /// - `$bclk`, `$ws`: TX bit clock and word select output signals.
        /// - `$bclk_rx`, `$ws_rx`: RX bit clock and word select output signals.
        /// - `$dout`, `$din`: data-out and data-in signal names.
        /// - `$pdm_tx`, `$pdm_rx`: whether the instance supports PDM TX/RX.
        /// - `$pcm2pdm`, `$pdm2pcm`: whether the instance supports the hardware PCM<->PDM format conversion filter on TX/RX.
        ///
        /// Example data:
        /// - `(I2S0, I2s0, I2S_MCLK, I2SO_BCK, I2SO_WS, I2SI_BCK, I2SI_WS, [I2SO_SD], [I2SI_SD], true, true, true, true)`
        #for_each
    }
}
