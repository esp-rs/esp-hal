use std::collections::BTreeMap;

use convert_case::{Case, Casing};
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

/// A single DMA channel definition.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaChannelDef {
    /// The name of the peripheral singleton for this channel.
    pub name: String,
    /// Whether this channel supports memory-to-memory transfers.
    #[serde(default)]
    #[expect(unused)]
    pub mem2mem: bool,
    /// PAC peripheral type backing this channel's register block.
    /// When absent the singleton is virtual (GDMA channels).
    /// When present the singleton maps to the given PAC type (PDMA channels).
    #[serde(default)]
    pub pac: Option<String>,
    /// PDMA only: peripheral names this channel is wired to.
    /// Non-empty means the channel is restricted to those peripherals.
    /// Empty (the default, and always the case for GDMA) means compatible with all.
    #[serde(default)]
    pub compatible_with: Vec<String>,
    /// Shared interrupt name (set when the channel has one interrupt for both directions).
    #[serde(default)]
    pub interrupt: Option<String>,
    /// RX/IN direction interrupt name (set when the channel has separate in/out interrupts).
    #[serde(default)]
    pub interrupt_in: Option<String>,
    /// TX/OUT direction interrupt name (set when the channel has separate in/out interrupts).
    #[serde(default)]
    pub interrupt_out: Option<String>,
}

/// A peripheral instance that can use a DMA engine.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaPeripheralInstance {
    /// The peripheral singleton name (e.g. `"SPI2"`, `"AES"`).
    pub name: String,
    /// The DMA peripheral selector ID (the old `dma_peripheral` value).
    pub dma_id: u32,
}

/// A single DMA engine with its channels.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaEngineDef {
    /// The name of the engine (e.g. `"AHB_GDMA"`, `"SPI_DMA"`, `"I2S_DMA"`).
    pub name: String,
    /// Driver config names whose peripherals can use this engine
    /// (e.g. `"aes"`, `"sha"`, `"spi_master"`, `"spi_slave"`, `"rmt"`).
    #[serde(default)]
    pub drivers: Vec<String>,
    /// DMA-capable peripheral instances for this engine, with their selector IDs.
    #[serde(default)]
    pub peripheral_instances: Vec<DmaPeripheralInstance>,
    /// The channels belonging to this engine.
    pub channels: Vec<DmaChannelDef>,
}

/// All DMA engines on a device.
#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(transparent)]
pub struct DmaEngines(pub Vec<DmaEngineDef>);

impl GenericProperty for DmaEngines {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut cfgs: Vec<String> = self
            .0
            .iter()
            .flat_map(|e| e.channels.iter())
            .map(|ch| format!("soc_has_{}", ch.name.to_lowercase()))
            .collect();

        // Emit a DMA-support cfg symbol for each unique driver listed in any engine.
        let mut seen = std::collections::HashSet::new();
        for engine in &self.0 {
            for driver in &engine.drivers {
                if seen.insert(driver.clone()) {
                    cfgs.push(format!("{}.supports_dma", driver));
                }
            }
        }

        Some(cfgs)
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let mut shared = vec![];
        let mut split = vec![];
        let mut engines = vec![];
        let mut engine_channels = vec![];
        let mut engine_any_channels = vec![];
        // One entry per (channel, peripheral) pair from DMA `compatible_with` lists.
        // If the list is empty (GDMA), this contains nothing.
        let mut dma_pairs = vec![];
        let mut dma_any_pairs = vec![];
        // Accumulates engine name tokens per driver, in engine declaration order.
        let mut driver_engines: BTreeMap<&str, Vec<proc_macro2::TokenStream>> = BTreeMap::new();

        for engine in self.0.iter() {
            let engine_name = engine.name.as_str();
            engines.push(quote! { #engine_name });

            let channel = engine.name.from_case(Case::Snake).to_case(Case::Pascal);
            let any_channel = format_ident!("{channel}Channel");

            for driver in &engine.drivers {
                driver_engines
                    .entry(driver)
                    .or_default()
                    .push(quote! { #engine_name, #any_channel });
            }

            if engine.channels.len() > 1 {
                engine_any_channels.push(quote! { #engine_name, any_channel = #any_channel });

                for peri_ident in engine
                    .peripheral_instances
                    .iter()
                    .map(|p| format_ident!("{}", p.name))
                {
                    dma_any_pairs
                        .push(quote! { #engine_name, any_channel = #any_channel, #peri_ident });
                }
            }

            for (idx, channel) in engine.channels.iter().enumerate() {
                let idx = number(idx);
                let ch = quote::format_ident!("{}", channel.name);

                engine_channels.push(quote! { #engine_name, #ch });

                let compatible_peris = if channel.compatible_with.is_empty() {
                    engine
                        .peripheral_instances
                        .iter()
                        .map(|p| format_ident!("{}", p.name))
                        .collect::<Vec<_>>()
                } else {
                    channel
                        .compatible_with
                        .iter()
                        .map(|p| format_ident!("{p}"))
                        .collect::<Vec<_>>()
                };

                for peri_ident in compatible_peris.iter() {
                    let ch_ident = quote::format_ident!("{}", channel.name);
                    dma_pairs.push(quote! { #engine_name, #ch_ident, #peri_ident });
                }

                match (
                    &channel.interrupt,
                    &channel.interrupt_in,
                    &channel.interrupt_out,
                ) {
                    (Some(interrupt), None, None) => {
                        let interrupt = quote::format_ident!("{}", interrupt);
                        shared.push(quote! { #engine_name, #ch, #idx, interrupt = #interrupt, compatible = [#(#compatible_peris),*] });
                    }
                    (None, Some(interrupt_in), Some(interrupt_out)) => {
                        let interrupt_in = quote::format_ident!("{}", interrupt_in);
                        let interrupt_out = quote::format_ident!("{}", interrupt_out);
                        split.push(
                            quote! { #engine_name, #ch, #idx, interrupt_in = #interrupt_in, interrupt_out = #interrupt_out, compatible = [#(#compatible_peris),*] },
                        );
                    }
                    (None, None, None) => {
                        // No interrupt info – skip
                    }
                    _ => {
                        panic!(
                            "DMA channel '{}': set either `interrupt` (shared) or both \
                             `interrupt_in` and `interrupt_out` (split), not a mix",
                            channel.name
                        );
                    }
                }
            }
        }

        let dma_channel_macro = if !shared.is_empty() || !split.is_empty() {
            Some(generate_for_each_macro(
                "dma_channel",
                &[
                    ("names", &engine_channels),
                    ("separate_any_type", &engine_any_channels),
                    ("shared", &shared),
                    ("split", &split),
                ],
            ))
        } else {
            None
        };

        let dma_engines_macro = generate_for_each_macro("dma_engine", &[("all", &engines)]);

        // Always emit for_each_dma_channel_peri_pair! so drivers can call it
        // unconditionally. On GDMA chips it expands to nothing.
        let dma_pairs_macro = generate_for_each_macro(
            "dma_channel_peri_pair",
            &[("channels", &dma_pairs), ("any_channels", &dma_any_pairs)],
        );

        // One with_{driver}_dma_engine! macro per driver that has DMA support.
        // Each driver uses exactly one engine; assert that invariant here.
        let driver_engine_macros: Vec<_> = driver_engines
            .iter()
            .map(|(driver, engines)| {
                assert!(
                    engines.len() == 1,
                    "driver '{}' is associated with {} DMA engines but exactly 1 is required",
                    driver,
                    engines.len()
                );
                generate_with_macro(&format!("{driver}_dma_engine"), &engines[0])
            })
            .collect();

        Some(quote::quote! {
            #dma_engines_macro
            #dma_channel_macro
            #dma_pairs_macro
            #(#driver_engine_macros)*
        })
    }
}

/// Generates a `with_{name}!` macro that provides a single fixed token set.
///
/// Unlike `generate_for_each_macro`, this asserts that exactly one set of tokens
/// was provided (panicking at code-generation time otherwise), then emits a macro
/// that calls its body exactly once with those tokens.
fn generate_with_macro(name: &str, tokens: &TokenStream) -> TokenStream {
    let macro_name = format_ident!("with_{name}");
    let inner = format_ident!("_with_inner_{name}");

    quote! {
        #[macro_export]
        #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
        macro_rules! #macro_name {
            (
                $($pattern:tt => $code:tt;)*
            ) => {
                macro_rules! #inner {
                    $(($pattern) => $code;)*
                    ($other:tt) => {}
                }
                #inner!(( #tokens ));
            };
        }
    }
}
