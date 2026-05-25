use quote::quote;

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

/// A single DMA engine with its channels.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaEngineDef {
    /// The name of the engine (e.g. `"GDMA"`, `"SPI2"`, `"I2S0"`).
    #[expect(unused)]
    pub name: String,
    /// The channels belonging to this engine.
    pub channels: Vec<DmaChannelDef>,
}

/// All DMA engines on a device.
#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(transparent)]
pub struct DmaEngines(pub Vec<DmaEngineDef>);

impl GenericProperty for DmaEngines {
    fn cfgs(&self) -> Option<Vec<String>> {
        let cfgs = self
            .0
            .iter()
            .flat_map(|e| e.channels.iter())
            .map(|ch| format!("soc_has_{}", ch.name.to_lowercase()))
            .collect();
        Some(cfgs)
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let mut shared = vec![];
        let mut split = vec![];

        for engine in self.0.iter() {
            for (idx, channel) in engine.channels.iter().enumerate() {
                let idx = number(idx);
                match (
                    &channel.interrupt,
                    &channel.interrupt_in,
                    &channel.interrupt_out,
                ) {
                    (Some(interrupt), None, None) => {
                        let ch = quote::format_ident!("{}", channel.name);
                        let interrupt = quote::format_ident!("{}", interrupt);
                        shared.push(quote! { #ch, #idx, interrupt = #interrupt });
                    }
                    (None, Some(interrupt_in), Some(interrupt_out)) => {
                        let ch = quote::format_ident!("{}", channel.name);
                        let interrupt_in = quote::format_ident!("{}", interrupt_in);
                        let interrupt_out = quote::format_ident!("{}", interrupt_out);
                        split.push(
                            quote! { #ch, #idx, interrupt_in = #interrupt_in, interrupt_out = #interrupt_out },
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

        if shared.is_empty() && split.is_empty() {
            return None;
        }

        Some(generate_for_each_macro(
            "dma_channel",
            &[("shared", &shared), ("split", &split)],
        ))
    }
}
