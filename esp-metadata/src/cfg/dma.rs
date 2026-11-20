use std::collections::{BTreeMap, HashSet};

use anyhow::{Result, bail, ensure};
use convert_case::{Case, Casing};
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{
    cfg::{DmaProperties, GenericProperty, Value},
    generate_for_each_macro,
    number,
};

/// A single DMA channel definition.
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DmaChannelDef {
    /// The name of the peripheral singleton for this channel.
    pub name: String,
    /// Overrides the engine's PSRAM access for this channel.
    #[serde(default)]
    pub can_access_psram: Option<bool>,
    /// Whether this channel supports memory-to-memory transfers.
    #[serde(default)]
    pub mem2mem: bool,
    /// GDMA peripheral selector for memory-to-memory on this channel (dummy slot).
    /// Required when `mem2mem = true` and the device has `mem2mem_requires_peripheral = false`.
    /// Must not be set when `mem2mem_requires_peripheral = true`.
    #[serde(default)]
    pub mem2mem_id: Option<u32>,
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
#[serde(deny_unknown_fields)]
pub struct DmaPeripheralInstance {
    /// The peripheral singleton name (e.g. `"SPI2"`, `"AES"`).
    pub name: String,
    /// The DMA peripheral selector ID (the old `dma_peripheral` value).
    pub dma_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
enum AhbBurstMode {
    Single,
    Incr,
    Incr4,
    Incr8,
    Incr16,
}

/// A single DMA engine with its channels.
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DmaEngineDef {
    /// The name of the engine (e.g. `"AHB_GDMA"`, `"SPI_DMA"`, `"I2S_DMA"`).
    pub name: String,

    /// Register backend version, required for AHB GDMA.
    #[serde(default)]
    pub version: Option<u32>,

    /// AHB GDMA data burst modes. Silicon revisions can impose further restrictions.
    #[serde(default)]
    data_burst_modes: Vec<AhbBurstMode>,

    #[serde(default)]
    pub max_priority: Option<u32>,

    /// Whether channels of this DMA engine can access PSRAM by default.
    #[serde(default)]
    pub can_access_psram: bool,

    /// Supported AXI GDMA data burst sizes in bytes.
    #[serde(default)]
    pub data_burst_sizes: Vec<u32>,

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

impl DmaEngineDef {
    fn channel_can_access_psram(&self, channel: &DmaChannelDef) -> bool {
        channel.can_access_psram.unwrap_or(self.can_access_psram)
    }
}

/// All DMA engines on a device.
#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(transparent)]
pub struct DmaEngines(pub Vec<DmaEngineDef>);

impl DmaProperties {
    pub(crate) fn validate(&self) -> Result<()> {
        self.engines
            .validate_mem2mem(self.mem2mem_requires_peripheral)?;
        self.engines.validate_ahb()?;
        self.engines.validate_axi()
    }

    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        self.engines
            .0
            .iter()
            .find(|engine| engine.name == "AHB_GDMA")
            .map(|engine| ("dma.gdma_version", false, Value::from(engine.version)))
            .into_iter()
    }
}

impl DmaEngines {
    fn validate_ahb(&self) -> Result<()> {
        let mut has_ahb = false;
        for engine in &self.0 {
            if engine.name == "AHB_GDMA" {
                ensure!(!has_ahb, "Duplicate AHB_GDMA engine");
                ensure!(
                    matches!(engine.version, Some(1..=3)),
                    "AHB_GDMA requires a supported version (1, 2 or 3)"
                );
                let modes: HashSet<_> = engine.data_burst_modes.iter().copied().collect();
                ensure!(
                    modes.len() == engine.data_burst_modes.len(),
                    "AHB_GDMA data_burst_modes contains duplicate modes"
                );
                ensure!(
                    modes.contains(&AhbBurstMode::Single),
                    "AHB_GDMA data_burst_modes requires single"
                );
                if engine.version == Some(1) {
                    ensure!(
                        modes.len() == 2 && modes.contains(&AhbBurstMode::Incr),
                        "AHB_GDMA v1 requires single and incr"
                    );
                } else {
                    ensure!(
                        !modes.contains(&AhbBurstMode::Incr),
                        "AHB_GDMA fixed incr mode requires v1"
                    );
                }
                has_ahb = true;
            } else {
                ensure!(
                    engine.version.is_none() && engine.data_burst_modes.is_empty(),
                    "DMA engine '{}': version and data_burst_modes are only valid for AHB_GDMA",
                    engine.name
                );
            }
        }
        Ok(())
    }

    fn validate_axi(&self) -> Result<()> {
        for engine in &self.0 {
            let sizes = &engine.data_burst_sizes;
            if engine.name != "AXI_GDMA" {
                ensure!(
                    sizes.is_empty(),
                    "DMA engine '{}': data_burst_sizes is only valid for AXI_GDMA",
                    engine.name
                );
                continue;
            }
            ensure!(
                sizes.contains(&8),
                "AXI_GDMA requires the default 8-byte burst"
            );
            let mut seen = HashSet::new();
            for &size in sizes {
                ensure!(
                    matches!(size, 8 | 16 | 32 | 64 | 128),
                    "AXI_GDMA: invalid data burst size {size}"
                );
                ensure!(
                    seen.insert(size),
                    "AXI_GDMA: duplicate data burst size {size}"
                );
            }
        }

        Ok(())
    }

    fn validate_mem2mem(&self, requires_peripheral: bool) -> Result<()> {
        for engine in &self.0 {
            let mut seen_ids = std::collections::HashSet::new();
            for channel in &engine.channels {
                if !channel.mem2mem {
                    ensure!(
                        channel.mem2mem_id.is_none(),
                        "DMA channel '{}': mem2mem_id is only valid when mem2mem = true",
                        channel.name
                    );
                    continue;
                }

                match (requires_peripheral, channel.mem2mem_id) {
                    (true, Some(id)) => bail!(
                        "DMA channel '{}': mem2mem_id = {id} is forbidden when mem2mem_requires_peripheral is true",
                        channel.name
                    ),
                    (false, None) => bail!(
                        "DMA channel '{}': mem2mem_id is required when mem2mem_requires_peripheral is false",
                        channel.name
                    ),
                    (false, Some(id)) => {
                        ensure!(
                            seen_ids.insert(id),
                            "DMA channel '{}': duplicate mem2mem_id {id}",
                            channel.name
                        );
                    }
                    (true, None) => {}
                }
            }
        }
        Ok(())
    }

    fn max_priority(&self) -> Option<u32> {
        self.0.iter().flat_map(|e| e.max_priority).max()
    }
}

impl GenericProperty for DmaEngines {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut cfgs: Vec<String> = self
            .0
            .iter()
            .flat_map(|e| e.channels.iter())
            .map(|ch| format!("soc_has_{}", ch.name.to_lowercase()))
            .collect();

        if self
            .0
            .iter()
            .flat_map(|e| e.channels.iter())
            .any(|ch| ch.mem2mem)
        {
            cfgs.push("dma.supports_mem2mem".to_string());
        }

        // Emit a DMA-support cfg symbol for each unique driver listed in any engine.
        let mut seen = std::collections::HashSet::new();
        for engine in &self.0 {
            for driver in &engine.drivers {
                if seen.insert(driver.clone()) {
                    cfgs.push(format!("{}.supports_dma", driver));
                    cfgs.push(format!("{}_dma_engine=\"{}\"", driver, engine.name));
                }
            }
        }

        if self.0.iter().any(|engine| {
            engine
                .channels
                .iter()
                .any(|channel| engine.channel_can_access_psram(channel))
        }) {
            cfgs.push("dma.can_access_psram".to_string());
        }

        if let Some(axi) = self.0.iter().find(|engine| engine.name == "AXI_GDMA") {
            cfgs.push("dma.has_axi_gdma".to_string());
            cfgs.extend(
                axi.data_burst_sizes
                    .iter()
                    .map(|size| format!("dma.axi_data_burst=\"{size}\"")),
            );
        }

        // TODO: temporary
        if let Some(max_priority) = self.max_priority() {
            cfgs.push(format!("dma.max_priority=\"{}\"", max_priority));
            cfgs.push("dma.max_priority_is_set".to_string());
        }

        Some(cfgs)
    }

    fn property_macro_branches(&self) -> TokenStream {
        self.0
            .iter()
            .flat_map(|engine| {
                let engine_name = format_ident!("{}", engine.name);
                let can_access_psram = engine.can_access_psram;
                let uniform = engine
                    .channels
                    .iter()
                    .all(|channel| engine.channel_can_access_psram(channel) == can_access_psram);
                let engine_property = uniform.then(|| {
                    quote! {
                        ("dma.engine_can_access_psram", #engine_name) => { #can_access_psram };
                    }
                });
                let channel_properties = engine.channels.iter().map(|channel| {
                    let name = format_ident!("{}", channel.name);
                    let can_access_psram = engine.channel_can_access_psram(channel);
                    quote! {
                        ("dma.channel_can_access_psram", #name) => { #can_access_psram };
                    }
                });
                engine_property.into_iter().chain(channel_properties)
            })
            .collect()
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        // DMA channel information split into three lists based on interrupt signals
        let mut no_own_interrupt = vec![];
        let mut shared = vec![];
        let mut split = vec![];

        let mut engines = vec![];
        let mut engine_channels = vec![];
        let mut engine_any_channels = vec![];
        // One entry per (channel, peripheral) pair from DMA `compatible_with` lists.
        // If the list is empty (GDMA), this contains nothing.
        let mut dma_pairs = vec![];
        let mut dma_any_pairs = vec![];
        let mut mem2mem_channels = vec![];
        let mut mem2mem_engines = vec![];
        let mut mem2mem_engine_seen = HashSet::new();
        // Per-engine (hardware channel index, mem2mem peripheral id) for type-erased channels.
        let mut mem2mem_erased_by_engine: BTreeMap<&str, Vec<(TokenStream, TokenStream)>> =
            BTreeMap::new();
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

                if channel.mem2mem {
                    let mem2mem_id = number(channel.mem2mem_id.unwrap_or(0));
                    let variant = format_ident!(
                        "{}",
                        engine.name.from_case(Case::Snake).to_case(Case::Pascal)
                    );
                    mem2mem_channels
                        .push(quote! { #engine_name, #variant, #any_channel, #ch, #mem2mem_id });
                    // CRYPTO_DMA and COPY_DMA alias their any-channel type to the singleton
                    // peripheral type; an erased impl would conflict with the singleton impl.
                    if !matches!(engine_name, "CRYPTO_DMA" | "COPY_DMA") {
                        mem2mem_erased_by_engine
                            .entry(engine_name)
                            .or_default()
                            .push((idx.clone(), mem2mem_id));
                    }
                    if mem2mem_engine_seen.insert(engine_name) {
                        mem2mem_engines.push(quote! { #engine_name, #variant, #any_channel });
                    }
                }

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
                        no_own_interrupt.push(
                            quote! { #engine_name, #ch, #idx, compatible = [#(#compatible_peris),*] },
                        );
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
                    ("no_own_interrupt", &no_own_interrupt),
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

        let mem2mem_erased = mem2mem_erased_by_engine
            .iter()
            .map(|(engine_name, arms)| {
                let variant = format_ident!(
                    "{}",
                    engine_name.from_case(Case::Snake).to_case(Case::Pascal)
                );
                let any_channel = format_ident!("{variant}Channel");
                let engine_name = *engine_name;
                let (hws, ids): (Vec<_>, Vec<_>) = arms.iter().cloned().unzip();
                quote! { #engine_name, #variant, #any_channel, #( #hws, #ids ),* }
            })
            .collect::<Vec<_>>();

        let mem2mem_channel_macro = if !mem2mem_channels.is_empty() {
            Some(generate_for_each_macro(
                "mem2mem_channel",
                &[
                    ("channels", &mem2mem_channels),
                    ("erased", &mem2mem_erased),
                    ("engines", &mem2mem_engines),
                ],
            ))
        } else {
            None
        };

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
            #mem2mem_channel_macro
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
