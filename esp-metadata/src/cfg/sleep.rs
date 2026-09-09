use convert_case::{Boundary, Case, Casing};
use indexmap::IndexMap;
use quote::{format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

/// Light-sleep CPU retention capabilities and memory placement constraints.
///
/// Declared in `[device.sleep.retention]` so these keys are not captured by
/// [`WakeupSources`]'s flattened wakeup-source map.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SleepRetentionProperties {
    /// How the chip retains CPU state across light sleep with CPU power-down.
    cpu_retention: Option<String>,
    #[serde(default)]
    supports_top_power_down: bool,
    #[serde(default)]
    supports_tagmem_power_down: bool,
    /// Bytes the CPU frames need, including the DMA descriptor that precedes them.
    ///
    /// The size is the mark that esp-hal describes the frames of this chip, so it also drives the
    /// `supports_cpu_power_down` cfg. A chip that has the hardware but no frame layout keeps the
    /// CPU domain powered.
    cpu_retention_mem_size: Option<u32>,
    cpu_retention_mem_align: Option<u32>,
    /// First address the retention DMA can reach.
    cpu_retention_mem_start: Option<u32>,
    /// First address past the range the retention DMA can reach.
    cpu_retention_mem_end: Option<u32>,
}

impl GenericProperty for SleepRetentionProperties {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut cfgs = vec![];

        if let Some(mode) = &self.cpu_retention {
            cfgs.push(format!("cpu_retention=\"{mode}\""));
        }
        if self.cpu_retention_mem_size.is_some() {
            cfgs.push("supports_cpu_power_down".to_string());
        }
        if self.supports_top_power_down {
            cfgs.push("supports_top_power_down".to_string());
        }
        if self.supports_tagmem_power_down {
            cfgs.push("supports_tagmem_power_down".to_string());
        }

        if cfgs.is_empty() { None } else { Some(cfgs) }
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let mut branches = quote! {};

        if let Some(size) = self.cpu_retention_mem_size {
            let size = number(size);
            branches.extend(quote! {
                ("sleep.cpu_retention_mem_size") => {
                    #size
                };
            });
        }
        if let Some(align) = self.cpu_retention_mem_align {
            let align = number(align);
            branches.extend(quote! {
                ("sleep.cpu_retention_mem_align") => {
                    #align
                };
            });
        }
        if let Some(start) = self.cpu_retention_mem_start {
            let start = number(start);
            branches.extend(quote! {
                ("sleep.cpu_retention_mem_start") => {
                    #start
                };
            });
        }
        if let Some(end) = self.cpu_retention_mem_end {
            let end = number(end);
            branches.extend(quote! {
                ("sleep.cpu_retention_mem_end") => {
                    #end
                };
            });
        }

        branches
    }
}

/// The set of wakeup sources supported by a chip.
///
/// A wakeup source occupies a single bit in both the wakeup-enable ("config")
/// register and the wakeup-cause register. The two registers share their bit
/// layout, so a single `WakeupSource` enum variant models both the configurable
/// trigger and the reported wakeup cause.
///
/// This maps each supported source to its bit position in the wakeup registers;
/// a source that a chip lacks is simply absent from the map. The variant
/// documentation lives in `esp-hal` (see the `for_each_wakeup_source!`
/// invocation in `rtc_cntl`), not here.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub(crate) struct WakeupSources {
    /// The sources that can reject a sleep request, if the chip cannot reject on every source.
    ///
    /// A source rejects a sleep request if the source is already asserted at the
    /// request. Most chips can reject on every source that they can wake from, and
    /// do not set this field. esp32 has reject enables for `Gpio` and `Sdio` only.
    rejectable: Option<Vec<String>>,

    /// Maps a `WakeupSource` enum variant name to its bit position.
    ///
    /// Each key is used verbatim as the generated enum variant identifier, so it
    /// must be a valid PascalCase Rust identifier. An `IndexMap` is used to
    /// preserve the order in which sources are declared in `soc.toml`; that
    /// order becomes the variant order of the generated enum.
    #[serde(flatten)]
    sources: IndexMap<String, u8>,
}

impl WakeupSources {
    /// The mask of sources that can reject a sleep request.
    ///
    /// This is a mask, and not a branch of `for_each_wakeup_source!`, for two reasons. First,
    /// `esp-hal` needs no more than the mask, because it only intersects the mask with the
    /// enabled sources. Second, a second branch also repeats those sources in the individual
    /// matcher of the macro, which defines the `WakeupSource` enum.
    fn rejectable_mask(&self) -> u32 {
        let bit = |name: &String| {
            let bit = self.sources.get(name).unwrap_or_else(|| {
                panic!("`rejectable` names {name}, which is not a wakeup source")
            });
            1u32 << bit
        };

        match &self.rejectable {
            // An empty field means that every source can reject, which is the usual case.
            None => self.sources.values().map(|bit| 1u32 << bit).sum(),
            Some(names) => names.iter().map(bit).sum(),
        }
    }
}

/// Generates `for_each_wakeup_source!`, which drives the `WakeupSource` enum in
/// `esp-hal`. The `all` branch yields one `(variant, bit)` pair per source this
/// chip supports; the variant documentation is supplied by `esp-hal`.
impl super::GenericProperty for WakeupSources {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut sources = vec![];

        let mut uart_seen = false;
        for name in self.sources.keys() {
            if name.starts_with("Uart") {
                if !uart_seen {
                    uart_seen = true;
                    sources.push("sleep_has_wakeup_source_uart".to_string());
                }
                continue;
            }

            sources.push(format!(
                "sleep_has_wakeup_source_{}",
                name.from_case(Case::Pascal)
                    .without_boundaries(&Boundary::digits())
                    .to_case(Case::Snake)
            ));
        }

        Some(sources)
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        if self.sources.is_empty() {
            return quote! {};
        }

        let mask = number(self.rejectable_mask());
        quote! {
            ("sleep.rejectable_mask") => {
                #mask
            };
        }
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let sources = self
            .sources
            .iter()
            .map(|(name, bit)| {
                let variant = format_ident!("{name}");
                let bit = number(bit);
                quote! { #variant, #bit }
            })
            .collect::<Vec<_>>();

        // Chips without any wakeup source don't get the macro at all.
        if sources.is_empty() {
            return None;
        }

        Some(generate_for_each_macro(
            "wakeup_source",
            &[("all", &sources)],
        ))
    }
}
