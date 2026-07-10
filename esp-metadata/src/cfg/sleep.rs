use indexmap::IndexMap;
use quote::{format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{generate_for_each_macro, number};

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
    /// Maps a `WakeupSource` enum variant name to its bit position.
    ///
    /// Each key is used verbatim as the generated enum variant identifier, so it
    /// must be a valid PascalCase Rust identifier. An `IndexMap` is used to
    /// preserve the order in which sources are declared in `soc.toml`; that
    /// order becomes the variant order of the generated enum.
    #[serde(flatten)]
    sources: IndexMap<String, u8>,
}

/// Generates `for_each_wakeup_source!`, which drives the `WakeupSource` enum in
/// `esp-hal`. The `all` branch yields one `(variant, bit)` pair per source this
/// chip supports; the variant documentation is supplied by `esp-hal`.
impl super::GenericProperty for WakeupSources {
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
