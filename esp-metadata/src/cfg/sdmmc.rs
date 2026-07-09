use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

/// Per-slot configuration, used as an entry of [device.sdmmc.slot_config].
///
/// Each entry describes one of the controller's card slots. `iomux` records
/// whether the slot's bus signals (clock, command, data) are routed through the
/// IO_MUX (fixed pads) rather than the GPIO matrix. The signal names are only
/// meaningful for GPIO-matrix-routed signals; IO_MUX pads are selected through
/// the IO_MUX function tables instead and are intentionally not listed here.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SdmmcSlotConfig {
    /// Whether the slot's clock/command/data signals are routed through the
    /// IO_MUX rather than the GPIO matrix.
    #[serde(default)]
    pub iomux: bool,
    /// Card clock output signal (GPIO-matrix slots).
    #[serde(default)]
    pub clk: Option<String>,
    /// Command line input signal (GPIO-matrix slots).
    #[serde(default)]
    pub cmd_in: Option<String>,
    /// Command line output signal (GPIO-matrix slots).
    #[serde(default)]
    pub cmd_out: Option<String>,
    /// Data line input signals, ordered by data line index (GPIO-matrix slots).
    #[serde(default)]
    pub data_in: Vec<String>,
    /// Data line output signals, ordered by data line index (GPIO-matrix slots).
    #[serde(default)]
    pub data_out: Vec<String>,
    /// Card-detect input signal.
    #[serde(default)]
    pub cd: Option<String>,
    /// Write-protect input signal.
    #[serde(default)]
    pub wp: Option<String>,
    /// SDIO card-interrupt input signal.
    #[serde(default)]
    pub card_int: Option<String>,
    /// HS400 data-strobe input signal.
    #[serde(default)]
    pub data_strobe: Option<String>,
    /// eMMC reset output signal.
    #[serde(default)]
    pub rst: Option<String>,
}

/// The controller's card slots, used in [device.sdmmc.slot_config].
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct SdmmcSlots(Vec<SdmmcSlotConfig>);

/// Generates `for_each_sdmmc!` which materializes the driver's per-slot routing
/// table. The macro generates one branch per [device.sdmmc.slot_config[X]]
/// slot.
impl GenericProperty for SdmmcSlots {
    fn macros(&self) -> Option<TokenStream> {
        // Renders an optional signal name as a bracketed group: `[SIGNAL]` when
        // set, `[]` when absent. The brackets keep the branch shape uniform so a
        // single matcher can capture every slot regardless of which signals it
        // routes.
        let opt = |signal: &Option<String>| match signal {
            Some(name) => {
                let id = format_ident!("{name}");
                quote! { [#id] }
            }
            None => quote! { [] },
        };

        let slot_cfgs = self
            .0
            .iter()
            .enumerate()
            .map(|(idx, slot)| {
                let name = format_ident!("slot{idx}");
                let idx = number(idx);
                let iomux = slot.iomux;

                let clk = opt(&slot.clk);
                let cmd_in = opt(&slot.cmd_in);
                let cmd_out = opt(&slot.cmd_out);
                let cd = opt(&slot.cd);
                let wp = opt(&slot.wp);
                let card_int = opt(&slot.card_int);
                let data_strobe = opt(&slot.data_strobe);
                let rst = opt(&slot.rst);

                let data_in = slot.data_in.iter().map(|s| format_ident!("{s}"));
                let data_out = slot.data_out.iter().map(|s| format_ident!("{s}"));

                // The order and meaning of these tokens must match their use at
                // the `for_each_sdmmc!` call site in the driver.
                quote! {
                    #name, #idx, #iomux,
                    #clk, #cmd_in, #cmd_out,
                    [#(#data_in),*], [#(#data_out),*],
                    #cd, #wp, #card_int, #data_strobe, #rst
                }
            })
            .collect::<Vec<_>>();

        let for_each = generate_for_each_macro("sdmmc", &[("all", &slot_cfgs)]);

        Some(quote! {
            /// This macro can be used to generate code for each slot of the SDMMC/SDIO host driver.
            ///
            /// For an explanation on the general syntax, as well as usage of individual/repeated
            /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
            ///
            /// This macro has one option for its "Individual matcher" case:
            ///
            /// Syntax: `($slot:ident, $idx:literal, $iomux:literal, [$($clk:ident)?]
            /// [$($cmd_in:ident)?] [$($cmd_out:ident)?] [$($data_in:ident),*] [$($data_out:ident),*]
            /// [$($cd:ident)?] [$($wp:ident)?] [$($card_int:ident)?] [$($data_strobe:ident)?]
            /// [$($rst:ident)?])`
            ///
            /// Macro fragments:
            ///
            /// - `$slot`: the name of the slot (`slot0`, `slot1`).
            /// - `$idx`: the zero-based slot index.
            /// - `$iomux`: `true` if the slot's clock/command/data signals are IO_MUX-routed.
            /// - `$clk`, `$cmd_in`, `$cmd_out`, `$data_in`, `$data_out`: GPIO-matrix bus signal
            ///   names (absent for IO_MUX-routed slots).
            /// - `$cd`, `$wp`, `$card_int`, `$data_strobe`, `$rst`: auxiliary signal names, each
            ///   present only when the slot routes that signal through the GPIO matrix.
            ///
            /// Each optional signal is wrapped in brackets so the branch shape stays uniform: a set
            /// signal appears as `[SIGNAL]`, an absent one as `[]`.
            #for_each
        })
    }
}
