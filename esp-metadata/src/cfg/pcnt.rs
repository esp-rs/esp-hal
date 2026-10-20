use std::collections::BTreeSet;

use convert_case::{Case, Casing};
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::PcntProperties, generate_for_each_macro, number};

/// Instance configuration, used in `[device.pcnt]` `instances = { ... }`.
///
/// The map key is the PAC/virtual register-block singleton (`PCNT`, `PCNT1`).
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PcntInstanceConfig {
    /// The name of the instance in the `esp_hal::system::Peripheral` enum.
    pub sys_instance: String,
    /// Shared peripheral interrupt for this register block.
    pub interrupt: String,
    /// Units in this register block, in hardware index order.
    #[serde(default)]
    pub units: Vec<PcntUnitSignals>,
}

/// GPIO matrix signals for one unit in a PCNT register block.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PcntUnitSignals {
    pub sig_ch0: String,
    pub sig_ch1: String,
    pub ctrl_ch0: String,
    pub ctrl_ch1: String,
}

/// Index of a PCNT register block: `PCNT` → 0, `PCNT1` → 1.
fn pcnt_block_index(regs: &str) -> u32 {
    let Some(rest) = regs.strip_prefix("PCNT") else {
        panic!("PCNT instance name must be `PCNT` or `PCNTn`, got {regs}");
    };
    if rest.is_empty() {
        0
    } else {
        rest.parse()
            .unwrap_or_else(|_| panic!("PCNT instance name must be `PCNT` or `PCNTn`, got {regs}"))
    }
}

pub(crate) fn singleton_name(regs: &str, unit: usize) -> String {
    format!("PCNT{}_UNIT{unit}", pcnt_block_index(regs))
}

/// Generates `for_each_pcnt_unit!` which can be used to implement the PCNT
/// `Instance` trait for each unit singleton.
pub(crate) fn generate_pcnt_peripherals(pcnt: &PcntProperties) -> TokenStream {
    let names = pcnt
        .instances
        .iter()
        .flat_map(|instance| {
            instance
                .instance_config
                .units
                .iter()
                .enumerate()
                .map(|(unit, _)| {
                    let name = format_ident!("{}", singleton_name(&instance.name, unit));
                    quote! { #name }
                })
        })
        .collect::<Vec<_>>();

    let instance_cfgs = pcnt
        .instances
        .iter()
        .flat_map(|instance| {
            let cfg = &instance.instance_config;
            instance
                .instance_config
                .units
                .iter()
                .enumerate()
                .map(move |(unit, signals)| {
                    let peri = format_ident!("{}", singleton_name(&instance.name, unit));
                    let variant =
                        format_ident!("{}", singleton_name(&instance.name, unit).to_case(Case::Pascal));
                    let sys = format_ident!("{}", cfg.sys_instance);
                    let regs = format_ident!("{}", instance.name);
                    let unit = number(unit as u32);
                    let interrupt = format_ident!("{}", cfg.interrupt);
                    let sig_ch0 = format_ident!("{}", signals.sig_ch0);
                    let sig_ch1 = format_ident!("{}", signals.sig_ch1);
                    let ctrl_ch0 = format_ident!("{}", signals.ctrl_ch0);
                    let ctrl_ch1 = format_ident!("{}", signals.ctrl_ch1);

                    quote! {
                        #peri, #variant, #sys, #regs, #unit, #interrupt, #sig_ch0, #sig_ch1, #ctrl_ch0, #ctrl_ch1
                    }
                })
        })
        .collect::<Vec<_>>();

    let mut seen = BTreeSet::new();
    let interrupts = pcnt
        .instances
        .iter()
        .filter_map(|instance| {
            let irq = &instance.instance_config.interrupt;
            seen.insert(irq.clone()).then(|| {
                let interrupt = format_ident!("{irq}");
                quote! { #interrupt }
            })
        })
        .collect::<Vec<_>>();

    let mut seen = BTreeSet::new();
    let register_blocks = pcnt
        .instances
        .iter()
        .filter_map(|instance| {
            seen.insert(instance.name.clone()).then(|| {
                let regs = format_ident!("{}", instance.name);
                quote! { #regs }
            })
        })
        .collect::<Vec<_>>();

    let for_each = generate_for_each_macro(
        "pcnt_unit",
        &[
            ("names", &names),
            ("all", &instance_cfgs),
            ("interrupt", &interrupts),
            ("regs", &register_blocks),
        ],
    );

    quote! {
        /// This macro can be used to generate code for each PCNT unit.
        ///
        /// For an explanation on the general syntax, as well as usage of individual/repeated
        /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
        ///
        /// This macro has four options for its "Individual matcher" case:
        ///
        /// - `names`: `($instance:ident)`
        /// - `all`: `($peri:ident, $variant:ident, $sys:ident, $regs:ident, $unit:literal, $interrupt:ident, $sig_ch0:ident, $sig_ch1:ident, $ctrl_ch0:ident, $ctrl_ch1:ident)`
        /// - `interrupt`: `($interrupt:ident)` (repeated: one ident per unique IRQ)
        /// - `regs`: `($regs:ident)` (repeated: one ident per register block)
        ///
        /// Macro fragments:
        ///
        /// - `$peri`: unit singleton name (`PCNT0_UNIT0`, `PCNT1_UNIT0`, …)
        /// - `$variant`: `AnyPcntUnit` enum variant (`Pcnt0Unit0`, `Pcnt1Unit0`, …)
        /// - `$sys`: `esp_hal::system::Peripheral` variant for clocks
        /// - `$regs`: register-block singleton (`PCNT`, `PCNT1`)
        /// - `$unit`: hardware unit index within that register block
        /// - `$interrupt`: shared peripheral interrupt
        /// - `$sig_ch0`, `$sig_ch1`, `$ctrl_ch0`, `$ctrl_ch1`: GPIO matrix input signals
        #for_each
    }
}
