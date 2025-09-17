use proc_macro2::TokenStream;
use quote::quote;

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

/// The capabilities of an RMT channel, used in [device.rmt.channels]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Deserialize, serde::Serialize)]
pub(crate) enum RmtChannelCapability {
    Rx,
    Tx,
    RxTx,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub(crate) struct RmtChannelConfig(
    /// The capability of each channel
    Vec<RmtChannelCapability>,
);

/// Generates `for_each_rmt_channel!` which can be used to implement channel creators and the main
/// driver struct for the RMT peripheral.
///
/// The macro generates code for each [device.rmt.channels[X]] entry.
impl GenericProperty for RmtChannelConfig {
    fn for_each_macro(&self) -> Option<TokenStream> {
        let channel_cfgs = self
            .0
            .iter()
            .enumerate()
            .map(|(num, _)| {
                let num = number(num);
                quote! {
                    #num
                }
            })
            .collect::<Vec<_>>();

        let make_channel_cfgs = |filter: fn(RmtChannelCapability) -> bool| {
            self.0
                .iter()
                .enumerate()
                .filter(|&(_, &cap)| filter(cap))
                .enumerate()
                .map(|(idx, (num, _))| {
                    let num = number(num);
                    let idx = number(idx);
                    quote! {
                        #num, #idx
                    }
                })
                .collect::<Vec<_>>()
        };

        let tx_channel_cfgs = make_channel_cfgs(|cap| {
            matches!(cap, RmtChannelCapability::Tx | RmtChannelCapability::RxTx)
        });
        let rx_channel_cfgs = make_channel_cfgs(|cap| {
            matches!(cap, RmtChannelCapability::Rx | RmtChannelCapability::RxTx)
        });

        let for_each = generate_for_each_macro(
            "rmt_channel",
            &[
                ("all", &channel_cfgs),
                ("tx", &tx_channel_cfgs),
                ("rx", &rx_channel_cfgs),
            ],
        );

        Some(quote! {
            /// This macro can be used to generate code for each channel of the RMT peripheral.
            ///
            /// For an explanation on the general syntax, as well as usage of individual/repeated
            /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
            ///
            /// This macro has three options for its "Individual matcher" case:
            ///
            /// - `all`: `($num:literal)`
            /// - `tx`: `($num:literal, $idx:literal)`
            /// - `rx`: `($num:literal, $idx:literal)`
            ///
            /// Macro fragments:
            ///
            /// - `$num`: number of the channel, e.g. `0`
            /// - `$idx`: index of the channel among channels of the same capability, e.g. `0`
            ///
            /// Example data:
            ///
            /// - `all`: `(0)`
            /// - `tx`: `(1, 1)`
            /// - `rx`: `(2, 0)`
            #for_each
        })
    }
}
