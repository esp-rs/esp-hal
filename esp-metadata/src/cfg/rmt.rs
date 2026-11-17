use proc_macro2::TokenStream;
use quote::{format_ident, quote};

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
    pub(crate) Vec<RmtChannelCapability>,
);

/// Generates `for_each_rmt_channel!` which can be used to implement channel creators and the main
/// driver struct for the RMT peripheral.
///
/// The macro generates code for each [device.rmt.channels[X]] entry.
impl GenericProperty for RmtChannelConfig {
    fn macros(&self) -> Option<TokenStream> {
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

        // A TX channel can use the RAM blocks of the channels that follow it, whatever their
        // direction.
        let channel_count = self.0.len();
        let tx_tuple_cfgs = self
            .0
            .iter()
            .enumerate()
            .filter(|&(_, &cap)| cap != RmtChannelCapability::Rx)
            .flat_map(|(first, _)| {
                (2..=channel_count - first).map(move |blocks| {
                    let channels = (first..first + blocks).map(|ch| format_ident!("RMT_CH{ch}"));
                    let blocks = number(blocks);
                    quote! {
                        #blocks, (#(#channels),*)
                    }
                })
            })
            .collect::<Vec<_>>();

        let for_each = generate_for_each_macro(
            "rmt_channel",
            &[
                ("all", &channel_cfgs),
                ("tx", &tx_channel_cfgs),
                ("rx", &rx_channel_cfgs),
                ("tx_tuples", &tx_tuple_cfgs),
            ],
        );

        Some(quote! {
            /// This macro can be used to generate code for each channel of the RMT peripheral.
            ///
            /// For an explanation on the general syntax, as well as usage of individual/repeated
            /// matchers, refer to [the crate-level documentation][crate#for_each-macros].
            ///
            /// This macro has four options for its "Individual matcher" case:
            ///
            /// - `all`: `($num:literal)`
            /// - `tx`: `($num:literal, $idx:literal)`
            /// - `rx`: `($num:literal, $idx:literal)`
            /// - `tx_tuples`: `($blocks:literal, ($($peri:ident),+))`
            ///
            /// Macro fragments:
            ///
            /// - `$num`: number of the channel, e.g. `0`
            /// - `$idx`: index of the channel among channels of the same capability, e.g. `0`
            /// - `$blocks`: number of RAM blocks of a TX channel that uses the RAM of the
            ///   channels that follow it, e.g. `2`
            /// - `$peri`: the singletons of these channels, the TX channel first, e.g. `RMT_CH1`
            ///
            /// Example data:
            ///
            /// - `all`: `(0)`
            /// - `tx`: `(1, 1)`
            /// - `rx`: `(2, 0)`
            /// - `tx_tuples`: `(2, (RMT_CH1, RMT_CH2))`
            #for_each
        })
    }
}
