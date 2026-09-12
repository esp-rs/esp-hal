//! This module contains configuration used in [device.lp_io], as well as
//! functions that generate code for esp-hal.

use proc_macro2::TokenStream;
use quote::quote;

use crate::cfg::{
    GenericProperty,
    gpio::{IoMuxSignal, render_signals},
};

/// Additional properties (besides those defined in cfg.rs) for [device.lp_io].
///
/// Defining any signal means the chip has an LP GPIO matrix: LP peripheral signals can be routed
/// to any LP GPIO. Chips without these lists only expose LP functions on fixed pads via the LP IO
/// MUX.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LpIoSignals {
    /// The list of LP peripheral input signals that can be routed through the LP GPIO matrix.
    #[serde(default)]
    pub lp_input_signals: Vec<IoMuxSignal>,

    /// The list of LP peripheral output signals that can be routed through the LP GPIO matrix.
    #[serde(default)]
    pub lp_output_signals: Vec<IoMuxSignal>,
}

impl LpIoSignals {
    fn has_gpio_matrix(&self) -> bool {
        !self.lp_input_signals.is_empty() || !self.lp_output_signals.is_empty()
    }
}

impl GenericProperty for LpIoSignals {
    fn cfgs(&self) -> Option<Vec<String>> {
        self.has_gpio_matrix()
            .then(|| vec![String::from("lp_io.has_gpio_matrix")])
    }

    fn property_macro_branches(&self) -> TokenStream {
        let has_gpio_matrix = self.has_gpio_matrix();
        quote! {
            ("lp_io.has_gpio_matrix") => {
                #has_gpio_matrix
            };
        }
    }

    fn macros(&self) -> Option<TokenStream> {
        let lp_input_signals = render_signals("LpInputSignal", &self.lp_input_signals);
        let lp_output_signals = render_signals("LpOutputSignal", &self.lp_output_signals);

        Some(quote! {
            /// Defines the `LpInputSignal` and `LpOutputSignal` enums.
            ///
            /// This macro is intended to be called in esp-hal only.
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
            macro_rules! define_lp_io_signals {
                () => {
                    #lp_input_signals
                    #lp_output_signals
                };
            }
        })
    }
}
