use proc_macro2::TokenStream;

use crate::{cfg::GenericProperty, number};

#[derive(Clone, Debug, Default, serde::Deserialize)]
#[serde(transparent)]
pub struct DacChannels {
    pub(crate) count: u32,
}

impl GenericProperty for DacChannels {
    fn cfgs(&self) -> Option<Vec<String>> {
        Some(
            (1..=self.count)
                .map(|channel| format!("soc_has_dac{channel}"))
                .collect(),
        )
    }

    fn property_macro_branches(&self) -> TokenStream {
        let count = number(self.count);
        quote::quote! {
            ("dac.channel_count") => {
                #count
            };
            ("dac.channel_count", str) => {
                stringify!(#count)
            };
        }
    }
}
