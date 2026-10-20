use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

#[derive(Clone, Debug)]
pub struct SdmChannels {
    pub(crate) count: u32,
}

impl<'de> serde::Deserialize<'de> for SdmChannels {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Ok(Self {
            count: u32::deserialize(deserializer)?,
        })
    }
}

impl GenericProperty for SdmChannels {
    fn cfgs(&self) -> Option<Vec<String>> {
        Some(
            (0..self.count)
                .map(|channel| format!("soc_has_sdm_ch{channel}"))
                .collect(),
        )
    }

    fn macros(&self) -> Option<TokenStream> {
        let channels = (0..self.count)
            .map(|channel| {
                let peri = format_ident!("SDM_CH{channel}");
                let variant = format_ident!("SdmCh{channel}");
                let signal = format_ident!("GPIO_SD{channel}");
                let channel = number(channel);
                quote::quote! { #channel, #peri, #variant, #signal }
            })
            .collect::<Vec<_>>();

        Some(generate_for_each_macro(
            "sdm_channel",
            &[("channels", &channels)],
        ))
    }

    fn property_macro_branches(&self) -> TokenStream {
        let count = number(self.count);

        quote::quote! {
            ("sdm.channel_count") => {
                #count
            };
            ("sdm.channel_count", str) => {
                stringify!(#count)
            };
        }
    }
}
