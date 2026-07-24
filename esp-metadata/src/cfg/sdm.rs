use proc_macro2::TokenStream;
use quote::format_ident;

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

#[derive(Clone, Debug)]
pub struct SdmChannels {
    count: u32,
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
    fn macros(&self) -> Option<TokenStream> {
        let channels = (0..self.count)
            .map(|channel| {
                let channel = number(channel);
                let signal = format_ident!("GPIO_SD{channel}");
                quote::quote! { #channel, #signal }
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
