use quote::quote;

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct AesKeyLengthOption {
    /// The key length in bits.
    pub bits: u32,

    /// The ID of the encryption mode.
    pub encrypt_mode: u32,

    /// The ID of the decryption mode.
    pub decrypt_mode: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct AesKeyLength {
    /// The supported key length options.
    options: Vec<AesKeyLengthOption>,
}

impl GenericProperty for AesKeyLength {
    fn for_each_macro(&self) -> Option<proc_macro2::TokenStream> {
        let bits = self
            .options
            .iter()
            .map(|opt| number(opt.bits))
            .collect::<Vec<_>>();

        let modes = self
            .options
            .iter()
            .map(|opt| {
                let bits = number(opt.bits);
                let encrypt_mode = number(opt.encrypt_mode);
                let decrypt_mode = number(opt.decrypt_mode);
                quote! {
                    #bits, #encrypt_mode, #decrypt_mode
                }
            })
            .collect::<Vec<_>>();

        Some(generate_for_each_macro(
            "aes_key_length",
            &[("bits", &bits), ("modes", &modes)],
        ))
    }
}
