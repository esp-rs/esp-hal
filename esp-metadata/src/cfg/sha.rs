use proc_macro2::Ident;
use quote::{ToTokens, format_ident, quote};
use serde::{Deserialize, Serialize};

use crate::{generate_for_each_macro, number};

struct ShaAlgo {
    name: &'static str,
    ident: Ident,
    digest_len: u32,
    block_size: u32,
    message_len_bytes: u32,
    // These bits come from <https://en.wikipedia.org/wiki/Secure_Hash_Algorithms#Comparison_of_SHA_functions>
    insecure_against_collision: bool,
    insecure_against_length_extension: bool,
}

#[derive(Default, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ShaAlgoMap {
    sha1: Option<u32>,
    sha224: Option<u32>,
    sha256: Option<u32>,
    sha384: Option<u32>,
    sha512: Option<u32>,
    sha512_224: Option<u32>,
    sha512_256: Option<u32>,
    sha512_t: Option<u32>,
}

impl super::GenericProperty for ShaAlgoMap {
    fn for_each_macro(&self) -> Option<proc_macro2::TokenStream> {
        let modes = [
            ("SHA-1", self.sha1),
            ("SHA-224", self.sha224),
            ("SHA-256", self.sha256),
            ("SHA-384", self.sha384),
            ("SHA-512", self.sha512),
            ("SHA-512/224", self.sha512_224),
            ("SHA-512/256", self.sha512_256),
            ("SHA-512/t", self.sha512_t),
        ];

        let algos = modes
            .into_iter()
            .filter_map(|(name, mode)| mode.map(|m| (name, m)))
            .filter_map(|(name, mode)| ShaAlgo::new(name).map(|name| (name, mode)))
            .map(|(algo, mode)| {
                let mode = number(mode);
                quote! { #algo, #mode }
            })
            .collect::<Vec<_>>();

        Some(generate_for_each_macro(
            "sha_algorithm",
            &[("algos", &algos)],
        ))
    }
}

impl ShaAlgo {
    fn new(algo: &str) -> Option<Self> {
        let known = [
            Self {
                name: "SHA-1",
                ident: format_ident!("Sha1"),
                digest_len: 20,
                block_size: 64,
                message_len_bytes: 8,
                insecure_against_collision: true,
                insecure_against_length_extension: true,
            },
            Self {
                name: "SHA-224",
                ident: format_ident!("Sha224"),
                digest_len: 28,
                block_size: 64,
                message_len_bytes: 8,
                insecure_against_collision: false,
                insecure_against_length_extension: true,
            },
            Self {
                name: "SHA-256",
                ident: format_ident!("Sha256"),
                digest_len: 32,
                block_size: 64,
                message_len_bytes: 8,
                insecure_against_collision: false,
                insecure_against_length_extension: true,
            },
            Self {
                name: "SHA-384",
                ident: format_ident!("Sha384"),
                digest_len: 48,
                block_size: 128,
                message_len_bytes: 16,
                insecure_against_collision: false,
                insecure_against_length_extension: false,
            },
            Self {
                name: "SHA-512",
                ident: format_ident!("Sha512"),
                digest_len: 64,
                block_size: 128,
                message_len_bytes: 16,
                insecure_against_collision: false,
                insecure_against_length_extension: true,
            },
            Self {
                name: "SHA-512/224",
                ident: format_ident!("Sha512_224"),
                digest_len: 28,
                block_size: 128,
                message_len_bytes: 16,
                insecure_against_collision: false,
                insecure_against_length_extension: false,
            },
            Self {
                name: "SHA-512/256",
                ident: format_ident!("Sha512_256"),
                digest_len: 32,
                block_size: 128,
                message_len_bytes: 16,
                insecure_against_collision: false,
                insecure_against_length_extension: false,
            },
        ];

        for a in known {
            if a.name == algo {
                return Some(a);
            }
        }

        // Special-case (ignore) general truncated algo until we figure out how to describe its
        // `digest_words`.
        if algo == "SHA-512/t" {
            return None;
        }

        panic!("Unknown SHA algorithm: {algo}")
    }
}

impl ToTokens for ShaAlgo {
    fn to_tokens(&self, tokens: &mut proc_macro2::TokenStream) {
        let ident = &self.ident;
        let name = &self.name;
        let digest_len = number(self.digest_len);
        let block_size = number(self.block_size);
        let message_len_bytes = number(self.message_len_bytes);

        let mut insecure = vec![];

        if self.insecure_against_collision {
            insecure.push("collision");
        }
        if self.insecure_against_length_extension {
            insecure.push("length extension");
        }

        tokens.extend(
            quote! { #ident, #name (sizes: #block_size, #digest_len, #message_len_bytes) (insecure_against: #(#insecure),*) },
        );
    }
}
