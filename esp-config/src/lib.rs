#![doc = include_str!("../README.md")]
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![cfg_attr(not(feature = "build"), no_std)]

#[cfg(feature = "build")]
mod generate;
#[cfg(feature = "build")]
pub use generate::*;

#[macro_export]
// TODO from 1.82 we can use <$ty>::from_str_radix(env!($var), 10) instead
macro_rules! esp_config_int {
    ($ty:ty, $var:expr) => {
        const {
            let mut bytes = env!($var).as_bytes();
            let mut val: $ty = 0;
            while let [byte, rest @ ..] = bytes {
                ::core::assert!(b'0' <= *byte && *byte <= b'9', "invalid digit");
                val = val * 10 + (*byte - b'0') as $ty;
                bytes = rest;
            }
            val
        }
    };
}

#[macro_export]
macro_rules! esp_config_str {
    ($var:expr) => {
        env!($var)
    };
}

#[macro_export]
macro_rules! esp_config_bool {
    ($var:expr) => {
        match env!($var).as_bytes() {
            b"false" => false,
            _ => true,
        }
    };
}
