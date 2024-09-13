#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
mod generate;
#[cfg(feature = "std")]
pub use generate::*;

#[macro_export]
// TODO from 1.82 we can use <$ty>::from_str_radix(env!($var), 10) instead
macro_rules! esp_config_int {
    ($ty:ty, $var:expr) => {
        const {
            let mut bytes = env!($var).as_bytes();
            let mut val: $ty = 0;
            while let [byte, rest @ ..] = bytes {
                core::assert!(b'0' <= *byte && *byte <= b'9', "invalid digit");
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
