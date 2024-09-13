#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
mod generate;
#[cfg(feature = "std")]
pub use generate::*;

#[macro_export]
macro_rules! esp_config_int {
    ($ty:ty, $var:expr) => {
        match <$ty>::from_str_radix(env!($var), 10) {
            Ok(val) => val,
            _ => unreachable!(),
        };
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
