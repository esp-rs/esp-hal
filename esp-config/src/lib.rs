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
