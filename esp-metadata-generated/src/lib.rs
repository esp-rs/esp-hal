#![cfg_attr(not(feature = "build-script"), no_std)]

#[cfg(not(feature = "build-script"))]
pub mod macros {
    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

#[cfg(not(feature = "build-script"))]
pub mod peripherals {
    include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));
    include!(concat!(env!("OUT_DIR"), "/_generated_peris.rs"));
}

#[cfg(feature = "build-script")]
pub mod build {
    include!(concat!(env!("OUT_DIR"), "/_build_script_utils.rs"));
}
