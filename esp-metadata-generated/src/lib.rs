#![cfg_attr(not(feature = "build-script"), no_std)]

#[cfg(not(feature = "build-script"))]
include!(concat!(env!("OUT_DIR"), "/_generated.rs"));

#[cfg(feature = "build-script")]
pub mod build {
    include!(concat!(env!("OUT_DIR"), "/_build_script_utils.rs"));
}
