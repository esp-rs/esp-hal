//! Links the ESP-SR libraries that `tools/vendor.py` prepared from an ESP-SR release.

use std::{env, path::PathBuf};

/// The libraries MultiNet needs, in `lib/<chip>`.
const LIBRARIES: [&str; 6] = [
    "multinet",
    "flite_g2p",
    "fst",
    "dl_lib",
    "c_speech_features",
    "hufzip",
];

fn main() {
    // Host tests do not link the libraries.
    if env::var("CARGO_CFG_TARGET_ARCH").unwrap() != "riscv32" {
        return;
    }
    let chip = if env::var_os("CARGO_FEATURE_ESP32S31").is_some() {
        "esp32s31"
    } else {
        panic!("Enable the feature of the target chip, esp32s31");
    };
    let libraries = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("lib")
        .join(chip);
    println!("cargo:rerun-if-changed={}", libraries.display());
    println!("cargo:rustc-link-search=native={}", libraries.display());
    for library in LIBRARIES {
        println!("cargo:rustc-link-lib=static={library}");
    }
}
