use std::{env, fs, path::PathBuf};

use serde::Deserialize;

// Macros taken from:
// https://github.com/TheDan64/inkwell/blob/36c3b10/src/lib.rs#L81-L110

macro_rules! assert_unique_features {
    () => {};

    ( $first:tt $(,$rest:tt)* ) => {
        $(
            #[cfg(all(feature = $first, feature = $rest))]
            compile_error!(concat!("Features \"", $first, "\" and \"", $rest, "\" cannot be used together"));
        )*
        assert_unique_features!($($rest),*);
    };
}

macro_rules! assert_used_features {
    ( $($all:tt),* ) => {
        #[cfg(not(any($(feature = $all),*)))]
        compile_error!(concat!("One of the feature flags must be provided: ", $($all, ", "),*));
    }
}

macro_rules! assert_unique_used_features {
    ( $($all:tt),* ) => {
        assert_unique_features!($($all),*);
        assert_used_features!($($all),*);
    }
}

#[derive(Debug, Deserialize)]
enum Arch {
    #[serde(rename = "riscv")]
    RiscV,
    #[serde(rename = "xtensa")]
    Xtensa,
}

impl core::fmt::Display for Arch {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Arch::RiscV => "riscv",
                Arch::Xtensa => "xtensa",
            }
        )
    }
}

#[derive(Debug, Deserialize)]
enum CoreCount {
    #[serde(rename = "single_core")]
    Single,
    #[serde(rename = "multi_core")]
    Multi,
}

impl core::fmt::Display for CoreCount {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                CoreCount::Single => "single_core",
                CoreCount::Multi => "multi_core",
            }
        )
    }
}

#[derive(Debug, Deserialize)]
struct Device {
    pub arch: Arch,
    pub cores: CoreCount,
    pub peripherals: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct Config {
    pub device: Device,
}

fn main() {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!("esp32", "esp32c2", "esp32c3", "esp32c6", "esp32s2", "esp32s3");

    // Handle the features for the ESP32's different crystal frequencies:
    #[cfg(feature = "esp32")]
    {
        assert_unique_used_features!("esp32_26mhz", "esp32_40mhz");
    }

    // Handle the features for the ESP32-C2's different crystal frequencies:
    #[cfg(feature = "esp32c2")]
    {
        assert_unique_used_features!("esp32c2_26mhz", "esp32c2_40mhz");
    }

    // NOTE: update when adding new device support!
    // Determine the name of the configured device:
    let device_name = if cfg!(feature = "esp32") {
        "esp32"
    } else if cfg!(feature = "esp32c2") {
        "esp32c2"
    } else if cfg!(feature = "esp32c3") {
        "esp32c3"
    } else if cfg!(feature = "esp32c6") {
        "esp32c6"
    } else if cfg!(feature = "esp32s2") {
        "esp32s2"
    } else if cfg!(feature = "esp32s3") {
        "esp32s3"
    } else {
        unreachable!() // We've confirmed exactly one known device was selected
    };

    // Load the configuration file for the configured device:
    let chip_toml_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("devices")
        .join(format!("{}.toml", device_name))
        .canonicalize()
        .unwrap();

    let config = fs::read_to_string(chip_toml_path).unwrap();
    let config: Config = basic_toml::from_str(&config).unwrap();
    let device = config.device;

    // Define all necessary configuration symbols for the configured device:
    println!("cargo:rustc-cfg={}", device_name);
    println!("cargo:rustc-cfg={}", device.arch);
    println!("cargo:rustc-cfg={}", device.cores);

    for peripheral in device.peripherals {
        println!("cargo:rustc-cfg={peripheral}");
    }

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    if cfg!(feature = "esp32") || cfg!(feature = "esp32s2") || cfg!(feature = "esp32s3") {
        fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x")).unwrap();
        fs::copy("ld/xtensa/rom.x", out.join("alias.x")).unwrap();
    } else {
        fs::copy("ld/riscv/hal-defaults.x", out.join("hal-defaults.x")).unwrap();
        fs::copy("ld/riscv/asserts.x", out.join("asserts.x")).unwrap();
        fs::copy("ld/riscv/debug.x", out.join("debug.x")).unwrap();
    }
    copy_dir_all("ld/sections", out).unwrap();
}

fn copy_dir_all(
    src: impl AsRef<std::path::Path>,
    dst: impl AsRef<std::path::Path>,
) -> std::io::Result<()> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
        }
    }
    Ok(())
}
