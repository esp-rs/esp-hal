use std::{
    env,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, Write},
    path::{Path, PathBuf},
};

use serde::Deserialize;

// Macros taken from:
// https://github.com/TheDan64/inkwell/blob/36c3b10/src/lib.rs#L81-L110

// Given some features, assert that AT MOST one of the features is enabled.
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

// Given some features, assert that AT LEAST one of the features is enabled.
macro_rules! assert_used_features {
    ( $($all:tt),* ) => {
        #[cfg(not(any($(feature = $all),*)))]
        compile_error!(concat!("One of the feature flags must be provided: ", $($all, ", "),*));
    }
}

// Given some features, assert that EXACTLY one of the features is enabled.
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

impl std::fmt::Display for Arch {
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

impl std::fmt::Display for CoreCount {
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
    pub symbols: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct Config {
    pub device: Device,
}

fn main() -> Result<(), Box<dyn Error>> {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32s2", "esp32s3"
    );

    // Handle the features for the ESP32's and ESP32-C2's different crystal
    // frequencies:
    #[cfg(any(feature = "esp32", feature = "esp32c2"))]
    {
        assert_unique_used_features!("xtal-26mhz", "xtal-40mhz");
    }

    // If the `embassy` feature is enabled, ensure that a time driver implementation
    // is available:
    #[cfg(feature = "embassy")]
    {
        assert_unique_used_features!("embassy-time-systick", "embassy-time-timg0");
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
    } else if cfg!(feature = "esp32h2") {
        "esp32h2"
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
        .join(device_name)
        .join("device.toml")
        .canonicalize()?;

    let config = fs::read_to_string(chip_toml_path)?;
    let config: Config = basic_toml::from_str(&config)?;
    let device = config.device;

    // Check PSRAM features are only given if the target supports PSRAM:
    if !&device.symbols.contains(&String::from("psram"))
        && (cfg!(feature = "psram-2m") || cfg!(feature = "psram-4m") || cfg!(feature = "psram-8m"))
    {
        panic!("The target does not support PSRAM");
    }

    // Define all necessary configuration symbols for the configured device:
    println!("cargo:rustc-cfg={}", device_name);
    println!("cargo:rustc-cfg={}", device.arch);
    println!("cargo:rustc-cfg={}", device.cores);

    for peripheral in &device.peripherals {
        println!("cargo:rustc-cfg={peripheral}");
    }

    for symbol in &device.symbols {
        println!("cargo:rustc-cfg={symbol}");
    }

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    if cfg!(feature = "esp32") || cfg!(feature = "esp32s2") || cfg!(feature = "esp32s3") {
        fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x"))?;
        fs::copy("ld/xtensa/rom.x", out.join("alias.x"))?;
    } else {
        fs::copy("ld/riscv/hal-defaults.x", out.join("hal-defaults.x"))?;
        fs::copy("ld/riscv/asserts.x", out.join("asserts.x"))?;
        fs::copy("ld/riscv/debug.x", out.join("debug.x"))?;
    }

    copy_dir_all("ld/sections", &out)?;

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // Generate the eFuse table from the selected device's CSV file:
    gen_efuse_table(device_name, out)?;

    Ok(())
}

fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> std::io::Result<()> {
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

fn gen_efuse_table(device_name: &str, out_dir: impl AsRef<Path>) -> Result<(), Box<dyn Error>> {
    let src_path = PathBuf::from(format!("devices/{device_name}/efuse.csv"));
    let out_path = out_dir.as_ref().join("efuse_fields.rs");

    println!("cargo:rerun-if-changed={}", src_path.display());

    let mut writer = File::create(out_path)?;
    let mut reader = BufReader::new(File::open(src_path)?);
    let mut line = String::with_capacity(128);

    while reader.read_line(&mut line)? > 0 {
        if line.ends_with("\n") {
            line.pop();
            if line.ends_with("\r") {
                line.pop();
            }
        }
        // drop comment and trim
        line.truncate(
            if let Some((pfx, _cmt)) = line.split_once("#") {
                pfx
            } else {
                &line
            }
            .trim()
            .len(),
        );
        // skip empty
        if line.is_empty() {
            continue;
        }

        let mut fields = line.split(",");
        match (
            fields.next().map(|s| s.trim().replace(".", "_")),
            fields
                .next()
                .map(|s| s.trim().replace(|c: char| !c.is_ascii_digit(), "")),
            fields
                .next()
                .map(|s| s.trim())
                .and_then(|s| s.parse::<u32>().ok()),
            fields
                .next()
                .map(|s| s.trim())
                .and_then(|s| s.parse::<u32>().ok()),
            fields.next().map(|s| s.trim()),
        ) {
            (Some(name), Some(block), Some(bit_off), Some(bit_len), Some(desc)) => {
                let desc = desc.replace('[', "`[").replace(']', "]`");
                writeln!(writer, "/// {desc}")?;
                writeln!(
                    writer,
                    "pub const {name}: EfuseField = EfuseField::new(EfuseBlock::Block{block}, {bit_off}, {bit_len});"
                )?;
            }
            other => eprintln!("Invalid data: {other:?}"),
        }

        line.clear();
    }

    Ok(())
}
