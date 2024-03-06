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
    ( $all:tt ) => {
        #[cfg(not(feature = $all))]
        compile_error!(concat!("The feature flag must be provided: ", $all));
    };

    ( $($all:tt),+ ) => {
        #[cfg(not(any($(feature = $all),*)))]
        compile_error!(concat!("One of the feature flags must be provided: ", $($all, ", "),*));
    };
}

// Given some features, assert that EXACTLY one of the features is enabled.
macro_rules! assert_unique_used_features {
    ( $($all:tt),* ) => {
        assert_unique_features!($($all),*);
        assert_used_features!($($all),*);
    }
}

#[derive(Debug, Deserialize, PartialEq)]
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
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // If the `embassy` feature is enabled, ensure that a time driver implementation
    // is available:
    #[cfg(feature = "embassy")]
    {
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                assert_unique_used_features!("embassy-time-timg0");
            } else if #[cfg(feature = "esp32s2")] {
                assert_unique_used_features!("embassy-time-systick-80mhz", "embassy-time-timg0");
            } else {
                assert_unique_used_features!("embassy-time-systick-16mhz", "embassy-time-timg0");
            }
        }
    }

    #[cfg(feature = "flip-link")]
    {
        #[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
        panic!("flip-link is only available on ESP32-C6/ESP32-H2");
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
    } else if cfg!(feature = "esp32p4") {
        "esp32p4"
    } else if cfg!(feature = "esp32s2") {
        "esp32s2"
    } else if cfg!(feature = "esp32s3") {
        "esp32s3"
    } else {
        unreachable!() // We've confirmed exactly one known device was selected
    };

    if detect_atomic_extension("a") || detect_atomic_extension("s32c1i") {
        panic!(
            "Atomic emulation flags detected in `.cargo/config.toml`, this is no longer supported!"
        );
    }

    // Load the configuration file for the configured device:
    let chip_toml_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("devices")
        .join(device_name)
        .join("device.toml")
        .canonicalize()?;

    let config = fs::read_to_string(chip_toml_path)?;
    let config: Config = basic_toml::from_str(&config)?;
    let device = &config.device;

    // Check PSRAM features are only given if the target supports PSRAM:
    if !&device.symbols.contains(&String::from("psram"))
        && (cfg!(feature = "psram-2m") || cfg!(feature = "psram-4m") || cfg!(feature = "psram-8m"))
    {
        panic!("The target does not support PSRAM");
    }

    // Don't support "interrupt-preemption" and "direct-vectoring" on Xtensa and
    // RISC-V with CLIC:
    if (device.symbols.contains(&String::from("clic")) || device.arch == Arch::Xtensa)
        && (cfg!(feature = "direct-vectoring") || cfg!(feature = "interrupt-preemption"))
    {
        panic!("The target does not support interrupt-preemption and direct-vectoring");
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

    let mut config_symbols = Vec::new();
    let arch = device.arch.to_string();
    let cores = device.cores.to_string();
    config_symbols.push(device_name);
    config_symbols.push(&arch);
    config_symbols.push(&cores);

    for peripheral in &device.peripherals {
        config_symbols.push(peripheral);
    }

    for symbol in &device.symbols {
        config_symbols.push(symbol);
    }

    #[cfg(feature = "flip-link")]
    config_symbols.push("flip-link");

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    if cfg!(feature = "esp32") || cfg!(feature = "esp32s2") || cfg!(feature = "esp32s3") {
        fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x"))?;

        let (irtc, drtc) = if cfg!(feature = "esp32s3") {
            ("rtc_fast_seg", "rtc_fast_seg")
        } else {
            ("rtc_fast_iram_seg", "rtc_fast_dram_seg")
        };

        let alias = format!(
            r#"
            REGION_ALIAS("ROTEXT", irom_seg);
            REGION_ALIAS("RWTEXT", iram_seg);
            REGION_ALIAS("RODATA", drom_seg);
            REGION_ALIAS("RWDATA", dram_seg);
            REGION_ALIAS("RTC_FAST_RWTEXT", {});
            REGION_ALIAS("RTC_FAST_RWDATA", {});
        "#,
            irtc, drtc
        );

        fs::write(out.join("alias.x"), alias)?;
    } else {
        preprocess_file(
            &config_symbols,
            "ld/riscv/hal-defaults.x",
            out.join("hal-defaults.x"),
        )?;
        preprocess_file(&config_symbols, "ld/riscv/asserts.x", out.join("asserts.x"))?;
        preprocess_file(&config_symbols, "ld/riscv/debug.x", out.join("debug.x"))?;
    }

    copy_dir_all(&config_symbols, "ld/sections", &out)?;
    copy_dir_all(&config_symbols, format!("ld/{device_name}"), &out)?;

    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    File::create(out.join("memory_extras.x"))?.write_all(&generate_memory_extras())?;

    // Generate the eFuse table from the selected device's CSV file:
    gen_efuse_table(device_name, out)?;

    Ok(())
}

fn copy_dir_all(
    config_symbols: &Vec<&str>,
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> std::io::Result<()> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(
                config_symbols,
                entry.path(),
                dst.as_ref().join(entry.file_name()),
            )?;
        } else {
            preprocess_file(
                config_symbols,
                entry.path(),
                dst.as_ref().join(entry.file_name()),
            )?;
        }
    }
    Ok(())
}

/// A naive pre-processor for linker scripts
fn preprocess_file(
    config: &[&str],
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> std::io::Result<()> {
    let file = File::open(src)?;
    let mut out_file = File::create(dst)?;

    let mut take = Vec::new();
    take.push(true);

    for line in std::io::BufReader::new(file).lines() {
        let line = line?;
        let trimmed = line.trim();

        if let Some(stripped) = trimmed.strip_prefix("#IF ") {
            let condition = stripped;
            let should_take = take.iter().all(|v| *v);
            let should_take = should_take && config.contains(&condition);
            take.push(should_take);
            continue;
        } else if trimmed == "#ELSE" {
            let taken = take.pop().unwrap();
            let should_take = take.iter().all(|v| *v);
            let should_take = should_take && !taken;
            take.push(should_take);
            continue;
        } else if trimmed == "#ENDIF" {
            take.pop();
            continue;
        }

        if *take.last().unwrap() {
            out_file.write_all(line.as_bytes())?;
            let _ = out_file.write(b"\n")?;
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
        if line.ends_with('\n') {
            line.pop();
            if line.ends_with('\r') {
                line.pop();
            }
        }
        // drop comment and trim
        line.truncate(
            if let Some((pfx, _cmt)) = line.split_once('#') {
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

        let mut fields = line.split(',');
        match (
            fields.next().map(|s| s.trim().replace('.', "_")),
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

fn detect_atomic_extension(ext: &str) -> bool {
    let rustflags = env::var_os("CARGO_ENCODED_RUSTFLAGS")
        .unwrap()
        .into_string()
        .unwrap();

    // Users can pass -Ctarget-feature to the compiler multiple times, so we have to
    // handle that
    let target_flags = rustflags
        .split(0x1f as char)
        .filter_map(|s| s.strip_prefix("target-feature="));
    for tf in target_flags {
        let tf = tf
            .split(',')
            .map(|s| s.trim())
            .filter_map(|s| s.strip_prefix('+'));
        for tf in tf {
            if tf == ext {
                return true;
            }
        }
    }

    false
}

#[cfg(feature = "esp32")]
fn generate_memory_extras() -> Vec<u8> {
    let reserve_dram = if cfg!(feature = "bluetooth") {
        "0x10000"
    } else {
        "0x0"
    };

    format!(
        "
    /* reserved at the start of DRAM for e.g. the BT stack */
    RESERVE_DRAM = {reserve_dram};
        "
    )
    .as_bytes()
    .to_vec()
}

#[cfg(feature = "esp32s2")]
fn generate_memory_extras() -> Vec<u8> {
    let reserved_cache = if cfg!(any(
        feature = "psram-2m",
        feature = "psram-4m",
        feature = "psram-8m"
    )) {
        "0x4000"
    } else {
        "0x2000"
    };

    format!(
        "
        /* reserved at the start of DRAM/IRAM */
        RESERVE_CACHES = {reserved_cache};
        "
    )
    .as_bytes()
    .to_vec()
}
