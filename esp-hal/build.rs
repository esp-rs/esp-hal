use std::{
    collections::HashMap,
    env,
    error::Error,
    fs::{self, File},
    io::{BufRead, Write},
    path::{Path, PathBuf},
};

use esp_build::assert_unique_features;
use esp_config::{Value, generate_config_from_yaml_definition};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rustc-check-cfg=cfg(is_debug_build)");
    if let Ok(level) = std::env::var("OPT_LEVEL") {
        if level == "0" || level == "1" {
            println!("cargo:rustc-cfg=is_debug_build");
        }
    }

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    // Ensure that exactly one chip has been specified:
    let chip = Chip::from_cargo_feature()?;

    if chip.target() != std::env::var("TARGET").unwrap_or_default().as_str() {
        panic!("
        Seems you are building for an unsupported or wrong target (e.g. the host environment).
        Maybe you are missing the `target` in `.cargo/config.toml` or you have configs overriding it?

        See https://doc.rust-lang.org/cargo/reference/config.html#hierarchical-structure
        ");
    }

    // Load the configuration file for the configured device:
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml").unwrap();
    let cfg =
        generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(config.clone())).unwrap();

    // RISC-V and Xtensa devices each require some special handling and processing
    // of linker scripts:

    #[allow(unused_mut)]
    let mut config_symbols = config.all().collect::<Vec<_>>();

    for (key, value) in &cfg {
        if let Value::Bool(true) = value {
            config_symbols.push(key);
        }
    }

    if chip.is_xtensa() {
        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        File::create(out.join("memory_extras.x"))?.write_all(&generate_memory_extras())?;

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
            REGION_ALIAS("RTC_FAST_RWTEXT", {irtc});
            REGION_ALIAS("RTC_FAST_RWDATA", {drtc});
        "#
        );

        fs::write(out.join("alias.x"), alias)?;
        fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x"))?;
    } else {
        // RISC-V devices:

        preprocess_file(
            &config_symbols,
            &cfg,
            "ld/riscv/asserts.x",
            out.join("asserts.x"),
        )?;
        preprocess_file(
            &config_symbols,
            &cfg,
            "ld/riscv/debug.x",
            out.join("debug.x"),
        )?;
        preprocess_file(
            &config_symbols,
            &cfg,
            "ld/riscv/hal-defaults.x",
            out.join("hal-defaults.x"),
        )?;
    }

    // With the architecture-specific linker scripts taken care of, we can copy all
    // remaining linker scripts which are common to all devices:
    copy_dir_all(&config_symbols, &cfg, "ld/sections", &out)?;
    copy_dir_all(&config_symbols, &cfg, format!("ld/{chip}"), &out)?;

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

fn copy_dir_all(
    config_symbols: &[&str],
    cfg: &HashMap<String, Value>,
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
                cfg,
                entry.path(),
                dst.as_ref().join(entry.file_name()),
            )?;
        } else {
            preprocess_file(
                config_symbols,
                cfg,
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
    cfg: &HashMap<String, Value>,
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> std::io::Result<()> {
    println!("cargo:rerun-if-changed={}", src.as_ref().display());

    let file = File::open(src)?;
    let mut out_file = File::create(dst)?;

    let mut take = Vec::new();
    take.push(true);

    for line in std::io::BufReader::new(file).lines() {
        let line = substitute_config(cfg, &line?);
        let trimmed = line.trim();

        if let Some(condition) = trimmed.strip_prefix("#IF ") {
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

fn substitute_config(cfg: &HashMap<String, Value>, line: &str) -> String {
    let mut result = String::new();
    let mut chars = line.chars().peekable();

    while let Some(c) = chars.next() {
        if c != '$' {
            result.push(c);
            continue;
        }

        let Some('{') = chars.peek() else {
            result.push(c);
            continue;
        };
        chars.next();

        let mut key = String::new();
        for c in chars.by_ref() {
            if c == '}' {
                break;
            }
            key.push(c);
        }
        match cfg
            .get(&key)
            .unwrap_or_else(|| panic!("missing config key: {key}"))
        {
            Value::Bool(true) => result.push('1'),
            Value::Bool(false) => result.push('0'),
            Value::Integer(value) => result.push_str(&value.to_string()),
            Value::String(value) => result.push_str(value),
        }
    }

    result
}

#[cfg(feature = "esp32")]
fn generate_memory_extras() -> Vec<u8> {
    let reserve_dram = if cfg!(feature = "__bluetooth") {
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
    let reserved_cache = if cfg!(feature = "psram") {
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
