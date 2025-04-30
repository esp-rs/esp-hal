use std::{
    collections::HashMap,
    env,
    error::Error,
    fs::{self, File},
    io::{BufRead, Write},
    path::{Path, PathBuf},
};

use esp_build::assert_unique_features;
use esp_config::{ConfigOption, Stability, Validator, Value, generate_config};
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
    let cfg = generate_config(
        "esp_hal",
        &[
            ConfigOption {
                name: "place-spi-master-driver-in-ram",
                description: "Places the SPI master driver in RAM for better performance",
                default_value: Value::Bool(false),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "place-switch-tables-in-ram",
                description: "Places switch-tables, some lookup tables and constants related to \
                interrupt handling into RAM - resulting in better performance but slightly more \
                RAM consumption.",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Stable("1.0.0-beta.0"),
                active: true,
            },
            ConfigOption {
                name: "place-anon-in-ram",
                description: "Places anonymous symbols into RAM - resulting in better performance \
                at the cost of significant more RAM consumption. Best to be combined with \
                `place-switch-tables-in-ram`.",
                default_value: Value::Bool(false),
                constraint: None,
                stability: Stability::Stable("1.0.0-beta.0"),
                active: true,
            },
            // Ideally, we should be able to set any clock frequency for any chip. However,
            // currently only the 32 and C2 implements any sort of configurability, and
            // the rest have a fixed clock frequeny.
            ConfigOption {
                name: "xtal-frequency",
                description: "The frequency of the crystal oscillator, in MHz. Set to `auto` to \
                automatically detect the frequency. `auto` may not be able to identify the clock \
                frequency in some cases. Also, configuring a specific frequency may increase \
                performance slightly.",
                default_value: Value::String(match chip {
                    Chip::Esp32 | Chip::Esp32c2 => String::from("auto"),
                    // The rest has only one option
                    Chip::Esp32c3 | Chip::Esp32c6 | Chip::Esp32s2 | Chip::Esp32s3 => {
                        String::from("40")
                    }
                    Chip::Esp32h2 => String::from("32"),
                }),
                constraint: match chip {
                    Chip::Esp32 | Chip::Esp32c2 => Some(Validator::Enumeration(vec![
                        String::from("auto"),
                        String::from("26"),
                        String::from("40"),
                    ])),
                    // The rest has only one option
                    _ => None,
                },
                stability: Stability::Unstable,
                active: [Chip::Esp32, Chip::Esp32c2].contains(&chip),
            },
            ConfigOption {
                name: "spi-address-workaround",
                description: "Enables a workaround for the issue where SPI in \
                half-duplex mode incorrectly transmits the address on a single line if the \
                data buffer is empty.",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: chip == Chip::Esp32,
            },
            ConfigOption {
                name: "flip-link",
                description: "Move the stack to start of RAM to get zero-cost stack overflow protection.",
                default_value: Value::Bool(false),
                constraint: None,
                stability: Stability::Unstable,
                active: [Chip::Esp32c6, Chip::Esp32h2].contains(&chip),
            },
            // TODO: automate "enum of single choice" handling - they don't need
            // to be presented to the user
            ConfigOption {
                name: "psram-mode",
                description: "SPIRAM chip mode",
                default_value: Value::String(String::from("quad")),
                constraint: Some(Validator::Enumeration(
                    if config
                        .symbols()
                        .iter()
                        .any(|s| s.eq_ignore_ascii_case("octal_psram"))
                    {
                        vec![String::from("quad"), String::from("octal")]
                    } else {
                        vec![String::from("quad")]
                    },
                )),
                stability: Stability::Unstable,
                active: config
                    .symbols()
                    .iter()
                    .any(|s| s.eq_ignore_ascii_case("psram")),
            },
            // Rust's stack smashing protection configuration
            ConfigOption {
                name: "stack-guard-offset",
                description: "The stack guard variable will be placed this many bytes from \
                the stack's end.",
                default_value: Value::Integer(4096),
                constraint: None,
                stability: Stability::Stable("1.0.0-beta.0"),
                active: true,
            },
            ConfigOption {
                name: "stack-guard-value",
                description: "The value to be written to the stack guard variable.",
                default_value: Value::Integer(0xDEED_BAAD),
                constraint: None,
                stability: Stability::Stable("1.0.0-beta.0"),
                active: true,
            },
            ConfigOption {
                name: "impl-critical-section",
                description: "Provide a `critical-section` implementation. Note that if disabled, \
                you will need to provide a `critical-section` implementation which is \
                using `restore-state-u32`.",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
        ],
        cfg!(feature = "unstable"),
        true,
    );

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
    copy_dir_all(&config_symbols, &cfg, format!("ld/{}", chip), &out)?;

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
