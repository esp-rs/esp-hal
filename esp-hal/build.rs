use std::{
    env,
    error::Error,
    fs::{self, File},
    io::{BufRead, Write},
    path::{Path, PathBuf},
    str::FromStr,
};

use esp_build::assert_unique_used_features;
use esp_config::{generate_config, Validator, Value};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rustc-check-cfg=cfg(is_debug_build)");
    if let Ok(level) = std::env::var("OPT_LEVEL") {
        if level == "0" || level == "1" {
            println!("cargo:rustc-cfg=is_debug_build");
        }
    }

    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32s2", "esp32s3"
    );

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
    let chip = Chip::from_str(device_name)?;
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    let cfg = generate_config("esp_hal", &[
        (
            "place-spi-driver-in-ram",
            "Places the SPI driver in RAM for better performance",
            Value::Bool(false),
            None
        ),
        (
            "place-switch-tables-in-ram",
            "Places switch-tables, some lookup tables and constants related to interrupt handling into RAM - resulting in better performance but slightly more RAM consumption.",
            Value::Bool(true),
            None
        ),
        (
            "place-anon-in-ram",
            "Places anonymous symbols into RAM - resulting in better performance at the cost of significant more RAM consumption. Best to be combined with `place-switch-tables-in-ram`.",
            Value::Bool(false),
            None
        ),
        // Ideally, we should be able to set any clock frequency for any chip. However, currently
        // only the 32 and C2 implements any sort of configurability, and the rest have a fixed
        // clock frequeny.
        // TODO: only show this configuration for chips that have multiple valid options.
        (
            "xtal-frequency",
            "The frequency of the crystal oscillator, in MHz. Set to `auto` to automatically detect the frequency. `auto` may not be able to identify the clock frequency in some cases. Also, configuring a specific frequency may increase performance slightly.",
            Value::String(match device_name {
                "esp32" | "esp32c2" => String::from("auto"),
                // The rest has only one option
                "esp32c3" | "esp32c6" | "esp32s2" | "esp32s3" => String::from("40"),
                "esp32h2" => String::from("32"),
                _ => unreachable!(),
            }),
            Some(Validator::Enumeration(match device_name {
                "esp32" | "esp32c2" => vec![String::from("auto"), String::from("26"), String::from("40")],
                // The rest has only one option
                "esp32c3" | "esp32c6" | "esp32s2" | "esp32s3" => vec![String::from("40")],
                "esp32h2" => vec![String::from("32")],
                _ => unreachable!(),
            })),
        ),
        // ideally we should only offer this for ESP32 but the config system doesn't
        // support per target configs, yet
        (
            "spi-address-workaround",
            "(ESP32 only) Enables a workaround for the issue where SPI in half-duplex mode incorrectly transmits the address on a single line if the data buffer is empty.",
            Value::Bool(true),
            None
        ),
        // ideally we should only offer this for ESP32-C6/ESP32-H2 but the config system doesn't support per target configs, yet
        (
            "flip-link",
            "(ESP32-C6/ESP32-H2 only): Move the stack to start of RAM to get zero-cost stack overflow protection.",
            Value::Bool(false),
            None
        ),
        // ideally we should only offer this for ESP32, ESP32-S2 and `octal` only for ESP32-S3 but the config system doesn't support per target configs, yet
        (
            "psram-mode",
            "(ESP32, ESP32-S2 and ESP32-S3 only, `octal` is only supported for ESP32-S3) SPIRAM chip mode",
            Value::String(String::from("quad")),
            Some(Validator::Enumeration(
                vec![String::from("quad"), String::from("octal")]
            )),
        ),
        (
            "generate-esp-idf-app-desc",
            "Generate esp-idf compatible app descriptor.",
            Value::Bool(false),
            None
        ),
        (
            "esp-idf-build-time",
            "Build time",
            Value::String(String::from("00:00:00")),
            None
        ),
        (
            "esp-idf-build-date",
            "Build data",
            Value::String(String::from("1970-01-01")),
            None
        ),
        (
            "esp-idf-build-time",
            "Build time",
            Value::String(String::from("00:00:00")),
            None
        ),
        (
            "esp-idf-app-version",
            "Version",
            Value::String(String::from("0.0.0")),
            None
        ),
        (
            "esp-idf-app-name",
            "Project name",
            Value::String(String::from("esp-hal project")),
            None
        ),
    ], true);

    // RISC-V and Xtensa devices each require some special handling and processing
    // of linker scripts:

    #[allow(unused_mut)]
    let mut config_symbols = config.all().collect::<Vec<_>>();

    for (key, value) in &cfg {
        if let Value::Bool(true) = value {
            config_symbols.push(key);
        }
    }

    if cfg!(feature = "esp32") || cfg!(feature = "esp32s2") || cfg!(feature = "esp32s3") {
        // Xtensa devices:

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
            REGION_ALIAS("RTC_FAST_RWTEXT", {});
            REGION_ALIAS("RTC_FAST_RWDATA", {});
        "#,
            irtc, drtc
        );

        fs::write(out.join("alias.x"), alias)?;
        fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x"))?;
    } else {
        // RISC-V devices:

        preprocess_file(&config_symbols, "ld/riscv/asserts.x", out.join("asserts.x"))?;
        preprocess_file(&config_symbols, "ld/riscv/debug.x", out.join("debug.x"))?;
        preprocess_file(
            &config_symbols,
            "ld/riscv/hal-defaults.x",
            out.join("hal-defaults.x"),
        )?;
    }

    // With the architecture-specific linker scripts taken care of, we can copy all
    // remaining linker scripts which are common to all devices:
    copy_dir_all(&config_symbols, "ld/sections", &out)?;
    copy_dir_all(&config_symbols, format!("ld/{device_name}"), &out)?;

    if let Some(esp_idf_app_desc) = generate_idf_app_desc_if_requested(&cfg) {
        File::create(out.join("rodata_desc.x"))?.write_all(&esp_idf_app_desc)?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

fn copy_dir_all(
    config_symbols: &[&str],
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

fn generate_idf_app_desc_if_requested(
    cfg: &std::collections::HashMap<String, Value>,
) -> Option<Vec<u8>> {
    if let Value::Bool(false) = cfg["ESP_HAL_CONFIG_GENERATE_ESP_IDF_APP_DESC"] {
        return None;
    }

    const ESP_APP_DESC_MAGIC_WORD: u32 = 0xABCD5432;

    #[repr(C)]
    struct EspAppDesc {
        magic_word: u32,                       // Magic word ESP_APP_DESC_MAGIC_WORD
        secure_version: u32,                   // Secure version
        reserv1: [u32; 2],                     // reserv1
        version: [core::ffi::c_char; 32],      // Application version
        project_name: [core::ffi::c_char; 32], // Project name
        time: [core::ffi::c_char; 16],         // Compile time
        date: [core::ffi::c_char; 16],         // Compile date
        idf_ver: [core::ffi::c_char; 32],      // Version IDF
        app_elf_sha256: [u8; 32],              // sha256 of elf file
        min_efuse_blk_rev_full: u16,           // Minimal eFuse block revision supported by image
        max_efuse_blk_rev_full: u16,           // Maximal eFuse block revision supported by image
        mmu_page_size: u8,                     // MMU page size in log base 2 format
        reserv3: [u8; 3],                      // reserv3
        reserv2: [u32; 18],                    // reserv2
    }

    fn str_to_cstr_array<const C: usize>(s: &str) -> [::core::ffi::c_char; C] {
        let bytes = s.as_bytes();
        if bytes.len() >= C {
            assert!(true, "String is too long for the C-string field");
        }

        let mut ret: [::core::ffi::c_char; C] = [0; C];
        let mut i = 0;
        loop {
            ret[i] = bytes[i] as _;
            i += 1;
            if i >= bytes.len() {
                break;
            }
        }
        ret
    }

    let app_desc = EspAppDesc {
        magic_word: ESP_APP_DESC_MAGIC_WORD,
        secure_version: 0,
        reserv1: [0; 2],
        version: str_to_cstr_array(
            cfg["ESP_HAL_CONFIG_ESP_IDF_APP_VERSION"]
                .as_string()
                .as_str(),
        ),
        project_name: str_to_cstr_array(
            cfg["ESP_HAL_CONFIG_ESP_IDF_APP_NAME"].as_string().as_str(),
        ),
        time: str_to_cstr_array(
            cfg["ESP_HAL_CONFIG_ESP_IDF_BUILD_TIME"]
                .as_string()
                .as_str(),
        ),
        date: str_to_cstr_array(
            cfg["ESP_HAL_CONFIG_ESP_IDF_BUILD_DATE"]
                .as_string()
                .as_str(),
        ),
        // just pretending some esp-idf version here
        idf_ver: str_to_cstr_array("5.3.1"),
        app_elf_sha256: [0; 32],
        min_efuse_blk_rev_full: 0,
        max_efuse_blk_rev_full: u16::MAX,
        mmu_page_size: 0,
        reserv3: [0; 3],
        reserv2: [0; 18],
    };

    let app_desc = unsafe {
        core::slice::from_raw_parts(
            &app_desc as *const _ as *const u8,
            core::mem::size_of::<EspAppDesc>(),
        )
    }
    .to_vec();

    let mut desc_linker_script = String::new();

    desc_linker_script.push_str("SECTIONS { .rodata_desc  : ALIGN(4) {");
    for byte in app_desc {
        desc_linker_script.push_str(&format!("BYTE({byte})"));
    }
    desc_linker_script.push_str("} > RODATA }");

    Some(desc_linker_script.as_bytes().to_vec())
}
