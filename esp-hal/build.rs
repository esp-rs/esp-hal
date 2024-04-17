use std::{
    env,
    error::Error,
    fs::{self, File},
    io::{BufRead, Write},
    path::{Path, PathBuf},
    str::FromStr,
};

use esp_build::assert_unique_used_features;
use esp_metadata::{Chip, Config};

#[cfg(debug_assertions)]
esp_build::warning! {"
WARNING: use --release
  We *strongly* recommend using release profile when building esp-hal.
  The dev profile can potentially be one or more orders of magnitude
  slower than release, and may cause issues with timing-senstive
  peripherals and/or devices.
"}

fn main() -> Result<(), Box<dyn Error>> {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32s2", "esp32s3"
    );

    // If the `embassy` feature is enabled, ensure that a time driver implementation
    // is available:
    #[cfg(feature = "embassy")]
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            assert_unique_used_features!("embassy-time-timg0");
        } else if #[cfg(feature = "esp32s2")] {
            assert_unique_used_features!("embassy-time-systick-80mhz", "embassy-time-timg0");
        } else {
            assert_unique_used_features!("embassy-time-systick-16mhz", "embassy-time-timg0");
        }
    }

    #[cfg(all(
        feature = "flip-link",
        not(any(feature = "esp32c6", feature = "esp32h2"))
    ))]
    esp_build::error!("flip-link is only available on ESP32-C6/ESP32-H2");

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

    // Check PSRAM features are only given if the target supports PSRAM:
    if !config.contains(&String::from("psram"))
        && (cfg!(feature = "psram-2m") || cfg!(feature = "psram-4m") || cfg!(feature = "psram-8m"))
    {
        panic!("The target does not support PSRAM");
    }

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    #[allow(unused_mut)]
    let mut config_symbols = config.all();
    #[cfg(feature = "flip-link")]
    config_symbols.push("flip-link".to_owned());

    // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
    // files:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // RISC-V and Xtensa devices each require some special handling and processing
    // of linker scripts:

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

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

fn copy_dir_all(
    config_symbols: &Vec<String>,
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
    config: &[String],
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
            let condition = stripped.to_string();
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
