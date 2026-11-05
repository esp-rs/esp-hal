use std::{
    env,
    error::Error,
    fs,
    path::{Path, PathBuf},
};

use esp_config::generate_config_from_yaml_definition;
use esp_metadata_generated::{Chip, assert_unique_features};
use jiff::Timestamp;

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo::rustc-check-cfg=cfg(embedded_test)");

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    // `SOURCE_DATE_EPOCH` is defined by the reproducible-builds spec as the number of
    // *seconds* since the Unix epoch.
    println!("cargo:rerun-if-env-changed=SOURCE_DATE_EPOCH");
    let build_time = match env::var("SOURCE_DATE_EPOCH") {
        Ok(val) => Timestamp::from_second(val.parse::<i64>()?)?,
        Err(_) => Timestamp::now(),
    };

    let build_time_formatted = build_time.strftime("%H:%M:%S");
    let build_date_formatted = build_time.strftime("%Y-%m-%d");

    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_TIME={build_time_formatted}");
    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_DATE={build_date_formatted}");

    // Ensure that exactly one chip has been specified (unless the "std" feature is enabled)
    let chip = if !cfg!(feature = "std") {
        Some(Chip::from_cargo_feature().unwrap())
    } else {
        None
    };

    // Partition and OTA access is built on `esp_hal::flash`. Host tests replace it with a mock.
    println!("cargo::rustc-check-cfg=cfg(flash_driver_supported)");
    if chip.is_none_or(|chip| chip.contains("flash_driver_supported")) {
        println!("cargo::rustc-cfg=flash_driver_supported");
    }

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-bootloader-esp-idf");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, chip).unwrap();

    if let Some(chip) = chip {
        emit_linker_scripts(chip)?;
    }

    Ok(())
}

/// Copies the linker scripts describing the memory layout the ESP-IDF 2nd-stage bootloader expects
/// into `OUT_DIR`. They include `hal-defaults.x` and the `esp_hal_*.x` fragments from esp-hal's
/// `OUT_DIR`, so no file here may share a name with one there.
fn emit_linker_scripts(chip: Chip) -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    if chip.is_xtensa() {
        let (irtc, drtc) = if chip == Chip::Esp32s3 {
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
        copy_dir_all("ld/xtensa", &out)?;
    } else {
        copy_dir_all("ld/riscv", &out)?;
    }

    copy_dir_all("ld/sections", &out)?;
    copy_dir_all(format!("ld/{}", chip.name()), &out)?;

    Ok(())
}

fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let dst = dst.as_ref().join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_dir_all(entry.path(), dst)?;
        } else {
            println!("cargo:rerun-if-changed={}", entry.path().display());
            fs::copy(entry.path(), dst)?;
        }
    }
    Ok(())
}
