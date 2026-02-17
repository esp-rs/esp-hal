use std::{env, error::Error};

use esp_config::generate_config_from_yaml_definition;
use esp_metadata_generated::assert_unique_features;
use jiff::Timestamp;

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo::rustc-check-cfg=cfg(embedded_test)");

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    let build_time = match env::var("SOURCE_DATE_EPOCH") {
        Ok(val) => Timestamp::from_microsecond(val.parse::<i64>()?).unwrap(),
        Err(_) => Timestamp::now(),
    };

    let build_time_formatted = build_time.strftime("%H:%M:%S");
    let build_date_formatted = build_time.strftime("%Y-%m-%d");

    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_TIME={build_time_formatted}");
    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_DATE={build_date_formatted}");

    // Ensure that exactly one chip has been specified (unless the "std" feature is enabled)
    let chip = if !cfg!(feature = "std") {
        Some(esp_metadata_generated::Chip::from_cargo_feature().unwrap())
    } else {
        None
    };

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-bootloader-esp-idf");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, chip).unwrap();

    Ok(())
}
