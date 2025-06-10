use std::env;

use chrono::{TimeZone, Utc};
use esp_config::generate_config_from_yaml_definition;

fn main() {
    let build_time = match env::var("SOURCE_DATE_EPOCH") {
        Ok(val) => Utc.timestamp_opt(val.parse::<i64>().unwrap(), 0).unwrap(),
        Err(_) => Utc::now(),
    };

    let build_time_formatted = build_time.format("%H:%M:%S").to_string();
    let build_date_formatted = build_time.format("%Y-%m-%d").to_string();

    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_TIME={build_time_formatted}");
    println!("cargo::rustc-env=ESP_BOOTLOADER_BUILD_DATE={build_date_formatted}");

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml fro esp-bootloader-esp-idf");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, None).unwrap();
}
