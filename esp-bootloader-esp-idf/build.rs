use std::env;

use chrono::{TimeZone, Utc};
use esp_config::{generate_config, ConfigOption, Stability, Validator, Value};

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
    generate_config(
        "esp-bootloader-esp-idf",
        &[
            ConfigOption {
                name: "mmu_page_size",
                description: "ESP32-C2, ESP32-C6 and ESP32-H2 support configurable page sizes. \
                This is currently only used to populate the app descriptor.",
                default_value: Value::String(String::from("64k")),
                constraint: Some(Validator::Enumeration(vec![
                    String::from("8k"),
                    String::from("16k"),
                    String::from("32k"),
                    String::from("64k"),
                ])),
                stability: Stability::Unstable,
                active: true, // TODO we need to know the device here
            },
            ConfigOption {
                name: "esp_idf_version",
                description: "ESP-IDF version used in the application descriptor. Currently it's \
                not checked by the bootloader.",
                default_value: Value::String(String::from("0.0.0")),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "partition-table-offset",
                description: "The address of partition table (by default 0x8000). Allows you to \
                move the partition table, it gives more space for the bootloader. Note that the \
                bootloader and app will both need to be compiled with the same |
                PARTITION_TABLE_OFFSET value.",
                default_value: Value::Integer(0x8000),
                constraint: Some(Validator::PositiveInteger),
                stability: Stability::Unstable,
                active: true,
            },
        ],
        true,
        true,
    );
}
