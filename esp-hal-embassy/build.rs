use std::error::Error as StdError;

use esp_config::{Value, generate_config_from_yaml_definition};

fn main() -> Result<(), Box<dyn StdError>> {
    // Load the configuration for the configured device:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-hal-embassy");
    let crate_config =
        generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    println!("cargo:rustc-check-cfg=cfg(integrated_timers)");
    println!("cargo:rustc-check-cfg=cfg(single_queue)");
    println!("cargo:rustc-check-cfg=cfg(generic_timers)");

    match &crate_config["ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE"] {
        Value::String(s) if s.as_str() == "single-integrated" => {
            println!("cargo:rustc-cfg=integrated_timers");
            println!("cargo:rustc-cfg=single_queue");
        }
        Value::String(s) if s.as_str() == "multiple-integrated" => {
            println!("cargo:rustc-cfg=integrated_timers");
        }
        Value::String(s) if s.as_str() == "generic" => {
            println!("cargo:rustc-cfg=generic_timers");
            println!("cargo:rustc-cfg=single_queue");
        }
        _ => unreachable!(),
    }

    Ok(())
}
