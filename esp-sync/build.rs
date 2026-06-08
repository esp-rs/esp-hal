use std::error::Error;

use esp_config::generate_config_from_yaml_definition;

fn main() -> Result<(), Box<dyn Error>> {
    // Define all necessary configuration symbols for the configured device:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;
    chip.define_cfgs();

    // Derive `esp32p4_rev_lt_v3` from the configured minimum chip revision: pre-v3.0
    // ESP32-P4 lacks the `mintthresh` CSR the Zcmp critical-section workaround writes
    // (0x347 traps). Compile-time, since the workaround runs in hot/early paths.
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-sync");
    let cfg = generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    println!("cargo:rustc-check-cfg=cfg(esp32p4_rev_lt_v3)");
    if chip.name() == "esp32p4" {
        let min_rev = cfg
            .iter()
            .find(|(k, _)| k.to_ascii_lowercase().ends_with("min_chip_revision"))
            .map(|(_, v)| v.as_integer())
            .unwrap_or(0);
        if min_rev < 300 {
            println!("cargo:rustc-cfg=esp32p4_rev_lt_v3");
        }
    }

    Ok(())
}
