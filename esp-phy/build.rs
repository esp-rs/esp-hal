use std::error::Error as StdError;

use esp_config::generate_config_from_yaml_definition;

fn main() -> Result<(), Box<dyn StdError>> {
    // Load the configuration for the configured device:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-phy");
    let _cfg = generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    let linker_script = "phy_provides.x";
    let out_file = out.join("libesp-phy.a");
    let _ = std::fs::copy(linker_script, out_file);
    println!("cargo:rerun-if-changed={linker_script}");
    // exploit the fact that linkers treat an unknown library format as a linker
    // script
    println!("cargo:rustc-link-lib=esp-phy");

    Ok(())
}
