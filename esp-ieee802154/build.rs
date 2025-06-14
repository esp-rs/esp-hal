use std::{env, path::PathBuf};

use esp_config::generate_config_from_yaml_definition;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-ieee802154");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, None).unwrap();
}
