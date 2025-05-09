use esp_build::assert_unique_used_features;
use esp_config::generate_config_from_yaml_definition;

fn main() {
    // Ensure that only a single chip is specified:
    let _ = esp_metadata::Chip::from_cargo_feature().unwrap();

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if cfg!(feature = "custom-halt") && cfg!(feature = "halt-cores") {
        panic!("Only one of `custom-halt` and `halt-cores` can be enabled");
    }

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml").unwrap();
    generate_config_from_yaml_definition(&cfg_yaml, true, true, None).unwrap();
}
