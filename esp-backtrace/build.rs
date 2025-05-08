use esp_build::assert_unique_used_features;
use esp_config::{ConfigOption, generate_config};

fn main() {
    // Ensure that only a single chip is specified:
    let _ = esp_metadata::Chip::from_cargo_feature().unwrap();

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if cfg!(feature = "custom-halt") && cfg!(feature = "halt-cores") {
        panic!("Only one of `custom-halt` and `halt-cores` can be enabled");
    }

    // emit config
    generate_config(
        "esp_backtrace",
        &[ConfigOption::new(
            "backtrace-frames",
            "The maximum number of frames that will be printed in a backtrace",
            10,
        )],
        true,
        true,
    );
}
