use esp_build::assert_unique_used_features;
use esp_config::{ConfigOption, Stability, Value, generate_config};

fn main() {
    // Ensure that only a single chip is specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if cfg!(feature = "custom-halt") && cfg!(feature = "halt-cores") {
        panic!("Only one of `custom-halt` and `halt-cores` can be enabled");
    }

    // emit config
    generate_config(
        "esp_backtrace",
        &[ConfigOption {
            name: "backtrace-frames",
            description: "The maximum number of frames that will be printed in a backtrace",
            default_value: Value::Integer(10),
            constraint: None,
            stability: Stability::Unstable,
            active: true,
        }],
        true,
        true,
    );
}
