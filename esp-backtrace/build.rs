use esp_build::assert_unique_used_features;
use esp_config::{generate_config, Value};

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
        &[(
            "backtrace-frames",
            "The maximum number of frames that will be printed in a backtrace",
            Value::Integer(10),
            None,
        )],
        true,
    );
}
