use esp_build::assert_unique_used_features;

fn main() {
    // Ensure that only a single chip is specified
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // Ensure that only a single communication method is specified
    assert_unique_used_features!("jtag-serial", "uart", "auto");

    // Ensure that, if the `jtag-serial` communication method feature is enabled,
    // a compatible chip feature is also enabled.
    if cfg!(feature = "jtag-serial")
        && !(cfg!(feature = "esp32c3")
            || cfg!(feature = "esp32c6")
            || cfg!(feature = "esp32h2")
            || cfg!(feature = "esp32p4")
            || cfg!(feature = "esp32s3"))
    {
        panic!(
            "The `jtag-serial` feature is only supported by the ESP32-C3, ESP32-C6, ESP32-H2, ESP32-P4, and ESP32-S3"
        );
    }

    // Ensure that, if the `colors` is used with `log`.`
    if cfg!(feature = "colors") && !cfg!(feature = "log") {
        println!(
            "cargo:warning=The `colors` feature is only effective when using the `log` feature"
        );
    }
}
