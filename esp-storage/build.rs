fn main() -> Result<(), String> {
    // Ensure that only a single chip is specified
    esp_build::assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    if cfg!(feature = "esp32") {
        match std::env::var("OPT_LEVEL") {
            Ok(level) if std::env::var("CI").is_err() => {
                if level != "2" && level != "3" {
                    Err(format!("Building esp-storage for ESP32 needs optimization level 2 or 3 - yours is {}. See https://github.com/esp-rs/esp-storage", level))
                } else {
                    Ok(())
                }
            }
            _ => Ok(()),
        }
    } else {
        Ok(())
    }
}
