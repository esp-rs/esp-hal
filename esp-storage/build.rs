fn main() -> Result<(), String> {
    // Ensure that only a single chip is specified
    if !cfg!(feature = "emulation") {
        let _ = esp_metadata::Chip::from_cargo_feature().map_err(|e| format!("{e:?}"))?;
    }

    if cfg!(feature = "esp32") {
        match std::env::var("OPT_LEVEL") {
            Ok(level) if std::env::var("CI").is_err() => {
                if level != "2" && level != "3" {
                    Err(format!(
                        "Building esp-storage for ESP32 needs optimization level 2 or 3 - yours is {level}. See https://github.com/esp-rs/esp-storage"
                    ))
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
