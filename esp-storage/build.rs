use esp_metadata_generated::Chip;

fn main() -> Result<(), String> {
    println!("cargo::rustc-check-cfg=cfg(multi_core)");
    if !cfg!(feature = "emulation") {
        // Load the configuration file for the configured device:
        let chip = Chip::from_cargo_feature()?;

        // Define all necessary configuration symbols for the configured device:
        chip.define_cfgs();
    }
    if cfg!(feature = "esp32") {
        match std::env::var("OPT_LEVEL") {
            Ok(level) if std::env::var("CI").is_err() => {
                if level != "2" && level != "3" && level != "s" {
                    Err(format!(
                        "Building esp-storage for ESP32 needs optimization level 2, 3 or s - yours is {level}. See https://github.com/esp-rs/esp-storage"
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
