fn main() -> Result<(), String> {
    // Ensure that only a single chip is specified
    if !cfg!(feature = "emulation") {
        let _ = esp_metadata::Chip::from_cargo_feature().map_err(|e| format!("{e:?}"))?;
    }

    if cfg!(feature = "esp32") {
        // for now only use `libesp_rom.a` for the original ESP32
        let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
        println!("cargo:rustc-link-search={}", out.display());
        std::fs::copy("./libs/esp32/libesp_rom.a", out.join("libesp_rom.a")).unwrap();
        println!("cargo:rustc-link-lib=esp_rom");

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
