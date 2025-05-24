use std::error::Error;

use esp_config::generate_config_from_yaml_definition;
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    // Load the configuration file for the configured device:
    let chip = Chip::from_cargo_feature()?;
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    assert!(
        !cfg!(feature = "ble") || config.contains("bt"),
        r#"

        BLE is not supported on this target.

        "#
    );
    assert!(
        !cfg!(feature = "wifi") || config.contains("wifi"),
        r#"

        WiFi is not supported on this target.

        "#
    );
    assert!(
        !cfg!(feature = "coex") || (config.contains("wifi") && config.contains("bt")),
        r#"

        COEX is not supported on this target.

        See https://github.com/esp-rs/esp-wifi/issues/92.

        "#
    );

    if let Ok(level) = std::env::var("OPT_LEVEL") {
        if level != "2" && level != "3" {
            let message = format!(
                "esp-wifi should be built with optimization level 2 or 3 - yours is {level}.
                See https://github.com/esp-rs/esp-wifi",
            );
            print_warning(message);
        }
    }

    println!("cargo:rustc-check-cfg=cfg(coex)");
    #[cfg(feature = "coex")]
    {
        #[cfg(all(feature = "wifi", feature = "ble"))]
        println!("cargo:rustc-cfg=coex");

        #[cfg(not(feature = "wifi"))]
        println!("cargo:warning=coex is enabled but wifi is not");

        #[cfg(not(feature = "ble"))]
        println!("cargo:warning=coex is enabled but ble is not");
    }

    // emit config
    //
    // keep the defaults aligned with `esp_wifi_sys::include::*` e.g.
    // `esp_wifi_sys::include::CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM`
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml").unwrap();
    generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(config.clone())).unwrap();

    Ok(())
}

fn print_warning(message: impl core::fmt::Display) {
    println!("cargo:warning={message}");
}
