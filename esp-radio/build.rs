use std::{
    collections::HashMap,
    error::Error,
    fs::File,
    io::{BufRead, Write},
    path::Path,
};

use esp_config::{Value, generate_config_from_yaml_definition};
use esp_metadata_generated::Chip;

#[macro_export]
macro_rules! assert_unique_features {
    ($($feature:literal),+ $(,)?) => {
        assert!(
            (0 $(+ cfg!(feature = $feature) as usize)+ ) <= 1,
            "Exactly zero or one of the following features must be enabled: {}",
            [$($feature),+].join(", ")
        );
    };
}

fn main() -> Result<(), Box<dyn Error>> {
    // if using '"rust-analyzer.cargo.buildScripts.useRustcWrapper": true' we can detect this
    let suppress_panics = std::env::var("RUSTC_WRAPPER")
        .unwrap_or_default()
        .contains("rust-analyzer");

    // Load the configuration file for the configured device:
    let chip = Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    // If some library required unstable make sure unstable is actually enabled.
    if !suppress_panics && cfg!(feature = "requires-unstable") && !cfg!(feature = "unstable") {
        panic!(
            "\n\nThe `unstable` feature is required by a dependent crate but is not enabled.\n\n"
        );
    }

    #[cfg(not(feature = "__docs_build"))]
    if cfg!(feature = "ieee802154") && (cfg!(feature = "wifi") || cfg!(feature = "wifi-eap")) {
        panic!("\n\n802.15.4 and Wi-Fi won't work together\n\n");
    }

    if (cfg!(feature = "ble")
        || cfg!(feature = "coex")
        || cfg!(feature = "csi")
        || cfg!(feature = "esp-now")
        || cfg!(feature = "ieee802154")
        || cfg!(feature = "smoltcp")
        || cfg!(feature = "sniffer")
        || cfg!(feature = "wifi-eap"))
        && !cfg!(feature = "unstable")
        && !suppress_panics
    {
        panic!(
            "\n\nThe `unstable` feature was not provided, but is required for the following features: `ble`, `coex`, `csi`, `esp-now`, `ieee802154`, `smoltcp`, `sniffer`, `wifi-eap`.\n\n"
        )
    }

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    assert!(
        !cfg!(feature = "ble") || chip.contains("bt"),
        r#"

        BLE is not supported on this target.

        "#
    );
    assert!(
        !cfg!(feature = "wifi") || chip.contains("wifi"),
        r#"

        Wi-Fi is not supported on this target.

        "#
    );
    assert!(
        !cfg!(feature = "ieee802154") || chip.contains("ieee802154"),
        r#"

        IEEE 802.15.4 is not supported on this target.

        "#
    );

    if let Ok(level) = std::env::var("OPT_LEVEL")
        && level != "2"
        && level != "3"
        && level != "s"
    {
        let message = format!(
            "esp-radio should be built with optimization level 2, 3 or s - yours is {level}.
                See https://github.com/esp-rs/esp-hal/tree/main/esp-radio",
        );
        print_warning(message);
    }

    println!("cargo:rustc-check-cfg=cfg(coex)");
    #[cfg(feature = "coex")]
    {
        assert!(
            chip.contains("wifi") && chip.contains("bt"),
            r#"

            Wi-Fi/Bluetooth coexistence is not supported on this target.

            "#
        );

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
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-radio");
    let cfg = generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=./ld/");
    let config = [
        #[cfg(feature = "wifi")]
        "wifi",
        #[cfg(feature = "ble")]
        "ble",
    ];
    preprocess_file(
        &config,
        &cfg,
        format!("./ld/{}_provides.x", chip.name()),
        out.join("libesp-radio.a"),
    )?;
    // exploit the fact that linkers treat an unknown library format as a linker
    // script
    println!("cargo:rustc-link-lib=esp-radio");

    Ok(())
}

fn print_warning(message: impl core::fmt::Display) {
    println!("cargo:warning={message}");
}

fn preprocess_file(
    config: &[&str],
    cfg: &HashMap<String, Value>,
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> std::io::Result<()> {
    println!("cargo:rerun-if-changed={}", src.as_ref().display());

    let file = File::open(src)?;
    let mut out_file = File::create(dst)?;

    let mut take = Vec::new();
    take.push(true);

    for line in std::io::BufReader::new(file).lines() {
        let line = substitute_config(cfg, &line?);
        let trimmed = line.trim();

        if let Some(condition) = trimmed.strip_prefix("#IF ") {
            let should_take = take.iter().all(|v| *v);
            let should_take = should_take && config.contains(&condition);
            take.push(should_take);
            continue;
        } else if trimmed == "#ELSE" {
            let taken = take.pop().unwrap();
            let should_take = take.iter().all(|v| *v);
            let should_take = should_take && !taken;
            take.push(should_take);
            continue;
        } else if trimmed == "#ENDIF" {
            take.pop();
            continue;
        }

        if *take.last().unwrap() {
            out_file.write_all(line.as_bytes())?;
            let _ = out_file.write(b"\n")?;
        }
    }
    Ok(())
}

fn substitute_config(cfg: &HashMap<String, Value>, line: &str) -> String {
    let mut result = String::new();
    let mut chars = line.chars().peekable();

    while let Some(c) = chars.next() {
        if c != '$' {
            result.push(c);
            continue;
        }

        let Some('{') = chars.peek() else {
            result.push(c);
            continue;
        };
        chars.next();

        let mut key = String::new();
        for c in chars.by_ref() {
            if c == '}' {
                break;
            }
            key.push(c);
        }
        match cfg
            .get(&key)
            .unwrap_or_else(|| panic!("missing config key: {key}"))
        {
            Value::Bool(true) => result.push('1'),
            Value::Bool(false) => result.push('0'),
            Value::Integer(value) => result.push_str(&value.to_string()),
            Value::String(value) => result.push_str(value),
        }
    }

    result
}
