use std::{error::Error, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_config::{generate_config, Value};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    // Ensure that only a single chip is specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32s2", "esp32s3"
    );

    // NOTE: update when adding new device support!
    // Determine the name of the configured device:
    let device_name = if cfg!(feature = "esp32") {
        "esp32"
    } else if cfg!(feature = "esp32c2") {
        "esp32c2"
    } else if cfg!(feature = "esp32c3") {
        "esp32c3"
    } else if cfg!(feature = "esp32c6") {
        "esp32c6"
    } else if cfg!(feature = "esp32h2") {
        "esp32h2"
    } else if cfg!(feature = "esp32s2") {
        "esp32s2"
    } else if cfg!(feature = "esp32s3") {
        "esp32s3"
    } else {
        unreachable!() // We've confirmed exactly one known device was selected
    };

    // Load the configuration file for the configured device:
    let chip = Chip::from_str(device_name)?;
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    #[cfg(all(feature = "ble", feature = "esp32s2"))]
    {
        panic!(
            r#"

        BLE is not supported on this target.

        "#
        );
    }
    #[cfg(all(feature = "wifi", feature = "esp32h2"))]
    {
        panic!(
            r#"

        WiFi is not supported on this target.

        "#
        );
    }
    #[cfg(all(feature = "coex", any(feature = "esp32s2", feature = "esp32h2")))]
    {
        panic!(
            r#"

        COEX is not supported on this target.

        See https://github.com/esp-rs/esp-wifi/issues/92.

        "#
        );
    }
    if let Ok(level) = std::env::var("OPT_LEVEL") {
        if level != "2" && level != "3" {
            let message = format!(
                "esp-wifi should be built with optimization level 2 or 3 - yours is {level}.
                See https://github.com/esp-rs/esp-wifi",
            );
            print_warning(message);
        }
    }

    println!("cargo:rustc-cfg={}", device_name);

    #[cfg(feature = "coex")]
    {
        #[cfg(all(feature = "wifi", feature = "ble"))]
        println!("cargo:rustc-cfg=coex");

        #[cfg(not(feature = "wifi"))]
        println!("cargo:warning=coex is enabled but wifi is not");

        #[cfg(not(feature = "ble"))]
        println!("cargo:warning=coex is enabled but ble is not");
    }

    let version_output = std::process::Command::new(
        std::env::var_os("RUSTC").unwrap_or_else(|| std::ffi::OsString::from("rustc")),
    )
    .arg("-V")
    .output()
    .unwrap()
    .stdout;
    let version_string = String::from_utf8_lossy(&version_output);

    if version_string.contains("nightly") {
        println!("cargo:rustc-cfg=nightly");
    }

    // HACK: we detect the xtensa-enabled compiler by existence of the second
    // version string in parens
    // - upstream output format: rustc 1.75.0-nightly (cae0791da 2023-10-05)
    // - xtensa output format: rustc 1.73.0-nightly (9163a2087 2023-10-03)
    //   (1.73.0.0)
    // - gentoo format (non-xtensa): rustc 1.73.0-nightly (cc66ad468 2023-10-03)
    //   (gentoo)
    if version_string.chars().filter(|&c| c == '(').count() == 2 {
        let version = version_string
            .split('(')
            .last()
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        if let Some(version) = try_read_xtensa_rustc_version(version) {
            if version >= Version4(1, 73, 0, 1)
                // Patch accidentally missing from 1.74.0.0
                && version != Version4(1, 74, 0, 0)
            {
                println!("cargo:rustc-cfg=xtensa_has_vaarg");
            }
        }
    }

    // emit config
    generate_config(
        "esp_wifi",
        &[("dummy", Value::Number(44), "Tests something")],
    );

    Ok(())
}

fn try_read_xtensa_rustc_version(version: &str) -> Option<Version4> {
    let mut version = version.trim_start_matches('v').split('.');

    let major = version.next()?.parse::<u32>().ok()?;
    let minor = version.next()?.parse::<u32>().ok()?;
    let patch = version.next()?.parse::<u32>().ok()?;
    let release = version.next()?.parse::<u32>().ok()?;

    Some(Version4(major, minor, patch, release))
}

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32h2",
    feature = "esp32s2",
    feature = "esp32s3",
)))]
fn main() {
    panic!("Select a chip via it's cargo feature");
}

use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Version4(u32, u32, u32, u32);

impl PartialOrd for Version4 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.0.partial_cmp(&other.0) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.1.partial_cmp(&other.1) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.2.partial_cmp(&other.2) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        self.3.partial_cmp(&other.3)
    }
}

fn print_warning(message: impl core::fmt::Display) {
    println!("cargo:warning={}", message);
}
