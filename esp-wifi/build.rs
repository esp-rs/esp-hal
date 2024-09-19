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
        &[
            ("rx_queue_size", Value::Number(5), "Size of the RX queue in frames"),
            ("tx_queue_size", Value::Number(3), "Size of the TX queue in frames"),
            ("static_rx_buf_num", Value::Number(10), "WiFi static RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("dynamic_rx_buf_num", Value::Number(32), "WiFi dynamic RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("static_tx_buf_num", Value::Number(0), "WiFi static TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("dynamic_tx_buf_num", Value::Number(32), "WiFi dynamic TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("ampdu_rx_enable", Value::Bool(false), "WiFi AMPDU RX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("ampdu_tx_enable", Value::Bool(false), "WiFi AMPDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("amsdu_tx_enable", Value::Bool(false), "WiFi AMSDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("rx_ba_win", Value::Number(6), "WiFi Block Ack RX window size. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("max_burst_size", Value::Number(1), "See [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_burst_size)"),
            (
                "country_code",
                Value::String("CN".to_owned()),
                "Country code. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
            ),
            (
                "country_code_operating_class",
                Value::Number(0),
                "If not 0: Operating Class table number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
            ),
            ("mtu", Value::Number(1492), "MTU, see [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_transmission_unit)"),
            ("tick_rate_hz", Value::Number(100), "Tick rate of the internal task scheduler in hertz"),
            ("listen_interval", Value::Number(3), "Interval for station to listen to beacon from AP. The unit of listen interval is one beacon interval. For example, if beacon interval is 100 ms and listen interval is 3, the interval for station to listen to beacon is 300 ms"),
            ("beacon_timeout", Value::Number(6), "For Station, If the station does not receive a beacon frame from the connected SoftAP during the  inactive time, disconnect from SoftAP. Default 6s. Range 6-30"),
            ("ap_beacon_timeout", Value::Number(300), "For SoftAP, If the SoftAP doesn’t receive any data from the connected STA during inactive time, the SoftAP will force deauth the STA. Default is 300s"),
            ("failure_retry_cnt", Value::Number(1), "Number of connection retries station will do before moving to next AP. scan_method should be set as WIFI_ALL_CHANNEL_SCAN to use this config. Note: Enabling this may cause connection time to increase incase best AP doesn't behave properly. Defaults to 1"),
            ("scan_method", Value::Number(0), "0 = WIFI_FAST_SCAN, 1 = WIFI_ALL_CHANNEL_SCAN, defaults to 0"),
        ],
        true
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
