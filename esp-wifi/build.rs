use std::{error::Error, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_config::{ConfigOption, Stability, Validator, Value, generate_config};
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
    generate_config(
        "esp_wifi",
        &[
            ConfigOption {
                name: "rx_queue_size",
                description: "Size of the RX queue in frames",
                default_value: Value::Integer(5),
                constraint: Some(Validator::PositiveInteger),
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "tx_queue_size",
                description: "Size of the TX queue in frames",
                default_value: Value::Integer(3),
                constraint: Some(Validator::PositiveInteger),
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "static_rx_buf_num",
                description: "WiFi static RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Integer(10),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "dynamic_rx_buf_num",
                description: "WiFi dynamic RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Integer(32),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "static_tx_buf_num",
                description: "WiFi static TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Integer(0),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "dynamic_tx_buf_num",
                description: "WiFi dynamic TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Integer(32),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "ampdu_rx_enable",
                description: "WiFi AMPDU RX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "ampdu_tx_enable",
                description: "WiFi AMPDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "amsdu_tx_enable",
                description: "WiFi AMSDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Bool(false),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "rx_ba_win",
                description: "WiFi Block Ack RX window size. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                default_value: Value::Integer(6),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "max_burst_size",
                description: "See [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_burst_size)",
                default_value: Value::Integer(1),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "country_code",
                description: "Country code. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
                default_value: Value::String("CN".to_owned()),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "country_code_operating_class",
                description: "If not 0: Operating Class table number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
                default_value: Value::Integer(0),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "mtu",
                description: "MTU, see [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_transmission_unit)",
                default_value: Value::Integer(1492),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "tick_rate_hz",
                description: "Tick rate of the internal task scheduler in hertz",
                default_value: Value::Integer(100),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "listen_interval",
                description: "Interval for station to listen to beacon from AP. The unit of listen interval is one beacon interval. For example, if beacon interval is 100 ms and listen interval is 3, the interval for station to listen to beacon is 300 ms",
                default_value: Value::Integer(3),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "beacon_timeout",
                description: "For Station, If the station does not receive a beacon frame from the connected SoftAP during the  inactive time, disconnect from SoftAP. Default 6s. Range 6-30",
                default_value: Value::Integer(6),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "ap_beacon_timeout",
                description: "For SoftAP, If the SoftAP doesn't receive any data from the connected STA during inactive time, the SoftAP will force deauth the STA. Default is 300s",
                default_value: Value::Integer(300),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "failure_retry_cnt",
                description: "Number of connection retries station will do before moving to next AP. scan_method should be set as WIFI_ALL_CHANNEL_SCAN to use this config. Note: Enabling this may cause connection time to increase incase best AP doesn't behave properly. Defaults to 1",
                default_value: Value::Integer(1),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "scan_method",
                description: "0 = WIFI_FAST_SCAN, 1 = WIFI_ALL_CHANNEL_SCAN, defaults to 0",
                default_value: Value::Integer(0),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "dump_packets",
                description: "Dump packets via an info log statement",
                default_value: Value::Bool(false),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "phy_enable_usb",
                description: "Keeps USB running when using WiFi. This allows debugging and log messages via USB Serial JTAG. Turn off for best WiFi performance.",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
            ConfigOption {
                name: "initial_power_save_mode",
                description: "Which power save mode to use when initializing.",
                default_value: Value::String(String::from("none")),
                constraint: Some(Validator::Enumeration(vec![String::from("none"), String::from("min"), String::from("max")])),
            },
            ConfigOption {
                name: "phy_skip_calibration_after_deep_sleep",
                description: "Use PHY_RF_CAL_NONE after deep sleep.",
                default_value: Value::Bool(false),
                constraint: None
            },
            ConfigOption {
                name: "phy_full_calibration",
                description: "Use PHY_RF_CAL_FULL instead of PHY_RF_CAL_PARTIAL.",
                default_value: Value::Bool(true),
                constraint: None
            },
        ],
        true,
        true,
    );

    Ok(())
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

fn print_warning(message: impl core::fmt::Display) {
    println!("cargo:warning={}", message);
}
