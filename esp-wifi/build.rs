use std::{error::Error, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_config::{generate_config, Validator, Value};
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
            ("rx_queue_size", "Size of the RX queue in frames", Value::Integer(5), Some(Validator::PositiveInteger)),
            ("tx_queue_size", "Size of the TX queue in frames", Value::Integer(3), Some(Validator::PositiveInteger)),
            (
                "static_rx_buf_num",
                "WiFi static RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Integer(10),
                None
            ),
            (
                "dynamic_rx_buf_num",
                "WiFi dynamic RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Integer(32),
                None
            ),
            (
                "static_tx_buf_num",
                "WiFi static TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Integer(0),
                None
            ),
            (
                "dynamic_tx_buf_num",
                "WiFi dynamic TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Integer(32),
                None
            ),
            (
                "ampdu_rx_enable",
                "WiFi AMPDU RX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Bool(true),
                None
            ),
            (
                "ampdu_tx_enable",
                "WiFi AMPDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Bool(true),
                None
            ),
            (
                "amsdu_tx_enable",
                "WiFi AMSDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Bool(false),
                None
            ),
            (
                "rx_ba_win",
                "WiFi Block Ack RX window size. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)",
                Value::Integer(6),
                None
            ),
            (
                "max_burst_size",
                "See [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_burst_size)",
                Value::Integer(1),
                None
            ),
            (
                "country_code",
                "Country code. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
                Value::String("CN".to_owned()),
                None
            ),
            (
                "country_code_operating_class",
                "If not 0: Operating Class table number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
                Value::Integer(0),
                None
            ),
            (
                "mtu",
                "MTU, see [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_transmission_unit)",
                Value::Integer(1492),
                None
            ),
            (
                "tick_rate_hz",
                "Tick rate of the internal task scheduler in hertz",
                Value::Integer(100),
                None
            ),
            (
                "listen_interval",
                "Interval for station to listen to beacon from AP. The unit of listen interval is one beacon interval. For example, if beacon interval is 100 ms and listen interval is 3, the interval for station to listen to beacon is 300 ms",
                Value::Integer(3),
                None
            ),
            (
                "beacon_timeout",
                "For Station, If the station does not receive a beacon frame from the connected SoftAP during the  inactive time, disconnect from SoftAP. Default 6s. Range 6-30",
                Value::Integer(6),
                None
            ),
            (
                "ap_beacon_timeout",
                "For SoftAP, If the SoftAP doesn't receive any data from the connected STA during inactive time, the SoftAP will force deauth the STA. Default is 300s",
                Value::Integer(300),
                None
            ),
            (
                "failure_retry_cnt",
                "Number of connection retries station will do before moving to next AP. scan_method should be set as WIFI_ALL_CHANNEL_SCAN to use this config. Note: Enabling this may cause connection time to increase incase best AP doesn't behave properly. Defaults to 1",
                Value::Integer(1),
                None
            ),
            (
                "scan_method",
                "0 = WIFI_FAST_SCAN, 1 = WIFI_ALL_CHANNEL_SCAN, defaults to 0",
                Value::Integer(0),
                None
            ),
            (
                "dump_packets",
                "Dump packets via an info log statement",
                Value::Bool(false),
                None
            ),
            (
                "phy_enable_usb",
                "Keeps USB running when using WiFi. This allows debugging and log messages via USB Serial JTAG. Turn off for best WiFi performance.",
                Value::Bool(true),
                None
            ),
        ],
        true
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
