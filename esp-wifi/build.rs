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
            ("rx_queue_size", Value::UnsignedInteger(5), "Size of the RX queue in frames"),
            ("tx_queue_size", Value::UnsignedInteger(3), "Size of the TX queue in frames"),
            ("static_rx_buf_num", Value::UnsignedInteger(10), "WiFi static RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("dynamic_rx_buf_num", Value::UnsignedInteger(32), "WiFi dynamic RX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("static_tx_buf_num", Value::UnsignedInteger(0), "WiFi static TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("dynamic_tx_buf_num", Value::UnsignedInteger(32), "WiFi dynamic TX buffer number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("csi_enable", Value::Bool(false), "WiFi channel state information enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("ampdu_rx_enable", Value::Bool(true), "WiFi AMPDU RX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("ampdu_tx_enable", Value::Bool(true), "WiFi AMPDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("amsdu_tx_enable", Value::Bool(false), "WiFi AMSDU TX feature enable flag. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("rx_ba_win", Value::UnsignedInteger(6), "WiFi Block Ack RX window size. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv418wifi_init_config_t)"),
            ("max_burst_size", Value::UnsignedInteger(1), "See [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_burst_size)"),
            (
                "country_code",
                Value::String("CN".to_owned()),
                "Country code. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
            ),
            (
                "country_code_operating_class",
                Value::UnsignedInteger(0),
                "If not 0: Operating Class table number. See [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code)",
            ),
            ("mtu", Value::UnsignedInteger(1492), "MTU, see [smoltcp's documentation](https://docs.rs/smoltcp/0.10.0/smoltcp/phy/struct.DeviceCapabilities.html#structfield.max_transmission_unit)"),
            ("tick_rate_hz", Value::UnsignedInteger(100), "Tick rate of the internal task scheduler in hertz"),
            ("listen_interval", Value::UnsignedInteger(3), "Interval for station to listen to beacon from AP. The unit of listen interval is one beacon interval. For example, if beacon interval is 100 ms and listen interval is 3, the interval for station to listen to beacon is 300 ms"),
            ("beacon_timeout", Value::UnsignedInteger(6), "For Station, If the station does not receive a beacon frame from the connected SoftAP during the  inactive time, disconnect from SoftAP. Default 6s. Range 6-30"),
            ("ap_beacon_timeout", Value::UnsignedInteger(300), "For SoftAP, If the SoftAP doesn’t receive any data from the connected STA during inactive time, the SoftAP will force deauth the STA. Default is 300s"),
            ("failure_retry_cnt", Value::UnsignedInteger(1), "Number of connection retries station will do before moving to next AP. scan_method should be set as WIFI_ALL_CHANNEL_SCAN to use this config. Note: Enabling this may cause connection time to increase incase best AP doesn't behave properly. Defaults to 1"),
            ("scan_method", Value::UnsignedInteger(0), "0 = WIFI_FAST_SCAN, 1 = WIFI_ALL_CHANNEL_SCAN, defaults to 0"),
            ("dump_packets", Value::Bool(false), "Dump packets via an info log statement"),
            ("phy_enable_usb", Value::Bool(true), "Keeps USB running when using WiFi. This allows debugging and log messages via USB Serial JTAG. Turn off for best WiFi performance."),
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
