#[cfg(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32h2",
    feature = "esp32s2",
    feature = "esp32s3",
))]
fn main() -> Result<(), String> {
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
    #[cfg(all(feature = "coex", any(feature = "esp32s2")))]
    {
        panic!(
            r#"

        COEX is not yet supported on this target.

        See https://github.com/esp-rs/esp-wifi/issues/92.

        "#
        );
    }
    match std::env::var("OPT_LEVEL") {
        Ok(level) => {
            if level != "2" && level != "3" {
                let message = format!(
                    "esp-wifi should be built with optimization level 2 or 3 - yours is {level}.
                    See https://github.com/esp-rs/esp-wifi",
                );
                print_warning(message);
            }
        }
        Err(_err) => (),
    }

    #[cfg(feature = "esp32")]
    println!("cargo:rustc-cfg=esp32");

    #[cfg(feature = "esp32c2")]
    println!("cargo:rustc-cfg=esp32c2");

    #[cfg(feature = "esp32c3")]
    println!("cargo:rustc-cfg=esp32c3");

    #[cfg(feature = "esp32c6")]
    println!("cargo:rustc-cfg=esp32c6");

    #[cfg(feature = "esp32h2")]
    println!("cargo:rustc-cfg=esp32h2");

    #[cfg(feature = "esp32s2")]
    println!("cargo:rustc-cfg=esp32s2");

    #[cfg(feature = "esp32s3")]
    println!("cargo:rustc-cfg=esp32s3");

    #[cfg(feature = "coex")]
    {
        #[cfg(all(feature = "wifi", feature = "ble"))]
        println!("cargo:rustc-cfg=coex");

        #[cfg(not(feature = "wifi"))]
        println!("cargo:warning=coex is enabled but wifi is not");

        #[cfg(not(feature = "ble"))]
        println!("cargo:warning=coex is enabled but ble is not");
    }

    validate_config();

    let version_output = std::process::Command::new(
        std::env::var_os("RUSTC").unwrap_or_else(|| std::ffi::OsString::from("rustc")),
    )
    .arg("-V")
    .output()
    .unwrap()
    .stdout;
    let version_string = String::from_utf8_lossy(&version_output);

    // HACK: we detect the xtensa-enabled compiler by existence of the second version string in parens
    // - upstream output format: rustc 1.75.0-nightly (cae0791da 2023-10-05)
    // - xtensa output format: rustc 1.73.0-nightly (9163a2087 2023-10-03) (1.73.0.0)
    // - gentoo format (non-xtensa): rustc 1.73.0-nightly (cc66ad468 2023-10-03) (gentoo)
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

#[toml_cfg::toml_config]
/// Tunable parameters for the WiFi driver
struct Config {
    #[default(5)]
    rx_queue_size: usize,
    #[default(3)]
    tx_queue_size: usize,
    #[default(10)]
    static_rx_buf_num: usize,
    #[default(32)]
    dynamic_rx_buf_num: usize,
    #[default(0)]
    static_tx_buf_num: usize,
    #[default(32)]
    dynamic_tx_buf_num: usize,
    #[default(0)]
    ampdu_rx_enable: usize,
    #[default(0)]
    ampdu_tx_enable: usize,
    #[default(0)]
    amsdu_tx_enable: usize,
    #[default(6)]
    rx_ba_win: usize,
    #[default(1)]
    max_burst_size: usize,
    #[default("CN")]
    country_code: &'static str,
    #[default(0)]
    country_code_operating_class: u8,
    #[default(1492)]
    mtu: usize,
    #[default(65536)]
    heap_size: usize,
    #[default(200)]
    tick_rate_hz: u32,
    #[default(3)]
    listen_interval: u16,
    #[default(6)]
    beacon_timeout: u16,
    #[default(300)]
    ap_beacon_timeout: u16,
    #[default(1)]
    failure_retry_cnt: u8,
    #[default(0)]
    scan_method: u32,
}

fn validate_config() {
    if CONFIG.rx_ba_win > CONFIG.dynamic_rx_buf_num {
        print_warning(
            "WiFi configuration check: rx_ba_win should not be larger than dynamic_rx_buf_num!",
        );
    }

    if CONFIG.rx_ba_win > (CONFIG.static_rx_buf_num * 2) {
        print_warning("WiFi configuration check: rx_ba_win should not be larger than double of the static_rx_buf_num!");
    }
}
