use std::{env, path::PathBuf};

use esp_config::{ConfigOption, Stability, Validator, Value, generate_config};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    generate_config(
        "esp_ieee802154",
        &[ConfigOption {
            name: "rx_queue_size",
            description: "Size of the RX queue in frames",
            default_value: Value::Integer(50),
            constraint: Some(Validator::PositiveInteger),
            stability: Stability::Unstable,
            active: true,
        }],
        true,
        true,
    );
}
