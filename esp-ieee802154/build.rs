use std::{env, path::PathBuf};

use esp_config::{generate_config, Validator, Value};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    generate_config(
        "esp_ieee802154",
        &[(
            "rx_queue_size",
            "Size of the RX queue in frames",
            Value::Integer(50),
            Some(Validator::PositiveInteger),
        )],
        true,
    );
}
