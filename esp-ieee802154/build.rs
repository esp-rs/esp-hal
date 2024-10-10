use std::{env, path::PathBuf};

use esp_config::{generate_config, Value};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    generate_config(
        "esp_ieee802154",
        &[(
            "rx_queue_size",
            Value::UnsignedInteger(50),
            "Size of the RX queue in frames",
        )],
        true,
    );
}
