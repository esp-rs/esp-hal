use std::{env, path::PathBuf};

use esp_config::{ConfigOption, Validator, generate_config};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // emit config
    generate_config(
        "esp_ieee802154",
        &[
            ConfigOption::new("rx_queue_size", "Size of the RX queue in frames", 50)
                .constraint(Validator::PositiveInteger),
        ],
        true,
        true,
    );
}
