use std::{error::Error, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    // Ensure that only a single chip is specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if cfg!(feature = "custom-halt") && cfg!(feature = "halt-cores") {
        panic!("Only one of `custom-halt` and `halt-cores` can be enabled");
    }

    if (cfg!(feature = "coredump") || cfg!(feature = "coredump-all"))
        && !cfg!(feature = "exception-handler")
    {
        panic!("coredumps need the `exception-handler` feature");
    }

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
    config.define_symbols();

    Ok(())
}
