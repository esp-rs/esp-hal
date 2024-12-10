use std::{error::Error, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_config::{generate_config, Validator, Value};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
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

    // emit config
    generate_config(
        "esp_hal_embassy",
        &[(
            "low-power-wait",
            "Enables the lower-power wait if no tasks are ready to run on the thread-mode executor. This allows the MCU to use less power if the workload allows. Recommended for battery-powered systems. May impact analog performance.",
            Value::Bool(true),
            None
        ),
        (
            "generic-queue-size",
            "The size of the generic queue. Only used if `generic-queue` is enabled.",
            Value::Integer(64),
            Some(Validator::PositiveInteger),
        )],
        true,
    );

    println!("cargo:rustc-check-cfg=cfg(integrated_timers)");
    println!("cargo:rustc-check-cfg=cfg(single_queue)");
    println!("cargo:rustc-check-cfg=cfg(generic_timers)");

    if cfg!(feature = "generic-queue") {
        println!("cargo:rustc-cfg=generic_timers");
        println!("cargo:rustc-cfg=single_queue");
    } else {
        println!("cargo:rustc-cfg=integrated_timers");
        if cfg!(feature = "single-queue") {
            println!("cargo:rustc-cfg=single_queue");
        }
    }

    Ok(())
}
