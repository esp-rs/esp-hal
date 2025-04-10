use std::{error::Error as StdError, str::FromStr};

use esp_build::assert_unique_used_features;
use esp_config::{generate_config, ConfigOption, Error, Stability, Validator, Value};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn StdError>> {
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
    let crate_config = generate_config(
        "esp_hal_embassy",
        &[
            ConfigOption {
                name: "low-power-wait",
                description: "Enables the lower-power wait if no tasks are ready to run on the \
                thread-mode executor. This allows the MCU to use less power if the workload allows. \
                Recommended for battery-powered systems. May impact analog performance.",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
            },
            ConfigOption {
                name: "timer-queue",
                description: "<p>The flavour of the timer queue provided by this crate. Accepts \
                one of `single-integrated`, `multiple-integrated` or `generic`. Integrated queues \
                require the `executors` feature to be enabled.</p><p>If you use embassy-executor, \
                the `single-integrated` queue is recommended for ease of use, while the \
                `multiple-integrated` queue is recommended for performance. The \
                `multiple-integrated` option needs one timer per executor.</p><p>The `generic` \
                queue allows using embassy-time without the embassy executors.</p>",
                default_value: Value::String(if cfg!(feature = "executors") {
                    String::from("single-integrated")
                } else {
                    String::from("generic")
                }),
                constraint: Some(Validator::Custom(Box::new(|value| {
                    let Value::String(string) = value else {
                        return Err(Error::Validation(String::from("Expected a string")));
                    };

                    if !cfg!(feature = "executors") {
                        if string.as_str() != "generic" {
                            return Err(Error::Validation(format!("Expected 'generic' because the `executors` feature is not enabled. Found {string}")));
                        }
                        return Ok(());
                    }

                    match string.as_str() {
                        "single-integrated" => Ok(()), // preferred for ease of use
                        "multiple-integrated" => Ok(()), // preferred for performance
                        "generic" => Ok(()), // allows using embassy-time without the embassy executors
                        _ => Err(Error::Validation(format!("Expected 'single-integrated', 'multiple-integrated' or 'generic', found {string}")))
                    }
                }))),
                stability: Stability::Unstable,
            },
            ConfigOption {
                name: "generic-queue-size",
                description: "The capacity of the queue when the `generic` timer \
                queue flavour is selected.",
                default_value: Value::Integer(64),
                constraint: Some(Validator::PositiveInteger),
                stability: Stability::Unstable,
            },
        ],
        true,
        true,
    );

    println!("cargo:rustc-check-cfg=cfg(integrated_timers)");
    println!("cargo:rustc-check-cfg=cfg(single_queue)");
    println!("cargo:rustc-check-cfg=cfg(generic_timers)");

    match &crate_config["ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE"] {
        Value::String(s) if s.as_str() == "single-integrated" => {
            println!("cargo:rustc-cfg=integrated_timers");
            println!("cargo:rustc-cfg=single_queue");
        }
        Value::String(s) if s.as_str() == "multiple-integrated" => {
            println!("cargo:rustc-cfg=integrated_timers");
        }
        Value::String(s) if s.as_str() == "generic" => {
            println!("cargo:rustc-cfg=generic_timers");
            println!("cargo:rustc-cfg=single_queue");
        }
        _ => unreachable!(),
    }

    Ok(())
}
