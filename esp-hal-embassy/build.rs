use std::error::Error as StdError;

use esp_config::{ConfigOption, Stability, Validator, Value, generate_config};
use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn StdError>> {
    // Load the configuration file for the configured device:
    let chip = Chip::from_cargo_feature()?;
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
                active: true,
            },
            ConfigOption {
                name: "timer-queue",
                description: "The flavour of the timer queue provided by this crate. Integrated \
                queues require the `executors` feature to be enabled.</p><p>If you use \
                embassy-executor, the `single-integrated` queue is recommended for ease of use, \
                while the `multiple-integrated` queue is recommended for performance. The \
                `multiple-integrated` option needs one timer per executor.</p><p>The `generic` \
                queue allows using embassy-time without the embassy executors.",
                default_value: Value::String(if cfg!(feature = "executors") {
                    String::from("single-integrated")
                } else {
                    String::from("generic")
                }),
                constraint: Some(if cfg!(feature = "executors") {
                    Validator::Enumeration(vec![
                        String::from("generic"),
                        String::from("single-integrated"),
                        String::from("multiple-integrated"),
                    ])
                } else {
                    Validator::Enumeration(vec![String::from("generic")])
                }),
                stability: Stability::Unstable,
                active: cfg!(feature = "executors"),
            },
            ConfigOption {
                name: "generic-queue-size",
                description: "The capacity of the queue when the `generic` timer \
                queue flavour is selected.",
                default_value: Value::Integer(64),
                constraint: Some(Validator::PositiveInteger),
                stability: Stability::Unstable,
                active: true,
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
