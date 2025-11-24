use std::error::Error;

use esp_config::generate_config_from_yaml_definition;

#[macro_export]
macro_rules! assert_unique_features {
    ($($feature:literal),+ $(,)?) => {
        assert!(
            (0 $(+ cfg!(feature = $feature) as usize)+ ) <= 1,
            "At most one of the following features must be enabled: {}",
            [$($feature),+].join(", ")
        );
    };
}

#[macro_export]
macro_rules! assert_unique_used_features {
    ($($feature:literal),+ $(,)?) => {
        assert!(
            (0 $(+ cfg!(feature = $feature) as usize)+ ) == 1,
            "Exactly one of the following features must be enabled: {}",
            [$($feature),+].join(", ")
        );
    };
}

fn main() -> Result<(), Box<dyn Error>> {
    // Ensure that exactly one chip has been specified:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    // Ensure that there aren't multiple halt methods selected:
    assert_unique_features!("custom-halt", "halt-cores", "semihosting");

    if !cfg!(feature = "panic-handler")
        && cfg!(any(
            feature = "custom-halt",
            feature = "halt-cores",
            feature = "semihosting"
        ))
    {
        print_warning("A halt method is selected, but esp-backtrace is not the panic handler.")
    }

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-backtrace");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip))?;

    println!("cargo::rustc-check-cfg=cfg(stack_dump)");
    if !chip.is_xtensa()
        && !std::env::var("CARGO_ENCODED_RUSTFLAGS")
            .unwrap_or_default()
            .contains("force-frame-pointers")
    {
        println!("cargo::rustc-cfg=stack_dump");
    }

    Ok(())
}

fn print_warning(message: impl core::fmt::Display) {
    println!("cargo:warning={message}");
}
