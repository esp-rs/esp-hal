use std::{env, error::Error, fs, path::PathBuf};

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
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!("esp32c6", "esp32s2", "esp32s3");

    // NOTE: update when adding new device support!
    // Determine the name of the configured device:
    let device_name = if cfg!(feature = "esp32c6") {
        "esp32c6"
    } else if cfg!(feature = "esp32s2") {
        "esp32s2"
    } else if cfg!(feature = "esp32s3") {
        "esp32s3"
    } else {
        unreachable!() // We've confirmed exactly one known device was selected
    };

    // Define all necessary configuration symbols for the configured device:
    println!("cargo:rustc-cfg={device_name}");

    // Put the linker script somewhere the linker can find it:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // Copy the required linker script to the `out` directory:
    if cfg!(feature = "esp32c6") {
        fs::copy("ld/link-lp.x", out.join("link.x"))?;
        println!("cargo:rerun-if-changed=ld/link-lp.x");
    } else if cfg!(feature = "esp32s2") || cfg!(feature = "esp32s3") {
        fs::copy("ld/link-ulp.x", out.join("link.x"))?;
        println!("cargo:rerun-if-changed=ld/link-ulp.x");
    }

    // Done!
    Ok(())
}
