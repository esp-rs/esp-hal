use std::{env, error::Error, fs, path::PathBuf};

// Macros taken from:
// https://github.com/TheDan64/inkwell/blob/36c3b10/src/lib.rs#L81-L110

// Given some features, assert that AT MOST one of the features is enabled.
macro_rules! assert_unique_features {
    () => {};

    ( $first:tt $(,$rest:tt)* ) => {
        $(
            #[cfg(all(feature = $first, feature = $rest))]
            compile_error!(concat!("Features \"", $first, "\" and \"", $rest, "\" cannot be used together"));
        )*
        assert_unique_features!($($rest),*);
    };
}

// Given some features, assert that AT LEAST one of the features is enabled.
macro_rules! assert_used_features {
    ( $all:tt ) => {
        #[cfg(not(feature = $all))]
        compile_error!(concat!("The feature flag must be provided: ", $all));
    };

    ( $($all:tt),+ ) => {
        #[cfg(not(any($(feature = $all),*)))]
        compile_error!(concat!("One of the feature flags must be provided: ", $($all, ", "),*));
    };
}

// Given some features, assert that EXACTLY one of the features is enabled.
macro_rules! assert_unique_used_features {
    ( $($all:tt),* ) => {
        assert_unique_features!($($all),*);
        assert_used_features!($($all),*);
    }
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
    println!("cargo:rustc-cfg={}", device_name);

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
