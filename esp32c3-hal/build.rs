use std::{env, error::Error, fs, path::PathBuf};

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

assert_unique_features! {"direct-boot", "mcu-boot"}

#[cfg(not(any(feature = "direct-boot", feature = "mcu-boot")))]
fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/bl-esp32c3-memory.x", out.join("memory.x"))?;
    fs::copy("ld/bl-riscv-link.x", out.join("bl-riscv-link.x"))?;
    fs::copy("ld/bl-linkall.x", out.join("linkall.x"))?;

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}

#[cfg(feature = "direct-boot")]
fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/db-esp32c3-memory.x", out.join("memory.x"))?;
    fs::copy("ld/db-esp32c3-link.x", out.join("esp32c3-link.x"))?;
    fs::copy("ld/db-riscv-link.x", out.join("riscv-link.x"))?;
    fs::copy("ld/db-linkall.x", out.join("linkall.x"))?;

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}

#[cfg(feature = "mcu-boot")]
fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/mb-esp32c3-memory.x", out.join("memory.x"))?;
    fs::copy("ld/mb-esp32c3-link.x", out.join("esp32c3-link.x"))?;
    fs::copy("ld/mb-riscv-link.x", out.join("riscv-link.x"))?;
    fs::copy("ld/mb-linkall.x", out.join("linkall.x"))?;

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}
