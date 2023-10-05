use std::{env, error::Error, fs, path::PathBuf};

#[cfg(feature = "direct-boot")]
fn main() -> Result<(), Box<dyn Error>> {
    check_features();

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/db-esp32c2-memory.x", out.join("memory.x")).unwrap();
    fs::copy("ld/db-esp32c2-link.x", out.join("esp32c2-link.x")).unwrap();
    fs::copy("ld/db-riscv-link.x", out.join("riscv-link.x")).unwrap();
    fs::copy("ld/db-linkall.x", out.join("linkall.x")).unwrap();

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}

#[cfg(not(feature = "direct-boot"))]
fn main() -> Result<(), Box<dyn Error>> {
    check_features();

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/bl-esp32c2-memory.x", out.join("memory.x")).unwrap();
    fs::copy("ld/bl-riscv-link.x", out.join("bl-riscv-link.x")).unwrap();
    fs::copy("ld/bl-linkall.x", out.join("linkall.x")).unwrap();

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}

fn check_features() {
    if cfg!(feature = "xtal-40mhz") && cfg!(feature = "xtal-26mhz") {
        panic!("Only one xtal speed feature can be selected");
    }
}
