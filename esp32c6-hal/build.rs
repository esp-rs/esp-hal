use std::{env, error::Error, fs, path::PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/bl-esp32c6-memory.x", out.join("memory.x"))?;
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
