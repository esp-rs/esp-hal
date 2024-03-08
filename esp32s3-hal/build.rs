use std::{env, error::Error, path::PathBuf};

build_alert::yellow! {"

WARNING: package deprecated!

  The 'esp32s3-hal' package has been deprecated in favour of 'esp-hal'.

  Please refer to the migration guide for help with updating your projects
  to use the new 'esp-hal' package:

  https://github.com/esp-rs/esp-hal/releases/tag/v0.16.0

"}

fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}
