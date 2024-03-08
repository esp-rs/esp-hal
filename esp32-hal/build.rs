use std::{env, error::Error, fs::File, io::Write, path::PathBuf};

build_alert::yellow! {"

WARNING: package deprecated!

  The 'esp32-hal' package has been deprecated in favour of 'esp-hal'.

  Please refer to the migration guide for help with updating your projects
  to use the new 'esp-hal' package:

  https://github.com/esp-rs/esp-hal/releases/tag/v0.16.0

"}

fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    let memory_extras = generate_memory_extras();
    File::create(out.join("memory_extras.x"))?.write_all(&memory_extras)?;

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    Ok(())
}

fn generate_memory_extras() -> Vec<u8> {
    let reserve_dram = if cfg!(feature = "bluetooth") {
        "0x10000"
    } else {
        "0x0"
    };

    format!(
        "
    /* reserved at the start of DRAM for e.g. the BT stack */
    RESERVE_DRAM = {reserve_dram};
        "
    )
    .as_bytes()
    .to_vec()
}
