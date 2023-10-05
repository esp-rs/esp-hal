use std::{
    env,
    error::Error,
    fs::{self, File},
    io::Write,
    path::PathBuf,
};

fn main() -> Result<(), Box<dyn Error>> {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    fs::copy("ld/memory.x", out.join("memory.x"))?;
    fs::copy("ld/link-esp32.x", out.join("link-esp32.x"))?;
    fs::copy("ld/linkall.x", out.join("linkall.x"))?;

    fs::copy("ld/rom-functions.x", out.join("rom-functions.x"))?;

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
    
    /* reserved at the start of the RTC memories for use by the ULP processor */
    RESERVE_RTC_FAST = 0;
    RESERVE_RTC_SLOW = 0;
        "
    )
    .as_bytes()
    .to_vec()
}
