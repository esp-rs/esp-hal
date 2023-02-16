use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    check_features();
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/memory.x"))
        .unwrap();

    File::create(out.join("alias.x"))
        .unwrap()
        .write_all(include_bytes!("ld/rom.x"))
        .unwrap();

    File::create(out.join("hal-defaults.x"))
        .unwrap()
        .write_all(include_bytes!("ld/hal-defaults.x"))
        .unwrap();

    File::create(out.join("rom-functions.x"))
        .unwrap()
        .write_all(include_bytes!("ld/rom-functions.x"))
        .unwrap();

    File::create(out.join("linkall.x"))
        .unwrap()
        .write_all(include_bytes!("ld/linkall.x"))
        .unwrap();

    let memory_extras = generate_memory_extras();
    File::create(out.join("memory_extras.x"))
        .unwrap()
        .write_all(&memory_extras)
        .unwrap();

    File::create(out.join("link-esp32.x"))
        .unwrap()
        .write_all(include_bytes!("ld/link-esp32.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");
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
    
    /* define stack size for both cores */
    STACK_SIZE = 8k;"
    )
    .as_bytes()
    .to_vec()
}

fn check_features() {
    if cfg!(feature = "esp32_40mhz") && cfg!(feature = "esp32_26mhz") {
        panic!("Only one xtal speed feature can be selected");
    }
}
