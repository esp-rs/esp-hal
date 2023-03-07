use std::{env, fs::File, io::Write, path::PathBuf};

#[cfg(feature = "direct-boot")]
fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-esp32c6-memory.x"))
        .unwrap();

    File::create(out.join("esp32c6-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-esp32c6-link.x"))
        .unwrap();

    File::create(out.join("riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-riscv-link.x"))
        .unwrap();

    File::create(out.join("linkall.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-linkall.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    add_defaults();
}

#[cfg(not(feature = "direct-boot"))]
fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/bl-esp32c6-memory.x"))
        .unwrap();

    File::create(out.join("bl-riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/bl-riscv-link.x"))
        .unwrap();

    File::create(out.join("linkall.x"))
        .unwrap()
        .write_all(include_bytes!("ld/bl-linkall.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");

    add_defaults();
}

fn add_defaults() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("rom-functions.x"))
        .unwrap()
        .write_all(include_bytes!("ld/rom-functions.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
}
