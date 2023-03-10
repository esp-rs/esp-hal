use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/memory.x"))
        .unwrap();

    File::create(out.join("rom-functions.x"))
        .unwrap()
        .write_all(include_bytes!("ld/rom-functions.x"))
        .unwrap();

    File::create(out.join("linkall.x"))
        .unwrap()
        .write_all(include_bytes!("ld/linkall.x"))
        .unwrap();

    File::create(out.join("link-esp32s2.x"))
        .unwrap()
        .write_all(include_bytes!("ld/link-esp32s2.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=ld/memory.x");
}
