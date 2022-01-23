use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    File::create(out.join("alias.x"))
        .unwrap()
        .write_all(include_bytes!("rom.x"))
        .unwrap();

    File::create(out.join("hal-defaults.x"))
        .unwrap()
        .write_all(include_bytes!("hal-defaults.x"))
        .unwrap();

    println!("cargo:rustc-link-arg=-Thal-defaults.x");

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");
}
