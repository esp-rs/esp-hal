use std::{env, fs::File, io::Write, path::PathBuf};

#[cfg(not(feature = "normalboot"))]
fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("esp32c3-memory.x"))
        .unwrap();

    File::create(out.join("esp32c3-link.x"))
        .unwrap()
        .write_all(include_bytes!("esp32c3-link.x"))
        .unwrap();

    File::create(out.join("riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("riscv-link.x"))
        .unwrap();        

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Tesp32c3-link.x");
}

#[cfg(feature = "normalboot")]
fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("bl-esp32c3-memory.x"))
        .unwrap();

        File::create(out.join("bl-riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("bl-riscv-link.x"))
        .unwrap(); 

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tbl-riscv-link.x");
}
