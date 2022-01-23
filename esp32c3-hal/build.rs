use std::{
    env,
    fs::{self, File},
    io::Write,
    path::PathBuf,
};

use riscv_target::Target;

#[cfg(not(feature = "normalboot"))]
fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("db-esp32c3-memory.x"))
        .unwrap();

    File::create(out.join("esp32c3-link.x"))
        .unwrap()
        .write_all(include_bytes!("db-esp32c3-link.x"))
        .unwrap();

    File::create(out.join("riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("db-riscv-link.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Tesp32c3-link.x");

    add_defaults();
    prepare_trap();
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

    add_defaults();
    prepare_trap();
}

fn add_defaults() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("hal-defaults.x"))
        .unwrap()
        .write_all(include_bytes!("hal-defaults.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Thal-defaults.x");
}

fn prepare_trap() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let name = env::var("CARGO_PKG_NAME").unwrap();
    let target = env::var("TARGET").unwrap();

    if target.starts_with("riscv") {
        let mut target = Target::from_target_str(&target);
        target.retain_extensions("if");

        let target = target.to_string();

        fs::copy(
            format!("bin/asm_{}.a", target),
            out.join(format!("lib{}.a", name)),
        )
        .unwrap();

        println!("cargo:rustc-link-lib=static={}", name);
        println!("cargo:rustc-link-search={}", out.display());
    }
}
