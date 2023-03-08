use std::{env, fs::File, io::Write, path::PathBuf, process::exit};

// Thanks to kennytm and TheDan64 for the assert_used_features macro.
// Source:
// https://github.com/TheDan64/inkwell/blob/36c3b106e61b1b45295a35f94023d93d9328c76f/src/lib.rs#L81-L110
macro_rules! assert_unique_features {
    () => {};
    ($first:tt $(,$rest:tt)*) => {
        $(
            #[cfg(all(feature = $first, feature = $rest))]
            compile_error!(concat!("Features \"", $first, "\" and \"", $rest, "\" cannot be used together"));
        )*
        assert_unique_features!($($rest),*);
    }
}

assert_unique_features! {"mcu-boot", "direct-boot"}

#[cfg(feature = "direct-boot")]
fn main() {
    check_opt_level();

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-esp32c3-memory.x"))
        .unwrap();

    File::create(out.join("esp32c3-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/db-esp32c3-link.x"))
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

#[cfg(not(any(feature = "mcu-boot", feature = "direct-boot")))]
fn main() {
    check_opt_level();

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/bl-esp32c3-memory.x"))
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

#[cfg(feature = "mcu-boot")]
fn main() {
    check_opt_level();

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("ld/mb-esp32c3-memory.x"))
        .unwrap();

    File::create(out.join("esp32c3-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/mb-esp32c3-link.x"))
        .unwrap();

    File::create(out.join("riscv-link.x"))
        .unwrap()
        .write_all(include_bytes!("ld/mb-riscv-link.x"))
        .unwrap();

    File::create(out.join("linkall.x"))
        .unwrap()
        .write_all(include_bytes!("ld/mb-linkall.x"))
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

const OPT_LEVEL_Z_MSG: &str = r#"opt-level=z will produce broken 128-bit shifts (i.e. `1u128 << i`). The hal's interrupt handling relies on that operation, causing an 'attempt to subtract with overflow' panic if an enabled interrupt is triggered while using that opt-level.

Please use `opt-level="s"` in lieu of "z", or alternatively enable `features = ["allow-opt-level-z"]` to suppress this error. The latter option is only recommended if you:

  * Do not use interrupts, and
  * Do not have any shifts of 128-bit integers (either u128 or i128) in your code

See also: https://github.com/esp-rs/esp-hal/issues/196
     and: https://github.com/llvm/llvm-project/issues/57988         
"#;

// Once a rust nightly has a fix for https://github.com/llvm/llvm-project/issues/57988 , consider:
//   1. Removing this check in favor of bumping the minimum rust verison, and/or
//   2. Augmenting this check to ensure that version for opt-level=z
fn check_opt_level() {
    if cfg!(feature = "allow-opt-level-z") {
        return;
    }

    if let Some(ref opt_level) = env::var_os("OPT_LEVEL") {
        if opt_level == "z" {
            println!(
                "{}",
                OPT_LEVEL_Z_MSG
                    .lines()
                    .into_iter()
                    .map(|l| format!("cargo:warning={l}"))
                    .collect::<Vec<String>>()
                    .join("\n")
            );
            exit(1);
        }
    }
}
