use std::{env, fs, path::PathBuf};

// Chips with a 16 KB instruction cache or less. On these chips, code that runs for each Wi-Fi frame
// is slow from flash.
const HEAP_IN_RAM: &[&str] = &["esp32c2", "esp32c3", "esp32s2", "esp32s31"];
const OS_IN_RAM: &[&str] = &["esp32s2", "esp32s31"];

fn main() {
    let selected = |chips: &[&str]| {
        chips
            .iter()
            .any(|chip| env::var_os(format!("CARGO_FEATURE_{}", chip.to_uppercase())).is_some())
    };

    // `ESP_HAL_CONFIG_USE_RWTEXT_LD_HOOK` makes esp-hal's linker script include `rwtext_hook.x`.
    // On other chips the file is empty.
    let mut hook = String::new();
    if selected(HEAP_IN_RAM) {
        hook += &fs::read_to_string("rwtext_heap.x").unwrap();
    }
    if selected(OS_IN_RAM) {
        hook += &fs::read_to_string("rwtext_os.x").unwrap();
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out_dir.join("rwtext_hook.x"), hook).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=rwtext_heap.x");
    println!("cargo:rerun-if-changed=rwtext_os.x");
}
