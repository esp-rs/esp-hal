use std::{env, error::Error, fs, path::PathBuf};

use esp_metadata_generated::Chip;

fn main() -> Result<(), Box<dyn Error>> {
    // Determine the name of the configured device:
    let chip = Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    // Copy the required linker script to the `out` directory:
    let source_file = match chip {
        Chip::Esp32c6 => "ld/link-lp.x",
        Chip::Esp32s2 | Chip::Esp32s3 => "ld/link-ulp.x",
        _ => unreachable!(),
    };

    // Put the linker script somewhere the linker can find it:
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());
    fs::copy(source_file, out.join("link.x"))?;
    println!("cargo:rerun-if-changed=ld/link-ulp.x");

    // Done!
    Ok(())
}
