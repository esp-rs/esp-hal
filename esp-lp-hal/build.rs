use std::{env, error::Error, fs, path::PathBuf};

use esp_metadata_generated::Chip;

fn main() -> Result<(), Box<dyn Error>> {
    // Determine the name of the configured device:
    let chip = Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    // Copy the required linker scripts to the `out` directory:
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out_dir.display());

    match chip {
        Chip::Esp32c6 => {
            fs::write(out_dir.join("link.x"), include_bytes!("ld/link-lp.x"))?;
        },
        Chip::Esp32s2 | Chip::Esp32s3 => {
            fs::write(out_dir.join("link.x"), include_bytes!("ld/link-ulp.x"))?;
            fs::write(out_dir.join("memory.x"), include_bytes!("ld/memory-ulp.x"))?;
            fs::write(out_dir.join("interrupts.x"), include_bytes!("ld/interrupts-ulp.x"))?;
            fs::write(out_dir.join("exceptions.x"), include_bytes!("ld/exceptions-ulp.x"))?;
        },
        _ => unreachable!(),
    };

    println!("cargo:rerun-if-changed=ld/link-lp.x");
    println!("cargo:rerun-if-changed=ld/link-ulp.x");
    println!("cargo:rerun-if-changed=ld/memory-ulp.x");
    println!("cargo:rerun-if-changed=ld/interrupts-ulp.x");
    println!("cargo:rerun-if-changed=ld/exceptions-ulp.x");

    // Done!
    Ok(())
}
