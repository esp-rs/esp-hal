use std::{env, error::Error, path::PathBuf};

use esp_metadata_generated::Chip;

fn main() -> Result<(), Box<dyn Error>> {
    let chip = Chip::from_cargo_feature()?;
    chip.define_cfgs();

    // The ROM of these chips leaves PMA entries enabled that the 2nd-stage bootloader replaces.
    println!("cargo::rustc-check-cfg=cfg(rom_pma)");
    if matches!(
        chip,
        Chip::Esp32c5 | Chip::Esp32c61 | Chip::Esp32h4 | Chip::Esp32p4 | Chip::Esp32s31
    ) {
        println!("cargo::rustc-cfg=rom_pma");
    }

    emit_linker_scripts(chip)
}

/// Renders the linker scripts into `OUT_DIR`. They include `hal-defaults.x` and the `esp_hal_*.x`
/// fragments from esp-hal's `OUT_DIR`, so no file here may share a name with one there.
fn emit_linker_scripts(chip: Chip) -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    let renderer = chip.linker_script_renderer();
    if !chip.is_xtensa() {
        renderer.render_dir("ld/riscv", &out)?;
    }
    renderer.render_dir(format!("ld/{}", chip.name()), &out)?;

    Ok(())
}
