use esp_metadata_generated::{Chip, emit_check_cfg_directives};

fn main() -> Result<(), String> {
    println!("cargo::rustc-check-cfg=cfg(__test_esp_storage)");

    if !cfg!(feature = "emulation") {
        // Load the configuration file for the configured device:
        let chip = Chip::from_cargo_feature()?;

        // Define all necessary configuration symbols for the configured device:
        chip.define_cfgs();
    } else {
        // Even though we don't have a chip, make sure we're not warned about the config symbols.
        emit_check_cfg_directives();
    }

    Ok(())
}
