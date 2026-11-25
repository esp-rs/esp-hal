use std::error::Error;

use esp_metadata_generated::assert_unique_features;

fn main() -> Result<(), Box<dyn Error>> {
    // Log and defmt are mutually exclusive.
    assert_unique_features!("log-04", "defmt");

    // Declared here so host builds (no chip feature) still silence unexpected_cfgs.
    // Chip::define_cfgs() also emits this when a chip feature is selected.
    println!("cargo:rustc-check-cfg=cfg(semver_checks)");

    // Chip cfgs when a chip feature is selected (xtask / device builds).
    // Host unit tests may build with no chip feature.
    if let Ok(chip) = esp_metadata_generated::Chip::from_cargo_feature() {
        chip.define_cfgs();
    }

    Ok(())
}
