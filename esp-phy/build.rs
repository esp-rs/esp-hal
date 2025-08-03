use std::error::Error as StdError;


fn main() -> Result<(), Box<dyn StdError>> {
    // Load the configuration for the configured device:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();
    Ok(())
}
