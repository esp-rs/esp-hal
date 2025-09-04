use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Define all necessary configuration symbols for the configured device:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;
    chip.define_cfgs();

    Ok(())
}
