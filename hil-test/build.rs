use std::error::Error;

use esp_metadata::{Chip, Config};

fn main() -> Result<(), Box<dyn Error>> {
    // Load the configuration file for the configured device:
    let chip = Chip::from_cargo_feature()?;
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();

    Ok(())
}
