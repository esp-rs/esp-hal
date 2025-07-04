//! This is not how this crate is supposed to work. For convenience, we generate files in build
//! time, but this build script should be replaced by an xtask command that re-generates this whole
//! crate.
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    #[cfg(not(feature = "build-script"))]
    {
        // Ensure that exactly one chip has been specified:
        let chip = esp_metadata::Chip::from_cargo_feature()?;

        // Load the configuration file for the configured device:
        let config = esp_metadata::Config::for_chip(&chip);

        config.generate_metadata();
    }

    #[cfg(feature = "build-script")]
    esp_metadata::generate_build_script_utils("_build_script_utils.rs");

    Ok(())
}
