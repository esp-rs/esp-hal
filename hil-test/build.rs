use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Define all necessary configuration symbols for the configured device:
    esp_metadata_generated::Chip::from_cargo_feature()?.define_cfgs();
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");

    Ok(())
}
