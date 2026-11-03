use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Define all necessary configuration symbols for the configured device:
    esp_metadata_generated::Chip::from_cargo_feature()?.define_cfgs();
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");
    // Set by the `self_cal` configuration of the `adc` test.
    println!("cargo::rustc-check-cfg=cfg(__test_adc_self_cal)");

    println!("cargo::rustc-check-cfg=cfg(float_save_restore)");
    if std::env::var_os("DEP_ESP_RISCV_RT_FLOAT_SAVE_RESTORE").is_some() {
        println!("cargo::rustc-cfg=float_save_restore");
    }

    Ok(())
}
