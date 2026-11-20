fn main() -> Result<(), Box<dyn std::error::Error>> {
    esp_metadata_generated::Chip::from_cargo_feature()?.define_cfgs();

    // Allow building QA tests in CI in debug mode
    println!("cargo:rustc-check-cfg=cfg(is_not_release)");
    println!("cargo:rerun-if-env-changed=CI");
    if std::env::var("CI").is_err() {
        if let Ok(level) = std::env::var("OPT_LEVEL") {
            if level == "0" || level == "1" {
                println!("cargo::rustc-cfg=is_not_release");
            }
        }
    }

    // `#[psram(zeroed)]` fails to compile without esp-hal's `code-in-psram`, so
    // a PSRAM static needs a cfg. The build script reads the variable that
    // `qa_test::code_in_psram` reads.
    println!("cargo:rustc-check-cfg=cfg(code_in_psram)");
    println!("cargo:rerun-if-env-changed=ESP_HAL_CONFIG_CODE_IN_PSRAM");
    if std::env::var("ESP_HAL_CONFIG_CODE_IN_PSRAM").as_deref() == Ok("true") {
        println!("cargo::rustc-cfg=code_in_psram");
    }

    Ok(())
}
