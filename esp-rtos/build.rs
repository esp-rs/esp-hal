use std::error::Error;

use esp_config::generate_config_from_yaml_definition;

fn main() -> Result<(), Box<dyn Error>> {
    // Ensure that exactly one chip has been specified:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    // Emit `riscv_has_f` when the target has a hardware FPU.
    //
    // `target_feature = "f"` is not stable, so detect the F extension from the target name
    // instead (e.g. "riscv32imafc-unknown-none-elf").
    println!("cargo:rustc-check-cfg=cfg(riscv_has_f)");
    let target = std::env::var("TARGET").unwrap_or_default();
    let arch = target.split('-').next().unwrap_or("");
    if let Some(exts) = arch.strip_prefix("riscv32") {
        // Only look at the base ISA letters, ignoring any `_z...` extensions.
        let exts = exts.split('_').next().unwrap_or("");
        // 'g' = imafd; 'f' or 'd' implies FP registers.
        if exts.contains('f') || exts.contains('d') || exts.contains('g') {
            println!("cargo:rustc-cfg=riscv_has_f");
        }
    }

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-rtos");
    generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    // Emit the default stack guard offset if not set by the user.
    if std::env::var("ESP_HAL_CONFIG_STACK_GUARD_OFFSET").is_err() {
        println!("cargo:rustc-env=ESP_HAL_CONFIG_STACK_GUARD_OFFSET=60");
    }

    let debug_build = ["0", "1"];
    println!("cargo:rustc-check-cfg=cfg(debug_build)");
    if let Ok(level) = std::env::var("OPT_LEVEL")
        && debug_build.iter().any(|&x| x == level)
    {
        println!("cargo:rustc-cfg=debug_build")
    }

    Ok(())
}