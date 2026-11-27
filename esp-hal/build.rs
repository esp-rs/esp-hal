#[cfg(feature = "rt")]
use std::{env, path::PathBuf};
use std::{error::Error, fmt::Write as _};

#[cfg(feature = "rt")]
use esp_config::Value;
use esp_config::generate_config_from_yaml_definition;
use esp_metadata_generated::assert_unique_features;

macro_rules! bail {
    ($($arg:tt)*) => {
        return Err(format!($($arg)*).into());
    };
}

fn main() {
    if let Err(e) = try_main() {
        eprintln!("{}", e);
        std::process::exit(1);
    }
}

fn try_main() -> Result<(), Box<dyn Error>> {
    generate_version_macro()?;

    // if using '"rust-analyzer.cargo.buildScripts.useRustcWrapper": true' we can detect this
    let suppress_panics = std::env::var("RUSTC_WRAPPER")
        .unwrap_or_default()
        .contains("rust-analyzer");

    println!("cargo:rustc-check-cfg=cfg(is_debug_build)");
    // FIXME: i2c_slave_i2c1 is used conditionally in src/i2c/slave/mod.rs but may not be generated
    // by metadata for single-I2C chips (e.g. ESP32-C3). Register it here to avoid unexpected_cfgs
    // warnings on those targets.
    println!("cargo:rustc-check-cfg=cfg(i2c_slave_i2c1)");
    // HIL rustflags for exercising the ADC self-calibration fallback on boards with eFuse data.
    println!("cargo:rustc-check-cfg=cfg(__test_adc_self_cal)");
    // HIL rustflags for encrypted-write tests that run without burned eFuses.
    println!("cargo:rustc-check-cfg=cfg(__test_flash)");
    if let Ok(level) = std::env::var("OPT_LEVEL")
        && (level == "0" || level == "1")
    {
        println!("cargo:rustc-cfg=is_debug_build");
    }

    // If some library required unstable make sure unstable is actually enabled.
    if !suppress_panics && cfg!(feature = "requires-unstable") && !cfg!(feature = "unstable") {
        bail!(
            "\n\nThe `unstable` feature is required by a dependent crate but is not enabled.\n\n"
        );
    }

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    // Ensure that exactly one chip has been specified:
    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    if !suppress_panics && chip.target() != std::env::var("TARGET").unwrap_or_default().as_str() {
        bail!("
        Seems you are building for an unsupported or wrong target (e.g. the host environment).
        Maybe you are missing the `target` in `.cargo/config.toml` or you have configs overriding it?

        See https://doc.rust-lang.org/cargo/reference/config.html#hierarchical-structure
        ");
    }

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    println!("cargo:rustc-check-cfg=cfg(fpu_disabled_in_interrupts)");
    if std::env::var_os("DEP_ESP_RISCV_RT_FPU_DISABLED_IN_INTERRUPTS").is_some() {
        println!("cargo:rustc-cfg=fpu_disabled_in_interrupts");
    }

    // emit config
    println!("cargo:rerun-if-changed=./esp_config.yml");
    let cfg_yaml = std::fs::read_to_string("./esp_config.yml")
        .expect("Failed to read esp_config.yml for esp-hal");
    let cfg = generate_config_from_yaml_definition(&cfg_yaml, true, true, Some(chip)).unwrap();

    #[cfg(not(feature = "rt"))]
    let _ = cfg;

    // Only emit linker directives if the `rt` feature is enabled
    #[cfg(feature = "rt")]
    {
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

        println!("cargo:rustc-link-search={}", out.display());

        let mut renderer = chip.linker_script_renderer();
        for (name, value) in cfg.iter() {
            match value {
                Value::Bool(value) => renderer.bool(name, *value),
                Value::Integer(value) => renderer.string(name, value.to_string()),
                Value::String(value) => renderer.string(name, value.clone()),
            };
        }

        let hal_defaults = if chip.is_xtensa() {
            "ld/xtensa/hal-defaults.x"
        } else {
            "ld/riscv/hal-defaults.x"
        };
        renderer.render_file(hal_defaults, out.join("hal-defaults.x"))?;

        // The parts of the memory layout that depend on esp-hal's configuration. The linker
        // scripts of the bootloader support crate include them.
        renderer.render_dir("ld/fragments", &out)?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions
fn generate_version_macro() -> Result<(), Box<dyn Error>> {
    let major = std::env::var("CARGO_PKG_VERSION_MAJOR")?.parse::<u64>()?;
    let minor = std::env::var("CARGO_PKG_VERSION_MINOR")?.parse::<u64>()?;
    let patch = std::env::var("CARGO_PKG_VERSION_PATCH")?.parse::<u64>()?;
    let mut source = String::from(
        r#"
#[doc(hidden)]
#[macro_export]
macro_rules! __esp_hal_at_least_version {
"#,
    );

    // Any minor version of an earlier major version satisfies the requirement.
    for accepted_major in 0..major {
        writeln!(
            source,
            r#"    (({accepted_major}, $_minor:literal, $_patch:literal) => {{ $($selected:tt)* }} $($rest:tt)*) => {{
        $($selected)*
    }};"#
        )?;
    }

    // Any patch version of an earlier minor version of the current major satisfies it.
    for accepted_minor in 0..minor {
        writeln!(
            source,
            r#"    (({major}, {accepted_minor}, $_patch:literal) => {{ $($selected:tt)* }} $($rest:tt)*) => {{
        $($selected)*
    }};"#
        )?;
    }

    // For the current major and minor version, only this and earlier patches satisfy it.
    for accepted_patch in 0..=patch {
        writeln!(
            source,
            r#"    (({major}, {minor}, {accepted_patch}) => {{ $($selected:tt)* }} $($rest:tt)*) => {{
        $($selected)*
    }};"#
        )?;
    }

    source.push_str(
        r#"    (($major:literal, $minor:literal, $patch:literal) => { $($skipped:tt)* } $($rest:tt)*) => {
        $crate::__esp_hal_at_least_version! { $($rest)* }
    };
    (, $($rest:tt)*) => {
        $crate::__esp_hal_at_least_version! { $($rest)* }
    };
    (_ => { $($fallback:tt)* } $(,)?) => {
        $($fallback)*
    };
}
"#,
    );

    let out_dir = std::env::var_os("OUT_DIR").ok_or("OUT_DIR is not set")?;
    std::fs::write(
        std::path::PathBuf::from(out_dir).join("version_macro.rs"),
        source,
    )?;

    Ok(())
}
