#[cfg(feature = "rt")]
use std::{
    collections::HashMap,
    env,
    fs::{self, File},
    io::Write,
    path::{Path, PathBuf},
};
use std::{error::Error, fmt::Write as _};

#[cfg(feature = "rt")]
use esp_config::Value;
use esp_config::generate_config_from_yaml_definition;
#[cfg(feature = "rt")]
use esp_metadata_generated::Chip;
use esp_metadata_generated::assert_unique_features;
#[cfg(feature = "rt")]
use somni_template::{BlockStyle, Env, Syntax, Template};

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
        // Place all linker scripts in `OUT_DIR`, and instruct Cargo how to find these
        // files:
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

        println!("cargo:rustc-link-search={}", out.display());

        if chip.is_xtensa() {
            let (irtc, drtc) = if cfg!(feature = "esp32s3") {
                ("rtc_fast_seg", "rtc_fast_seg")
            } else {
                ("rtc_fast_iram_seg", "rtc_fast_dram_seg")
            };

            let alias = format!(
                r#"
            REGION_ALIAS("ROTEXT", irom_seg);
            REGION_ALIAS("RWTEXT", iram_seg);
            REGION_ALIAS("RODATA", drom_seg);
            REGION_ALIAS("RWDATA", dram_seg);
            REGION_ALIAS("RTC_FAST_RWTEXT", {irtc});
            REGION_ALIAS("RTC_FAST_RWDATA", {drtc});
        "#
            );

            fs::write(out.join("alias.x"), alias)?;
            fs::copy("ld/xtensa/hal-defaults.x", out.join("hal-defaults.x"))?;
        } else {
            // RISC-V devices:

            preprocess_file(chip, &cfg, "ld/riscv/asserts.x", out.join("asserts.x"))?;
            preprocess_file(
                chip,
                &cfg,
                "ld/riscv/hal-defaults.x",
                out.join("hal-defaults.x"),
            )?;
        }

        // With the architecture-specific linker scripts taken care of, we can copy all
        // remaining linker scripts which are common to all devices:
        copy_dir_all(chip, &cfg, "ld/sections", &out)?;
        copy_dir_all(chip, &cfg, format!("ld/{}", chip.name()), &out)?;
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

#[cfg(feature = "rt")]
fn copy_dir_all(
    chip: Chip,
    cfg: &HashMap<String, Value>,
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(
                chip,
                cfg,
                entry.path(),
                dst.as_ref().join(entry.file_name()),
            )?;
        } else {
            preprocess_file(
                chip,
                cfg,
                entry.path(),
                dst.as_ref().join(entry.file_name()),
            )?;
        }
    }
    Ok(())
}

/// A naive pre-processor for linker scripts
#[cfg(feature = "rt")]
fn preprocess_file(
    chip: Chip,
    cfg: &HashMap<String, Value>,
    src: impl AsRef<Path>,
    dst: impl AsRef<Path>,
) -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed={}", src.as_ref().display());

    let mut out_file = File::create(dst)?;

    let syntax = Syntax {
        expr: ("${".to_string(), "}".to_string()),
        block: BlockStyle::Line {
            prefix: "#".to_string(),
        },
    };

    let template_src = std::fs::read_to_string(src)?;
    let template = Template::compile(&template_src, &syntax)
        .map_err(|e| e.display_with(&template_src).to_string())?;

    let rendered = template
        .render(somni_env(chip, cfg))
        .map_err(|e| e.display_with(&template_src).to_string())?;
    out_file.write_all(rendered.as_bytes())?;

    Ok(())
}

#[cfg(feature = "rt")]
fn somni_env(chip: Chip, cfg: &HashMap<String, Value>) -> Env {
    let mut env = Env::new();

    // cargo features
    env.function("CARGO_FEATURE", |feature: &str| {
        let vars = std::env::var("CARGO_CFG_FEATURE").unwrap_or_default();
        let enabled_features = vars.split(',').collect::<Vec<_>>();
        enabled_features.contains(&feature)
    });

    // esp-metadata
    for c in Chip::all_possible_symbols() {
        if let Some((name, _)) = c.split_once(',') {
            // empty string is never a valid value, but we need the variables to exist
            env.value(name, "");
        } else {
            env.value(c, false);
        }
    }
    for c in chip.all_symbols() {
        if let Some((name, value)) = c.split_once('=') {
            env.value(name, value.trim().trim_matches('"'));
        } else {
            env.value(c, true);
        }
    }

    // esp-config values
    for (name, value) in cfg.iter() {
        match value {
            // bools are usually used in conditions
            Value::Bool(value) => env.value(name, *value),
            // the rest are usually substituted, so convert them to strings
            Value::Integer(value) => env.value(name, value.to_string()),
            Value::String(value) => env.value(name, value.clone()),
        };
    }
    env
}
