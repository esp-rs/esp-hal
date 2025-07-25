use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Result, anyhow};
use cargo::CargoAction;
use esp_metadata::{Chip, Config, TokenStream};
use serde::{Deserialize, Serialize};

use crate::{
    cargo::{CargoArgsBuilder, CargoToml},
    firmware::Metadata,
};

pub mod cargo;
pub mod changelog;
pub mod commands;
pub mod documentation;
pub mod firmware;
pub mod git;

#[cfg(feature = "semver-checks")]
pub mod semver_check;

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    clap::ValueEnum,
    strum::Display,
    strum::EnumIter,
    strum::AsRefStr,
    serde::Deserialize,
    serde::Serialize,
)]
#[serde(rename_all = "kebab-case")]
#[strum(serialize_all = "kebab-case")]
pub enum Package {
    EspAlloc,
    EspBacktrace,
    EspBootloaderEspIdf,
    EspConfig,
    EspHal,
    EspHalEmbassy,
    EspHalProcmacros,
    EspRomSys,
    EspIeee802154,
    EspLpHal,
    EspMetadata,
    EspMetadataGenerated,
    EspPrintln,
    EspRiscvRt,
    EspStorage,
    EspRadio,
    EspRadioPreemptDriver,
    EspRadioPreemptBaremetal,
    Examples,
    HilTest,
    QaTest,
    XtensaLx,
    XtensaLxRt,
    XtensaLxRtProcMacros,
}

impl Package {
    /// Does the package have chip-specific cargo features?
    pub fn has_chip_features(&self) -> bool {
        use Package::*;

        matches!(
            self,
            EspBacktrace
                | EspBootloaderEspIdf
                | EspHal
                | EspHalEmbassy
                | EspMetadataGenerated
                | EspRomSys
                | EspIeee802154
                | EspLpHal
                | EspPrintln
                | EspRadioPreemptBaremetal
                | EspStorage
                | EspRadio
        )
    }

    /// Does the package have inline assembly?
    pub fn has_inline_assembly(&self, workspace: &Path) -> bool {
        // feature(asm_experimental_arch) is enabled in all crates that use Xtensa
        // assembly, which covers crates that use assembly AND are used for both
        // architectures (e.g. esp-backtrace).
        // But RISC-V doesn't need this feature, so we can either scrape the crate
        // source, or check in a list of packages.
        if matches!(self, Package::EspRiscvRt | Package::EspLpHal) {
            return true;
        }

        let lib_rs_path = workspace.join(self.to_string()).join("src").join("lib.rs");
        let Ok(source) = std::fs::read_to_string(&lib_rs_path) else {
            return false;
        };

        source
            .lines()
            .filter(|line| line.starts_with("#!["))
            .any(|line| line.contains("asm_experimental_arch"))
    }

    pub fn has_migration_guide(&self, workspace: &Path) -> bool {
        let package_path = workspace.join(self.to_string());

        // Check if the package directory exists
        let Ok(entries) = std::fs::read_dir(&package_path) else {
            return false;
        };

        // Look for files matching the pattern "MIGRATING-*.md"
        for entry in entries.flatten() {
            if let Some(file_name) = entry.file_name().to_str() {
                if file_name.starts_with("MIGRATING-") && file_name.ends_with(".md") {
                    return true;
                }
            }
        }

        false
    }

    pub fn needs_build_std(&self) -> bool {
        use Package::*;

        !matches!(self, EspConfig | EspMetadata)
    }

    /// Do the package's chip-specific cargo features affect the public API?
    pub fn chip_features_matter(&self) -> bool {
        use Package::*;

        matches!(
            self,
            EspHal
                | EspLpHal
                | EspRadio
                | EspHalEmbassy
                | EspRomSys
                | EspBootloaderEspIdf
                | EspMetadataGenerated
                | EspRadioPreemptBaremetal
        )
    }

    /// Should documentation be built for the package, and should the package be
    /// published?
    pub fn is_published(&self, workspace: &Path) -> bool {
        // TODO: we should use some sort of cache instead of parsing the TOML every
        // time, but for now this should be good enough.
        let toml =
            crate::cargo::CargoToml::new(workspace, *self).expect("Failed to parse Cargo.toml");
        toml.is_published()
    }

    /// Build on host
    pub fn build_on_host(&self, features: &[String]) -> bool {
        match self {
            Self::EspConfig | Self::EspMetadata => true,
            Self::EspMetadataGenerated if features.iter().any(|f| f == "build-script") => true,
            _ => false,
        }
    }

    /// Given a device config, return the features which should be enabled for
    /// this package.
    pub fn feature_rules(&self, config: &Config) -> Vec<String> {
        let mut features = vec![];
        match self {
            Package::EspBacktrace => features.push("defmt".to_owned()),
            Package::EspConfig => features.push("build".to_owned()),
            Package::EspHal => {
                features.push("unstable".to_owned());
                features.push("rt".to_owned());
                if config.contains("psram") {
                    // TODO this doesn't test octal psram (since `ESP_HAL_CONFIG_PSRAM_MODE`
                    // defaults to `quad`) as it would require a separate build
                    features.push("psram".to_owned())
                }
                if config.contains("usb0") {
                    features.push("__usb_otg".to_owned());
                }
                if config.contains("bt") {
                    features.push("__bluetooth".to_owned());
                }
            }
            Package::EspRadio => {
                features.push("esp-hal/unstable".to_owned());
                features.push("esp-hal/rt".to_owned());
                features.push("defmt".to_owned());
                if config.contains("wifi") {
                    features.push("wifi".to_owned());
                    features.push("esp-now".to_owned());
                    features.push("sniffer".to_owned());
                    features.push("smoltcp/proto-ipv4".to_owned());
                    features.push("smoltcp/proto-ipv6".to_owned());
                }
                if config.contains("bt") {
                    features.push("ble".to_owned());
                }
                if config.contains("wifi") && config.contains("bt") {
                    features.push("coex".to_owned());
                }
            }
            Package::EspHalProcmacros => {
                features.push("embassy".to_owned());
            }
            Package::EspHalEmbassy => {
                features.push("esp-hal/unstable".to_owned());
                features.push("esp-hal/rt".to_owned());
                features.push("defmt".to_owned());
                features.push("executors".to_owned());
            }
            Package::EspIeee802154 => {
                features.push("defmt".to_owned());
                features.push("esp-hal/unstable".to_owned());
                features.push("esp-hal/rt".to_owned());
            }
            Package::EspLpHal => {
                if config.contains("lp_core") {
                    features.push("embedded-io".to_owned());
                }
                features.push("embedded-hal".to_owned());
            }
            Package::EspPrintln => {
                features.push("auto".to_owned());
                features.push("defmt-espflash".to_owned());
            }
            Package::EspStorage => {}
            Package::EspBootloaderEspIdf => {
                features.push("defmt".to_owned());
                features.push("validation".to_owned());
            }
            Package::EspAlloc => {
                features.push("defmt".to_owned());
            }
            Package::EspMetadataGenerated => {}
            Package::EspRadioPreemptBaremetal => features.push("esp-hal/unstable".to_owned()),
            _ => {}
        }

        features
    }

    /// Additional feature rules to test subsets of features for a package.
    pub fn lint_feature_rules(&self, _config: &Config) -> Vec<Vec<String>> {
        let mut cases = Vec::new();

        match self {
            Package::EspHal => {
                // This checks if the `esp-hal` crate compiles with the no features (other than the
                // chip selection)

                // This tests that disabling the `rt` feature works
                cases.push(vec![]);
                // This checks if the `esp-hal` crate compiles _without_ the `unstable` feature
                // enabled
                cases.push(vec!["rt".to_owned()]);
            }
            Package::EspRadio => {
                // Minimal set of features that when enabled _should_ still compile:
                cases.push(vec!["esp-hal/rt".to_owned(), "esp-hal/unstable".to_owned()]);
            }
            Package::EspMetadataGenerated => {
                cases.push(vec!["build-script".to_owned()]);
            }
            Package::EspRadioPreemptBaremetal => {
                cases.push(vec!["esp-alloc".to_owned(), "esp-hal/unstable".to_owned()])
            }
            _ => {}
        }

        cases
    }

    /// Return the target triple for a given package/chip pair.
    pub fn target_triple(&self, chip: &Chip) -> Result<String> {
        if *self == Package::EspLpHal {
            chip.lp_target().map(ToString::to_string)
        } else {
            Ok(chip.target())
        }
    }

    /// Validate that the specified chip is valid for the specified package.
    pub fn validate_package_chip(&self, chip: &Chip) -> Result<()> {
        let device = Config::for_chip(chip);

        let check = match self {
            Package::EspIeee802154 => device.contains("ieee802154"),
            Package::EspLpHal => chip.has_lp_core(),
            Package::XtensaLx | Package::XtensaLxRt | Package::XtensaLxRtProcMacros => {
                chip.is_xtensa()
            }
            Package::EspRiscvRt => chip.is_riscv(),
            _ => true,
        };

        if check {
            Ok(())
        } else {
            Err(anyhow!(
                "Invalid chip provided for package '{}': '{}'",
                self,
                chip
            ))
        }
    }

    pub fn tag(&self, version: &semver::Version) -> String {
        format!("{self}-v{version}")
    }

    #[cfg(feature = "release")]
    fn is_semver_checked(&self) -> bool {
        [Self::EspHal].contains(self)
    }
}

#[derive(Debug, Clone, Copy, strum::Display, clap::ValueEnum, Serialize, Deserialize)]
#[strum(serialize_all = "lowercase")]
pub enum Version {
    Major,
    Minor,
    Patch,
}

/// Run or build the specified test or example for the specified chip.
pub fn execute_app(
    package_path: &Path,
    chip: Chip,
    target: &str,
    app: &Metadata,
    action: CargoAction,
    repeat: usize,
    debug: bool,
    toolchain: Option<&str>,
    timings: bool,
) -> Result<()> {
    let package = app.example_path().strip_prefix(package_path)?;
    log::info!("Building example '{}' for '{}'", package.display(), chip);

    if !app.configuration().is_empty() {
        log::info!("  Configuration: {}", app.configuration());
    }

    let mut features = app.feature_set().to_vec();
    if !features.is_empty() {
        log::info!("  Features:      {}", features.join(", "));
    }
    features.push(chip.to_string());

    let env_vars = app.env_vars();
    for (key, value) in env_vars {
        log::info!("  esp-config:    {} = {}", key, value);
    }

    let mut builder = CargoArgsBuilder::default()
        .target(target)
        .features(&features);

    let bin_arg = if package.starts_with("src/bin") {
        format!("--bin={}", app.binary_name())
    } else if package.starts_with("tests") {
        format!("--test={}", app.binary_name())
    } else {
        format!("--example={}", app.binary_name())
    };
    builder.add_arg(bin_arg);

    let subcommand = if matches!(action, CargoAction::Build(_)) {
        "build"
    } else if package.starts_with("tests") {
        "test"
    } else {
        "run"
    };
    builder = builder.subcommand(subcommand);

    for config in app.cargo_config() {
        log::info!(" Cargo --config: {config}");
        builder.add_arg("--config").add_arg(config);
        // Some configuration requires nightly rust, so let's just assume it. May be
        // overwritten by the esp toolchain on xtensa.
        builder = builder.toolchain("nightly");
    }

    if !debug {
        builder.add_arg("--release");
    }
    if timings {
        builder.add_arg("--timings");
    }

    let toolchain = match toolchain {
        // Preserve user choice
        Some(tc) => Some(tc),
        // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
        _ if target.starts_with("xtensa") => Some("esp"),
        _ => None,
    };

    if let Some(toolchain) = toolchain {
        if toolchain.starts_with("esp") {
            builder = builder.arg("-Zbuild-std=core,alloc");
        }
        builder = builder.toolchain(toolchain);
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    if let CargoAction::Build(out_dir) = action {
        cargo::run_with_env(&args, package_path, env_vars, false)?;

        // Now that the build has succeeded and we printed the output, we can
        // rerun the build again quickly enough to capture JSON. We'll use this to
        // copy the binary to the output directory.
        builder.add_arg("--message-format=json");
        let args = builder.build();
        let output = cargo::run_with_env(&args, package_path, env_vars, true)?;
        for line in output.lines() {
            if let Ok(artifact) = serde_json::from_str::<cargo::Artifact>(line) {
                let out_dir = out_dir.join(chip.to_string());
                std::fs::create_dir_all(&out_dir)?;

                let output_file = out_dir.join(app.output_file_name());
                std::fs::copy(artifact.executable, &output_file)?;
                log::info!("Output ready: {}", output_file.display());
            }
        }
    } else {
        for i in 0..repeat {
            if repeat != 1 {
                log::info!("Run {}/{}", i + 1, repeat);
            }
            cargo::run_with_env(&args, package_path, env_vars.clone(), false)?;
        }
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

// Copy an entire directory recursively.
// https://stackoverflow.com/a/65192210
pub fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> Result<()> {
    fs::create_dir_all(&dst)?;

    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;

        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
        }
    }

    Ok(())
}

/// Return a (sorted) list of paths to each valid Cargo package in the
/// workspace.
pub fn package_paths(workspace: &Path) -> Result<Vec<PathBuf>> {
    let mut paths = Vec::new();
    for entry in fs::read_dir(workspace)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() && entry.path().join("Cargo.toml").exists() {
            paths.push(entry.path());
        }
    }

    paths.sort();

    Ok(paths)
}

/// Parse the version from the specified package's Cargo manifest.
pub fn package_version(workspace: &Path, package: Package) -> Result<semver::Version> {
    CargoToml::new(workspace, package).map(|toml| toml.package_version())
}

/// Make the path "Windows"-safe
pub fn windows_safe_path(path: &Path) -> PathBuf {
    PathBuf::from(path.to_str().unwrap().to_string().replace("\\\\?\\", ""))
}

pub fn format_package(workspace: &Path, package: Package, check: bool) -> Result<()> {
    log::info!("Formatting package: {}", package);
    let path = workspace.join(package.as_ref());

    // we need to list all source files since modules in `unstable_module!` macros
    // won't get picked up otherwise
    let source_files = walkdir::WalkDir::new(path.join("src"))
        .into_iter()
        .filter_map(|entry| {
            let path = entry.unwrap().into_path();
            if let Some("rs") = path.extension().unwrap_or_default().to_str() {
                Some(String::from(path.to_str().unwrap()))
            } else {
                None
            }
        });

    let mut cargo_args = CargoArgsBuilder::default()
        .toolchain("nightly")
        .subcommand("fmt")
        .arg("--all")
        .build();

    if check {
        cargo_args.push("--check".into());
    }

    cargo_args.push("--".into());
    cargo_args.push(format!(
        "--config-path={}/rustfmt.toml",
        workspace.display()
    ));
    cargo_args.extend(source_files);

    cargo::run(&cargo_args, &path)?;

    Ok(())
}

pub fn update_metadata(workspace: &Path, check: bool) -> Result<()> {
    update_chip_support_table(workspace)?;
    generate_metadata(workspace, save)?;

    format_package(workspace, Package::EspMetadataGenerated, false)?;

    if check {
        let res = std::process::Command::new("git")
            .args(["diff", "HEAD", "esp-metadata-generated"])
            .output()?;
        if !res.stdout.is_empty() {
            return Err(anyhow::Error::msg(
                "detected `esp-metadata-generated` changes. Run `cargo xtask update-metadata`, and commit the changes.",
            ));
        }
    }

    Ok(())
}

fn generate_metadata(
    workspace: &Path,
    call_for_file: fn(&Path, TokenStream) -> Result<()>,
) -> Result<()> {
    use strum::IntoEnumIterator;

    let out_path = workspace.join("esp-metadata-generated").join("src");

    for chip in Chip::iter() {
        let config = esp_metadata::Config::for_chip(&chip);
        call_for_file(
            &out_path.join(format!("_generated_{chip}.rs")),
            config.generate_metadata(),
        )?;
    }

    call_for_file(
        &out_path.join("_build_script_utils.rs"),
        esp_metadata::generate_build_script_utils(),
    )?;

    call_for_file(&out_path.join("lib.rs"), esp_metadata::generate_lib_rs())?;

    Ok(())
}

fn save(out_path: &Path, tokens: TokenStream) -> Result<()> {
    let source = tokens.to_string();

    let syntax_tree = syn::parse_file(&source)?;
    let mut source = String::from(
        "// Do NOT edit this file directly. Make your changes to esp-metadata,\n// then run `cargo xtask update-metadata`.\n\n",
    );
    source.push_str(&prettyplease::unparse(&syntax_tree));

    std::fs::write(out_path, source)?;

    Ok(())
}

fn update_chip_support_table(workspace: &Path) -> Result<()> {
    let mut output = String::new();
    let readme = std::fs::read_to_string(workspace.join("esp-hal").join("README.md"))?;

    let mut in_support_table = false;
    let mut generate_support_table = true;
    for line in readme.lines() {
        let mut copy_line = true;
        if line.trim() == "<!-- start chip support table -->" {
            in_support_table = true;
        } else if line.trim() == "<!-- end chip support table -->" {
            in_support_table = false;
        } else {
            copy_line = !in_support_table;
        }
        if !copy_line {
            continue;
        }
        output.push_str(line);
        output.push('\n');

        if in_support_table && generate_support_table {
            esp_metadata::generate_chip_support_status(&mut output)?;

            generate_support_table = false;
        }
    }

    std::fs::write(workspace.join("esp-hal").join("README.md"), output)?;

    Ok(())
}
