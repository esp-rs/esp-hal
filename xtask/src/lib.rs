use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, anyhow, bail, ensure};
use cargo::CargoAction;
use clap::ValueEnum;
use esp_metadata::{Chip, Config};
use semver::Prerelease;
use strum::{Display, EnumIter, IntoEnumIterator as _};

use crate::{cargo::CargoArgsBuilder, firmware::Metadata};

pub mod cargo;
pub mod commands;
pub mod documentation;
pub mod firmware;

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Display,
    EnumIter,
    ValueEnum,
    serde::Deserialize,
    serde::Serialize,
)]
#[serde(rename_all = "kebab-case")]
#[strum(serialize_all = "kebab-case")]
pub enum Package {
    EspAlloc,
    EspBacktrace,
    EspBootloaderEspIdf,
    EspBuild,
    EspConfig,
    EspHal,
    EspHalEmbassy,
    EspHalProcmacros,
    EspIeee802154,
    EspLpHal,
    EspMetadata,
    EspPrintln,
    EspRiscvRt,
    EspStorage,
    EspWifi,
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
                | EspHal
                | EspHalEmbassy
                | EspIeee802154
                | EspLpHal
                | EspPrintln
                | EspStorage
                | EspWifi
                | XtensaLxRt
        )
    }

    /// Do the package's chip-specific cargo features affect the public API?
    pub fn chip_features_matter(&self) -> bool {
        use Package::*;

        matches!(self, EspHal | EspLpHal | EspWifi | EspHalEmbassy)
    }

    /// Should documentation be built for the package?
    pub fn is_published(&self) -> bool {
        !matches!(self, Package::Examples | Package::HilTest | Package::QaTest)
    }

    /// Build on host
    pub fn build_on_host(&self) -> bool {
        use Package::*;

        matches!(self, EspBuild | EspConfig | EspMetadata)
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
            Package::EspWifi => {
                features.push("esp-hal/unstable".to_owned());
                features.push("defmt".to_owned());
                if config.contains("wifi") {
                    features.push("wifi".to_owned());
                    features.push("esp-now".to_owned());
                    features.push("sniffer".to_owned());
                    features.push("smoltcp/proto-ipv4".to_owned());
                    features.push("smoltcp/proto-ipv6".to_owned());
                }
                if config.contains("ble") {
                    features.push("ble".to_owned());
                }
                if config.contains("coex") {
                    features.push("coex".to_owned());
                }
            }
            Package::EspHalProcmacros => {
                features.push("embassy".to_owned());
            }
            Package::EspHalEmbassy => {
                features.push("esp-hal/unstable".to_owned());
                features.push("defmt".to_owned());
                features.push("executors".to_owned());
            }
            Package::EspIeee802154 => {
                features.push("defmt".to_owned());
                features.push("esp-hal/unstable".to_owned());
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
            _ => {}
        }

        features
    }

    /// Additional feature rules to test subsets of features for a package.
    pub fn lint_feature_rules(&self, _config: &Config) -> Vec<Vec<String>> {
        let mut cases = Vec::new();

        if self == &Package::EspWifi {
            // Minimal set of features that when enabled _should_ still compile:
            cases.push(vec![
                "esp-hal/unstable".to_owned(),
                "builtin-scheduler".to_owned(),
            ]);
        }

        cases
    }

    /// Return the target triple for a given package/chip pair.
    pub fn target_triple(&self, chip: &Chip) -> Result<&'static str> {
        if *self == Package::EspLpHal {
            chip.lp_target()
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
}

#[derive(Debug, Clone, Copy, Display, ValueEnum)]
#[strum(serialize_all = "lowercase")]
pub enum Version {
    Major,
    Minor,
    Patch,
    Beta,
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

    if !debug {
        builder.add_arg("--release");
    }

    // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
    if target.starts_with("xtensa") {
        builder = builder.toolchain("esp");
        builder.add_arg("-Zbuild-std=core,alloc");
    }

    if subcommand == "test" && chip == Chip::Esp32c2 {
        builder.add_arg("--").add_arg("--speed").add_arg("15000");
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

/// Bump the version of the specified package by the specified amount.
pub fn bump_version(workspace: &Path, package: Package, amount: Version) -> Result<()> {
    let manifest_path = workspace.join(package.to_string()).join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .with_context(|| format!("Could not read {}", manifest_path.display()))?;

    let mut manifest = manifest.parse::<toml_edit::DocumentMut>()?;

    let version = manifest["package"]["version"]
        .to_string()
        .trim()
        .trim_matches('"')
        .to_string();
    let prev_version = &version;

    let mut version = semver::Version::parse(&version)?;
    match amount {
        Version::Major => {
            version.major += 1;
            version.minor = 0;
            version.patch = 0;
            version.pre = Prerelease::EMPTY;
        }
        Version::Minor => {
            version.minor += 1;
            version.patch = 0;
            version.pre = Prerelease::EMPTY;
        }
        Version::Patch => {
            version.patch += 1;
            version.pre = Prerelease::EMPTY;
        }
        Version::Beta => {
            ensure!(
                !version.pre.is_empty(),
                "No pre-release version found for {package}"
            );
            let pre_str = version.pre.as_str();
            // Increment the number after the last dot:
            if let Some(separator) = pre_str.rfind('.') {
                let (pre, num) = pre_str.split_at(separator);
                let num = num.trim_start_matches('.');
                let num = num.parse::<u32>()?;
                version.pre = Prerelease::new(&format!("{pre}.{}", num + 1)).unwrap();
            } else if pre_str.is_empty() {
                version.pre = Prerelease::new("beta.0").unwrap();
            } else {
                bail!("Unexpected pre-release version format found for {package}: {pre_str}");
            };
        }
    }

    log::info!("Bumping version for package: {package} ({prev_version} -> {version})");

    manifest["package"]["version"] = toml_edit::value(version.to_string());
    fs::write(manifest_path, manifest.to_string())?;

    for pkg in
        Package::iter().filter(|p| ![package, Package::Examples, Package::HilTest].contains(p))
    {
        let manifest_path = workspace.join(pkg.to_string()).join("Cargo.toml");
        let manifest = fs::read_to_string(&manifest_path)
            .with_context(|| format!("Could not read {}", manifest_path.display()))?;

        let mut manifest = manifest.parse::<toml_edit::DocumentMut>()?;

        if manifest["dependencies"]
            .as_table()
            .unwrap()
            .contains_key(&package.to_string())
        {
            log::info!(
                "  Bumping {package} version for package {pkg}: ({prev_version} -> {version})"
            );

            if let Some(table) = manifest["dependencies"].as_table_mut() {
                table[&package.to_string()]["version"] = toml_edit::value(version.to_string());
            }

            fs::write(&manifest_path, manifest.to_string())
                .with_context(|| format!("Could not write {}", manifest_path.display()))?;
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
    #[derive(Debug, serde::Deserialize)]
    pub struct Manifest {
        package: Package,
    }

    #[derive(Debug, serde::Deserialize)]
    pub struct Package {
        version: semver::Version,
    }

    let path = workspace.join(package.to_string()).join("Cargo.toml");
    let path = windows_safe_path(&path);
    let manifest =
        fs::read_to_string(&path).with_context(|| format!("Could not read {}", path.display()))?;
    let manifest: Manifest = basic_toml::from_str(&manifest)?;

    Ok(manifest.package.version)
}

/// Make the path "Windows"-safe
pub fn windows_safe_path(path: &Path) -> PathBuf {
    PathBuf::from(path.to_str().unwrap().to_string().replace("\\\\?\\", ""))
}
