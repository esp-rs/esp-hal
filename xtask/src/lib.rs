use std::{
    collections::VecDeque,
    fs::{self, File},
    io::Write as _,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result};
use cargo::CargoAction;
use clap::ValueEnum;
use esp_metadata::{Chip, Config};
use strum::{Display, EnumIter, IntoEnumIterator as _};

use crate::{cargo::CargoArgsBuilder, firmware::Metadata};

pub mod cargo;
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
    serde::Serialize,
)]
#[serde(rename_all = "kebab-case")]
#[strum(serialize_all = "kebab-case")]
pub enum Package {
    EspAlloc,
    EspBacktrace,
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
}

#[derive(Debug, Clone, Copy, Display, ValueEnum)]
#[strum(serialize_all = "lowercase")]
pub enum Version {
    Major,
    Minor,
    Patch,
}

/// Build the documentation for the specified package and device.
pub fn build_documentation(workspace: &Path, package: Package, chip: Chip) -> Result<PathBuf> {
    let package_name = package.to_string();
    let package_path = windows_safe_path(&workspace.join(&package_name));

    log::info!("Building '{package_name}' documentation targeting '{chip}'");

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(package, &chip)?;

    // We need `nightly` for building the docs, unfortunately:
    let toolchain = if chip.is_xtensa() { "esp" } else { "nightly" };

    let mut features = vec![chip.to_string()];

    let chip = Config::for_chip(&chip);

    features.extend(apply_feature_rules(&package, chip));

    // Build up an array of command-line arguments to pass to `cargo`:
    let builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("doc")
        .target(target)
        .features(&features)
        .arg("-Zbuild-std=alloc,core")
        .arg("-Zrustdoc-map")
        .arg("--lib")
        .arg("--no-deps");

    let args = builder.build();
    log::debug!("{args:#?}");

    // Execute `cargo doc` from the package root:
    cargo::run_with_env(
        &args,
        &package_path,
        [("RUSTDOCFLAGS", "--cfg docsrs --cfg not_really_docsrs")],
        false,
    )?;

    let docs_path = windows_safe_path(
        &workspace
            .join(package.to_string())
            .join("target")
            .join(target)
            .join("doc"),
    );

    Ok(docs_path)
}

fn apply_feature_rules(package: &Package, config: &Config) -> Vec<String> {
    let chip_name = &config.name();

    let mut features = vec![];
    match package {
        Package::EspHal => {
            features.push("unstable".to_owned());
            features.push("ci".to_owned());
            match chip_name.as_str() {
                "esp32" => features.push("psram".to_owned()),
                "esp32s2" => features.push("psram".to_owned()),
                "esp32s3" => features.push("psram".to_owned()),
                _ => {}
            };
        }
        Package::EspWifi => {
            features.push("esp-hal/unstable".to_owned());
            if config.contains("wifi") {
                features.push("wifi".to_owned());
                features.push("esp-now".to_owned());
                features.push("sniffer".to_owned());
                features.push("utils".to_owned());
                features.push("smoltcp/proto-ipv4".to_owned());
                features.push("smoltcp/proto-ipv6".to_owned());
            }
            if config.contains("ble") {
                features.push("ble".to_owned());
            }
            if config.contains("wifi") && config.contains("ble") {
                features.push("coex".to_owned());
            }
        }
        Package::EspHalEmbassy => {
            features.push("esp-hal/unstable".to_owned());
        }
        _ => {}
    }
    features
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
    log::info!(
        "Building example '{}' for '{}'",
        app.example_path().display(),
        chip
    );

    let mut features = app.feature_set().to_vec();
    if !features.is_empty() {
        log::info!("Features: {}", features.join(","));
    }
    features.push(chip.to_string());

    let package = app.example_path().strip_prefix(package_path)?;
    log::info!("Package: {}", package.display());

    let mut builder = CargoArgsBuilder::default()
        .target(target)
        .features(&features);

    let bin_arg = if package.starts_with("src/bin") {
        format!("--bin={}", app.name())
    } else if package.starts_with("tests") {
        format!("--test={}", app.name())
    } else {
        format!("--example={}", app.name())
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
        cargo::run(&args, package_path)?;

        // Now that the build has succeeded and we printed the output, we can
        // rerun the build again quickly enough to capture JSON. We'll use this to
        // copy the binary to the output directory.
        builder.add_arg("--message-format=json");
        let args = builder.build();
        let output = cargo::run_and_capture(&args, package_path)?;
        for line in output.lines() {
            if let Ok(artifact) = serde_json::from_str::<cargo::Artifact>(line) {
                let out_dir = out_dir.join(&chip.to_string());
                std::fs::create_dir_all(&out_dir)?;

                let output_file = out_dir.join(app.name());
                std::fs::copy(artifact.executable, &output_file)?;
                log::info!("Output ready: {}", output_file.display());
            }
        }
    } else {
        for i in 0..repeat {
            if repeat != 1 {
                log::info!("Run {}/{}", i + 1, repeat);
            }
            cargo::run(&args, package_path)?;
        }
    }

    Ok(())
}

/// Build the specified package, using the given toolchain/target/features if
/// provided.
pub fn build_package(
    package_path: &Path,
    features: Vec<String>,
    no_default_features: bool,
    toolchain: Option<String>,
    target: Option<String>,
) -> Result<()> {
    log::info!("Building package '{}'", package_path.display());
    if !features.is_empty() {
        log::info!("  Features: {}", features.join(","));
    }
    if let Some(ref target) = target {
        log::info!("  Target:   {}", target);
    }

    let mut builder = CargoArgsBuilder::default()
        .subcommand("build")
        .arg("--release");

    if let Some(toolchain) = toolchain {
        builder = builder.toolchain(toolchain);
    }

    if let Some(target) = target {
        // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
        if target.starts_with("xtensa") {
            builder = builder.toolchain("esp");
            builder = builder.arg("-Zbuild-std=core,alloc")
        }
        builder = builder.target(target);
    }

    if !features.is_empty() {
        builder = builder.features(&features);
    }

    if no_default_features {
        builder = builder.arg("--no-default-features");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run(&args, package_path)?;

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
        }
        Version::Minor => {
            version.minor += 1;
            version.patch = 0;
        }
        Version::Patch => {
            version.patch += 1;
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

            manifest["dependencies"].as_table_mut().map(|table| {
                table[&package.to_string()]["version"] = toml_edit::value(version.to_string())
            });

            fs::write(&manifest_path, manifest.to_string())
                .with_context(|| format!("Could not write {}", manifest_path.display()))?;
        }
    }

    Ok(())
}

// File header for the generated eFuse fields.
const EFUSE_FIELDS_RS_HEADER: &str = r#"
//! eFuse fields for the $CHIP.
//!
//! This file was automatically generated, please do not edit it manually!
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
//!
//! Generated on:   $DATE
//! ESP-IDF Commit: $HASH

use super::EfuseBlock;
use crate::soc::efuse_field::EfuseField;
"#;

#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
struct EfuseField {
    field_name: String,
    efuse_block: String,
    bit_start: u32,
    bit_count: u32,
    description: String,
}

/// Generate Rust constants for each eFuse field defined in the given CSV file.
pub fn generate_efuse_table(
    chip: &Chip,
    idf_path: impl AsRef<Path>,
    out_path: impl AsRef<Path>,
) -> Result<()> {
    let idf_path = idf_path.as_ref();
    let out_path = out_path.as_ref();

    // We will put the date of generation in the file header:
    let date = chrono::Utc::now().date_naive();

    // Determine the commit (short) hash of the HEAD commit in the
    // provided ESP-IDF repository:
    let output = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(idf_path)
        .output()?;
    let idf_hash = String::from_utf8_lossy(&output.stdout[0..=7]).to_string();

    // Read the CSV file containing the eFuse field definitions:
    let csv_path = idf_path
        .join("components")
        .join("efuse")
        .join(chip.to_string())
        .join("esp_efuse_table.csv");

    // Create the reader and writer from our source and destination file paths:
    let mut reader = csv::ReaderBuilder::new()
        .comment(Some(b'#'))
        .has_headers(false)
        .trim(csv::Trim::All)
        .from_path(csv_path)?;
    let mut writer = File::create(out_path)?;

    // Write the header to the destination file:
    writeln!(
        writer,
        "{}",
        EFUSE_FIELDS_RS_HEADER
            .trim_start()
            .replace("$CHIP", chip.pretty_name())
            .replace("$DATE", &date.to_string())
            .replace("$HASH", &idf_hash)
    )?;

    // Build a vector of parsed eFuse fields; we build this vector up first rather
    // than writing directly to the destination file, as we need to do some
    // pre-processing first:
    let mut fields = VecDeque::new();
    for result in reader.deserialize() {
        // We will print a warning and just ignore any fields which cannot be
        // successfull parsed:
        let mut efuse_field: EfuseField = match result {
            Ok(field) => field,
            Err(e) => {
                log::warn!("{e}");
                continue;
            }
        };

        // Remove any comments from the eFuse field descriptions:
        efuse_field.description.truncate(
            if let Some((prefix, _comment)) = efuse_field.description.split_once('#') {
                prefix
            } else {
                &efuse_field.description
            }
            .trim_end()
            .len(),
        );

        // Link to other eFuse fields in documentation, using code blocks:
        efuse_field.description = efuse_field
            .description
            .replace('[', "`[")
            .replace(']', "]`");

        // Convert the eFuse field name into a valid Rust iddentifier:
        efuse_field.field_name = efuse_field.field_name.replace('.', "_");

        // Replace any non-digit characters in the eFuse block:
        efuse_field.efuse_block = efuse_field
            .efuse_block
            .replace(|c: char| !c.is_ascii_digit(), "");

        fields.push_back(efuse_field);
    }

    // Now that we've parsed all eFuse field definitions, we can perform our
    // pre-processing; right now, this just means handling any multi-world
    // fields:
    let mut i = 0;
    while i < fields.len() {
        let field = fields[i].clone();

        if field.field_name.is_empty() {
            let mut prev = fields[i - 1].clone();
            prev.bit_start = field.bit_start;
            prev.bit_count += field.bit_count;
            fields[i - 1] = prev;

            fields.retain(|x| *x != field);
        } else {
            i += 1;
        }
    }

    // Finally, write out each eFuse field definition to the destination file:
    while let Some(EfuseField {
        field_name,
        efuse_block,
        bit_start,
        bit_count,
        description,
    }) = fields.pop_front()
    {
        writeln!(writer, "/// {description}")?;
        writeln!(writer,
            "pub const {field_name}: EfuseField = EfuseField::new(EfuseBlock::Block{efuse_block}, {bit_start}, {bit_count});"
        )?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

/// Return a (sorted) list of paths to each valid Cargo package in the
/// workspace.
pub fn package_paths(workspace: &Path) -> Result<Vec<PathBuf>> {
    let mut paths = Vec::new();
    for entry in fs::read_dir(workspace)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            if entry.path().join("Cargo.toml").exists() {
                paths.push(entry.path());
            }
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

/// Return the target triple for a given package/chip pair.
pub fn target_triple(package: Package, chip: &Chip) -> Result<&str> {
    if package == Package::EspLpHal {
        chip.lp_target()
    } else {
        Ok(chip.target())
    }
}
