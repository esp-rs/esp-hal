use std::{
    collections::VecDeque,
    fs::{self, File},
    io::{BufRead as _, BufReader, Write as _},
    path::{Path, PathBuf},
};

use anyhow::{bail, Result};
use clap::ValueEnum;
use strum::{Display, EnumIter, IntoEnumIterator};

use self::cargo::CargoArgsBuilder;

pub mod cargo;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Display, EnumIter, ValueEnum)]
#[strum(serialize_all = "kebab-case")]
pub enum Package {
    EspHal,
    EspHalProcmacros,
    EspHalSmartled,
    EspLpHal,
    EspRiscvRt,
    Examples,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Display, EnumIter, ValueEnum)]
#[strum(serialize_all = "kebab-case")]
pub enum Chip {
    Esp32,
    Esp32c2,
    Esp32c3,
    Esp32c6,
    Esp32h2,
    Esp32p4,
    Esp32s2,
    Esp32s3,
}

impl Chip {
    pub fn target(&self) -> &str {
        use Chip::*;

        match self {
            Esp32 => "xtensa-esp32-none-elf",
            Esp32c2 | Esp32c3 => "riscv32imc-unknown-none-elf",
            Esp32c6 | Esp32h2 => "riscv32imac-unknown-none-elf",
            Esp32p4 => "riscv32imafc-unknown-none-elf",
            Esp32s2 => "xtensa-esp32s2-none-elf",
            Esp32s3 => "xtensa-esp32s3-none-elf",
        }
    }

    pub fn has_lp_core(&self) -> bool {
        use Chip::*;

        matches!(self, Esp32c6 | Esp32p4 | Esp32s2 | Esp32s3)
    }

    pub fn lp_target(&self) -> Result<&str> {
        use Chip::*;

        match self {
            Esp32c6 => Ok("riscv32imac-unknown-none-elf"),
            Esp32s2 | Esp32s3 => Ok("riscv32imc-unknown-none-elf"),
            _ => bail!("Chip does not contain an LP core: '{}'", self),
        }
    }

    pub fn pretty_name(&self) -> &str {
        match self {
            Chip::Esp32 => "ESP32",
            Chip::Esp32c2 => "ESP32-C2",
            Chip::Esp32c3 => "ESP32-C3",
            Chip::Esp32c6 => "ESP32-C6",
            Chip::Esp32h2 => "ESP32-H2",
            Chip::Esp32p4 => "ESP32-P4",
            Chip::Esp32s2 => "ESP32-S2",
            Chip::Esp32s3 => "ESP32-S3",
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Metadata {
    example_path: PathBuf,
    chips: Vec<Chip>,
    features: Vec<String>,
}

impl Metadata {
    pub fn new(example_path: &Path, chips: Vec<Chip>, features: Vec<String>) -> Self {
        let chips = if chips.is_empty() {
            Chip::iter().collect()
        } else {
            chips
        };

        Self {
            example_path: example_path.to_path_buf(),
            chips,
            features,
        }
    }

    /// Absolute path to the example.
    pub fn example_path(&self) -> &Path {
        &self.example_path
    }

    /// Name of the example.
    pub fn name(&self) -> String {
        self.example_path()
            .file_name()
            .unwrap()
            .to_string_lossy()
            .replace(".rs", "")
    }

    /// A list of all features required for building a given examples.
    pub fn features(&self) -> &[String] {
        &self.features
    }

    /// If the specified chip is in the list of chips, then it is supported.
    pub fn supports_chip(&self, chip: Chip) -> bool {
        self.chips.contains(&chip)
    }
}

#[derive(Debug, Clone, Copy, Display, ValueEnum)]
#[strum(serialize_all = "lowercase")]
pub enum Version {
    Major,
    Minor,
    Patch,
}

/// Build the documentation for the specified package and device.
pub fn build_documentation(
    workspace: &Path,
    package: Package,
    chip: Chip,
    target: &str,
    open: bool,
) -> Result<()> {
    let package_name = package.to_string();
    let package_path = windows_safe_path(&workspace.join(&package_name));

    log::info!("Building '{package_name}' documentation targeting '{chip}'");

    // Build up an array of command-line arguments to pass to `cargo`:
    let mut builder = CargoArgsBuilder::default()
        .subcommand("doc")
        .arg("-Zbuild-std=core") // Required for Xtensa, for some reason
        .target(target)
        .features(&[chip.to_string()]);

    if open {
        builder = builder.arg("--open");
    }

    // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
    if target.starts_with("xtensa") {
        builder = builder.toolchain("esp");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    // Execute `cargo doc` from the package root:
    cargo::run(&args, &package_path)?;

    Ok(())
}

/// Load all examples at the given path, and parse their metadata.
pub fn load_examples(path: &Path) -> Result<Vec<Metadata>> {
    let mut examples = Vec::new();

    for entry in fs::read_dir(path)? {
        let path = windows_safe_path(&entry?.path());
        let text = fs::read_to_string(&path)?;

        let mut chips = Vec::new();
        let mut features = Vec::new();

        // We will indicate metadata lines using the `//%` prefix:
        for line in text.lines().filter(|line| line.starts_with("//%")) {
            let mut split = line
                .trim_start_matches("//%")
                .trim()
                .split_ascii_whitespace()
                .map(|s| s.to_string())
                .collect::<VecDeque<_>>();

            if split.len() < 2 {
                bail!(
                    "Expected at least two elements (key, value), found {}",
                    split.len()
                );
            }

            // The trailing ':' on metadata keys is optional :)
            let key = split.pop_front().unwrap();
            let key = key.trim_end_matches(':');

            if key == "CHIPS" {
                chips = split
                    .iter()
                    .map(|s| Chip::from_str(s, false).unwrap())
                    .collect::<Vec<_>>();
            } else if key == "FEATURES" {
                features = split.into();
            } else {
                log::warn!("Unregognized metadata key '{key}', ignoring");
            }
        }

        examples.push(Metadata::new(&path, chips, features));
    }

    Ok(examples)
}

/// Build the specified example for the specified chip.
pub fn build_example(
    package_path: &Path,
    chip: Chip,
    target: &str,
    example: &Metadata,
) -> Result<()> {
    log::info!(
        "Building example '{}' for '{}'",
        example.example_path().display(),
        chip
    );
    if !example.features().is_empty() {
        log::info!("  Features: {}", example.features().join(","));
    }

    let bin = if example
        .example_path()
        .strip_prefix(package_path)?
        .starts_with("src/bin")
    {
        format!("--bin={}", example.name())
    } else {
        format!("--example={}", example.name())
    };

    let mut features = example.features().to_vec();
    features.push(chip.to_string());

    let mut builder = CargoArgsBuilder::default()
        .subcommand("build")
        .arg("-Zbuild-std=alloc,core")
        .arg("--release")
        .target(target)
        .features(&features)
        .arg(bin);

    // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
    if target.starts_with("xtensa") {
        builder = builder.toolchain("esp");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run(&args, package_path)?;

    Ok(())
}

/// Run the specified example for the specified chip.
pub fn run_example(
    package_path: &Path,
    chip: Chip,
    target: &str,
    example: &Metadata,
) -> Result<()> {
    log::info!(
        "Building example '{}' for '{}'",
        example.example_path().display(),
        chip
    );
    if !example.features().is_empty() {
        log::info!("  Features: {}", example.features().join(","));
    }

    let bin = if example
        .example_path()
        .strip_prefix(package_path)?
        .starts_with("src/bin")
    {
        format!("--bin={}", example.name())
    } else {
        format!("--example={}", example.name())
    };

    let mut features = example.features().to_vec();
    features.push(chip.to_string());

    let mut builder = CargoArgsBuilder::default()
        .subcommand("run")
        .arg("-Zbuild-std=alloc,core")
        .arg("--release")
        .target(target)
        .features(&features)
        .arg(bin);

    // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
    if target.starts_with("xtensa") {
        builder = builder.toolchain("esp");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run_with_input(&args, package_path)?;
    Ok(())
}

/// Build the specified package, using the given toolchain/target/features if
/// provided.
pub fn build_package(
    package_path: &Path,
    features: Vec<String>,
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
        .arg("-Zbuild-std=core")
        .arg("--release");

    if let Some(toolchain) = toolchain {
        builder = builder.toolchain(toolchain);
    }

    if let Some(target) = target {
        builder = builder.target(target);
    }

    if !features.is_empty() {
        builder = builder.features(&features);
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run(&args, package_path)?;

    Ok(())
}

/// Bump the version of the specified package by the specified amount.
pub fn bump_version(workspace: &Path, package: Package, amount: Version) -> Result<()> {
    let manifest_path = workspace.join(package.to_string()).join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)?;

    let mut manifest = manifest.parse::<toml_edit::Document>()?;

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

use super::EfuseBlock;
use crate::soc::efuse_field::EfuseField;
"#;

/// Generate Rust constants for each eFuse field defined in the given CSV file.
pub fn generate_efuse_table(
    chip: &Chip,
    csv_path: impl AsRef<Path>,
    out_path: impl AsRef<Path>,
) -> Result<()> {
    let csv_path = csv_path.as_ref();
    let out_path = out_path.as_ref();

    // Create the reader and writer from our source and destination file paths:
    let mut reader = BufReader::new(File::open(csv_path)?);
    let mut writer = File::create(out_path)?;

    // Write the header to the destination file:
    writeln!(
        writer,
        "{}",
        EFUSE_FIELDS_RS_HEADER
            .trim_start()
            .replace("$CHIP", chip.pretty_name())
    )?;

    // Generate constants from the CSV eFuse table, and write them out to
    // the destination file:
    let mut line = String::with_capacity(128);
    while reader.read_line(&mut line)? > 0 {
        line = line
            .trim_end_matches('\n')
            .trim_end_matches('\r')
            .to_string();

        // Drop comment and trim:
        line.truncate(
            if let Some((prefix, _comment)) = line.split_once('#') {
                prefix
            } else {
                &line
            }
            .trim()
            .len(),
        );

        // Skip empty lines (and in turn, comments):
        if line.is_empty() {
            continue;
        }

        let mut fields = line.split(',');

        match (
            fields.next().map(|s| s.trim().replace('.', "_")),
            fields
                .next()
                .map(|s| s.trim().replace(|c: char| !c.is_ascii_digit(), "")),
            fields
                .next()
                .map(|s| s.trim())
                .and_then(|s| s.parse::<u32>().ok()),
            fields
                .next()
                .map(|s| s.trim())
                .and_then(|s| s.parse::<u32>().ok()),
            fields.next().map(|s| s.trim()),
        ) {
            (Some(name), Some(block), Some(bit_off), Some(bit_len), Some(desc)) => {
                let desc = desc.replace('[', "`[").replace(']', "]`");
                writeln!(writer, "/// {desc}")?;
                writeln!(
                    writer,
                    "pub const {name}: EfuseField = EfuseField::new(EfuseBlock::Block{block}, {bit_off}, {bit_len});"
                )?;
            }
            other => eprintln!("Invalid data: {other:?}"),
        }

        line.clear();
    }

    Ok(())
}

/// Make the path "Windows"-safe
pub fn windows_safe_path(path: &Path) -> PathBuf {
    PathBuf::from(path.to_str().unwrap().to_string().replace("\\\\?\\", ""))
}
