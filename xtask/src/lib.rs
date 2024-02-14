use std::{
    collections::VecDeque,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{bail, Result};
use clap::ValueEnum;
use strum::{Display, EnumIter, IntoEnumIterator};

use self::cargo::CargoArgsBuilder;

mod cargo;

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

/// Build the documentation for the specified package and device.
pub fn build_documentation(
    workspace: &Path,
    package: Package,
    chip: Chip,
    target: &str,
    open: bool,
) -> Result<()> {
    let package_name = package.to_string();
    let package_path = workspace.join(&package_name);

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
        let path = entry?.path();
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

