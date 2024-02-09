use std::{
    collections::VecDeque,
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

use anyhow::{bail, Result};
use clap::ValueEnum;
use strum::{Display, EnumIter, IntoEnumIterator};

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

    pub fn toolchain(&self) -> &str {
        use Chip::*;

        match self {
            Esp32 | Esp32s2 | Esp32s3 => "xtensa",
            _ => "nightly",
        }
    }
}

#[derive(Debug)]
pub struct Metadata {
    path: PathBuf,
    chips: Vec<Chip>,
    features: Vec<String>,
}

impl Metadata {
    pub fn new(path: PathBuf, chips: Vec<Chip>, features: Vec<String>) -> Self {
        let chips = if chips.is_empty() {
            Chip::iter().collect()
        } else {
            chips
        };

        Self {
            path,
            chips,
            features,
        }
    }

    /// Absolute path to the example.
    pub fn path(&self) -> &Path {
        &self.path
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

pub fn load_examples(workspace: &Path) -> Result<Vec<Metadata>> {
    let bin_path = workspace
        .join("examples")
        .join("src")
        .join("bin")
        .canonicalize()?;

    let mut examples = Vec::new();

    for entry in fs::read_dir(bin_path)? {
        let path = entry?.path();
        let text = fs::read_to_string(&path)?;

        let mut chips = Vec::new();
        let mut features = Vec::new();

        // We will indicate metadata lines using the `//%` prefix:
        let lines = text
            .lines()
            .filter(|line| line.starts_with("//%"))
            .collect::<Vec<_>>();

        for line in lines {
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

        let meta = Metadata::new(path, chips, features);
        examples.push(meta);
    }

    Ok(examples)
}

pub fn build_example(workspace: &Path, chip: Chip, example: &Metadata) -> Result<()> {
    let path = example.path();
    let features = example.features();

    log::info!("Building example '{}' for '{}'", path.display(), chip);
    if !features.is_empty() {
        log::info!("  Features: {}", features.join(","));
    }

    let bin_name = example
        .path()
        .file_name()
        .unwrap()
        .to_string_lossy()
        .replace(".rs", "");

    let args = &[
        &format!("+{}", chip.toolchain()),
        "build",
        "--release",
        &format!("--target={}", chip.target()),
        &format!("--features={},{}", chip, features.join(",")),
        &format!("--bin={bin_name}"),
    ];
    log::debug!("{args:#?}");

    Command::new("cargo")
        .args(args)
        .current_dir(workspace.join("examples"))
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .output()?;

    Ok(())
}
