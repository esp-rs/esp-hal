//! Chip metadata, read from the cache written by the `esp-metadata` tool.
//!
//! The devtool deliberately does not depend on `esp-metadata` as a library:
//! that would rebuild (and relink) the devtool every time the shape of the
//! metadata changes. Instead it reads a cache of the data it needs, and asks
//! the tool to regenerate when the metadata inputs have changed. Regenerating
//! also refreshes `esp-metadata-generated`, which anything we build afterwards
//! needs to be up to date anyway.

use std::{
    collections::{BTreeMap, HashMap},
    path::{Path, PathBuf},
    process::Command,
    sync::OnceLock,
};

use anyhow::{Context, Result, bail};
use strum::IntoEnumIterator;

/// Cache layout this devtool understands. Must match `esp-metadata`.
const CACHE_VERSION: u32 = 1;

/// Supported devices.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
    strum::EnumString,
    strum::AsRefStr,
    clap::ValueEnum,
)]
#[serde(rename_all = "kebab-case")]
#[strum(serialize_all = "kebab-case")]
pub enum Chip {
    /// ESP32
    Esp32,
    /// ESP32-C2, ESP8684
    Esp32c2,
    /// ESP32-C3, ESP8685
    Esp32c3,
    /// ESP32-C5
    Esp32c5,
    /// ESP32-C6
    Esp32c6,
    /// ESP32-C61
    Esp32c61,
    /// ESP32-H2
    Esp32h2,
    /// ESP32-P4 (chip revision v3.x / eco5 only)
    Esp32p4,
    /// ESP32-S2
    Esp32s2,
    /// ESP32-S3
    Esp32s3,
    /// ESP32-S31
    Esp32s31,
}

impl Chip {
    pub fn target(&self) -> String {
        cache().chip(self).target.clone()
    }

    pub fn has_lp_core(&self) -> bool {
        cache().chip(self).has_lp_core
    }

    pub fn lp_target(&self) -> Result<&'static str> {
        match cache().chip(self).lp_target.as_deref() {
            Some(target) => Ok(target),
            None => bail!("Chip does not contain an LP core: '{self}'"),
        }
    }

    pub fn pretty_name(&self) -> &'static str {
        &cache().chip(self).pretty_name
    }

    pub fn is_xtensa(&self) -> bool {
        cache().chip(self).arch == "xtensa"
    }

    pub fn is_riscv(&self) -> bool {
        !self.is_xtensa()
    }

    /// Every valueless symbol defined by any chip.
    pub fn all_symbols() -> &'static [String] {
        &cache().symbols
    }

    /// Every symbol with a value defined by any chip.
    pub fn all_kv_symbols() -> &'static [String] {
        &cache().kv_symbols
    }
}

/// The configuration symbols of a single device.
#[derive(Debug, Default, Clone)]
pub struct Config {
    /// The name of the device.
    pub name: String,

    /// The valueless symbols of the device.
    pub symbols: Vec<String>,

    /// The symbols of the device that have a value.
    pub kv_values: BTreeMap<String, String>,
}

impl Config {
    /// The configuration for the specified chip.
    pub fn for_chip(chip: &Chip) -> &'static Self {
        static CONFIGS: OnceLock<HashMap<Chip, Config>> = OnceLock::new();
        &CONFIGS.get_or_init(|| {
            Chip::iter()
                .map(|chip| {
                    let cached = cache().chip(&chip);
                    let config = Config {
                        name: chip.to_string(),
                        symbols: cached.symbols.clone(),
                        kv_values: cached.kv_values.clone(),
                    };
                    (chip, config)
                })
                .collect()
        })[chip]
    }
}

#[derive(Debug, serde::Deserialize)]
struct Cache {
    version: u32,
    hash: String,
    symbols: Vec<String>,
    kv_symbols: Vec<String>,
    chips: BTreeMap<String, ChipCache>,
}

#[derive(Debug, serde::Deserialize)]
struct ChipCache {
    pretty_name: String,
    arch: String,
    target: String,
    lp_target: Option<String>,
    has_lp_core: bool,
    symbols: Vec<String>,
    kv_values: BTreeMap<String, String>,
}

impl Cache {
    fn chip(&self, chip: &Chip) -> &ChipCache {
        // `load` has verified that every chip is present.
        &self.chips[&chip.to_string()]
    }
}

fn cache() -> &'static Cache {
    static CACHE: OnceLock<Cache> = OnceLock::new();
    CACHE.get_or_init(|| load().expect("Failed to read chip metadata"))
}

fn load() -> Result<Cache> {
    let workspace = workspace_root()?;
    let path = workspace.join("target").join("esp-metadata-cache.toml");

    let mut hash = input_hash(&workspace)?;

    // A cache that is missing, unreadable, stale or of an unexpected version is
    // regenerated once. Anything still wrong afterwards is a real mismatch
    // between the devtool and the tool, which regenerating cannot resolve.
    if !probe(&path)
        .is_some_and(|(version, cached_hash)| version == CACHE_VERSION && cached_hash == hash)
    {
        log::debug!("Refreshing chip metadata cache");
        refresh(&workspace)?;

        // Building the tool writes hashed inputs of its own: `Cargo.lock` is
        // not checked in, so a fresh checkout gains one here. The hash the tool
        // recorded is the one from after that, so recompute ours to match.
        hash = input_hash(&workspace)?;
    }

    let cache = read_cache(&path)?;
    if cache.version != CACHE_VERSION {
        bail!(
            "{} has version {}, but this devtool expects {CACHE_VERSION}. Rebuild the devtool.",
            path.display(),
            cache.version
        );
    }
    if cache.hash != hash {
        bail!("{} is stale after regenerating it", path.display());
    }

    for chip in Chip::iter() {
        // Guards against the chip list here drifting from the metadata.
        if !cache.chips.contains_key(&chip.to_string()) {
            bail!(
                "`esp-metadata` does not know about '{chip}'. The devtool's chip list is out of date."
            );
        }
    }
    if cache.chips.len() != Chip::iter().count() {
        bail!(
            "`esp-metadata` defines chips the devtool does not know about. Add them to `Chip`. Known: {:?}",
            cache.chips.keys().collect::<Vec<_>>()
        );
    }

    Ok(cache)
}

/// Reads the version and hash of a cache, without requiring the rest of it to
/// match the layout this devtool expects.
fn probe(path: &Path) -> Option<(u32, String)> {
    #[derive(serde::Deserialize)]
    struct Probe {
        version: u32,
        hash: String,
    }

    let contents = std::fs::read_to_string(path).ok()?;
    let probe: Probe = toml_edit::de::from_str(&contents).ok()?;

    Some((probe.version, probe.hash))
}

fn read_cache(path: &Path) -> Result<Cache> {
    let contents = std::fs::read_to_string(path)
        .with_context(|| format!("`esp-metadata` did not write {}", path.display()))?;

    toml_edit::de::from_str(&contents)
        .with_context(|| format!("Failed to parse {}", path.display()))
}

fn refresh(workspace: &Path) -> Result<()> {
    log::info!("Regenerating `esp-metadata-generated` and cache");
    let manifest = workspace.join("esp-metadata").join("Cargo.toml");
    let status = Command::new("cargo")
        .args(["run", "--quiet", "--manifest-path"])
        .arg(&manifest)
        .args(["--", "generate"])
        .current_dir(workspace)
        .status()
        .context("Failed to run the `esp-metadata` tool")?;

    if !status.success() {
        bail!("The `esp-metadata` tool failed with {status}");
    }

    Ok(())
}

fn workspace_root() -> Result<PathBuf> {
    let cwd = std::env::current_dir().context("Failed to get the current dir")?;
    for dir in cwd.ancestors() {
        if dir.join("esp-metadata").join("devices").is_dir() {
            return Ok(dir.to_path_buf());
        }
    }

    bail!("Could not find the esp-hal workspace in {}", cwd.display())
}

/// Hash of everything the metadata cache is derived from.
///
/// Mirrors `esp_metadata::input_hash`; keep both in sync.
fn input_hash(workspace: &Path) -> Result<String> {
    use sha2::Digest;

    let root = workspace.join("esp-metadata");
    let mut files = vec![];
    for entry in walkdir::WalkDir::new(&root)
        .sort_by_file_name()
        .into_iter()
        .filter_entry(|e| e.file_name() != "target")
    {
        let path = entry?.into_path();
        if path.is_file() {
            files.push(path);
        }
    }

    let mut hasher = sha2::Sha256::new();
    for file in files {
        let relative = file.strip_prefix(&root).unwrap();
        hasher.update(relative.to_string_lossy().replace('\\', "/").as_bytes());
        hasher.update(
            std::fs::read(&file).with_context(|| format!("Failed to read {}", file.display()))?,
        );
    }

    Ok(format!("{:x}", hasher.finalize()))
}
