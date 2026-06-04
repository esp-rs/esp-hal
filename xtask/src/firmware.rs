use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use esp_metadata::{Chip, Config};
use serde::Deserialize;
use strum::IntoEnumIterator as _;

use crate::windows_safe_path;

/// A single, configured example (or test).
#[derive(Debug, Clone)]
pub struct Metadata {
    example_path: PathBuf,
    chip: Chip,
    configuration_name: String,
    features: Vec<String>,
    tag: Option<String>,
    description: Option<String>,
    harness_firmware: Option<String>,
    support_firmware: bool,
    env_vars: HashMap<String, String>,
    cargo_config: Vec<String>,
}

impl Metadata {
    /// Absolute path to the example.
    pub fn example_path(&self) -> &Path {
        &self.example_path
    }

    /// Name of the example.
    pub fn binary_name(&self) -> String {
        self.example_path()
            .file_name()
            .unwrap()
            .to_string_lossy()
            .replace(".rs", "")
    }

    /// Name of the example, including the name of the configuration.
    pub fn output_file_name(&self) -> String {
        if self.configuration_name.is_empty() {
            self.binary_name()
        } else {
            format!("{}_{}", self.binary_name(), self.configuration_name)
        }
    }

    /// The name of the configuration.
    pub fn configuration(&self) -> &str {
        &self.configuration_name
    }

    /// Name of the example, including the name of the configuration.
    pub fn name_with_configuration(&self) -> String {
        if self.configuration_name.is_empty() {
            self.binary_name()
        } else {
            format!("{} ({})", self.binary_name(), self.configuration_name)
        }
    }

    /// A list of all features required for building a given example.
    pub fn feature_set(&self) -> &[String] {
        &self.features
    }

    /// A list of all env vars to build a given example.
    pub fn env_vars(&self) -> &HashMap<String, String> {
        &self.env_vars
    }

    /// A list of all cargo `--config` values to use.
    pub fn cargo_config(&self) -> &[String] {
        &self.cargo_config
    }

    /// If the specified chip is in the list of chips, then it is supported.
    pub fn supports_chip(&self, chip: Chip) -> bool {
        self.chip == chip
    }

    /// Optional tag of the example.
    pub fn tag(&self) -> Option<String> {
        self.tag.clone()
    }

    /// Optional description of the example.
    pub fn description(&self) -> Option<String> {
        self.description.clone()
    }

    /// Optional support firmware binary to run on a second target.
    pub fn harness_firmware(&self) -> Option<&str> {
        self.harness_firmware.as_deref()
    }

    /// True if this artifact is support firmware and should not run as a DUT test.
    pub fn is_support_firmware(&self) -> bool {
        self.support_firmware
    }

    /// Check if the example matches the given filter.
    pub fn matches(&self, filter: Option<&str>) -> bool {
        let Some(filter) = filter else {
            return false;
        };

        filter == self.binary_name() || filter == self.output_file_name()
    }

    /// Check if the example matches the given name (case insensitive).
    pub fn matches_name(&self, name: &str) -> bool {
        name.to_lowercase() == self.binary_name() || name.to_lowercase() == self.output_file_name()
    }
}

/// A single configuration of an example, as parsed from metadata lines.
#[derive(Debug, Default, Clone)]
pub struct Configuration {
    chips: Vec<Chip>,
    name: String,
    cargo_config: Vec<String>,
    features: Vec<String>,
    esp_config: HashMap<String, String>,
    tag: Option<String>,
    harness_firmware: Option<String>,
    support_firmware: Option<bool>,
}

struct ConfigurationCollector<'a> {
    configurations: &'a mut HashMap<String, Configuration>,
    all_configurations: &'a mut Configuration,
    meta_line: &'a MetaLine,
}

impl ConfigurationCollector<'_> {
    fn apply(&mut self, callback: impl Fn(&mut Configuration)) {
        if self.meta_line.config_names.is_empty() {
            callback(self.all_configurations);
        } else {
            for config_name in &self.meta_line.config_names {
                let meta = self
                    .configurations
                    .entry(config_name.clone())
                    .or_insert_with(|| Configuration {
                        name: config_name.clone(),
                        ..Configuration::default()
                    });
                callback(meta);
            }
        }
    }
}

struct MetaLine {
    key: String,
    config_names: Vec<String>,
    value: String,
}

/// Parse a metadata line from an example file.
///
/// Metadata lines come in the form of:
///
/// - `//% METADATA_KEY: value` or
/// - `//% METADATA_KEY(config_name_1, config_name_2, ...): value`.
fn parse_meta_line(line: &str) -> anyhow::Result<MetaLine> {
    let Some((key, value)) = line.trim_start_matches("//%").split_once(':') else {
        bail!("Metadata line is missing ':': {}", line);
    };

    let (key, config_names) = if let Some((key, config_names)) = key.split_once('(') {
        let config_names = config_names
            .trim_end_matches(')')
            .split(',')
            .map(str::trim)
            .map(ToString::to_string)
            .collect();
        (key.trim(), config_names)
    } else {
        (key, Vec::new())
    };

    let key = key.trim();
    let value = value.trim();

    Ok(MetaLine {
        key: key.to_string(),
        config_names,
        value: value.to_string(),
    })
}

/// Returns the chips selected by a `CHIP_FILTER` expression (a boolean expression over
/// cfg symbols and chip names, e.g. `cfg_symbol && !esp32` or `esp32c6 || esp32h2`).
fn parse_chips(expr: &str) -> anyhow::Result<Vec<Chip>> {
    fn symbol_to_ident(s: &String) -> Option<String> {
        s.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.')
            .then_some(s.replace(".", "_"))
    }

    let possible_symbols = Chip::list_of_possible_symbols()
        .iter()
        .filter_map(|(sym, values)| {
            if values.is_none() {
                symbol_to_ident(sym)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();

    let mut chips = Vec::new();
    for chip in Chip::iter() {
        let config = Config::for_chip(&chip);
        let chip_symbols = config
            .all()
            .iter()
            .filter_map(symbol_to_ident)
            .collect::<Vec<_>>();

        let mut ctx = somni_expr::Context::new();

        // All known symbols are initially false
        for sym in possible_symbols.iter() {
            ctx.add_variable(sym.as_str(), false);
        }

        // All defined symbols for this chip are true
        for sym in chip_symbols.iter() {
            ctx.add_variable(sym.as_str(), true);
        }

        let selected = ctx.evaluate::<bool>(expr).map_err(|err| {
            anyhow::anyhow!("{err:?}").context("Failed to evaluate chip expression")
        })?;
        if selected {
            chips.push(chip);
        }
    }
    Ok(chips)
}

/// Load all examples at the given path, and parse their metadata.
pub fn load(path: &Path) -> Result<Vec<Metadata>> {
    let mut examples = Vec::new();

    for entry in fs::read_dir(path).context("Failed to read {path}")? {
        let entry = entry?;
        if !entry.file_type()?.is_file() {
            continue;
        }
        log::debug!("Loading example from path: {}", path.display());
        let path = windows_safe_path(&entry.path());
        let text = fs::read_to_string(&path)
            .with_context(|| format!("Could not read {}", path.display()))?;

        let description = parse_description(&text);

        // When the list of configuration names is missing, the metadata is applied to
        // all configurations. Each configuration encountered will create a
        // separate Metadata entry. Different metadata lines referring to the
        // same configuration will be merged.
        //
        // If there are no named configurations, an unnamed default is created.
        let mut all_configuration = Configuration {
            chips: Chip::iter().collect::<Vec<_>>(),
            ..Configuration::default()
        };

        let mut configurations = HashMap::<String, Configuration>::new();

        // Unless specified, an example is assumed to be valid for all chips.
        for (line_no, line) in text
            .lines()
            .enumerate()
            .filter(|(_, line)| line.starts_with("//%"))
        {
            let meta_line = parse_meta_line(line)
                .with_context(|| format!("Failed to parse line {}", line_no + 1))?;

            let mut relevant_metadata = ConfigurationCollector {
                configurations: &mut configurations,
                all_configurations: &mut all_configuration,
                meta_line: &meta_line,
            };

            match meta_line.key.as_str() {
                "CHIP_FILTER" => {
                    let chips = parse_chips(meta_line.value.as_str())?;
                    relevant_metadata.apply(|meta| meta.chips = chips.clone());
                }
                // A list of cargo `--config` configurations.
                "CARGO-CONFIG" => {
                    relevant_metadata
                        .apply(|meta| meta.cargo_config.push(meta_line.value.to_string()));
                }
                // Cargo features to enable for the current configuration.
                "FEATURES" => {
                    let mut values = meta_line
                        .value
                        .split_ascii_whitespace()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>();

                    // Sort the features so they are in a deterministic order:
                    values.sort();

                    relevant_metadata.apply(|meta| meta.features.extend_from_slice(&values));
                }
                // esp-config env vars, one per line
                "ENV" => {
                    let (env_var, value) = meta_line
                        .value
                        .split_once('=')
                        .with_context(|| "CONFIG metadata must be in the form 'CONFIG=VALUE'")?;

                    let env_var = env_var.trim();
                    let value = value.trim();

                    relevant_metadata.apply(|meta| {
                        meta.esp_config
                            .insert(env_var.to_string(), value.to_string());
                    });
                }
                // Tags by which the user can filter examples.
                "TAG" => {
                    relevant_metadata.apply(|meta| meta.tag = Some(meta_line.value.to_string()));
                }
                // Optional support firmware binary that must run on another target.
                "HARNESS-FIRMWARE" => {
                    relevant_metadata
                        .apply(|meta| meta.harness_firmware = Some(meta_line.value.to_string()));
                }
                // Mark this artifact as support firmware (not a DUT test).
                "SUPPORT-FIRMWARE" | "TEST-SUPPORT-FIRMWARE" => {
                    let support = parse_bool(&meta_line.value).with_context(|| {
                        format!("{} metadata must be true/false", meta_line.key.as_str())
                    })?;
                    relevant_metadata.apply(|meta| meta.support_firmware = Some(support));
                }
                key => log::warn!("Unrecognized metadata key '{key}', ignoring"),
            }
        }

        // Merge "all" into configurations
        for meta in configurations.values_mut() {
            // Chips is a filter, inherit if empty
            if meta.chips.is_empty() {
                meta.chips = all_configuration.chips.clone();
            }

            // Tag is an ID, inherit if empty
            if meta.tag.is_none() {
                meta.tag = all_configuration.tag.clone();
            }

            // Harness firmware is a selector, inherit if empty
            if meta.harness_firmware.is_none() {
                meta.harness_firmware = all_configuration.harness_firmware.clone();
            }

            // Support firmware marker inherits if not explicitly set.
            if meta.support_firmware.is_none() {
                meta.support_firmware = all_configuration.support_firmware;
            }

            // Other values are merged
            meta.features.extend_from_slice(&all_configuration.features);
            meta.esp_config.extend(all_configuration.esp_config.clone());
            meta.cargo_config
                .extend(all_configuration.cargo_config.clone());
        }

        // If no configurations are specified, fall back to the unnamed one. Otherwise
        // ignore it, it has been merged into the others.
        if configurations.is_empty() {
            configurations.insert(String::new(), all_configuration);
        }

        // Generate metadata

        for configuration in configurations.values_mut() {
            // Sort the features so they are in a deterministic order:
            configuration.features.sort();

            for chip in &configuration.chips {
                examples.push(Metadata {
                    // File properties
                    example_path: path.clone(),
                    description: description.clone(),

                    // Configuration
                    chip: *chip,
                    configuration_name: configuration.name.clone(),
                    features: configuration.features.clone(),
                    tag: configuration.tag.clone(),
                    harness_firmware: configuration.harness_firmware.clone(),
                    support_firmware: configuration.support_firmware.unwrap_or(false),
                    env_vars: configuration.esp_config.clone(),
                    cargo_config: configuration.cargo_config.clone(),
                })
            }
        }
    }

    // Sort by feature set, to prevent rebuilding packages if not necessary.
    examples.sort_by_key(|e| e.feature_set().join(","));

    Ok(examples)
}

#[derive(Debug, Deserialize)]
struct CargoToml {
    features: HashMap<String, Vec<String>>,
}

/// Parse the chip set from `//% CHIP_FILTER:` annotations in a source file.
/// Returns `None` if the annotation is not present.
fn parse_chips_from_annotation(
    text: &str,
) -> anyhow::Result<Option<std::collections::HashSet<Chip>>> {
    let mut found = false;
    let mut chips: Vec<Chip> = Chip::iter().collect();

    for (line_no, line) in text
        .lines()
        .enumerate()
        .filter(|(_, l)| l.starts_with("//%"))
    {
        let meta = parse_meta_line(line)
            .with_context(|| format!("Failed to parse line {}", line_no + 1))?;
        match meta.key.as_str() {
            "CHIP_FILTER" => {
                found = true;
                chips = parse_chips(meta.value.as_str())?;
            }
            _ => {}
        }
    }

    if !found {
        return Ok(None);
    }

    Ok(Some(chips.into_iter().collect()))
}

/// Load all examples by finding all packages in the given path, and parsing their metadata.
pub fn load_cargo_toml(examples_path: &Path) -> Result<Vec<Metadata>> {
    let mut examples = Vec::new();

    let mut packages = crate::find_packages(examples_path)?;
    packages.sort();

    for package_path in packages {
        log::debug!("Loading package from path: {}", package_path.display());
        let cargo_toml_path = package_path.join("Cargo.toml");
        let main_rs_path = package_path.join("src").join("main.rs");

        if !cargo_toml_path.exists() || !main_rs_path.exists() {
            continue;
        }

        let text = fs::read_to_string(&main_rs_path)?;
        let description = parse_description(&text);

        let toml = fs::read_to_string(&cargo_toml_path)?;
        let toml: CargoToml = toml_edit::de::from_str(&toml)?;

        let cargo_chips: Vec<Chip> = toml
            .features
            .keys()
            .filter_map(|k| Chip::from_str(k, true).ok())
            .collect();

        let chips_from_annotations = parse_chips_from_annotation(&text).with_context(|| {
            format!("Failed to parse annotations in {}", main_rs_path.display())
        })?;

        // If the annotation requires chips that are not declared in Cargo.toml, bail.
        if let Some(ref required) = chips_from_annotations {
            let missing = required
                .iter()
                .filter(|c| !cargo_chips.contains(c))
                .collect::<Vec<_>>();
            if !missing.is_empty() {
                anyhow::bail!(
                    "{}: chips {missing:?} are required by the annotation but missing from Cargo.toml",
                    package_path.display()
                );
            }
        }

        let chips = cargo_chips.into_iter().filter(|c| {
            chips_from_annotations
                .as_ref()
                .map_or(true, |set| set.contains(c))
        });

        for chip in chips {
            examples.push(Metadata {
                example_path: package_path.clone(),
                chip,
                configuration_name: String::new(),
                features: vec![],
                tag: None,
                description: description.clone(),
                harness_firmware: None,
                support_firmware: false,
                env_vars: HashMap::new(),
                cargo_config: Vec::new(),
            });
        }
    }

    Ok(examples)
}

/// Find the metadata entry for an artifact/test name.
pub fn find_test_by_name<'a>(tests: &'a [Metadata], name: &str) -> Option<&'a Metadata> {
    tests.iter().find(|test| {
        test.binary_name() == name
            || test.output_file_name() == name
            || test.name_with_configuration() == name
    })
}

fn parse_bool(value: &str) -> Result<bool> {
    match value.trim().to_ascii_lowercase().as_str() {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => bail!("invalid boolean value: {value}"),
    }
}

fn parse_description(text: &str) -> Option<String> {
    let mut description = None;

    for line in text.lines().filter(|line| line.starts_with("//!")) {
        let line = line.trim_start_matches("//!");
        let mut descr: String = description.unwrap_or_default();
        descr.push_str(line);
        descr.push('\n');
        description = Some(descr);
    }

    log::debug!("Parsed description: {:?}", description);

    description
}
