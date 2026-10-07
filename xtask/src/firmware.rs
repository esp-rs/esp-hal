use std::{
    collections::{BTreeSet, HashMap},
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use serde::Deserialize;
use strum::IntoEnumIterator as _;
use toml_edit::DocumentMut;

use crate::{Package, ScriptContext, metadata::Chip, windows_safe_path};

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

        self.matches_name(filter)
    }

    /// Returns this example's path relative to `package_root` for matching.
    ///
    /// Drops a `.rs` suffix and a `src/bin/` prefix, so nested projects become
    /// `ota/update` and bins stay `sleep_timer`.
    pub fn lookup_name(&self, package_root: &Path) -> String {
        let path = self.example_path();
        let relative = path.strip_prefix(package_root).unwrap_or(path);
        let mut name = relative.to_string_lossy().replace('\\', "/");
        if let Some(stripped) = name.strip_suffix(".rs") {
            name = stripped.to_string();
        }
        if let Some(stripped) = name.strip_prefix("src/bin/") {
            name = stripped.to_string();
        }
        name
    }

    /// Checks if the example matches the given name (case insensitive).
    ///
    /// Accepts the binary name, the output file name, and a relative path
    /// (`ota/update`, `examples/ota/update`).
    pub fn matches_name(&self, name: &str) -> bool {
        let name = normalize_source_name(name);
        if name.is_empty() {
            return false;
        }
        if name == self.binary_name().to_lowercase()
            || name == self.output_file_name().to_lowercase()
        {
            return true;
        }
        let path = normalize_source_name(&self.example_path().to_string_lossy());
        path.ends_with(&format!("/{name}"))
    }
}

fn normalize_source_name(name: &str) -> String {
    name.trim()
        .trim_end_matches(".rs")
        .replace('\\', "/")
        .to_lowercase()
}

/// A single configuration of an example, as parsed from metadata lines.
#[derive(Debug, Default, Clone)]
pub struct Configuration {
    chips: Option<Vec<Chip>>,
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
///
/// `ENV-IF(expr)` and `FEATURES-IF(expr)` overlay env or features on existing
/// configurations. `expr` is a `CHIP_FILTER` expression; unlike `ENV(name)` it does not
/// create a new configuration binary.
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
/// cfg symbols, key-value cfg symbols, and chip names, e.g. `cfg_symbol && !esp32`,
/// `esp32c6 || esp32h2`, or `interrupt_controller != "clic"`).
fn parse_chips(expr: &str) -> Result<Vec<Chip>> {
    let mut chips = Vec::new();
    for chip in Chip::iter() {
        if chip_matches(chip, expr)? {
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
            chips: Some(Chip::iter().collect::<Vec<_>>()),
            ..Configuration::default()
        };

        let mut configurations = HashMap::<String, Configuration>::new();
        let mut env_overlays: Vec<(Vec<Chip>, String, String)> = Vec::new();
        let mut feature_overlays: Vec<(Vec<Chip>, Vec<String>)> = Vec::new();

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
                    relevant_metadata.apply(|meta| meta.chips = Some(chips.clone()));
                }
                // A list of cargo `--config` configurations.
                "CARGO-CONFIG" => {
                    relevant_metadata
                        .apply(|meta| meta.cargo_config.push(meta_line.value.to_string()));
                }
                // Cargo features to enable for the current configuration.
                "FEATURES" => {
                    let values = parse_feature_list(&meta_line.value);
                    relevant_metadata.apply(|meta| meta.features.extend_from_slice(&values));
                }
                // Same language as CHIP_FILTER. Adds these features on matching chips.
                "FEATURES-IF" => {
                    if meta_line.config_names.is_empty() {
                        bail!("FEATURES-IF needs a chip expression in parentheses");
                    }
                    feature_overlays.push((
                        parse_chips(&meta_line.config_names.join(" || "))?,
                        parse_feature_list(&meta_line.value),
                    ));
                }
                // esp-config env vars, one per line
                "ENV" => {
                    let (env_var, value) = parse_env_assignment(&meta_line.value)?;
                    relevant_metadata.apply(|meta| {
                        meta.esp_config.insert(env_var.clone(), value.clone());
                    });
                }
                // Same language as CHIP_FILTER. Adds or replaces that one variable on
                // matching chips, does not create a configuration.
                "ENV-IF" => {
                    if meta_line.config_names.is_empty() {
                        bail!("ENV-IF needs a chip expression in parentheses");
                    }
                    let (env_var, value) = parse_env_assignment(&meta_line.value)?;
                    env_overlays.push((
                        parse_chips(&meta_line.config_names.join(" || "))?,
                        env_var,
                        value,
                    ));
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
            // Chips is a filter, inherit if unset
            if meta.chips.is_none() {
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

            for chip in configuration.chips.as_deref().unwrap_or(&[]) {
                let mut features = configuration.features.clone();
                for (chips, extra) in &feature_overlays {
                    if chips.contains(chip) {
                        features.extend(extra.iter().cloned());
                    }
                }
                features.sort();
                features.dedup();

                let mut env_vars = configuration.esp_config.clone();
                for (chips, key, value) in &env_overlays {
                    if chips.contains(chip) {
                        env_vars.insert(key.clone(), value.clone());
                    }
                }

                examples.push(Metadata {
                    // File properties
                    example_path: path.clone(),
                    description: description.clone(),

                    // Configuration
                    chip: *chip,
                    configuration_name: configuration.name.clone(),
                    features,
                    tag: configuration.tag.clone(),
                    harness_firmware: configuration.harness_firmware.clone(),
                    support_firmware: configuration.support_firmware.unwrap_or(false),
                    env_vars,
                    cargo_config: configuration.cargo_config.clone(),
                })
            }
        }
    }

    // Sort by feature set, to prevent rebuilding packages if not necessary.
    examples.sort_by_key(|e| e.feature_set().join(","));

    Ok(examples)
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
        if meta.key.as_str() == "CHIP_FILTER" {
            found = true;
            chips = parse_chips(meta.value.as_str())?;
        }
    }

    if !found {
        return Ok(None);
    }

    Ok(Some(chips.into_iter().collect()))
}

/// Load all examples by finding all packages in the given path, and parsing their metadata.
///
/// Two shapes coexist under `examples/` and `compile-tests/`:
///
/// - A project with no per-chip `[features]` table: chips come from its `//% CHIP_FILTER`
///   annotation (every chip when absent), and xtask forwards `<dep>/<chip>` to each chip-aware
///   dependency. This is how compile-tests work.
/// - A self-contained project (e.g. `examples/async/embassy_ethernet`), each buildable on its own
///   and meant to be copied elsewhere as a starting point, declares one `[features]` key per
///   supported chip; that is its chip set, narrowed by an optional `//% CHIP_FILTER`.
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

        let toml_str = fs::read_to_string(&cargo_toml_path)?;
        let doc = toml_str
            .parse::<DocumentMut>()
            .with_context(|| format!("Failed to parse {}", cargo_toml_path.display()))?;

        let annotation_chips = parse_chips_from_annotation(&text).with_context(|| {
            format!("Failed to parse annotations in {}", main_rs_path.display())
        })?;
        let feature_chips = feature_table_chips(&doc);

        if feature_chips.is_empty() {
            let deps = chip_aware_deps(&doc);
            for chip in selected_chips(&annotation_chips) {
                let features = deps.iter().map(|dep| format!("{dep}/{chip}")).collect();
                examples.push(Metadata {
                    example_path: package_path.clone(),
                    chip,
                    configuration_name: String::new(),
                    features,
                    tag: None,
                    description: description.clone(),
                    harness_firmware: None,
                    support_firmware: false,
                    env_vars: HashMap::new(),
                    cargo_config: Vec::new(),
                });
            }
            continue;
        }

        // A `[features]` chip table, narrowed by an optional `//% CHIP_FILTER`. Without an
        // annotation the project must list every chip, otherwise a newly added chip silently
        // drops out of its coverage.
        let missing: Vec<Chip> = Chip::iter()
            .filter(|chip| match &annotation_chips {
                Some(annotated) => annotated.contains(chip),
                None => true,
            })
            .filter(|chip| !feature_chips.contains(chip))
            .collect();
        if !missing.is_empty() {
            bail!(
                "{}: chips {missing:?} are missing from Cargo.toml. Add them, or narrow the set \
                 with a `//% CHIP_FILTER:` annotation.",
                package_path.display()
            );
        }

        for chip in feature_chips {
            if annotation_chips
                .as_ref()
                .is_some_and(|set| !set.contains(&chip))
            {
                continue;
            }
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

/// Chips selected by a `//% CHIP_FILTER` annotation, or every chip when absent.
fn selected_chips(annotation: &Option<std::collections::HashSet<Chip>>) -> Vec<Chip> {
    match annotation {
        Some(set) => Chip::iter().filter(|c| set.contains(c)).collect(),
        None => Chip::iter().collect(),
    }
}

/// Whether the standalone project at `dir` declares a per-chip `[features]`
/// table. Compile-tests do not (their chip rides on forwarded `<dep>/<chip>`
/// features), so the bare chip feature must not be enabled for them.
pub(crate) fn project_has_chip_feature_table(dir: &Path) -> bool {
    std::fs::read_to_string(dir.join("Cargo.toml"))
        .ok()
        .and_then(|s| s.parse::<DocumentMut>().ok())
        .is_some_and(|doc| !feature_table_chips(&doc).is_empty())
}

/// The chips named by a project's `[features]` keys.
fn feature_table_chips(doc: &DocumentMut) -> Vec<Chip> {
    doc.get("features")
        .and_then(|f| f.as_table())
        .map(|table| {
            table
                .iter()
                .filter_map(|(key, _)| Chip::from_str(key, true).ok())
                .collect()
        })
        .unwrap_or_default()
}

/// The `esp-*` dependencies that declare chip features; xtask forwards each as
/// `<dep>/<chip>` for the selected chips. Chip-agnostic deps (esp-alloc) and
/// third-party crates are left out.
fn chip_aware_deps(doc: &DocumentMut) -> Vec<Package> {
    let Some(table) = doc.get("dependencies").and_then(|d| d.as_table()) else {
        return Vec::new();
    };
    let mut deps = Vec::new();
    for (name, item) in table.iter() {
        let real = item.get("package").and_then(|p| p.as_str()).unwrap_or(name);
        if let Ok(pkg) = Package::from_str(real, true)
            && pkg.has_chip_features()
            && !deps.contains(&pkg)
        {
            deps.push(pkg);
        }
    }
    deps.sort();
    deps
}

/// Whether `expr` holds for `chip`, evaluated against the chip's device metadata.
fn chip_matches(chip: Chip, expr: &str) -> Result<bool> {
    let script_ctx = ScriptContext::new();
    let mut ctx = script_ctx.for_chip(chip);
    ctx.evaluate(expr)
}

/// The chips each compile-test project selects (from its `//% CHIP_FILTER`),
/// keyed by project directory name. Report-only helper for the plan and PR body.
pub fn compile_test_coverage(workspace: &Path) -> Result<Vec<(String, Vec<Chip>)>> {
    let root = windows_safe_path(&workspace.join(Package::CompileTests.directory()));
    let mut packages = crate::find_packages(&root)?;
    packages.sort();

    let mut coverage = Vec::new();
    for package_path in packages {
        let main_rs_path = package_path.join("src").join("main.rs");
        if !main_rs_path.exists() {
            continue;
        }
        let text = fs::read_to_string(&main_rs_path)?;
        let chips = selected_chips(&parse_chips_from_annotation(&text)?);
        let name = package_path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        coverage.push((name, chips));
    }

    coverage.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(coverage)
}

/// Render [`compile_test_coverage`] as one `- project: chip, chip` line each.
pub fn format_compile_test_coverage(coverage: &[(String, Vec<Chip>)]) -> String {
    coverage
        .iter()
        .map(|(project, chips)| {
            let chips = if chips.is_empty() {
                "(no chips)".to_string()
            } else {
                chips
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            format!("- {project}: {chips}")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The feature names declared for `version` in a crates.io sparse-index body
/// (newline-delimited JSON). Merges `features` and `features2`, since cargo
/// splits weak/`dep:` syntax into the latter but the names are equally valid.
pub fn features_for_version(
    index_body: &str,
    version: &semver::Version,
) -> Result<BTreeSet<String>> {
    #[derive(Deserialize)]
    struct IndexEntry {
        vers: String,
        #[serde(default)]
        features: HashMap<String, Vec<String>>,
        #[serde(default)]
        features2: Option<HashMap<String, Vec<String>>>,
    }

    for line in index_body.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let entry: IndexEntry = serde_json::from_str(line)
            .with_context(|| format!("Failed to parse crates.io index line: {line}"))?;
        // Compare parsed versions so build metadata on one side doesn't cause a miss.
        let Ok(entry_version) = semver::Version::parse(&entry.vers) else {
            continue;
        };
        if entry_version == *version {
            let mut names: BTreeSet<String> = entry.features.into_keys().collect();
            if let Some(features2) = entry.features2 {
                names.extend(features2.into_keys());
            }
            return Ok(names);
        }
    }

    bail!("version {version} was not found in the crates.io index response");
}

/// The highest non-yanked published version satisfying `req`, from a sparse-index body.
fn max_version_matching(
    index_body: &str,
    req: &semver::VersionReq,
) -> Result<Option<semver::Version>> {
    #[derive(Deserialize)]
    struct VersionEntry {
        vers: String,
        #[serde(default)]
        yanked: bool,
    }

    let mut best: Option<semver::Version> = None;
    for line in index_body.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let entry: VersionEntry = serde_json::from_str(line)
            .with_context(|| format!("Failed to parse crates.io index line: {line}"))?;
        if entry.yanked {
            continue;
        }
        let Ok(version) = semver::Version::parse(&entry.vers) else {
            continue;
        };
        if req.matches(&version) && best.as_ref().is_none_or(|b| version > *b) {
            best = Some(version);
        }
    }

    Ok(best)
}

/// The crates.io sparse-index URL for a crate, following the registry's prefix layout.
fn sparse_index_url(crate_name: &str) -> String {
    let name = crate_name.to_lowercase();
    let prefix = match name.len() {
        1 => "1".to_string(),
        2 => "2".to_string(),
        3 => format!("3/{}", &name[0..1]),
        _ => format!("{}/{}", &name[0..2], &name[2..4]),
    };
    format!("https://index.crates.io/{prefix}/{name}")
}

/// Fetch a crate's sparse-index document via `curl`, or `None` when curl or the
/// network is unavailable. curl avoids a dependency for one best-effort check.
fn fetch_sparse_index(crate_name: &str) -> Option<String> {
    let url = sparse_index_url(crate_name);
    let output = std::process::Command::new("curl")
        .args(["-sSf", &url])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout).ok()
}

/// Whether every `chip-deps` crate of the compile-test project at `project_path`
/// declares the `<chip>` cargo feature at the version that will resolve.
///
/// A frozen dependency line that predates the chip returns `Ok(false)` so the
/// build skips this project for the chip instead of hitting a `<dep>/<chip>`
/// feature cargo cannot find. A crate this release publishes that lacks the
/// feature is a hard error (see [`chip_dep_declares_feature`]).
pub fn compile_test_project_supports_chip(
    workspace: &Path,
    project_path: &Path,
    chip: Chip,
) -> Result<bool> {
    let cargo_toml_path = project_path.join("Cargo.toml");
    let toml_str = fs::read_to_string(&cargo_toml_path)?;
    let doc = toml_str
        .parse::<DocumentMut>()
        .with_context(|| format!("Failed to parse {}", cargo_toml_path.display()))?;

    let mut index_cache: HashMap<String, Option<String>> = HashMap::new();
    for dep in chip_aware_deps(&doc) {
        let dep = dep.to_string();
        if !chip_dep_declares_feature(workspace, project_path, &doc, &dep, chip, &mut index_cache)?
        {
            log::info!(
                "Skipping compile-test {} for {chip}: dependency `{dep}` does not support it at \
                 the version that resolves",
                project_path.display()
            );
            return Ok(false);
        }
    }

    Ok(true)
}

fn chip_dep_declares_feature(
    workspace: &Path,
    project: &Path,
    doc: &DocumentMut,
    dep: &str,
    chip: Chip,
    index_cache: &mut HashMap<String, Option<String>>,
) -> Result<bool> {
    let chip_feature = chip.to_string();

    let Some(req_str) = dependency_requirement(doc, dep) else {
        bail!(
            "{}: `{dep}` is a compile-test chip-dep but has no [dependencies] entry",
            project.display()
        );
    };
    let req = semver::VersionReq::parse(&req_str).with_context(|| {
        format!(
            "{}: invalid version requirement `{req_str}` for {dep}",
            project.display()
        )
    })?;

    let tree = if let Ok(package) = Package::from_str(dep, true)
        && let Ok(toml) = crate::cargo::CargoToml::new(workspace, package)
    {
        Some((
            toml.package_version(),
            manifest_declares_feature(&toml.manifest, &chip_feature),
        ))
    } else {
        None
    };

    let index_body = index_cache
        .entry(dep.to_string())
        .or_insert_with(|| fetch_sparse_index(dep));
    let max_published = match index_body.as_deref() {
        Some(body) => max_version_matching(body, &req)?,
        None => None,
    };

    // Trust the tree only when cargo will resolve that version (it is newer
    // than, or the only match for, the requirement). A published tree version
    // older than the index max is crates.io's crate, whose features can lag.
    if let Some((tree_version, has_feature)) = &tree
        && tree_is_resolving_version(tree_version, &req, max_published.as_ref())
    {
        if *has_feature {
            return Ok(true);
        }
        bail!(
            "{}: crate `{dep}` {tree_version} (working tree) does not declare the \
             `{chip_feature}` feature required for {chip}. A chip was added to the metadata \
             without a matching feature in {dep}.",
            project.display()
        );
    }

    // Frozen line: read the published index. A missing feature means this line
    // predates the chip, so the project does not cover it - skip, do not error.
    let Some(index_body) = index_body.as_deref() else {
        log::warn!(
            "Cannot verify `{dep}` ({chip}) against the crates.io index (offline?); attempting the build"
        );
        return Ok(true);
    };

    let Some(resolved) = max_published else {
        log::warn!("No published `{dep}` satisfies `{req_str}`; attempting the build");
        return Ok(true);
    };

    let features = features_for_version(index_body, &resolved)?;
    Ok(features.contains(&chip_feature))
}

/// Whether the working-tree crate is the version cargo will resolve.
///
/// A published tree version that is older than the index max must not win:
/// cargo picks the published crate, whose feature table can lag the tree.
fn tree_is_resolving_version(
    tree_version: &semver::Version,
    req: &semver::VersionReq,
    max_published: Option<&semver::Version>,
) -> bool {
    req.matches(tree_version) && max_published.is_none_or(|published| tree_version > published)
}

/// The version requirement string for `dep` in a manifest's `[dependencies]`,
/// whether written as `dep = "x"`, `dep = { version = "x" }`, or a table.
fn dependency_requirement(doc: &DocumentMut, dep: &str) -> Option<String> {
    let item = doc.get("dependencies")?.get(dep)?;
    item.as_str()
        .or_else(|| item.get("version").and_then(|v| v.as_str()))
        .map(String::from)
}

fn manifest_declares_feature(manifest: &DocumentMut, feature: &str) -> bool {
    manifest
        .get("features")
        .and_then(|f| f.as_table())
        .is_some_and(|table| table.contains_key(feature))
}

/// Load every example or test the given package owns.
///
/// Packages keep their firmware in one of three shapes: a directory of standalone projects, a
/// `src/bin` directory, or an `examples` directory.
pub fn load_package(workspace: &Path, package: Package) -> Result<Vec<Metadata>> {
    let root = windows_safe_path(&workspace.join(package.directory()));
    if package.contains_standalone_projects() {
        return load_cargo_toml(&root);
    }

    let bins = match package {
        Package::QaTest | Package::HilTest | Package::HilTestRadio => root.join("src").join("bin"),
        _ => root.join("examples"),
    };

    let mut firmware = load(&bins)?;
    // hil-test-radio keeps the tests and their harness firmware in subdirectories.
    for nested in ["tests", "support"] {
        let dir = bins.join(nested);
        if dir.exists() {
            firmware.extend(load(&dir)?);
        }
    }

    Ok(firmware)
}

/// Find the metadata entry for an artifact/test name.
pub fn find_test_by_name<'a>(tests: &'a [Metadata], name: &str) -> Option<&'a Metadata> {
    tests.iter().find(|test| {
        test.binary_name() == name
            || test.output_file_name() == name
            || test.name_with_configuration() == name
    })
}

fn parse_env_assignment(value: &str) -> Result<(String, String)> {
    let (env_var, value) = value
        .split_once('=')
        .with_context(|| "CONFIG metadata must be in the form 'CONFIG=VALUE'")?;
    Ok((env_var.trim().to_string(), value.trim().to_string()))
}

fn parse_feature_list(value: &str) -> Vec<String> {
    let mut values = value
        .split_ascii_whitespace()
        .map(ToString::to_string)
        .collect::<Vec<_>>();
    values.sort();
    values
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

#[cfg(test)]
mod tests {
    use super::*;

    fn chips(expr: &str) -> Vec<Chip> {
        parse_chips(expr).expect("expression should evaluate")
    }

    #[test]
    fn wifi_predicate_selects_wifi_chips() {
        let selected = chips("wifi_driver_supported");
        // The wifi compile-test must cover both incident chips: esp32s31 (#6334,
        // beta.1 shipped against emg lacking its Wi-Fi metadata) and esp32c5
        // (Wi-Fi 6 metadata not exercised). esp32h2 and esp32p4 have no Wi-Fi.
        assert!(selected.contains(&Chip::Esp32s31));
        assert!(selected.contains(&Chip::Esp32c5));
        assert!(!selected.contains(&Chip::Esp32h2));
        assert!(!selected.contains(&Chip::Esp32p4));
    }

    #[test]
    fn bt_predicate_selects_bt_chips() {
        let selected = chips("bt_driver_supported");
        // esp32h2 has a Bluetooth driver but no Wi-Fi one, and esp32s31 gained a
        // BLE driver (#6332). Radio-less chips (esp32s2, esp32p4) are excluded.
        assert!(selected.contains(&Chip::Esp32h2));
        assert!(selected.contains(&Chip::Esp32s31));
        assert!(!selected.contains(&Chip::Esp32s2));
        assert!(!selected.contains(&Chip::Esp32p4));
    }

    #[test]
    fn hal_predicate_selects_every_chip() {
        // `true` is the `hal` project's predicate: it must cover esp32p4, which
        // nothing else selects, so no chip is left without a compile-test.
        let selected = chips("true");
        assert!(selected.contains(&Chip::Esp32p4));
        assert_eq!(selected.len(), Chip::iter().count());
    }

    #[test]
    fn false_predicate_selects_no_chip() {
        // A predicate no chip satisfies yields an empty set; the build path turns
        // this into a hard zero-coverage error.
        assert!(chips("false").is_empty());
    }

    #[test]
    fn chip_aware_deps_are_the_esp_crates_with_chip_features() {
        // esp-hal and esp-alloc both declare per-chip features and are forwarded;
        // embassy-executor is third-party (not a workspace crate) and is left out.
        let doc = r#"
            [dependencies]
            esp-hal = "1.1.0"
            esp-alloc = "0.10.0"
            embassy-executor = "0.10.0"
        "#
        .parse::<DocumentMut>()
        .unwrap();

        let deps = chip_aware_deps(&doc);
        assert!(deps.contains(&Package::EspHal));
        assert!(deps.contains(&Package::EspAlloc));
        assert_eq!(deps.len(), 2);
    }

    #[test]
    fn feature_table_chips_reads_only_chip_keys() {
        let doc = r#"
            [features]
            default = []
            esp32 = []
            esp32c6 = []
        "#
        .parse::<DocumentMut>()
        .unwrap();

        let chips = feature_table_chips(&doc);
        assert!(chips.contains(&Chip::Esp32));
        assert!(chips.contains(&Chip::Esp32c6));
        assert_eq!(chips.len(), 2);
    }

    #[test]
    fn features_for_version_reports_declared_feature_names() {
        // Two versions of the same crate; only 0.18.0 declares `esp32c6`.
        let body = concat!(
            r#"{"name":"esp-radio","vers":"0.17.0","deps":[],"features":{"wifi":[],"esp32":[]},"cksum":"a","yanked":false}"#,
            "\n",
            r#"{"name":"esp-radio","vers":"0.18.0","deps":[],"features":{"wifi":[]},"features2":{"esp32c6":[]},"cksum":"b","yanked":false}"#,
            "\n",
        );

        let older = features_for_version(body, &semver::Version::parse("0.17.0").unwrap()).unwrap();
        assert!(!older.contains("esp32c6"));

        // The resolved (max satisfying) version and its merged feature set.
        let req = semver::VersionReq::parse("^0.18.0").unwrap();
        let resolved = max_version_matching(body, &req).unwrap().unwrap();
        assert_eq!(resolved, semver::Version::parse("0.18.0").unwrap());
        let features = features_for_version(body, &resolved).unwrap();
        assert!(features.contains("esp32c6"));
        assert!(features.contains("wifi"));
    }

    #[test]
    fn features_for_version_flags_missing_chip_feature() {
        // The mechanism behind #6334 (beta.1) and new-chip coverage: the
        // published version that resolves lacks the required chip feature, so the
        // feature check can skip an old dependency line for that chip instead of
        // failing the build.
        let body = concat!(
            r#"{"name":"dep","vers":"1.0.0","deps":[],"features":{"esp32":[]},"cksum":"a","yanked":false}"#,
            "\n",
            r#"{"name":"dep","vers":"1.1.0","deps":[],"features":{"esp32":[]},"cksum":"b","yanked":false}"#,
            "\n",
        );
        let req = semver::VersionReq::parse("^1.0").unwrap();
        let resolved = max_version_matching(body, &req).unwrap().unwrap();
        assert_eq!(resolved, semver::Version::parse("1.1.0").unwrap());
        let features = features_for_version(body, &resolved).unwrap();
        assert!(!features.contains("esp32s31"));
    }

    #[test]
    fn real_compile_tests_cover_every_chip() {
        // Exercises the real projects end to end: derives each one's chip set
        // from its `//% CHIP_FILTER` against the live metadata, and asserts every
        // chip is covered so no release chip is silently untested.
        let workspace = crate::repo_root_for_tests();
        let coverage = compile_test_coverage(&workspace).expect("coverage should compute");

        for chip in Chip::iter() {
            assert!(
                coverage.iter().any(|(_, chips)| chips.contains(&chip)),
                "chip {chip} is not covered by any compile-test project"
            );
        }

        // `hal` is the catch-all, and esp32p4 relies on it exclusively.
        let hal = coverage
            .iter()
            .find(|(name, _)| name == "hal")
            .expect("hal project present");
        assert_eq!(hal.1.len(), Chip::iter().count());

        let p4_projects = coverage
            .iter()
            .filter(|(_, chips)| chips.contains(&Chip::Esp32p4))
            .map(|(name, _)| name.as_str())
            .collect::<Vec<_>>();
        assert_eq!(p4_projects, ["hal"]);
    }

    #[test]
    fn tree_is_resolving_when_newer_than_published() {
        let req = semver::VersionReq::parse("1.2").unwrap();
        let tree = semver::Version::parse("1.3.0").unwrap();
        let published = semver::Version::parse("1.2.2").unwrap();
        assert!(tree_is_resolving_version(&tree, &req, Some(&published)));
    }

    #[test]
    fn tree_is_not_resolving_when_published_is_ahead() {
        // Tree 1.2.0 with unreleased chip features; cargo resolves crates.io 1.2.2.
        let req = semver::VersionReq::parse("1.2").unwrap();
        let tree = semver::Version::parse("1.2.0").unwrap();
        let published = semver::Version::parse("1.2.2").unwrap();
        assert!(!tree_is_resolving_version(&tree, &req, Some(&published)));
    }

    #[test]
    fn tree_is_not_resolving_when_equal_to_published() {
        let req = semver::VersionReq::parse("1.2").unwrap();
        let tree = semver::Version::parse("1.2.2").unwrap();
        let published = semver::Version::parse("1.2.2").unwrap();
        assert!(!tree_is_resolving_version(&tree, &req, Some(&published)));
    }

    #[test]
    fn tree_is_resolving_when_nothing_is_published() {
        let req = semver::VersionReq::parse("1.3").unwrap();
        let tree = semver::Version::parse("1.3.0").unwrap();
        assert!(tree_is_resolving_version(&tree, &req, None));
    }

    #[test]
    fn frozen_line_tree_does_not_match_req() {
        let req = semver::VersionReq::parse("~1.1.0").unwrap();
        let tree = semver::Version::parse("1.2.0").unwrap();
        let published = semver::Version::parse("1.1.1").unwrap();
        assert!(!tree_is_resolving_version(&tree, &req, Some(&published)));
    }

    #[test]
    fn sparse_index_url_follows_registry_prefix_layout() {
        assert_eq!(sparse_index_url("a"), "https://index.crates.io/1/a");
        assert_eq!(sparse_index_url("bc"), "https://index.crates.io/2/bc");
        assert_eq!(sparse_index_url("abc"), "https://index.crates.io/3/a/abc");
        assert_eq!(
            sparse_index_url("esp-hal"),
            "https://index.crates.io/es/p-/esp-hal"
        );
    }
}
