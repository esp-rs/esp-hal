use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, anyhow, bail};
use cargo::CargoAction;
use parking_lot::{MappedMutexGuard, Mutex, MutexGuard};
use pretty_yaml::{config::FormatOptions, format_text};
use serde::{Deserialize, Serialize};
use toml_edit::{InlineTable, Item, Value};
use walkdir::WalkDir;

use crate::{
    cargo::{CargoArgsBuilder, CargoCommandBatcher, CargoToml},
    firmware::Metadata,
    metadata::{Chip, Config},
};

pub mod cargo;
pub mod changelog;
pub mod commands;
pub mod detect;
pub mod documentation;
pub mod firmware;
pub mod git;
pub mod metadata;
pub mod pr_changelog;
pub mod resolve;

/// GitHub repository used for all `gh` CLI calls.
pub const UPSTREAM_REPO: &str = "esp-rs/esp-hal";
pub mod radio_hil_runner;

// ---------------------------------------------------------------------------
// MCP tool registration

/// Registration record for a single MCP tool.
///
/// Populated by the `#[mcp_tool]` attribute macro via `inventory::submit!`.
#[cfg(feature = "mcp")]
pub struct McpToolRegistration {
    /// Tool name exposed over MCP (snake_case, matches the subcommand).
    pub name: &'static str,
    /// Human-readable description shown in `tools/list`.
    pub description: &'static str,
    /// Returns the JSON Schema for the tool's input as a `serde_json::Value`.
    pub input_schema_fn: fn() -> serde_json::Value,
    /// Deserialises `json` into the tool's input type and executes the tool as
    /// a subprocess, returning captured stdout/stderr.
    pub execute_fn: fn(serde_json::Value) -> anyhow::Result<String>,
}

#[cfg(feature = "mcp")]
inventory::collect!(McpToolRegistration);

#[cfg(feature = "semver-checks")]
pub mod semver_check;

/// One `check-configs` or `clippy-configs` case from `[package.metadata.espressif]`.
#[derive(Debug, Clone, Default)]
pub struct CheckConfig {
    pub features: Vec<String>,
    pub env: HashMap<String, String>,
}

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
/// Represents the packages in the `esp-hal` workspace.
pub enum Package {
    EspAlloc,
    EspBacktrace,
    EspBootloaderEspIdf,
    EspConfig,
    EspHal,
    EspHalProcmacros,
    EspRomSys,
    EspLpHal,
    EspMetadata,
    EspMetadataGenerated,
    EspPhy,
    EspPrintln,
    EspRiscvRt,
    EspStorage,
    EspSync,
    EspRadio,
    EspRadioRtosDriver,
    EspRtos,
    Examples,
    HilTest,
    HilTestRadio,
    QaTest,
    XtensaLx,
    XtensaLxRt,
    XtensaLxRtProcMacros,

    CompileTests,
}

// Returns the root directory of the esp-hal repository.
fn repo_root() -> PathBuf {
    let cwd = std::env::current_dir().unwrap();
    let mut cwd = cwd.as_path();
    loop {
        if cwd.join("xtask").exists() && cwd.join("esp-hal").exists() {
            return cwd.to_path_buf();
        }
        let Some(parent) = cwd.parent() else {
            panic!("Looks like you are not in the esp-hal repository");
        };
        cwd = parent;
    }
}

static TOML: Mutex<Option<HashMap<Package, Option<CargoToml>>>> = Mutex::new(None);

impl Package {
    /// Source directory relative to the workspace root.
    ///
    /// Usually matches the clap/package name. QA tests live under `examples/qa`.
    pub fn directory(&self) -> &str {
        match self {
            Self::QaTest => "examples/qa",
            other => other.as_ref(),
        }
    }

    /// Does the package have chip-specific cargo features?
    pub fn has_chip_features(&self) -> bool {
        use strum::IntoEnumIterator;
        let chips = Chip::iter()
            .map(|chip| chip.to_string())
            .collect::<Vec<_>>();
        let toml = self.toml();
        let Some(ref toml) = *toml else {
            return false;
        };
        let Some(Item::Table(features)) = toml.manifest.get("features") else {
            return false;
        };

        // This is intended to opt-out in case there are features that look like chip names, but
        // aren't supposed to be handled like them.
        if let Some(has_chip_features) = toml.espressif_metadata_bool("has_chip_features") {
            return has_chip_features;
        }

        features
            .iter()
            .any(|(feature, _)| chips.iter().any(|c| c == feature))
    }

    /// Should doc tests be skipped for this package?
    ///
    /// Controlled by the `skip-doctests` key in the package's
    /// `[package.metadata.espressif]` table. Defaults to `false`.
    ///
    /// If a package does not define `doc-config`, doc tests are skipped by default.
    // FIXME: doc tests should slowly be enabled (by removing the `skip-doctests` key) as the docs
    // are fixed up, and eventually this opt-out should be removed entirely.
    pub fn skip_doctests(&self) -> bool {
        let toml = self.toml();
        let Some(ref toml) = *toml else {
            // No Cargo.toml in the package; it is a directory of standalone projects.
            return true;
        };
        toml.espressif_metadata_bool("skip-doctests")
            .unwrap_or(false)
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

    /// Is the package compatible with the given chip?
    pub fn supports_chip(&self, chip: Chip) -> bool {
        if !self.has_chip_features() {
            // Chip-independent package
            return true;
        }

        let toml = self.toml();
        let Some(ref toml) = *toml else {
            return false;
        };
        let Some(Item::Table(features)) = toml.manifest.get("features") else {
            unreachable!("has_chip_features() already checked for a features table");
        };

        let chip_name = chip.to_string();
        features.iter().any(|(feature, _)| feature == chip_name)
    }

    /// Does the package have a migration guide?
    pub fn has_migration_guide(&self, workspace: &Path) -> bool {
        let package_path = workspace.join(self.to_string());

        // Check if the package directory exists
        let Ok(entries) = std::fs::read_dir(&package_path) else {
            return false;
        };

        // Look for files matching the pattern "MIGRATING-*.md"
        for entry in entries.flatten() {
            if let Some(file_name) = entry.file_name().to_str()
                && file_name.starts_with("MIGRATING-")
                && file_name.ends_with(".md")
            {
                return true;
            }
        }

        false
    }

    /// Does the package have any host tests?
    ///
    /// Returns true when `src/**/*.rs` contains `#[test]`. Packages that return
    /// true must also have a match arm in [`run_host_tests`]; see `xtask/README.md`
    /// ("Host tests").
    pub fn has_host_tests(&self, workspace: &Path) -> bool {
        if *self == Package::HilTest || *self == Package::HilTestRadio {
            return false;
        }
        let package_path = workspace.join(self.to_string()).join("src");

        walkdir::WalkDir::new(package_path)
            .into_iter()
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|ext| ext == "rs"))
            .any(|entry| {
                std::fs::read_to_string(entry.path()).is_ok_and(|src| src.contains("#[test]"))
            })
    }

    /// Does the package need to be built with the standard library?
    pub fn needs_build_std(&self) -> bool {
        self.toml()
            .as_ref()
            .and_then(|toml| toml.espressif_metadata_bool("needs-build-std"))
            .unwrap_or(true)
    }

    /// Do the package's chip-specific cargo features affect the public API?
    pub fn chip_features_matter(&self) -> bool {
        self.toml()
            .as_ref()
            .and_then(|toml| toml.espressif_metadata_bool("chip-features-matter"))
            .unwrap_or(false)
    }

    /// Is this "package" a directory of standalone projects rather than a single crate?
    pub fn contains_standalone_projects(&self) -> bool {
        matches!(self, Package::Examples | Package::CompileTests)
    }

    /// Should documentation be built for the package, and should the package be
    /// published?
    pub fn is_published(&self) -> bool {
        let toml = self.toml();
        if let Some(ref toml) = *toml {
            toml.is_published()
        } else {
            false
        }
    }

    /// Build on host
    pub fn build_on_host(&self, features: &[String]) -> bool {
        match self {
            Self::EspConfig | Self::EspMetadata => true,
            Self::EspMetadataGenerated if features.iter().any(|f| f == "build-script") => true,
            _ => false,
        }
    }

    fn parse_required_features(table: &InlineTable) -> Vec<String> {
        let mut features = Vec::new();
        if let Some(config_features) = table.get("features") {
            let Value::Array(config_features) = config_features else {
                panic!("features must be an array.");
            };

            for feature in config_features {
                let feature = feature.as_str().expect("features must be strings.");
                features.push(feature.to_owned());
            }
        }

        features
    }

    fn parse_env_table(table: &InlineTable) -> HashMap<String, String> {
        let mut env = HashMap::new();
        if let Some(env_value) = table.get("env") {
            let Value::InlineTable(env_table) = env_value else {
                panic!("env must be an inline table.");
            };

            for (key, value) in env_table.iter() {
                let value = value.as_str().expect("env values must be strings.");
                env.insert(key.to_string(), value.to_owned());
            }
        }

        env
    }

    fn parse_conditional_append(
        table: &InlineTable,
        config: &Config,
    ) -> Option<(Option<Vec<String>>, Option<HashMap<String, String>>)> {
        let script_ctx = ScriptContext::new();
        let mut ctx = script_ctx.for_config(config);

        if let Some(condition) = table.get("if") {
            let Some(expr) = condition.as_str() else {
                panic!("`if` condition must be a string.");
            };

            if !ctx.evaluate(expr).expect("Failed to evaluate expression") {
                return None;
            }
        }

        let features = if table.contains_key("features") {
            Some(Self::parse_required_features(table))
        } else {
            None
        };

        let env = if table.contains_key("env") {
            Some(Self::parse_env_table(table))
        } else {
            None
        };

        if features.is_none() && env.is_none() {
            panic!("append items must specify at least one of `features` or `env`.");
        }

        Some((features, env))
    }

    fn parse_config_set(table: &InlineTable, config: &Config) -> Option<CheckConfig> {
        let script_ctx = ScriptContext::new();
        let mut ctx = script_ctx.for_config(config);

        if let Some(condition) = table.get("if") {
            let Some(expr) = condition.as_str() else {
                panic!("`if` condition must be a string.");
            };

            if !ctx.evaluate(expr).expect("Failed to evaluate expression") {
                return None;
            }
        }

        let mut features = Self::parse_required_features(table);
        let mut env = Self::parse_env_table(table);

        if let Some(conditionals) = table.get("append") {
            let Value::Array(conditionals) = conditionals else {
                panic!("append must be an array.");
            };
            for cond in conditionals {
                let Value::InlineTable(cond_table) = cond else {
                    panic!("append items must be inline tables.");
                };
                if let Some((append_features, append_env)) =
                    Self::parse_conditional_append(cond_table, config)
                {
                    if let Some(append_features) = append_features {
                        features.extend(append_features);
                    }
                    if let Some(append_env) = append_env {
                        env.extend(append_env);
                    }
                }
            }
        }

        Some(CheckConfig { features, env })
    }

    fn single_config_rule_from_metadata(
        &self,
        config: &Config,
        metadata_key: &str,
    ) -> Option<CheckConfig> {
        let toml = self.toml();
        let toml = toml.as_ref()?;

        if let Some(metadata) = toml.espressif_metadata()
            && let Some(config_meta) = metadata.get(metadata_key)
        {
            let Item::Value(Value::InlineTable(table)) = config_meta else {
                panic!("'{}' must be an inline table.", metadata_key);
            };

            return Self::parse_config_set(table, config);
        }

        None
    }

    /// Documentation build configuration for this package.
    ///
    /// If the `doc-config` table is not found, this function returns `None`. This differs from
    /// specifying an empty set of features, which returns `Some` with empty `features` and `env`.
    // TODO: perhaps we should use the docs.rs metadata for doc features for packages that have no
    // chip-specific features.
    pub fn doc_config_rules(&self, config: &Config) -> Option<CheckConfig> {
        if *self == Self::Examples {
            return None;
        }

        let config = self.single_config_rule_from_metadata(config, "doc-config");

        log::debug!("Doc config for package '{}': {:?}", self, config);
        config
    }

    /// Configuration for semver checking of this package.
    #[cfg(feature = "semver-checks")]
    pub fn semver_config_rules(&self, config: &Config) -> CheckConfig {
        let config = self
            .single_config_rule_from_metadata(config, "semver-config")
            .unwrap_or_default();

        log::debug!("Semver config for package '{}': {:?}", self, config);
        config
    }

    fn config_rules_from_metadata(
        &self,
        config: &Config,
        metadata_key: &str,
    ) -> Option<Vec<CheckConfig>> {
        let toml = self.toml();
        let toml = toml.as_ref()?;
        let mut cases = Vec::new();

        if let Some(metadata) = toml.espressif_metadata()
            && let Some(config_meta) = metadata.get(metadata_key)
        {
            let Item::Value(Value::Array(tables)) = config_meta else {
                panic!(
                    "'{}' must be an array of tables. {:?}",
                    metadata_key, config_meta
                );
            };

            for table in tables {
                let Value::InlineTable(table) = table else {
                    panic!("'{}' items must be inline tables.", metadata_key);
                };
                if let Some(case) = Self::parse_config_set(table, config) {
                    cases.push(case);
                }
            }
        }

        if cases.is_empty() { None } else { Some(cases) }
    }

    /// Check configurations to compile for this package in CI.
    pub fn check_config_rules(&self, config: &Config) -> Vec<CheckConfig> {
        let mut cases = self
            .config_rules_from_metadata(config, "check-configs")
            .unwrap_or_default();

        if cases.is_empty() {
            cases.push(CheckConfig::default());
        }

        log::debug!("Check configs for package '{}': {:?}", self, cases);
        cases
    }

    /// Clippy configurations to run for this package in CI.
    pub fn lint_config_rules(&self, config: &Config) -> Vec<CheckConfig> {
        let cases = self
            .config_rules_from_metadata(config, "clippy-configs")
            .unwrap_or_default();

        log::debug!("Clippy configs for package '{}': {:?}", self, cases);
        cases
    }

    fn toml(&self) -> MappedMutexGuard<'_, Option<CargoToml>> {
        let tomls = TOML.lock();

        MutexGuard::map(tomls, |tomls| {
            let tomls = tomls.get_or_insert_default();

            tomls
                .entry(*self)
                .or_insert_with(|| CargoToml::new(&repo_root(), *self).ok())
        })
    }

    #[cfg(feature = "rel-check")]
    fn remove_toml_from_cache(&self) {
        let mut tomls = TOML.lock();
        if let Some(ref mut tomls) = *tomls {
            tomls.remove(self);
        }
    }

    fn targets_lp_core(&self) -> bool {
        if self.contains_standalone_projects() {
            return false;
        }

        let toml = self.toml();
        let Some(ref toml) = *toml else {
            return false;
        };
        toml.espressif_metadata_bool("targets_lp_core")
            .unwrap_or(false)
    }

    /// Return the target triple for a given package/chip pair.
    pub fn target_triple(&self, chip: &Chip) -> Result<String> {
        if self.targets_lp_core() {
            chip.lp_target().map(ToString::to_string)
        } else {
            Ok(chip.target())
        }
    }

    /// Validate that the specified chip is valid for the specified package.
    pub fn validate_package_chip(&self, chip: &Chip) -> Result<()> {
        if self.targets_lp_core() && !chip.has_lp_core() {
            return Err(anyhow!(
                "Package '{self}' requires an LP core, but '{chip}' does not have one",
            ));
        }

        if !self.supports_chip(*chip) {
            return Err(anyhow!(
                "Package '{self}' does not have a chip feature for {chip}"
            ));
        }

        let toml = self.toml();
        let Some(ref toml) = *toml else {
            return Ok(());
        };

        if let Some(metadata) = toml.espressif_metadata()
            && let Some(Item::Value(Value::Array(targets))) = metadata.get("requires_target")
            && !targets.iter().any(|t| t.as_str() == Some(&chip.target()))
        {
            return Err(anyhow!(
                "Package '{self}' is not compatible with {chip_target} chips",
                chip_target = chip.target()
            ));
        }

        Ok(())
    }

    /// Creates a tag string for this [`Package`] combined with a semantic version.
    pub fn tag(&self, version: &semver::Version) -> String {
        log::debug!(
            "Creating tag for package '{}' with version '{}'",
            self,
            version
        );
        format!("{self}-v{version}")
    }

    #[cfg(feature = "release")]
    fn is_semver_checked(&self) -> bool {
        let toml = self.toml();
        let Some(ref toml) = *toml else {
            // No Cargo.toml in the package, must be the examples
            return false;
        };
        toml.espressif_metadata_bool("semver-checked")
            .unwrap_or(false)
    }

    #[cfg(feature = "semver-checks")]
    pub(crate) fn clean_semver_check(&self, dest_path: &Path) -> anyhow::Result<()> {
        if self == &Package::EspRomSys && dest_path.exists() {
            fs::remove_file(dest_path)
                .context("Failed to remove existing generated_rom_symbols.rs")?;

            // Create new file with placeholder content
            fs::write(dest_path, "// Do not delete - placeholder for fmt!\n")
                .context("Failed to create new generated_rom_symbols.rs placeholder")?;
        }

        Ok(())
    }

    #[cfg(feature = "semver-checks")]
    pub(crate) fn prepare_semver_check(
        &self,
        package_path: &Path,
        chip: &Chip,
    ) -> anyhow::Result<()> {
        if self == &Package::EspRomSys {
            log::info!("Generating ROM symbol markers for chip: {}", chip);
            crate::commands::generate_rom_symbols::generate_rom_symbols(&package_path, chip)?;
        }
        Ok(())
    }
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, strum::Display, clap::ValueEnum, Serialize, Deserialize,
)]
#[strum(serialize_all = "lowercase")]
/// Represents the versioning scheme for a package.
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
    debug: bool,
    toolchain: Option<&str>,
    timings: bool,
    extra_args: &[&str],
) -> Result<()> {
    let package = app.example_path().strip_prefix(package_path)?;
    log::info!("Building example '{}' for '{}'", package.display(), chip);

    let builder = generate_build_command(
        package_path,
        chip,
        target,
        app,
        action,
        debug,
        toolchain,
        timings,
        extra_args,
    )?;

    let command = CargoCommandBatcher::build_one_for_cargo(&builder);

    command.run(false)?;

    Ok(())
}

pub fn generate_build_command(
    package_path: &Path,
    chip: Chip,
    target: &str,
    app: &Metadata,
    action: CargoAction,
    debug: bool,
    toolchain: Option<&str>,
    timings: bool,
    extra_args: &[&str],
) -> Result<CargoArgsBuilder> {
    let package = app.example_path().strip_prefix(package_path)?;
    log::info!(
        "Building command: {} '{}' for '{}'",
        if matches!(action, CargoAction::Build(_)) {
            "Build"
        } else {
            "Run"
        },
        package.display(),
        chip
    );

    let mut features = app.feature_set().to_vec();
    if !features.is_empty() {
        log::info!("  Features:      {}", features.join(", "));
    }
    features.push(chip.to_string());

    let cwd = if package_path.ends_with("examples") || package_path.ends_with("compile-tests") {
        package_path.join(package).to_path_buf()
    } else {
        package_path.to_path_buf()
    };

    let mut builder = CargoArgsBuilder::new(app.output_file_name())
        .manifest_path(cwd.join("Cargo.toml"))
        .config_path(cwd.join(".cargo").join("config.toml"))
        .target(target)
        .features(&features);

    let subcommand = if matches!(action, CargoAction::Build(_)) {
        "build"
    } else {
        "run"
    };
    builder = builder.subcommand(subcommand);

    let bin_arg = if package.starts_with("src/bin") {
        Some(format!("--bin={}", app.binary_name()))
    } else if !package_path.ends_with("examples") && !package_path.ends_with("compile-tests") {
        Some(format!("--example={}", app.binary_name()))
    } else {
        None
    };

    if let Some(arg) = bin_arg {
        builder.add_arg(arg);
    }

    if !app.configuration().is_empty() {
        log::info!("  Configuration: {}", app.configuration());
    }

    for config in app.cargo_config() {
        log::info!(" Cargo --config: {config}");
        builder.add_config("--config").add_config(config);
        // Some configuration requires nightly rust, so let's just assume it. May be
        // overwritten by the esp toolchain on xtensa.
        builder = builder.toolchain("nightly");
    }

    let env_vars = app.env_vars();
    for (key, value) in env_vars {
        log::info!("  esp-config:    {} = {}", key, value);
        builder.add_env_var(key, value);
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
        builder = builder.toolchain(toolchain);
    }

    if let CargoAction::Build(Some(out_dir)) = action {
        // We have to place the binary into a directory named after the app, because
        // we can't set the binary name.
        builder.add_arg("--artifact-dir");
        builder.add_arg(
            out_dir
                .join("tmp") // This will be deleted in one go
                .join(app.output_file_name()) // This sets the name of the binary
                .display()
                .to_string(),
        );
    }

    let builder = builder.args(extra_args);

    Ok(builder)
}

// ----------------------------------------------------------------------------
// Helper Functions

/// Copy an entire directory recursively.
// https://stackoverflow.com/a/65192210
pub fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> Result<()> {
    log::debug!(
        "Copying directory '{}' to '{}'",
        src.as_ref().display(),
        dst.as_ref().display()
    );
    fs::create_dir_all(&dst).with_context(|| "Failed to create a {dst}")?;

    for entry in fs::read_dir(src).with_context(|| "Failed to read {src}")? {
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
    for entry in fs::read_dir(workspace).context("Failed to read {workspace}")? {
        let entry = entry?;
        if entry.file_type()?.is_dir() && entry.path().join("Cargo.toml").exists() {
            paths.push(entry.path());
        }
    }

    paths.sort();

    log::debug!(
        "Found {} packages in workspace '{}':",
        paths.len(),
        workspace.display()
    );

    Ok(paths)
}

/// Parse the version from the specified package's Cargo manifest.
pub fn package_version(_workspace: &Path, package: Package) -> Result<semver::Version> {
    let toml = package.toml();
    let Some(ref toml) = *toml else {
        return Err(anyhow!("Failed to parse Cargo.toml for package {package}"));
    };
    Ok(toml.package_version())
}

/// Make the path "Windows"-safe
pub fn windows_safe_path(path: &Path) -> PathBuf {
    PathBuf::from(path.to_str().unwrap().to_string().replace("\\\\?\\", ""))
}

/// Format the specified package in the workspace using `cargo fmt`.
pub fn format_package(
    workspace: &Path,
    package: Package,
    check: bool,
    format_rules: Option<&Path>,
) -> Result<()> {
    log::info!("Formatting package: {}", package);
    let package_path = workspace.join(package.directory());

    let paths = if package.contains_standalone_projects() {
        crate::find_packages(&package_path)?
    } else {
        vec![package_path]
    };

    for path in &paths {
        format_package_path(workspace, path, check, format_rules)?;
        format_yml(check, path)?;
    }

    Ok(())
}

/// Run the host tests for the specified package.
///
/// Called by `cargo xtask host-tests` for every package where [`Package::has_host_tests`]
/// is true. **When adding `#[test]` functions to a package, add a match arm here**
/// with the correct `cargo test` flags/features. See `xtask/README.md` ("Host tests").
pub fn run_host_tests(workspace: &Path, package: Package) -> Result<()> {
    log::info!("Running host tests for package: {}", package);
    let package_path = workspace.join(package.directory());

    let cmd = CargoArgsBuilder::default();

    match package {
        Package::EspConfig => cargo::run(
            &cmd.clone()
                .subcommand("test")
                .features(&["build".into(), "tui".into()])
                .build(),
            &package_path,
        ),

        Package::EspBootloaderEspIdf => cargo::run(
            &cmd.clone()
                .subcommand("test")
                .arg("--lib")
                .arg("--tests")
                .features(&["std".into(), "embedded-storage".into()])
                .arg("--")
                .arg("--test-threads=1")
                .build(),
            &package_path,
        ),

        Package::EspStorage => {
            cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&["emulation".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )?;

            cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&["emulation".into(), "bytewise-read".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )?;

            log::info!("Running miri host tests for package: {}", package);

            cargo::run(
                &cmd.clone()
                    .toolchain("nightly")
                    .subcommand("miri")
                    .subcommand("test")
                    .features(&["emulation".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )?;

            cargo::run(
                &cmd.clone()
                    .toolchain("nightly")
                    .subcommand("miri")
                    .subcommand("test")
                    .features(&["emulation".into(), "bytewise-read".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )
        }
        Package::EspHalProcmacros => cargo::run(
            &cmd.clone()
                .subcommand("test")
                .features(&[
                    "has-lp-core".into(),
                    "is-lp-core".into(),
                    "rtc-slow".into(),
                    "rtc-fast".into(),
                ])
                .build(),
            &package_path,
        ),
        Package::EspMetadata => cargo::run(&cmd.clone().subcommand("test").build(), &package_path),
        _ => Err(anyhow!(
            "Instructions for host testing were not provided for: '{}'",
            package,
        )),
    }
}

/// How many times an operation that writes source files is attempted before
/// giving up.
const WRITE_ATTEMPTS: usize = 5;

/// Runs `operation`, retrying up to [`WRITE_ATTEMPTS`] times.
///
/// Writing files can transiently fail if another process (an editor, a virus
/// scanner, ...) holds a lock on them, so give the lock a chance to disappear
/// before propagating the error.
fn retry_on_failure<T>(what: &str, mut operation: impl FnMut() -> Result<T>) -> Result<T> {
    for attempt in 1..=WRITE_ATTEMPTS {
        match operation() {
            Ok(value) => return Ok(value),
            Err(error) if attempt < WRITE_ATTEMPTS => {
                log::warn!("{what} failed (attempt {attempt}/{WRITE_ATTEMPTS}), retrying...");
                log::debug!("{error:?}");
                std::thread::sleep(Duration::from_millis(200 * attempt as u64));
            }
            Err(error) => return Err(error),
        }
    }

    unreachable!()
}

/// Write `contents` to `path`, retrying if the file is temporarily locked.
fn write_file(path: &Path, contents: impl AsRef<[u8]>) -> Result<()> {
    let contents = contents.as_ref();
    retry_on_failure(&format!("Writing {}", path.display()), || {
        fs::write(path, contents).with_context(|| format!("Failed to write {}", path.display()))
    })
}

/// Format a package directory in the workspace using `cargo fmt`.
pub fn format_package_path(
    workspace: &Path,
    package_path: &Path,
    check: bool,
    format_rules: Option<&Path>,
) -> Result<()> {
    // We need to list all source files since modules in `unstable_module!` macros
    // won't get picked up otherwise
    let source_files = walkdir::WalkDir::new(package_path.join("src"))
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
        .build();

    if check {
        cargo_args.push("--check".into());
    }

    cargo_args.push("--".into());
    let mut config_file_path;
    let config_file = if let Some(rules) = format_rules {
        rules
    } else {
        config_file_path = package_path.join("rustfmt.toml");
        if !config_file_path.exists() {
            config_file_path = workspace.join("rustfmt.toml");
        }
        &config_file_path
    };
    cargo_args.push(format!("--config-path={}", config_file.display()));
    cargo_args.extend(source_files);

    log::debug!("{cargo_args:#?}");

    if check {
        return cargo::run(&cargo_args, package_path);
    }

    retry_on_failure(&format!("Formatting {}", package_path.display()), || {
        cargo::run(&cargo_args, package_path)
    })
}

/// Recursively format all `.yml` files in the `.github/` directory.
pub fn format_yml<P: AsRef<Path>>(check: bool, path: P) -> Result<()> {
    WalkDir::new(path)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| e.path().extension().is_some_and(|ext| ext == "yml"))
        .try_for_each(|entry| -> Result<()> {
            let path = entry.path();
            let content = fs::read_to_string(path)?;

            let formatted = format_text(&content, &FormatOptions::default())
                .context(format!("Failed to format {:?} yml!", path))?;

            if content.replace("\r\n", "\n") != formatted.replace("\r\n", "\n") {
                if check {
                    anyhow::bail!("File not formatted: {:?}", path);
                }

                log::info!("Fixing format: {:?}", path);
                write_file(path, formatted)?;
            }

            Ok(())
        })?;

    Ok(())
}

/// Recursively find all packages in the given path that contain a `Cargo.toml` file.
pub fn find_packages(path: &Path) -> Result<Vec<PathBuf>> {
    let mut packages = Vec::new();

    for result in
        fs::read_dir(path).with_context(|| format!("Failed to read {}", path.display()))?
    {
        log::debug!("Inspecting path: {}", path.display());
        let entry = result?;
        if entry.path().is_file() {
            continue;
        }

        // Path is a directory:
        if entry.path().join("Cargo.toml").exists() {
            packages.push(entry.path());
        } else {
            packages.extend(find_packages(&entry.path())?);
        }
    }

    log::debug!(
        "Found {} packages in path '{}':",
        packages.len(),
        path.display()
    );

    Ok(packages)
}

struct ScriptContext;

struct ChipFilterEval<'a> {
    config: &'a Config,
}

impl ScriptContext {
    pub fn new() -> Self {
        Self
    }

    pub fn for_chip(&self, chip: Chip) -> ChipFilterEval<'_> {
        self.for_config(Config::for_chip(&chip))
    }

    pub fn for_config<'a>(&self, config: &'a Config) -> ChipFilterEval<'a> {
        ChipFilterEval { config }
    }
}

impl ChipFilterEval<'_> {
    pub fn evaluate<'s>(&'s mut self, expr: &'s str) -> anyhow::Result<bool> {
        let mut ctx = somni_expr::Context::new();

        // All known symbols are initially false
        for sym in Chip::all_symbols() {
            ctx.add_variable(sym, false);
        }
        for sym in Chip::all_kv_symbols() {
            // empty string is not a valid value, chips that don't define the symbol won't match
            ctx.add_variable(sym, "");
        }

        // All defined symbols for this chip are true
        for sym in &self.config.symbols {
            ctx.add_variable(sym, true);
        }
        for (k, v) in &self.config.kv_values {
            ctx.add_variable(k, v.as_str());
        }

        match ctx.evaluate::<bool>(expr) {
            Ok(result) => Ok(result),
            Err(err) => {
                Err(anyhow::anyhow!("{err:?}").context("Failed to evaluate chip expression"))
            }
        }
    }
}

/// The repository root, for tests which need to inspect the real workspace.
#[cfg(test)]
pub(crate) fn repo_root_for_tests() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask must live in a subdirectory of the repository")
        .to_path_buf()
}

/// Spawn `command`, capture stdout/stderr, kill it if it exceeds `timeout`.
pub(crate) fn run_command_with_output_timeout(
    mut command: Command,
    name: &str,
    timeout: Duration,
) -> Result<std::process::Output> {
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| anyhow!("Failed to spawn {name}: {e}"))?;
    let start = Instant::now();

    loop {
        if child.try_wait()?.is_some() {
            return child
                .wait_with_output()
                .map_err(|e| anyhow!("Failed to collect {name} output: {e}"));
        }

        if start.elapsed() > timeout {
            let _ = child.kill();
            let _ = child.wait();
            bail!("{name} timed out after {}s", timeout.as_secs());
        }

        thread::sleep(Duration::from_millis(100));
    }
}

#[cfg(test)]
mod tests {
    use strum::IntoEnumIterator;

    use super::*;

    #[test]
    fn packages_are_either_crates_or_collections_of_standalone_projects() {
        let workspace = repo_root_for_tests();

        for package in Package::iter() {
            let package_path = workspace.join(package.directory());
            assert!(
                package_path.is_dir(),
                "package '{package}' has no directory at {}",
                package_path.display()
            );

            let has_manifest = package_path.join("Cargo.toml").exists();
            assert_eq!(
                has_manifest,
                !package.contains_standalone_projects(),
                "package '{package}' has {} Cargo.toml, but contains_standalone_projects() is {}",
                if has_manifest { "a" } else { "no" },
                package.contains_standalone_projects(),
            );
        }

        assert_eq!(Package::QaTest.to_string(), "qa-test");
        assert_eq!(Package::QaTest.directory(), "examples/qa");
    }

    #[test]
    fn collections_of_standalone_projects_are_not_empty() {
        let workspace = repo_root_for_tests();

        for package in Package::iter().filter(Package::contains_standalone_projects) {
            let projects = find_packages(&workspace.join(package.directory()))
                .unwrap_or_else(|err| panic!("could not enumerate projects of '{package}': {err}"));

            assert!(
                !projects.is_empty(),
                "package '{package}' contains no standalone projects"
            );
        }
    }
}
