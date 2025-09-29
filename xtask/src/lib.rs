use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, anyhow};
use cargo::CargoAction;
use esp_metadata::{Chip, Config, TokenStream};
use parking_lot::{MappedMutexGuard, Mutex, MutexGuard};
use serde::{Deserialize, Serialize};
use toml_edit::{InlineTable, Item, Value};

use crate::{
    cargo::{CargoArgsBuilder, CargoCommandBatcher, CargoToml},
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
    QaTest,
    XtensaLx,
    XtensaLxRt,
    XtensaLxRtProcMacros,
}

impl Package {
    /// Does the package have chip-specific cargo features?
    pub fn has_chip_features(&self) -> bool {
        use strum::IntoEnumIterator;
        let chips = Chip::iter()
            .map(|chip| chip.to_string())
            .collect::<Vec<_>>();
        let toml = self.toml();
        let Some(Item::Table(features)) = toml.manifest.get("features") else {
            return false;
        };

        // This is intended to opt-out in case there are features that look like chip names, but
        // aren't supposed to be handled like them.
        if let Some(metadata) = toml.espressif_metadata() {
            if let Some(Item::Value(ov)) = metadata.get("has_chip_features") {
                let Value::Boolean(ov) = ov else {
                    log::warn!("Invalid value for 'has_chip_features' in metadata");
                    return false;
                };

                return *ov.value();
            }
        }

        features
            .iter()
            .any(|(feature, _)| chips.iter().any(|c| c == feature))
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

    /// Does the package have a migration guide?
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

    /// Does the package have any host tests?
    pub fn has_host_tests(&self, workspace: &Path) -> bool {
        if *self == Package::HilTest {
            return false;
        }
        let package_path = workspace.join(self.to_string()).join("src");

        walkdir::WalkDir::new(package_path)
            .into_iter()
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|ext| ext == "rs"))
            .any(|entry| {
                std::fs::read_to_string(entry.path()).map_or(false, |src| src.contains("#[test]"))
            })
    }

    /// Does the package need to be built with the standard library?
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
                | EspPhy
                | EspRomSys
                | EspBootloaderEspIdf
                | EspMetadataGenerated
                | EspRtos
        )
    }

    /// Should documentation be built for the package, and should the package be
    /// published?
    pub fn is_published(&self) -> bool {
        if *self == Package::Examples {
            // The `examples/` directory does not contain `Cargo.toml` in its root, and even if it
            // did nothing in this directory will be published.
            false
        } else {
            self.toml().is_published()
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

    fn parse_conditional_features(table: &InlineTable, config: &Config) -> Option<Vec<String>> {
        let mut eval_context = somni_expr::Context::new();
        eval_context.add_function("chip_has", |symbol: &str| {
            config.all().iter().any(|sym| sym == symbol)
        });
        eval_context.add_variable("chip", config.name());

        if let Some(condition) = table.get("if") {
            let Some(expr) = condition.as_str() else {
                panic!("`if` condition must be a string.");
            };

            if !eval_context
                .evaluate::<bool>(expr)
                .expect("Failed to evaluate expression")
            {
                return None;
            }
        }

        let Some(config_features) = table.get("features") else {
            panic!("Missing features array.");
        };
        let Value::Array(config_features) = config_features else {
            panic!("features must be an array.");
        };

        let mut features = Vec::new();
        for feature in config_features {
            let feature = feature.as_str().expect("features must be strings.");
            features.push(feature.to_owned());
        }

        Some(features)
    }

    fn parse_feature_set(table: &InlineTable, config: &Config) -> Option<Vec<String>> {
        // Base features. If their condition is not met, the whole item is ignored.
        let mut features = Self::parse_conditional_features(table, config)?;

        if let Some(conditionals) = table.get("append") {
            // Optional features. If their conditions are met, they are appended to the base
            // features.
            let Value::Array(conditionals) = conditionals else {
                panic!("append must be an array.");
            };
            for cond in conditionals {
                let Value::InlineTable(cond_table) = cond else {
                    panic!("append items must be inline tables.");
                };
                if let Some(cond_features) = Self::parse_conditional_features(cond_table, config) {
                    features.extend(cond_features);
                }
            }
        };

        Some(features)
    }

    /// Given a device config, return the features which should be enabled for
    /// this package.
    ///
    /// Features are read from Cargo.toml metadata, from the `doc-config` table. Currently only
    /// one feature set is supported.
    // TODO: perhaps we should use the docs.rs metadata for doc features for packages that have no
    // chip-specific features.
    pub fn doc_feature_rules(&self, config: &Config) -> Vec<String> {
        let mut features = vec![];

        let toml = self.toml();
        if let Some(metadata) = toml.espressif_metadata()
            && let Some(config_meta) = metadata.get("doc-config")
        {
            let Item::Value(Value::InlineTable(table)) = config_meta else {
                panic!("doc-config must be inline tables.");
            };

            if let Some(fs) = Self::parse_feature_set(table, config) {
                features = fs;
            }
        } else {
            // Nothing
        }

        log::debug!("Doc features for package '{}': {:?}", self, features);
        features
    }

    /// Additional feature rules to test subsets of features for a package.
    pub fn check_feature_rules(&self, config: &Config) -> Vec<Vec<String>> {
        let mut cases = vec![];

        let toml = self.toml();
        if let Some(metadata) = toml.espressif_metadata()
            && let Some(config_meta) = metadata.get("check-configs")
        {
            let Item::Value(Value::Array(tables)) = config_meta else {
                panic!(
                    "check-configs must be an array of tables. {:?}",
                    config_meta
                );
            };

            for table in tables {
                let Value::InlineTable(table) = table else {
                    panic!("check-configs items must be inline tables.");
                };
                if let Some(features) = Self::parse_feature_set(table, config) {
                    cases.push(features);
                }
            }
        } else {
            // Nothing specified, just test no features
            cases.push(vec![]);
        }

        log::debug!("Check features for package '{}': {:?}", self, cases);
        cases
    }

    /// Additional feature rules to test subsets of features for a package.
    pub fn lint_feature_rules(&self, config: &Config) -> Vec<Vec<String>> {
        let mut cases = vec![];

        let toml = self.toml();
        if let Some(metadata) = toml.espressif_metadata()
            && let Some(config_meta) = metadata.get("clippy-configs")
        {
            let Item::Value(Value::Array(tables)) = config_meta else {
                panic!(
                    "clippy-configs must be an array of tables. {:?}",
                    config_meta
                );
            };

            for table in tables {
                let Value::InlineTable(table) = table else {
                    panic!("clippy-configs items must be inline tables.");
                };
                if let Some(features) = Self::parse_feature_set(table, config) {
                    cases.push(features);
                }
            }
        }

        log::debug!("Check features for package '{}': {:?}", self, cases);
        cases
    }

    fn toml(&self) -> MappedMutexGuard<'_, CargoToml> {
        static TOML: Mutex<Option<HashMap<Package, CargoToml>>> = Mutex::new(None);

        let tomls = TOML.lock();

        MutexGuard::map(tomls, |tomls| {
            let tomls = tomls.get_or_insert_default();

            tomls.entry(*self).or_insert_with(|| {
                CargoToml::new(&std::env::current_dir().unwrap(), *self)
                    .expect("Failed to parse Cargo.toml")
            })
        })
    }

    fn targets_lp_core(&self) -> bool {
        if *self == Package::Examples {
            return false;
        }

        let toml = self.toml();
        let Some(metadata) = toml.espressif_metadata() else {
            return false;
        };

        let Some(Item::Value(targets_lp_core)) = metadata.get("targets_lp_core") else {
            return false;
        };

        targets_lp_core
            .as_bool()
            .expect("targets_lp_core must be a boolean")
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
        if *self == Package::Examples {
            return Ok(());
        }

        if self.targets_lp_core() && !chip.has_lp_core() {
            return Err(anyhow!(
                "Package '{self}' requires an LP core, but '{chip}' does not have one",
            ));
        }

        let toml = self.toml();

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
        let Some(metadata) = toml.espressif_metadata() else {
            return false;
        };

        let Some(Item::Value(semver_checked)) = metadata.get("semver_checked") else {
            return false;
        };

        semver_checked
            .as_bool()
            .expect("semver_checked must be a boolean")
    }
}

#[derive(Debug, Clone, Copy, strum::Display, clap::ValueEnum, Serialize, Deserialize)]
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

    let cwd = if package_path.ends_with("examples") {
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
    } else if !package_path.ends_with("examples") {
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
    Ok(package.toml().package_version())
}

/// Make the path "Windows"-safe
pub fn windows_safe_path(path: &Path) -> PathBuf {
    PathBuf::from(path.to_str().unwrap().to_string().replace("\\\\?\\", ""))
}

/// Format the specified package in the workspace using `cargo fmt`.
pub fn format_package(workspace: &Path, package: Package, check: bool) -> Result<()> {
    log::info!("Formatting package: {}", package);
    let package_path = workspace.join(package.as_ref());

    let paths = if package == Package::Examples {
        crate::find_packages(&package_path)?
    } else {
        vec![package_path]
    };

    for path in &paths {
        format_package_path(workspace, path, check)?;
    }

    Ok(())
}

/// Run the host tests for the specified package.
pub fn run_host_tests(workspace: &Path, package: Package) -> Result<()> {
    log::info!("Running host tests for package: {}", package);
    let package_path = workspace.join(package.as_ref());

    let cmd = CargoArgsBuilder::default();

    match package {
        Package::EspConfig => {
            return cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&vec!["build".into(), "tui".into()])
                    .build(),
                &package_path,
            );
        }

        Package::EspBootloaderEspIdf => {
            return cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&vec!["std".into()])
                    .build(),
                &package_path,
            );
        }

        Package::EspStorage => {
            cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&vec!["emulation".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )?;

            cargo::run(
                &cmd.clone()
                    .subcommand("test")
                    .features(&vec!["emulation".into(), "bytewise-read".into()])
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
                    .features(&vec!["emulation".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            )?;

            return cargo::run(
                &cmd.clone()
                    .toolchain("nightly")
                    .subcommand("miri")
                    .subcommand("test")
                    .features(&vec!["emulation".into(), "bytewise-read".into()])
                    .arg("--")
                    .arg("--test-threads=1")
                    .build(),
                &package_path,
            );
        }
        _ => Err(anyhow!(
            "Instructions for host testing were not provided for: '{}'",
            package,
        )),
    }
}

fn format_package_path(workspace: &Path, package_path: &Path, check: bool) -> Result<()> {
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

    log::debug!("{cargo_args:#?}");

    cargo::run(&cargo_args, &package_path)
}

/// Update the metadata and chip support table in the esp-hal README.
pub fn update_metadata(workspace: &Path, check: bool) -> Result<()> {
    log::info!("Updating esp-metadata and chip support table...");
    update_chip_support_table(workspace)?;
    generate_metadata(workspace, save)?;

    format_package(workspace, Package::EspMetadataGenerated, false)?;

    if check {
        let res = std::process::Command::new("git")
            .args(["diff", "HEAD", "esp-metadata-generated"])
            .output()
            .context("Failed to run `git diff HEAD esp-metadata-generated`")?;
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
    log::debug!("Updating chip support table in README.md...");
    let mut output = String::new();
    let readme = std::fs::read_to_string(workspace.join("esp-hal").join("README.md"))
        .context("Failed to read {workspace}")?;

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
