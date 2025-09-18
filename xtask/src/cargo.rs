//! Tools for working with Cargo.

use std::{
    collections::HashMap,
    ffi::OsStr,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

use anyhow::{Context as _, Result, bail};
use clap::ValueEnum as _;
use serde::{Deserialize, Serialize};
use toml_edit::{DocumentMut, Formatted, Item, Table, Value};

use crate::{Package, windows_safe_path};

/// Actions that can be performed with Cargo.
#[derive(Clone, Debug, PartialEq)]
pub enum CargoAction {
    Build(Option<PathBuf>),
    Run,
}

/// Information about a built artifact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Artifact {
    pub executable: PathBuf,
}

/// Execute cargo with the given arguments and from the specified directory.
pub fn run(args: &[String], cwd: &Path) -> Result<()> {
    run_with_env::<[(&str, &str); 0], _, _>(args, cwd, [], false).with_context(|| {
        format!("Failed to execute cargo with given arguments {args:?} in cwd {cwd:?}",)
    })?;
    Ok(())
}

/// Execute cargo with the given arguments and from the specified directory.
pub fn run_with_env<I, K, V>(args: &[String], cwd: &Path, envs: I, capture: bool) -> Result<String>
where
    I: IntoIterator<Item = (K, V)> + core::fmt::Debug,
    K: AsRef<OsStr>,
    V: AsRef<OsStr>,
{
    if !cwd.is_dir() {
        bail!("The `cwd` argument MUST be a directory");
    }

    // Make sure to not use a UNC as CWD!
    // That would make `OUT_DIR` a UNC which will trigger things like the one fixed in https://github.com/dtolnay/rustversion/pull/51
    // While it's fixed in `rustversion` it's not fixed for other crates we are
    // using now or in future!
    let cwd = windows_safe_path(cwd);

    log::debug!(
        "Running `cargo {}` in {:?} - Environment {:?}",
        args.join(" "),
        cwd,
        envs
    );

    let mut command = Command::new(get_cargo());

    command
        .args(args)
        .current_dir(cwd)
        .envs(envs)
        .stdout(if capture {
            Stdio::piped()
        } else {
            Stdio::inherit()
        })
        .stderr(if capture {
            Stdio::piped()
        } else {
            Stdio::inherit()
        });

    if args.iter().any(|a| a.starts_with('+')) {
        // Make sure the right cargo runs
        command.env_remove("CARGO");
    }

    let output = command
        .stdin(Stdio::inherit())
        .output()
        .with_context(|| format!("Couldn't get output for command {command:?}"))?;

    // Make sure that we return an appropriate exit code here, as Github Actions
    // requires this in order to function correctly:
    if output.status.success() {
        Ok(String::from_utf8_lossy(&output.stdout).to_string())
    } else {
        bail!(
            "Failed to execute cargo subcommand `cargo {}`",
            args.join(" "),
        )
    }
}

fn get_cargo() -> String {
    // On Windows when executed via `cargo run` (e.g. via the xtask alias) the
    // `cargo` on the search path is NOT the cargo-wrapper but the `cargo` from the
    // toolchain - that one doesn't understand `+toolchain`
    #[cfg(target_os = "windows")]
    let cargo = if let Ok(cargo) = std::env::var("CARGO_HOME") {
        format!("{cargo}/bin/cargo")
    } else {
        String::from("cargo")
    };

    #[cfg(not(target_os = "windows"))]
    let cargo = String::from("cargo");

    cargo
}

/// A builder for constructing cargo command line arguments.
#[derive(Clone, Debug, Default)]
pub struct CargoArgsBuilder {
    pub(crate) artifact_name: String,
    pub(crate) config_path: Option<PathBuf>,
    pub(crate) manifest_path: Option<PathBuf>,
    pub(crate) toolchain: Option<String>,
    pub(crate) subcommand: String,
    pub(crate) target: Option<String>,
    pub(crate) features: Vec<String>,
    pub(crate) args: Vec<String>,
    pub(crate) configs: Vec<String>,
    pub(crate) env_vars: HashMap<String, String>,
}

impl CargoArgsBuilder {
    pub fn new(artifact_name: String) -> Self {
        Self {
            artifact_name,
            ..Default::default()
        }
    }

    /// Set the path to the Cargo manifest file (Cargo.toml)
    #[must_use]
    pub fn manifest_path(mut self, path: PathBuf) -> Self {
        self.manifest_path = Some(path);
        self
    }

    /// Set the path to the Cargo configuration file (.cargo/config.toml)
    #[must_use]
    pub fn config_path(mut self, path: PathBuf) -> Self {
        self.config_path = Some(path);
        self
    }

    /// Set the Rust toolchain to use.
    #[must_use]
    pub fn toolchain<S>(mut self, toolchain: S) -> Self
    where
        S: Into<String>,
    {
        self.toolchain = Some(toolchain.into());
        self
    }

    /// Set the cargo subcommand to use.
    #[must_use]
    pub fn subcommand<S>(mut self, subcommand: S) -> Self
    where
        S: Into<String>,
    {
        self.subcommand = subcommand.into();
        self
    }

    /// Set the compilation target to use.
    #[must_use]
    pub fn target<S>(mut self, target: S) -> Self
    where
        S: Into<String>,
    {
        self.target = Some(target.into());
        self
    }

    /// Set the cargo features to use.
    #[must_use]
    pub fn features(mut self, features: &[String]) -> Self {
        self.features = features.to_vec();
        self
    }

    /// Add a single argument to the cargo command line.
    #[must_use]
    pub fn arg<S>(mut self, arg: S) -> Self
    where
        S: Into<String>,
    {
        self.args.push(arg.into());
        self
    }

    /// Add multiple arguments to the cargo command line.
    #[must_use]
    pub fn args<S>(mut self, args: &[S]) -> Self
    where
        S: Clone + Into<String>,
    {
        for arg in args {
            self.args.push(arg.clone().into());
        }
        self
    }

    /// Add a single argument to the cargo command line.
    pub fn add_arg<S>(&mut self, arg: S) -> &mut Self
    where
        S: Into<String>,
    {
        self.args.push(arg.into());
        self
    }

    /// Adds a raw configuration argument (--config, -Z, ...)
    #[must_use]
    pub fn config<S>(mut self, arg: S) -> Self
    where
        S: Into<String>,
    {
        self.add_config(arg);
        self
    }

    /// Adds a raw configuration argument (--config, -Z, ...)
    pub fn add_config<S>(&mut self, arg: S) -> &mut Self
    where
        S: Into<String>,
    {
        self.configs.push(arg.into());
        self
    }

    /// Adds an environment variable
    pub fn add_env_var<S>(&mut self, key: S, value: S) -> &mut Self
    where
        S: Into<String>,
    {
        self.env_vars.insert(key.into(), value.into());
        self
    }

    /// Build the final list of cargo command line arguments.
    #[must_use]
    pub fn build(&self) -> Vec<String> {
        let mut args = vec![];

        if let Some(ref toolchain) = self.toolchain {
            args.push(format!("+{toolchain}"));
        }

        args.push(self.subcommand.clone());

        if let Some(manifest_path) = &self.manifest_path {
            args.push("--manifest-path".to_string());
            args.push(manifest_path.display().to_string());
        }

        if let Some(config_path) = &self.config_path {
            args.push("--config".to_string());
            args.push(config_path.display().to_string());
        }

        if let Some(ref target) = self.target {
            args.push(format!("--target={target}"));
        }

        for config in self.configs.iter() {
            args.push(config.clone());
        }

        if !self.features.is_empty() {
            args.push(format!("--features={}", self.features.join(",")));
        }

        for arg in self.args.iter() {
            args.push(arg.clone());
        }

        log::debug!("Built cargo args: {:?}", args);
        args
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
struct BatchKey {
    config_file: String,
    toolchain: Option<String>,
    config: Vec<String>,
    env_vars: Vec<(String, String)>,
}

impl BatchKey {
    fn from_command(command: &CargoArgsBuilder) -> Self {
        let config_file = if let Some(config_path) = &command.config_path {
            std::fs::read_to_string(config_path).unwrap_or_default()
        } else {
            String::new()
        };

        Self {
            toolchain: command.toolchain.clone(),
            config: command.configs.clone(),
            config_file,
            env_vars: {
                let mut env_vars = command
                    .env_vars
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect::<Vec<_>>();

                env_vars.sort();
                env_vars
            },
        }
    }
}

#[derive(Debug)]
pub struct CargoCommandBatcher {
    commands: HashMap<BatchKey, Vec<CargoArgsBuilder>>,
}

#[derive(Debug, Clone)]
pub struct BuiltCommand {
    pub artifact_name: String,
    pub command: Vec<String>,
    pub env_vars: Vec<(String, String)>,
}

impl BuiltCommand {
    pub fn run(&self, capture: bool) -> Result<String> {
        let env_vars = self.env_vars.clone();
        let cwd = std::env::current_dir()?;
        run_with_env(&self.command, &cwd, env_vars, capture)
    }
}

impl CargoCommandBatcher {
    pub fn new() -> Self {
        Self {
            commands: HashMap::new(),
        }
    }

    pub fn push(&mut self, command: CargoArgsBuilder) {
        let key = BatchKey::from_command(&command);
        self.commands.entry(key).or_default().push(command);
    }

    fn build_for_cargo_batch(&self) -> Vec<BuiltCommand> {
        let mut all = Vec::new();

        for (key, group) in self.commands.iter() {
            // No need to batch if there's only one command
            if group.len() == 1 {
                all.push(Self::build_one_for_cargo(&group[0]));
                continue;
            }

            let mut command = Vec::new();
            let mut batch_len = 0;
            let mut commands_in_batch = 0;

            // Windows be Windows, it has a command length limit.
            let limit = if cfg!(target_os = "windows") {
                Some(8191)
            } else {
                None
            };

            for item in group.iter() {
                // Only some commands can be batched
                let batchable = ["build", "doc", "check"];
                if !batchable
                    .iter()
                    .any(|&subcommand| subcommand == item.subcommand)
                {
                    all.push(Self::build_one_for_cargo(item));
                    continue;
                }

                // Build the new command
                let mut c = item.clone();

                c.toolchain = None;
                c.configs = Vec::new();
                c.config_path = None;

                let args = c.build();

                let command_chars = 4 + args.iter().map(|arg| arg.len() + 1).sum::<usize>();

                if !command.is_empty()
                    && let Some(limit) = limit
                    && batch_len + command_chars > limit
                {
                    // Command would be too long, cut here.
                    all.push(BuiltCommand {
                        artifact_name: String::from("batch"),
                        command: std::mem::take(&mut command),
                        env_vars: key.env_vars.clone(),
                    });
                }

                // Set up head part if empty
                if command.is_empty() {
                    if let Some(tc) = key.toolchain.as_ref() {
                        command.push(format!("+{tc}"));
                    }

                    command.push("batch".to_string());
                    if !key.config_file.is_empty()
                        && let Some(config_path) = &group[0].config_path
                    {
                        // All grouped projects have the same config file content, pick one:
                        command.push("--config".to_string());
                        command.push(config_path.display().to_string());
                    }
                    command.extend_from_slice(&key.config);

                    commands_in_batch = 0;
                    batch_len = command.iter().map(|s| s.len() + 1).sum::<usize>() - 1;
                }

                // Append the new command

                command.push("---".to_string());
                command.extend_from_slice(&args);

                commands_in_batch += 1;
                batch_len += command_chars;
            }

            if commands_in_batch > 0 {
                all.push(BuiltCommand {
                    artifact_name: String::from("batch"),
                    command,
                    env_vars: key.env_vars.clone(),
                });
            }
        }

        all
    }

    fn build_for_cargo(&self) -> Vec<BuiltCommand> {
        let mut all = Vec::new();

        for group in self.commands.values() {
            for item in group.iter() {
                all.push(Self::build_one_for_cargo(item));
            }
        }

        all
    }

    pub fn build_one_for_cargo(item: &CargoArgsBuilder) -> BuiltCommand {
        BuiltCommand {
            artifact_name: item.artifact_name.clone(),
            command: {
                let mut args = item.build();

                if item.args.iter().any(|arg| arg == "--artifact-dir") {
                    args.push("-Zunstable-options".to_string());
                }

                args
            },
            env_vars: item
                .env_vars
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
    }

    pub fn build(&self, no_batch: bool) -> Vec<BuiltCommand> {
        let cargo_batch_available = Command::new("cargo")
            .arg("batch")
            .arg("-h")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false);

        if cargo_batch_available && !no_batch {
            self.build_for_cargo_batch()
        } else {
            if !no_batch {
                log::warn!("You don't have cargo batch installed. Falling back to cargo.");
                log::warn!("You should really install cargo-batch.");
                log::warn!(
                    "cargo install --git https://github.com/embassy-rs/cargo-batch cargo --bin cargo-batch --locked"
                );
            }
            self.build_for_cargo()
        }
    }
}

impl Drop for CargoCommandBatcher {
    fn drop(&mut self) {}
}

/// A representation of a Cargo.toml file for a specific package.
pub struct CargoToml {
    /// The workspace path where the Cargo.toml is located.
    pub workspace: PathBuf,
    /// The package this Cargo.toml belongs to.
    pub package: Package,
    /// The parsed Cargo.toml manifest.
    pub manifest: toml_edit::DocumentMut,
}

const DEPENDENCY_KINDS: [&'static str; 3] =
    ["dependencies", "dev-dependencies", "build-dependencies"];

impl CargoToml {
    /// Load and parse the Cargo.toml for the specified package in the given workspace.
    pub fn new(workspace: &Path, package: Package) -> Result<Self> {
        let package_path = workspace.join(package.to_string());
        let manifest_path = package_path.join("Cargo.toml");
        if !manifest_path.exists() {
            bail!(
                "Could not find Cargo.toml for package {package} at {}",
                manifest_path.display()
            );
        }

        let manifest = std::fs::read_to_string(&manifest_path)
            .with_context(|| format!("Could not read {}", manifest_path.display()))?;

        Self::from_str(workspace, package, &manifest)
    }

    pub fn espressif_metadata(&self) -> Option<&Table> {
        let Some(package) = self.manifest.get("package") else {
            return None;
        };
        let Some(metadata) = package.get("metadata") else {
            return None;
        };
        let Some(espressif) = metadata.get("espressif") else {
            return None;
        };
        Some(espressif.as_table()?)
    }

    /// Create a `CargoToml` instance from a manifest string.
    pub fn from_str(workspace: &Path, package: Package, manifest: &str) -> Result<Self> {
        // Parse the manifest string into a mutable TOML document.
        Ok(Self {
            workspace: workspace.to_path_buf(),
            package,
            manifest: manifest
                .parse::<DocumentMut>()
                .with_context(|| format!("Manifest {manifest} parsing failed!"))?,
        })
    }

    /// Check if the package is published to crates.io.
    pub fn is_published(&self) -> bool {
        // Check if the package is published by looking for the `publish` key
        // in the manifest.
        let Item::Table(package) = &self.manifest["package"] else {
            unreachable!("The package table is missing in the manifest");
        };

        let Some(publish) = package.get("publish") else {
            return true;
        };

        publish.as_bool().unwrap_or(true)
    }

    /// Get the absolute path to the package directory.
    pub fn package_path(&self) -> PathBuf {
        self.workspace.join(self.package.to_string())
    }

    /// Get the absolute path to the Cargo.toml file of the package.
    pub fn manifest_path(&self) -> PathBuf {
        self.package_path().join("Cargo.toml")
    }

    /// Get the current version of the package.
    pub fn version(&self) -> &str {
        self.manifest["package"]["version"]
            .as_str()
            .unwrap()
            .trim()
            .trim_matches('"')
    }

    /// Get the current version of the package as a `semver::Version`.
    pub fn package_version(&self) -> semver::Version {
        semver::Version::parse(self.version()).expect("Failed to parse version")
    }

    /// Set the version of the package to the specified version.
    pub fn set_version(&mut self, version: &semver::Version) {
        log::info!(
            "Bumping version for package: {} ({} -> {version})",
            self.package,
            self.version(),
        );
        self.manifest["package"]["version"] = toml_edit::value(version.to_string());
    }

    /// Save the modified Cargo.toml back to disk.
    pub fn save(&self) -> Result<()> {
        let manifest_path = self.manifest_path();
        std::fs::write(&manifest_path, self.manifest.to_string())
            .with_context(|| format!("Could not write {}", manifest_path.display()))?;

        Ok(())
    }

    /// Calls a callback for each table that contains dependencies.
    ///
    /// Callback arguments:
    /// - `path`: The path to the table (e.g. `dependencies.package`)
    /// - `dependency_kind`: The kind of dependency (e.g. `dependencies`, `dev-dependencies`)
    /// - `table`: The table itself
    pub fn visit_dependencies(
        &mut self,
        mut handle_dependencies: impl FnMut(&str, &'static str, &mut toml_edit::Table),
    ) {
        fn recurse_dependencies(
            path: String,
            table: &mut toml_edit::Table,
            handle_dependencies: &mut impl FnMut(&str, &'static str, &mut toml_edit::Table),
        ) {
            // Walk through tables recursively so that we can find *all* dependencies.
            for (key, item) in table.iter_mut() {
                if let Item::Table(table) = item {
                    let path = if path.is_empty() {
                        key.to_string()
                    } else {
                        format!("{path}.{key}")
                    };
                    recurse_dependencies(path, table, handle_dependencies);
                }
            }
            for dependency_kind in DEPENDENCY_KINDS {
                let Some(Item::Table(table)) = table.get_mut(dependency_kind) else {
                    continue;
                };

                handle_dependencies(&path, dependency_kind, table);
            }
        }

        recurse_dependencies(
            String::new(),
            self.manifest.as_table_mut(),
            &mut handle_dependencies,
        );
    }

    /// Returns the package this Cargo.toml belongs to.
    pub fn package(&self) -> Package {
        self.package
    }

    /// Returns all dependencies of the package, that come from the repository.
    ///
    /// For example, for esp-println this will return [esp-metadata]
    /// (at the time of writing).
    pub fn repo_dependencies(&mut self) -> Vec<Package> {
        let mut dependencies = Vec::new();
        self.visit_dependencies(|_, _, table| {
            for (name, value) in table.iter() {
                let name = if let Item::Value(Value::InlineTable(t)) = value {
                    if let Some(Value::String(name)) = t.get("package") {
                        name.value()
                    } else {
                        name
                    }
                } else {
                    name
                };

                if let Ok(package) = Package::from_str(name, true) {
                    if !dependencies.contains(&package) {
                        dependencies.push(package);
                    }
                }
            }
        });
        dependencies
    }

    pub(crate) fn change_version_of_dependency(
        &mut self,
        package_name: &str,
        version: &semver::Version,
    ) -> bool {
        let mut changed = false;

        self.visit_dependencies(|_, _, table| {
            // Update dependencies which specify a version:
            match &mut table[&package_name] {
                Item::Value(Value::String(table)) => {
                    // package = "version"
                    *table = Formatted::new(version.to_string());
                    changed = true;
                }
                Item::Table(table) if table.contains_key("version") => {
                    // [package]
                    // version = "version"
                    table["version"] = toml_edit::value(version.to_string());
                    changed = true;
                }
                Item::Value(Value::InlineTable(table)) if table.contains_key("version") => {
                    // package = { version = "version" }
                    table["version"] = version.to_string().into();
                    changed = true;
                }
                Item::None => {
                    // alias = { package = "foo", version = "version" }
                    let update_renamed_dep = table.get_values().iter().find_map(|(k, p)| {
                        if let Value::InlineTable(table) = p {
                            if let Some(Value::String(name)) = &table.get("package") {
                                if name.value() == &package_name {
                                    // Return the actual key of this dependency, e.g.:
                                    // `procmacros = { package = "esp-hal-procmacros" }`
                                    //  ^^^^^^^^^^
                                    return Some(k.last().unwrap().get().to_string());
                                }
                            }
                        }

                        None
                    });

                    if let Some(dependency_name) = update_renamed_dep {
                        table[&dependency_name]["version"] = version.to_string().into();
                        changed = true;
                    }
                }
                _ => {}
            }
        });

        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bump_version() {
        struct TestCase {
            input: &'static str,
            expected: &'static str,
        }

        let test_cases = [
            // Simple dependencies
            TestCase {
                input: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                esp-hal-procmacros = "0.1.0"

                [dev-dependencies]
                esp-hal-procmacros = "0.1.0"

                [build-dependencies]
                esp-hal-procmacros = "0.1.0"
                "#,
                expected: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                esp-hal-procmacros = "0.2.0"

                [dev-dependencies]
                esp-hal-procmacros = "0.2.0"

                [build-dependencies]
                esp-hal-procmacros = "0.2.0"
                "#,
            },
            // Dependencies with inline tables
            TestCase {
                input: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                esp-hal-procmacros = { version = "0.1.0", foo = "bar" }

                [dev-dependencies]
                esp-hal-procmacros = { version = "0.1.0", foo = "bar" }

                [build-dependencies]
                esp-hal-procmacros = { version = "0.1.0", foo = "bar" }
                "#,
                expected: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                esp-hal-procmacros = { version = "0.2.0", foo = "bar" }

                [dev-dependencies]
                esp-hal-procmacros = { version = "0.2.0", foo = "bar" }

                [build-dependencies]
                esp-hal-procmacros = { version = "0.2.0", foo = "bar" }
                "#,
            },
            // Dependencies with renamed keys
            TestCase {
                input: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.1.0", foo = "bar" }

                [dev-dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.1.0", foo = "bar" }

                [build-dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.1.0", foo = "bar" }
                "#,
                expected: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.2.0", foo = "bar" }

                [dev-dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.2.0", foo = "bar" }

                [build-dependencies]
                procmacros = { package = "esp-hal-procmacros", version = "0.2.0", foo = "bar" }
                "#,
            },
            // Dependencies specified as tables
            TestCase {
                input: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies.'esp-hal-procmacros']
                version = "0.1.0"

                [dev-dependencies.'esp-hal-procmacros']
                version = "0.1.0"

                [build-dependencies.'esp-hal-procmacros']
                version = "0.1.0"
                "#,
                expected: r#"
                [package]
                name = "test-package"
                version = "0.1.0"

                [dependencies.'esp-hal-procmacros']
                version = "0.2.0"

                [dev-dependencies.'esp-hal-procmacros']
                version = "0.2.0"

                [build-dependencies.'esp-hal-procmacros']
                version = "0.2.0"
                "#,
            },
        ];

        for test_case in test_cases {
            let mut cargo_toml =
                CargoToml::from_str(Path::new("."), Package::EspAlloc, test_case.input).unwrap();

            let changed = cargo_toml.change_version_of_dependency(
                "esp-hal-procmacros",
                &semver::Version::parse("0.2.0").unwrap(),
            );

            assert!(changed);
            pretty_assertions::assert_eq!(test_case.expected, cargo_toml.manifest.to_string());
        }
    }
}
