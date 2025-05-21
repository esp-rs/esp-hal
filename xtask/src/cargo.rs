//! Tools for working with Cargo.

use std::{
    ffi::OsStr,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

use anyhow::{Context as _, Result, bail};
use serde::{Deserialize, Serialize};
use toml_edit::{DocumentMut, Item};

use crate::{Package, windows_safe_path};

#[derive(Clone, Debug, PartialEq)]
pub enum CargoAction {
    Build(PathBuf),
    Run,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Artifact {
    pub executable: PathBuf,
}

/// Execute cargo with the given arguments and from the specified directory.
pub fn run(args: &[String], cwd: &Path) -> Result<()> {
    run_with_env::<[(&str, &str); 0], _, _>(args, cwd, [], false)?;
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

    let output = Command::new(get_cargo())
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
        })
        .stdin(Stdio::inherit())
        .output()?;

    // Make sure that we return an appropriate exit code here, as Github Actions
    // requires this in order to function correctly:
    if output.status.success() {
        Ok(String::from_utf8_lossy(&output.stdout).to_string())
    } else {
        bail!("Failed to execute cargo subcommand")
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

#[derive(Debug, Default)]
pub struct CargoArgsBuilder {
    toolchain: Option<String>,
    subcommand: String,
    target: Option<String>,
    features: Vec<String>,
    args: Vec<String>,
}

impl CargoArgsBuilder {
    #[must_use]
    pub fn toolchain<S>(mut self, toolchain: S) -> Self
    where
        S: Into<String>,
    {
        self.toolchain = Some(toolchain.into());
        self
    }

    #[must_use]
    pub fn subcommand<S>(mut self, subcommand: S) -> Self
    where
        S: Into<String>,
    {
        self.subcommand = subcommand.into();
        self
    }

    #[must_use]
    pub fn target<S>(mut self, target: S) -> Self
    where
        S: Into<String>,
    {
        self.target = Some(target.into());
        self
    }

    #[must_use]
    pub fn features(mut self, features: &[String]) -> Self {
        self.features = features.to_vec();
        self
    }

    #[must_use]
    pub fn arg<S>(mut self, arg: S) -> Self
    where
        S: Into<String>,
    {
        self.args.push(arg.into());
        self
    }

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

    pub fn add_arg<S>(&mut self, arg: S) -> &mut Self
    where
        S: Into<String>,
    {
        self.args.push(arg.into());
        self
    }

    #[must_use]
    pub fn build(&self) -> Vec<String> {
        let mut args = vec![];

        if let Some(ref toolchain) = self.toolchain {
            args.push(format!("+{toolchain}"));
        }

        args.push(self.subcommand.clone());

        if let Some(ref target) = self.target {
            args.push(format!("--target={target}"));
        }

        if !self.features.is_empty() {
            args.push(format!("--features={}", self.features.join(",")));
        }

        for arg in self.args.iter() {
            args.push(arg.clone());
        }

        args
    }
}

pub struct CargoToml<'a> {
    pub workspace: &'a Path,
    pub package: Package,
    pub manifest: toml_edit::DocumentMut,
}

const DEPENDENCY_KINDS: [&'static str; 3] =
    ["dependencies", "dev-dependencies", "build-dependencies"];

impl<'a> CargoToml<'a> {
    pub fn new(workspace: &'a Path, package: Package) -> Result<Self> {
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

        Ok(Self {
            workspace,
            package,
            manifest: manifest.parse::<DocumentMut>()?,
        })
    }

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

    pub fn package_path(&self) -> PathBuf {
        self.workspace.join(self.package.to_string())
    }

    pub fn manifest_path(&self) -> PathBuf {
        self.package_path().join("Cargo.toml")
    }

    pub fn version(&self) -> &str {
        self.manifest["package"]["version"]
            .as_str()
            .unwrap()
            .trim()
            .trim_matches('"')
    }

    pub fn set_version(&mut self, version: &semver::Version) {
        log::info!(
            "Bumping version for package: {} ({} -> {version})",
            self.package,
            self.version(),
        );
        self.manifest["package"]["version"] = toml_edit::value(version.to_string());
    }

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
    /// - `dependency_kind`: The kind of dependency (e.g. `dependencies`,
    ///   `dev-dependencies`)
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
}
