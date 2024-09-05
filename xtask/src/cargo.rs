//! Tools for working with Cargo.

use std::{
    path::Path,
    process::{Command, Stdio},
};

use anyhow::{bail, Result};

use crate::windows_safe_path;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CargoAction {
    Build,
    Run,
}

/// Execute cargo with the given arguments and from the specified directory.
pub fn run(args: &[String], cwd: &Path) -> Result<()> {
    if !cwd.is_dir() {
        bail!("The `cwd` argument MUST be a directory");
    }

    // Make sure to not use a UNC as CWD!
    // That would make `OUT_DIR` a UNC which will trigger things like the one fixed in https://github.com/dtolnay/rustversion/pull/51
    // While it's fixed in `rustversion` it's not fixed for other crates we are
    // using now or in future!
    let cwd = windows_safe_path(cwd);

    let status = Command::new(get_cargo())
        .args(args)
        .current_dir(cwd)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .stdin(Stdio::inherit())
        .status()?;

    // Make sure that we return an appropriate exit code here, as Github Actions
    // requires this in order to function correctly:
    if status.success() {
        Ok(())
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

    pub fn add_arg<S>(&mut self, arg: S) -> &mut Self
    where
        S: Into<String>,
    {
        self.args.push(arg.into());
        self
    }

    #[must_use]
    pub fn build(self) -> Vec<String> {
        let mut args = vec![];

        if let Some(toolchain) = self.toolchain {
            args.push(format!("+{toolchain}"));
        }

        args.push(self.subcommand);

        if let Some(target) = self.target {
            args.push(format!("--target={target}"));
        }

        if !self.features.is_empty() {
            args.push(format!("--features={}", self.features.join(",")));
        }

        for arg in self.args {
            args.push(arg);
        }

        args
    }
}
