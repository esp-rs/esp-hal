//! Tools for working with Cargo.

use std::{
    path::Path,
    process::{Command, Stdio},
};

use anyhow::{bail, Result};

/// Execute cargo with the given arguments and from the specified directory.
pub fn run(args: &[String], cwd: &Path) -> Result<()> {
    if !cwd.is_dir() {
        bail!("The `cwd` argument MUST be a directory");
    }

    let status = Command::new(get_cargo())
        .args(args)
        .current_dir(cwd)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .status()?;

    // Make sure that we return an appropriate exit code here, as Github Actions
    // requires this in order to function correctly:
    if status.success() {
        Ok(())
    } else {
        bail!("Failed to execute cargo subcommand")
    }
}

/// Execute cargo with the given arguments and from the specified directory.
pub fn run_with_input(args: &[String], cwd: &Path) -> Result<()> {
    if !cwd.is_dir() {
        bail!("The `cwd` argument MUST be a directory");
    }

    let _status = Command::new(get_cargo())
        .args(args)
        .current_dir(cwd)
        .stdout(std::process::Stdio::inherit())
        .stderr(std::process::Stdio::inherit())
        .stdin(std::process::Stdio::inherit())
        .status()?;

    Ok(())
}

fn get_cargo() -> String {
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
