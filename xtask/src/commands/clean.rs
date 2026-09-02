use std::path::Path;

use anyhow::{Context, Result};
use clap::Args;
use strum::IntoEnumIterator;

use crate::{Package, cargo::CargoArgsBuilder};

/// Arguments for the `clean` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run cargo clean for the specified packages",
        command = "clean"
    )
)]
#[derive(Debug, Args)]
pub struct CleanArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Run `cargo clean` for each package directory that contains a Cargo.toml.
pub fn clean(workspace: &Path, args: CleanArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        let path = workspace.join(package.directory());
        for dir in walkdir::WalkDir::new(path) {
            if let Ok(dir) = dir
                && let path = dir.path()
                && path.join("Cargo.toml").exists()
            {
                log::info!("Cleaning folder: {}", path.display());
                let cargo_args = CargoArgsBuilder::default()
                    .subcommand("clean")
                    .arg("--target-dir")
                    .arg(path.join("target").display().to_string())
                    .build();

                crate::cargo::run(&cargo_args, path).with_context(|| {
                    format!(
                        "Failed to run `cargo run` with {cargo_args:?} in {}",
                        path.display()
                    )
                })?;
            }
        }
    }

    Ok(())
}
