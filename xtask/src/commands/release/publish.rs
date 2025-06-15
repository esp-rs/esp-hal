use std::path::Path;

use anyhow::{Result, ensure};
use clap::Args;

use crate::{Package, cargo::CargoArgsBuilder, windows_safe_path};

#[derive(Debug, Args)]
pub struct PublishArgs {
    /// Package to publish (performs a dry-run by default).
    #[arg(value_enum)]
    package: Package,

    /// Do not pass the `--dry-run` argument, actually try to publish.
    #[arg(long)]
    no_dry_run: bool,
}

pub fn publish(workspace: &Path, args: PublishArgs) -> Result<()> {
    let package_name = args.package.to_string();
    let package_path = windows_safe_path(&workspace.join(&package_name));

    ensure!(
        args.package.is_published(workspace),
        "Invalid package '{}' specified, this package should not be published!",
        args.package
    );

    let mut publish_args = if args.package.has_chip_features() {
        vec!["--no-verify"]
    } else {
        vec![]
    };

    if !args.no_dry_run {
        publish_args.push("--dry-run");
    }

    let builder = CargoArgsBuilder::default()
        .subcommand("publish")
        .args(&publish_args);

    let args = builder.build();
    log::debug!("{args:#?}");

    // Execute `cargo publish` command from the package root:
    crate::cargo::run(&args, &package_path)?;

    Ok(())
}
