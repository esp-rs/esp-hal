use std::path::Path;

use anyhow::{Result, bail};
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

    use Package::*;
    let mut publish_args = match args.package {
        Examples | HilTest | QaTest => {
            bail!(
                "Invalid package '{}' specified, this package should not be published!",
                args.package
            )
        }

        EspBacktrace | EspHal | EspHalEmbassy | EspIeee802154 | EspLpHal | EspPrintln
        | EspRiscvRt | EspStorage | EspWifi | XtensaLxRt => vec!["--no-verify"],

        _ => vec![],
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
