use std::path::Path;

use anyhow::Result;
use clap::Args;
use strum::IntoEnumIterator;

use crate::Package;

/// Arguments for the `fmt` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Format all packages in the workspace with rustfmt",
        command = "fmt"
    )
)]
#[derive(Debug, Args)]
pub struct FmtPackagesArgs {
    /// Run in 'check' mode; exits with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    pub check: bool,

    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Format workspace packages with rustfmt.
pub fn fmt_packages(workspace: &Path, args: FmtPackagesArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        crate::format_package(workspace, package, args.check, None)?;
    }

    // Keep formatting "helper" crates that are intentionally not in
    // the Package enum.
    for package_path in ["xtask", "xtask-mcp-macros"] {
        log::info!("Formatting package: {}", package_path);
        crate::format_package_path(workspace, &workspace.join(package_path), args.check, None)?;
    }

    // format ymls in .github/
    crate::format_yml(args.check, "./.github")?;

    Ok(())
}
