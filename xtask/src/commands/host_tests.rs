use std::path::Path;

use anyhow::Result;
use clap::Args;
use strum::IntoEnumIterator;

use crate::Package;

/// Arguments for the `host-tests` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run host-tests in the workspace with cargo test",
        command = "host-tests"
    )
)]
#[derive(Debug, Args)]
pub struct HostTestsArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Run host-tests for packages that declare them.
pub fn host_tests(workspace: &Path, args: HostTestsArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        log::debug!("Running host-tests for package: {}", package);
        if package.has_host_tests(workspace) {
            crate::run_host_tests(workspace, package)?;
        }
    }

    Ok(())
}
