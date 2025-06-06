use std::fs;

use anyhow::Result;
use semver::Version;
use strum::IntoEnumIterator;

use crate::Package;
use super::PLACEHOLDER;

#[derive(Debug, Clone, clap::Args)]
pub struct PostReleaseArgs {
    /// Package to run the post release step on.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,
    /// Actually execute post-release actions.
    #[arg(long)]
    no_dry_run: bool,
}

pub fn post_release(workspace: &std::path::Path, args: PostReleaseArgs) -> Result<()> {
    for package in args
        .packages
        .iter()
        .filter(|p| p.has_migration_guide(workspace))
    {
        // Get the package's directory path
        let package_path = workspace.join(package.to_string());
        let cargo_toml_path = package_path.join("Cargo.toml");

        // Read and parse Cargo.toml
        let cargo_toml_content = fs::read_to_string(&cargo_toml_path)?;
        let cargo_toml = cargo_toml_content.parse::<toml_edit::DocumentMut>()?;

        // Extract version from Cargo.toml
        let version_str = cargo_toml["package"]["version"].as_str().ok_or_else(|| {
            anyhow::anyhow!(
                "Could not find version in Cargo.toml for package {:?}",
                package
            )
        })?;

        // Parse version using semver and zero out patch version
        let mut version = Version::parse(version_str)?;
        version.patch = 0;

        // Generate migration guide filename
        let migration_file_name = format!("MIGRATING-{}.md", version);
        let migration_file_path = package_path.join(&migration_file_name);

        // Create the migration guide file if it doesn't exist
        if !migration_file_path.exists() {
            // Create the title content
            let title = format!("# Migration Guide from {} to {}\n", version, PLACEHOLDER);

            if args.no_dry_run {
                fs::write(&migration_file_path, title)?;
                log::info!("Created migration guide: {}", migration_file_path.display());
            } else {
                log::info!(
                    "Would create migration guide: {}",
                    migration_file_path.display()
                );
                continue;
            }
        } else {
            log::info!(
                "Migration guide already exists: {}",
                migration_file_path.display()
            );
        }
    }

    Ok(())
}
