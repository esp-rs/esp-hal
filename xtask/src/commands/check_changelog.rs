use std::path::Path;

use anyhow::{Context, Result, bail};

use crate::{Package, changelog::Changelog};

/// Check the changelogs for the specified packages. If `normalize` is true, rewrite
/// the changelogs in a normalized format.
pub fn check_changelog(workspace: &Path, packages: &[Package], normalize: bool) -> Result<()> {
    let mut failed = false;
    for package in packages {
        if let Err(e) = check_changelog_for_package(workspace, *package, normalize) {
            eprintln!("Error checking changelog for package {}: {:?}", package, e);
            failed = true;
        }
    }

    if failed {
        bail!("One or more changelogs failed to check.");
    }

    Ok(())
}

fn check_changelog_for_package(workspace: &Path, package: Package, normalize: bool) -> Result<()> {
    let changelog_path = workspace.join(package.to_string()).join("CHANGELOG.md");

    if !changelog_path.exists() {
        // No changelog exists for this package
        return Ok(());
    }

    log::info!("  Checking changelog for package: {package}");

    // Let's parse the old changelog first
    let changelog_str = std::fs::read_to_string(&changelog_path)
        .with_context(|| format!("Could not read {}", changelog_path.display()))?;

    let changelog = Changelog::parse(&changelog_str)
        .with_context(|| format!("Could not parse {}", changelog_path.display()))?;

    if normalize {
        std::fs::write(&changelog_path, changelog.to_string()).with_context(|| {
            format!(
                "Failed to write changelog into {}",
                changelog_path.display()
            )
        })?;
    }

    Ok(())
}
