use std::{fs, path::Path};

use anyhow::{Context, Result, bail};
use clap::Args;
use semver::Prerelease;
use strum::IntoEnumIterator;
use toml_edit::{Item, Value};

use crate::{Package, Version, changelog::Changelog};

#[derive(Debug, Args)]
pub struct BumpVersionArgs {
    /// How much to bump the version by.
    ///
    /// If the version is a pre-release, this will just remove the pre-release
    /// version tag.
    #[arg(value_enum)]
    amount: Version,
    /// Pre-release version to append to the version.
    ///
    /// If the package is already a pre-release, this will ignore `amount`. If
    /// the package is not a pre-release, this will bump the version by
    /// `amount` and append the pre-release version as `<pre>.0`.
    #[arg(long)]
    pre: Option<String>,
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

pub fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    // Bump the version by the specified amount for each given package:
    for package in args.packages {
        let new_version = bump_crate_version(workspace, package, args.amount, args.pre.as_deref())?;
        finalize_changelog(workspace, package, new_version)?;
    }

    Ok(())
}

/// Bump the version of the specified package by the specified amount.
fn bump_crate_version(
    workspace: &Path,
    package: Package,
    amount: Version,
    pre: Option<&str>,
) -> Result<semver::Version> {
    let manifest_path = workspace.join(package.to_string()).join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .with_context(|| format!("Could not read {}", manifest_path.display()))?;

    let mut manifest = manifest.parse::<toml_edit::DocumentMut>()?;

    let version = manifest["package"]["version"]
        .to_string()
        .trim()
        .trim_matches('"')
        .to_string();
    let prev_version = &version;

    let version = do_version_bump(&version, package, amount, pre)?;

    log::info!("Bumping version for package: {package} ({prev_version} -> {version})");

    manifest["package"]["version"] = toml_edit::value(version.to_string());
    fs::write(manifest_path, manifest.to_string())?;

    let dependency_kinds = ["dependencies", "dev-dependencies", "build-dependencies"];

    for pkg in Package::iter() {
        let manifest_path = workspace.join(pkg.to_string()).join("Cargo.toml");
        let manifest = fs::read_to_string(&manifest_path)
            .with_context(|| format!("Could not read {}", manifest_path.display()))?;

        let mut manifest = manifest.parse::<toml_edit::DocumentMut>()?;

        let mut changed = false;
        let package_name = package.to_string();
        for dependency_kind in dependency_kinds {
            let Some(table) = manifest[dependency_kind].as_table_mut() else {
                continue;
            };

            let dependency = &mut table[package_name.as_str()];

            // Update dependencies which specify a version:
            match dependency {
                Item::Table(table) if table.contains_key("version") => {
                    table["version"] = toml_edit::value(version.to_string());
                    changed = true;
                }
                Item::Value(Value::InlineTable(table)) if table.contains_key("version") => {
                    table["version"] = version.to_string().into();
                    changed = true;
                }
                _ => continue,
            }
        }

        if changed {
            log::info!(
                "  Bumping {package_name} version for package {pkg}: ({prev_version} -> {version})"
            );

            fs::write(&manifest_path, manifest.to_string())
                .with_context(|| format!("Could not write {}", manifest_path.display()))?;
        }
    }

    Ok(version)
}

fn do_version_bump(
    version: &str,
    package: Package,
    amount: Version,
    pre: Option<&str>,
) -> Result<semver::Version> {
    fn bump_version_number(version: &mut semver::Version, amount: Version) {
        match amount {
            Version::Major => {
                version.major += 1;
                version.minor = 0;
                version.patch = 0;
            }
            Version::Minor => {
                version.minor += 1;
                version.patch = 0;
            }
            Version::Patch => {
                version.patch += 1;
            }
        }
    }
    let mut version = semver::Version::parse(version)?;

    if let Some(pre) = pre {
        if let Some(pre_version) = version.pre.as_str().strip_prefix(&format!("{pre}.")) {
            let pre_version = pre_version.parse::<u32>()?;
            version.pre = Prerelease::new(&format!("{pre}.{}", pre_version + 1)).unwrap();
        } else if version.pre.as_str().is_empty() {
            // Start a new pre-release
            bump_version_number(&mut version, amount);
            version.pre = Prerelease::new(&format!("{pre}.0")).unwrap();
        } else {
            bail!(
                "Unexpected pre-release version format found for {package}: {}",
                version.pre.as_str()
            );
        }
    } else if !version.pre.is_empty() {
        version.pre = Prerelease::EMPTY;
    } else {
        bump_version_number(&mut version, amount);
    }

    Ok(version)
}

fn finalize_changelog(
    workspace: &Path,
    package: Package,
    new_version: semver::Version,
) -> Result<()> {
    let changelog_path = workspace.join(package.to_string()).join("CHANGELOG.md");

    if !changelog_path.exists() {
        // No changelog exists for this package
        return Ok(());
    }

    log::info!("  Updating changelog for package: {package}");

    // Let's parse the old changelog first
    let changelog_str = fs::read_to_string(&changelog_path)
        .with_context(|| format!("Could not read {}", changelog_path.display()))?;

    let mut changelog = Changelog::parse(&changelog_str)
        .with_context(|| format!("Could not parse {}", changelog_path.display()))?;

    changelog.finalize(package, new_version, jiff::Timestamp::now());

    std::fs::write(&changelog_path, changelog.to_string())?;

    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_version_bump() {
        let test_cases = vec![
            ("0.1.0", Version::Patch, None, "0.1.1"),
            ("0.1.0", Version::Minor, None, "0.2.0"),
            ("0.1.0", Version::Major, None, "1.0.0"),
            ("0.1.0", Version::Patch, Some("alpha"), "0.1.1-alpha.0"),
            ("0.1.0", Version::Minor, Some("alpha"), "0.2.0-alpha.0"),
            ("0.1.0", Version::Major, Some("alpha"), "1.0.0-alpha.0"),
            // amount is ignored, assuming same release cycle
            ("0.1.0-beta.0", Version::Minor, None, "0.1.0"),
            ("0.1.0-beta.0", Version::Major, None, "0.1.0"),
            ("0.1.0-beta.0", Version::Minor, Some("beta"), "0.1.0-beta.1"),
            ("0.1.0-beta.0", Version::Major, Some("beta"), "0.1.0-beta.1"),
        ];

        for (version, amount, pre, expected) in test_cases {
            let new_version = do_version_bump(version, Package::EspHal, amount, pre).unwrap();
            assert_eq!(new_version.to_string(), expected);
        }
    }
}
