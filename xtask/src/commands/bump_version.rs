use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use clap::Args;
use semver::Prerelease;
use strum::IntoEnumIterator;
use toml_edit::{DocumentMut, Item, Value};

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

struct BumpedPackage<'a> {
    workspace: &'a Path,
    package: Package,
    manifest_path: PathBuf,
    manifest: toml_edit::DocumentMut,
}

impl<'a> BumpedPackage<'a> {
    fn new(workspace: &'a Path, package: Package) -> Result<Self> {
        let manifest_path = workspace.join(package.to_string()).join("Cargo.toml");
        if !manifest_path.exists() {
            bail!(
                "Could not find Cargo.toml for package {package} at {}",
                manifest_path.display()
            );
        }

        let manifest = fs::read_to_string(&manifest_path)
            .with_context(|| format!("Could not read {}", manifest_path.display()))?;

        Ok(Self {
            workspace,
            package,
            manifest_path,
            manifest: manifest.parse::<DocumentMut>()?,
        })
    }

    fn version(&self) -> &str {
        self.manifest["package"]["version"]
            .as_str()
            .unwrap()
            .trim()
            .trim_matches('"')
    }

    fn set_version(&mut self, version: &semver::Version) {
        log::info!(
            "Bumping version for package: {} ({} -> {version})",
            self.package,
            self.version(),
        );
        self.manifest["package"]["version"] = toml_edit::value(version.to_string());
    }
}

pub fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    // Bump the version by the specified amount for each given package:
    for package in args.packages {
        let mut package = BumpedPackage::new(workspace, package)?;
        let new_version = bump_crate_version(&mut package, args.amount, args.pre.as_deref())?;
        finalize_changelog(&package, new_version)?;
    }

    Ok(())
}

/// Bump the version of the specified package by the specified amount.
fn bump_crate_version(
    bumped_package: &mut BumpedPackage<'_>,
    amount: Version,
    pre: Option<&str>,
) -> Result<semver::Version> {
    let prev_version = bumped_package.version().to_string();

    let version = do_version_bump(&prev_version, amount, pre)
        .with_context(|| format!("Failed to bump version of {}", bumped_package.package))?;

    bumped_package.set_version(&version);

    fs::write(
        &bumped_package.manifest_path,
        bumped_package.manifest.to_string(),
    )?;

    let dependency_kinds = ["dependencies", "dev-dependencies", "build-dependencies"];

    let package_name = bumped_package.package.to_string();
    for pkg in Package::iter() {
        let manifest_path = bumped_package
            .workspace
            .join(pkg.to_string())
            .join("Cargo.toml");
        let manifest = fs::read_to_string(&manifest_path)
            .with_context(|| format!("Could not read {}", manifest_path.display()))?;

        let mut manifest = manifest.parse::<toml_edit::DocumentMut>()?;

        let mut changed = false;
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

fn do_version_bump(version: &str, amount: Version, pre: Option<&str>) -> Result<semver::Version> {
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
                "Unexpected pre-release version format found: {}",
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
    bumped_package: &BumpedPackage<'_>,
    new_version: semver::Version,
) -> Result<()> {
    let changelog_path = bumped_package
        .workspace
        .join(bumped_package.package.to_string())
        .join("CHANGELOG.md");

    if !changelog_path.exists() {
        // No changelog exists for this package
        return Ok(());
    }

    log::info!(
        "  Updating changelog for package: {}",
        bumped_package.package
    );

    // Let's parse the old changelog first
    let changelog_str = fs::read_to_string(&changelog_path)
        .with_context(|| format!("Could not read {}", changelog_path.display()))?;

    let mut changelog = Changelog::parse(&changelog_str)
        .with_context(|| format!("Could not parse {}", changelog_path.display()))?;

    changelog.finalize(bumped_package.package, new_version, jiff::Timestamp::now());

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
            let new_version = do_version_bump(version, amount, pre).unwrap();
            assert_eq!(new_version.to_string(), expected);
        }
    }
}
