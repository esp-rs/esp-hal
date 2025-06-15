use std::{
    fmt::Write,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use clap::Args;
use semver::Prerelease;
use serde::{Deserialize, Serialize};
use strum::IntoEnumIterator;
use toml_edit::{Item, TableLike, Value};

use crate::{cargo::CargoToml, changelog::Changelog, commands::PLACEHOLDER, Package, Version};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub enum VersionBump {
    PreRelease(String),
    Patch,
    Minor,
    Major,
}

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
        let version = if let Some(pre) = &args.pre {
            VersionBump::PreRelease(pre.clone())
        } else {
            match args.amount {
                Version::Major => VersionBump::Major,
                Version::Minor => VersionBump::Minor,
                Version::Patch => VersionBump::Patch,
            }
        };
        let mut package = CargoToml::new(workspace, package)?;
        update_package(&mut package, &version, false)?;
    }

    Ok(())
}

pub fn update_package(
    package: &mut CargoToml<'_>,
    version: &VersionBump,
    dry_run: bool,
) -> Result<semver::Version> {
    check_crate_before_bumping(package)?;
    let new_version = bump_crate_version(package, version, dry_run)?;
    finalize_changelog(package, &new_version, dry_run)?;
    finalize_placeholders(package, &new_version, dry_run)?;

    Ok(new_version)
}

fn check_crate_before_bumping(manifest: &mut CargoToml<'_>) -> Result<()> {
    // Collect errors into a vector to preserve order.
    let mut errors = Vec::new();

    manifest.visit_dependencies(|path, dep_kind, table| {
        // Display the section in case we have a more complex path to the dependency
        // (like [target.'cfg(target_arch = "riscv32")'.dependencies])
        let path = if path.is_empty() {
            String::new()
        } else {
            format!("[{path}.{dep_kind}] ")
        };

        for (name, item) in table.iter() {
            let Err(e) = check_dependency_before_bumping(item) else {
                // Dependency is well-formed
                continue;
            };

            let position = match errors.iter().position(|(k, _)| *k == dep_kind) {
                Some(position) => position,
                None => {
                    errors.push((dep_kind, vec![]));
                    errors.len() - 1
                }
            };
            errors[position].1.push((format!("{path}{name}"), e));
        }
    });

    // We've processed the dependencies, let's assemble the error message:
    if !errors.is_empty() {
        let mut error_message = String::new();
        for (dep_kind, errors) in errors {
            if !error_message.is_empty() {
                error_message.push_str("\n");
            }
            writeln!(&mut error_message, "In [{dep_kind}]:").unwrap();
            for (krate, error) in errors {
                writeln!(&mut error_message, "- {krate}: {error}").unwrap();
            }
        }
        bail!(
            "The following errors were found in the dependencies of {}:\n{error_message}",
            manifest.package
        );
    }

    Ok(())
}

fn check_dependency_before_bumping(item: &Item) -> Result<()> {
    fn validate_simple_version(version: &str) -> Result<()> {
        if version == "*" {
            bail!("Dependency specifies a wildcard version.");
        }
        Ok(())
    }

    fn check_for_non_version_deps<'a, T: TableLike>(dependency: &T) -> Result<()> {
        if let Some(version) = dependency.get("version") {
            if let Some(version) = version.as_str() {
                validate_simple_version(version)?;
            }
        } else {
            bail!("Dependency does not specify a version.");
        }
        if dependency.contains_key("git") {
            // These are not allowed because usually they indicate that the dependency
            // (usually a PAC) needs to be released. `cargo publish` would still
            // succeed, but the end result would not be what we want.
            bail!("Dependency specifies a git dependency.");
        }
        Ok(())
    }

    match item {
        Item::Value(Value::String(version)) => {
            // package = "version"
            validate_simple_version(version.value())?;
        }
        Item::Value(Value::InlineTable(table)) => {
            // package = { version = "version" }
            check_for_non_version_deps(table)?
        }
        Item::Table(table) => {
            // [dependency.package]
            // version = "version"
            check_for_non_version_deps(table)?
        }
        other => unreachable!("{other:#?}"),
    };

    Ok(())
}

/// Bump the version of the specified package by the specified amount.
fn bump_crate_version(
    bumped_package: &mut CargoToml<'_>,
    amount: &VersionBump,
    dry_run: bool,
) -> Result<semver::Version> {
    let prev_version = bumped_package.package_version();

    let version = do_version_bump(&prev_version, amount)
        .with_context(|| format!("Failed to bump version of {}", bumped_package.package))?;

    if dry_run {
        log::info!(
            "Dry run: would bump {} version to {version}",
            bumped_package.package,
        );
    } else {
        log::info!("Update {} to {version}", bumped_package.package);
        bumped_package.set_version(&version);
        bumped_package.save()?;
    }

    let package_name = bumped_package.package.to_string();
    for pkg in Package::iter() {
        let mut dependent = CargoToml::new(bumped_package.workspace, pkg)
            .with_context(|| format!("Could not load Cargo.toml of {pkg}"))?;

        if dependent.change_version_of_dependency(&package_name, &version) {
            if dry_run {
                log::info!(
                    "  Dry run: would update {} in {}: ({prev_version} -> {version})",
                    package_name,
                    pkg,
                );
            } else {
                log::info!(
                    "  Bumping {package_name} version for package {pkg}: ({prev_version} -> {version})"
                );
                dependent.save()?;
            }
        }
    }

    Ok(version)
}

pub fn do_version_bump(version: &semver::Version, amount: &VersionBump) -> Result<semver::Version> {
    fn bump_version_number(version: &mut semver::Version, amount: &VersionBump) {
        match amount {
            VersionBump::Major => {
                version.major += 1;
                version.minor = 0;
                version.patch = 0;
            }
            VersionBump::Minor => {
                version.minor += 1;
                version.patch = 0;
            }
            VersionBump::Patch => {
                version.patch += 1;
            }
            VersionBump::PreRelease(_) => unreachable!(),
        }
    }
    let mut version = version.clone();

    match amount {
        VersionBump::Major | VersionBump::Minor | VersionBump::Patch => {
            // If the version is a pre-release, we need to remove it
            if !version.pre.is_empty() {
                version.pre = Prerelease::EMPTY;
            } else {
                // If the version is not a pre-release, we need to bump the version
                bump_version_number(&mut version, &amount);
            }
        }
        VersionBump::PreRelease(pre) => {
            if let Some(pre_version) = version.pre.as_str().strip_prefix(&format!("{pre}.")) {
                let pre_version = pre_version.parse::<u32>()?;
                version.pre = Prerelease::new(&format!("{pre}.{}", pre_version + 1)).unwrap();
            } else if version.pre.as_str().is_empty() {
                // Start a new pre-release
                bump_version_number(&mut version, &amount);
                version.pre = Prerelease::new(&format!("{pre}.0")).unwrap();
            } else {
                bail!(
                    "Unexpected pre-release version format found: {}",
                    version.pre.as_str()
                );
            }
        }
    }

    Ok(version)
}

fn finalize_changelog(
    bumped_package: &CargoToml<'_>,
    new_version: &semver::Version,
    dry_run: bool,
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

    if dry_run {
        log::info!("  Dry run: would update {}", changelog_path.display());
    } else {
        changelog.finalize(bumped_package.package, new_version, jiff::Timestamp::now());

        std::fs::write(&changelog_path, changelog.to_string())?;
    }

    Ok(())
}

fn finalize_placeholders(
    bumped_package: &CargoToml<'_>,
    new_version: &semver::Version,
    dry_run: bool,
) -> Result<()> {
    let skip_paths = [bumped_package.package_path().join("target")];

    fn walk_dir(dir: &Path, skip_paths: &[PathBuf], callback: &mut impl FnMut(&Path)) {
        for entry in dir.read_dir().unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();
            if skip_paths.contains(&path) {
                continue;
            }
            if path.is_dir() {
                walk_dir(&path, skip_paths, callback);
            } else if path.is_file() {
                callback(&path);
            }
        }
    }

    walk_dir(&bumped_package.package_path(), &skip_paths, &mut |path| {
        let content = match fs::read_to_string(path) {
            Ok(content) => content,
            Err(e) => {
                log::debug!("  Could not read {}: {}", path.display(), e);
                return;
            }
        };
        if content.contains(PLACEHOLDER) {
            if dry_run {
                log::info!("  Would replace version placeholders in {}", path.display());
            } else {
                log::info!("  Replacing placeholders in {}", path.display());
                let new_content = content.replace(PLACEHOLDER, &new_version.to_string());
                fs::write(path, new_content).unwrap();
            }
        }
    });

    Ok(())
}

#[cfg(test)]
mod test {
    use toml_edit::DocumentMut;

    use super::*;

    #[test]
    fn test_version_bump() {
        let test_cases = vec![
            ("0.1.0", VersionBump::Patch, "0.1.1"),
            ("0.1.0", VersionBump::Minor, "0.2.0"),
            ("0.1.0", VersionBump::Major, "1.0.0"),
            (
                "0.1.0",
                VersionBump::PreRelease("alpha".to_string()),
                "0.1.1-alpha.0",
            ),
            (
                "0.1.0",
                VersionBump::PreRelease("alpha".to_string()),
                "0.2.0-alpha.0",
            ),
            (
                "0.1.0",
                VersionBump::PreRelease("alpha".to_string()),
                "1.0.0-alpha.0",
            ),
            // amount is ignored, assuming same release cycle
            ("0.1.0-beta.0", VersionBump::Minor, "0.1.0"),
            ("0.1.0-beta.0", VersionBump::Major, "0.1.0"),
            (
                "0.1.0-beta.0",
                VersionBump::PreRelease("beta".to_string()),
                "0.1.0-beta.1",
            ),
            (
                "0.1.0-beta.0",
                VersionBump::PreRelease("beta".to_string()),
                "0.1.0-beta.1",
            ),
        ];

        for (version, amount, expected) in test_cases {
            let version = semver::Version::parse(version).unwrap();
            let new_version = do_version_bump(&version, &amount).unwrap();
            assert_eq!(new_version.to_string(), expected);
        }
    }

    #[test]
    fn test_rejected_dependencies() {
        let toml = r#"
            [dependencies]
            package = { version = "*" }
            package2 = "*"

            [dev-dependencies]
            package3 = { version = "1.0", git = "" }

            [target.'cfg(target_arch = "riscv32")'.dependencies]
            package4 = {  }
            "#;

        let mut doc = CargoToml {
            manifest: toml.parse::<DocumentMut>().unwrap(),
            package: Package::EspHal,
            workspace: Path::new(""),
        };
        let errors = check_crate_before_bumping(&mut doc);
        pretty_assertions::assert_eq!(
            errors.unwrap_err().to_string(),
            r#"The following errors were found in the dependencies of esp-hal:
In [dependencies]:
- [target.'cfg(target_arch = "riscv32")'.dependencies] package4: Dependency does not specify a version.
- package: Dependency specifies a wildcard version.
- package2: Dependency specifies a wildcard version.

In [dev-dependencies]:
- package3: Dependency specifies a git dependency.
"#
        );
    }
}
