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

use crate::{
    Package,
    Version,
    cargo::CargoToml,
    changelog::Changelog,
    commands::PLACEHOLDER,
    find_packages,
};

/// How to bump the version of a package.
///
/// This is a direct reflection of the `bump-version` CLI arguments. The four
/// combinations of `base` and `pre` each mean something different; see
/// [`do_version_bump`] for the full state table.
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq, Eq)]
pub struct VersionBump {
    /// Bump the base `major.minor.patch` by this amount. `None` means keep
    /// the current base.
    pub base: Option<Version>,
    /// Apply or continue this pre-release identifier (e.g. `"alpha"`).
    /// `None` means a stable release.
    pub pre: Option<String>,
}

impl VersionBump {
    pub fn major() -> Self {
        Self {
            base: Some(Version::Major),
            pre: None,
        }
    }
    pub fn minor() -> Self {
        Self {
            base: Some(Version::Minor),
            pre: None,
        }
    }
    pub fn patch() -> Self {
        Self {
            base: Some(Version::Patch),
            pre: None,
        }
    }
    pub fn pre(id: impl Into<String>) -> Self {
        Self {
            base: None,
            pre: Some(id.into()),
        }
    }
    pub fn base_and_pre(base: Version, id: impl Into<String>) -> Self {
        Self {
            base: Some(base),
            pre: Some(id.into()),
        }
    }
}

/// Arguments for bumping the version of packages.
#[derive(Debug, Args)]
pub struct BumpVersionArgs {
    /// How much to bump the base version by.
    ///
    /// - Without `--pre`: a regular stable bump. If the current version is a pre-release, the
    ///   pre-release tag is dropped (finalizing the cycle) and `amount` is ignored.
    /// - With `--pre`: start (or restart) a pre-release cycle on the bumped base, e.g.
    ///   `bump-version minor --pre alpha` on `1.0.0` produces `1.1.0-alpha.0`.
    #[arg(value_enum, required_unless_present = "pre")]
    amount: Option<Version>,
    /// Pre-release identifier to apply.
    ///
    /// - Without `amount`: continue the pre-release cycle on the *current* base. If the current
    ///   version already carries this identifier the counter is incremented; if it carries a
    ///   different one the counter is reset to `<pre>.0`. Starting a pre-release cycle from a
    ///   stable version without also passing `amount` is an error, as it would produce a version
    ///   lower than the current one.
    /// - With `amount`: bump the base then append `<pre>.0`.
    #[arg(long)]
    pre: Option<String>,
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

/// Bump the version of the specified packages by the specified amount.
pub fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    let bump = VersionBump {
        base: args.amount,
        pre: args.pre,
    };

    // Bump the version for each given package:
    for package in args.packages {
        let mut package = CargoToml::new(workspace, package)?;
        update_package(&mut package, &bump, false)?;
    }

    Ok(())
}

/// Update the specified package by bumping its version, updating its changelog,
pub fn update_package(
    package: &mut CargoToml,
    version: &VersionBump,
    dry_run: bool,
) -> Result<semver::Version> {
    check_crate_before_bumping(package)?;
    let new_version = bump_crate_version(package, version, dry_run)?;
    finalize_changelog(package, &new_version, dry_run)?;
    finalize_placeholders(package, &new_version, dry_run)?;

    Ok(new_version)
}

fn check_crate_before_bumping(manifest: &mut CargoToml) -> Result<()> {
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
    bumped_package: &mut CargoToml,
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

    let examples = find_packages(&bumped_package.workspace.join("examples"))?
        .into_iter()
        .map(|p| p.join("Cargo.toml"));

    let tomls = Package::iter()
        .filter(|&p| p != Package::Examples)
        .map(|p| {
            CargoToml::new(&bumped_package.workspace, p)
                .with_context(|| format!("Could not load Cargo.toml of {p}"))
        })
        // special case, examples are stand alone projects so we must add these in a special way
        .chain(examples.into_iter().map(|p| {
            let content = fs::read_to_string(&p)
                .with_context(|| format!("Could not read {}", p.display()))?;
            CargoToml::from_str(&bumped_package.workspace, Package::Examples, &content)
                .with_context(|| format!("Could not parse Cargo.toml"))
        }));

    for dependent in tomls {
        let mut dependent = dependent?;
        if dependent.change_version_of_dependency(&package_name, &version) {
            if dry_run {
                log::info!(
                    "  Dry run: would update {} in {}: ({prev_version} -> {version})",
                    package_name,
                    dependent.package(),
                );
            } else {
                log::info!(
                    "  Bumping {package_name} version for package {}: ({prev_version} -> {version})",
                    dependent.package()
                );
                dependent.save()?;
            }
        }
    }

    Ok(version)
}

/// Bump only the base version (`major.minor.patch`).
fn bump_base(version: &mut semver::Version, amount: Version) {
    log::debug!("Bumping base version: {version} by {amount:?}");
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

/// Perform the actual version bump logic.
pub fn do_version_bump(version: &semver::Version, bump: &VersionBump) -> Result<semver::Version> {
    let mut version = version.clone();

    // The four `(base, pre)` combinations each mean something different:
    //
    // | `base` | `pre`   | Behavior                                                          |
    // |--------|---------|-------------------------------------------------------------------|
    // | None   | None    | Error — nothing to do.                                            |
    // | Some   | None    | Stable bump. If currently pre, drop the pre tag (finalize).       |
    // | Some   | Some    | Start / restart a pre-release cycle on a freshly-bumped base.     |
    // | None   | Some    | Continue a pre-release cycle on the current base. Bails if the    |
    // |        |         | current version is stable, as it would produce a lower version.   |
    match (bump.base, bump.pre.as_deref()) {
        (None, None) => bail!(
            "invalid bump request: neither an amount nor `--pre` was specified; \
             pass a bump amount (`major`/`minor`/`patch`) and/or `--pre <id>`"
        ),

        // Stable bump. If currently a pre-release, finalize by dropping the
        // pre tag and ignore `amount` (matches historical behavior).
        (Some(amount), None) => {
            if !version.pre.is_empty() {
                version.pre = Prerelease::EMPTY;
            } else {
                bump_base(&mut version, amount);
            }
        }

        // Start / restart a pre-release cycle on a freshly-bumped base.
        // e.g. 1.0.0 + (Minor, alpha) -> 1.1.0-alpha.0
        //      1.1.0-alpha.5 + (Minor, alpha) -> 1.2.0-alpha.0
        (Some(amount), Some(pre)) => {
            version.pre = Prerelease::EMPTY;
            bump_base(&mut version, amount);
            version.pre = Prerelease::new(&format!("{pre}.0"))
                .with_context(|| format!("invalid pre-release identifier {pre:?}"))?;
        }

        // Pre-release work on the *current* base.
        (None, Some(pre)) => {
            if version.pre.is_empty() {
                bail!(
                    "cannot start pre-release {pre:?} from stable version {version}: \
                     the result would be semver-less than the current version. \
                     Pass a bump amount as well, e.g. `bump-version minor --pre {pre}`."
                );
            } else if let Some(pre_version) = version.pre.as_str().strip_prefix(&format!("{pre}."))
            {
                // Same identifier -> increment counter.
                let pre_version = pre_version.parse::<u32>().with_context(|| {
                    format!(
                        "could not parse pre-release counter from {:?}",
                        version.pre.as_str()
                    )
                })?;
                // This branch is structurally safe: the strip_prefix match above
                // requires the current version's pre to already start with "{pre}.",
                // so `pre` has already been accepted by `Prerelease::new` once.
                version.pre = Prerelease::new(&format!("{pre}.{}", pre_version + 1))
                    .with_context(|| format!("invalid pre-release identifier {pre:?}"))?;
            } else {
                // Different identifier on same base -> reset counter.
                // e.g. 1.1.0-alpha.5 + (None, beta) -> 1.1.0-beta.0
                version.pre = Prerelease::new(&format!("{pre}.0"))
                    .with_context(|| format!("invalid pre-release identifier {pre:?}"))?;
            }
        }
    }

    Ok(version)
}

fn finalize_changelog(
    bumped_package: &CargoToml,
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
    bumped_package: &CargoToml,
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
mod tests {
    use toml_edit::DocumentMut;

    use super::*;

    #[test]
    fn test_version_bump() {
        let test_cases = vec![
            // --- Stable bumps from stable versions ---
            ("0.1.0", VersionBump::patch(), "0.1.1"),
            ("0.1.0", VersionBump::minor(), "0.2.0"),
            ("0.1.0", VersionBump::major(), "1.0.0"),
            ("1.0.0", VersionBump::patch(), "1.0.1"),
            ("1.0.0", VersionBump::minor(), "1.1.0"),
            ("1.0.0", VersionBump::major(), "2.0.0"),
            // --- Finalizing a pre-release: amount is ignored, pre is dropped ---
            ("0.1.0-beta.0", VersionBump::minor(), "0.1.0"),
            ("0.1.0-beta.0", VersionBump::major(), "0.1.0"),
            ("1.1.0-alpha.5", VersionBump::minor(), "1.1.0"),
            // --- Continuing a pre-release cycle on the *same* base ---
            // Same identifier -> increment counter.
            ("0.1.0-beta.0", VersionBump::pre("beta"), "0.1.0-beta.1"),
            ("0.1.0-beta.7", VersionBump::pre("beta"), "0.1.0-beta.8"),
            ("1.1.0-alpha.0", VersionBump::pre("alpha"), "1.1.0-alpha.1"),
            // Different identifier on same base -> reset counter.
            ("0.1.0-beta.0", VersionBump::pre("rc"), "0.1.0-rc.0"),
            ("1.1.0-alpha.5", VersionBump::pre("beta"), "1.1.0-beta.0"),
            // --- Starting a new pre-release cycle on a freshly-bumped base ---
            // This is the scenario from esp-rs/esp-hal#3801.
            (
                "1.0.0",
                VersionBump::base_and_pre(Version::Minor, "alpha"),
                "1.1.0-alpha.0",
            ),
            (
                "1.0.0",
                VersionBump::base_and_pre(Version::Major, "alpha"),
                "2.0.0-alpha.0",
            ),
            (
                "1.0.0",
                VersionBump::base_and_pre(Version::Patch, "alpha"),
                "1.0.1-alpha.0",
            ),
            // Restart a cycle from an existing pre-release onto a new base.
            (
                "1.1.0-alpha.5",
                VersionBump::base_and_pre(Version::Minor, "alpha"),
                "1.2.0-alpha.0",
            ),
            (
                "1.1.0-alpha.5",
                VersionBump::base_and_pre(Version::Major, "beta"),
                "2.0.0-beta.0",
            ),
            // Starting a pre-release on a 0.x package.
            (
                "0.5.0",
                VersionBump::base_and_pre(Version::Minor, "alpha"),
                "0.6.0-alpha.0",
            ),
        ];

        for (version, amount, expected) in test_cases {
            let version = semver::Version::parse(version).unwrap();
            let new_version = do_version_bump(&version, &amount).unwrap();
            assert_eq!(
                new_version.to_string(),
                expected,
                "bump {amount:?} of {version}",
            );
        }
    }

    #[test]
    fn test_version_bump_errors() {
        let test_cases = vec![
            // Nothing to bump.
            (
                "1.0.0",
                VersionBump {
                    base: None,
                    pre: None,
                },
            ),
            // Pre-release from stable without a base bump would go backwards (#3801).
            ("1.0.0", VersionBump::pre("alpha")),
            // Invalid pre-release identifiers, both construction paths.
            (
                "1.0.0",
                VersionBump::base_and_pre(Version::Minor, "has space"),
            ),
            ("1.0.0-alpha.0", VersionBump::pre("has space")),
        ];

        for (version, bump) in test_cases {
            let version = semver::Version::parse(version).unwrap();
            assert!(
                do_version_bump(&version, &bump).is_err(),
                "expected error for bump {bump:?} of {version}",
            );
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
            workspace: PathBuf::new(),
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
