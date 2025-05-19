use std::{
    fmt::Write,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use clap::Args;
use semver::Prerelease;
use strum::IntoEnumIterator;
use toml_edit::{DocumentMut, Item, TableLike, Value};

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

struct CargoToml<'a> {
    workspace: &'a Path,
    package: Package,
    manifest: toml_edit::DocumentMut,
}

const DEPENDENCY_KINDS: [&'static str; 3] =
    ["dependencies", "dev-dependencies", "build-dependencies"];

impl<'a> CargoToml<'a> {
    fn new(workspace: &'a Path, package: Package) -> Result<Self> {
        let package_path = workspace.join(package.to_string());
        let manifest_path = package_path.join("Cargo.toml");
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
            manifest: manifest.parse::<DocumentMut>()?,
        })
    }

    fn package_path(&self) -> PathBuf {
        self.workspace.join(self.package.to_string())
    }

    fn manifest_path(&self) -> PathBuf {
        self.package_path().join("Cargo.toml")
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

    fn save(&self) -> Result<()> {
        let manifest_path = self.manifest_path();
        fs::write(&manifest_path, self.manifest.to_string())
            .with_context(|| format!("Could not write {}", manifest_path.display()))?;

        Ok(())
    }

    /// Calls a callback for each table that contains dependencies.
    ///
    /// Callback arguments:
    /// - `path`: The path to the table (e.g. `dependencies.package`)
    /// - `dependency_kind`: The kind of dependency (e.g. `dependencies`,
    ///   `dev-dependencies`)
    /// - `table`: The table itself
    fn visit_dependencies(
        &mut self,
        mut handle_dependencies: impl FnMut(&str, &'static str, &mut toml_edit::Table),
    ) {
        fn recurse_dependencies(
            path: String,
            table: &mut toml_edit::Table,
            handle_dependencies: &mut impl FnMut(&str, &'static str, &mut toml_edit::Table),
        ) {
            // Walk through tables recursively so that we can find *all* dependencies.
            for (key, item) in table.iter_mut() {
                if let Item::Table(table) = item {
                    let path = if path.is_empty() {
                        key.to_string()
                    } else {
                        format!("{path}.{key}")
                    };
                    recurse_dependencies(path, table, handle_dependencies);
                }
            }
            for dependency_kind in DEPENDENCY_KINDS {
                let Some(Item::Table(table)) = table.get_mut(dependency_kind) else {
                    continue;
                };

                handle_dependencies(&path, dependency_kind, table);
            }
        }

        recurse_dependencies(
            String::new(),
            self.manifest.as_table_mut(),
            &mut handle_dependencies,
        );
    }
}

pub fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    // Bump the version by the specified amount for each given package:
    for package in args.packages {
        let mut package = CargoToml::new(workspace, package)?;
        check_crate_before_bumping(&mut package)?;
        let new_version = bump_crate_version(&mut package, args.amount, args.pre.as_deref())?;
        finalize_changelog(&package, &new_version)?;
        finalize_placeholders(&package, &new_version)?;
    }

    Ok(())
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
    bumped_package: &mut CargoToml<'_>,
    amount: Version,
    pre: Option<&str>,
) -> Result<semver::Version> {
    let prev_version = bumped_package.version().to_string();

    let version = do_version_bump(&prev_version, amount, pre)
        .with_context(|| format!("Failed to bump version of {}", bumped_package.package))?;

    bumped_package.set_version(&version);

    bumped_package.save()?;

    let package_name = bumped_package.package.to_string();
    for pkg in Package::iter() {
        let mut dependent = CargoToml::new(bumped_package.workspace, pkg)
            .with_context(|| format!("Could not load Cargo.toml of {pkg}"))?;

        let mut changed = false;

        dependent.visit_dependencies(|_, _, table| {
            // Update dependencies which specify a version:
            match &mut table[&package_name] {
                Item::Table(table) if table.contains_key("version") => {
                    table["version"] = toml_edit::value(version.to_string());
                    changed = true;
                }
                Item::Value(Value::InlineTable(table)) if table.contains_key("version") => {
                    table["version"] = version.to_string().into();
                    changed = true;
                }
                Item::None => {
                    // Maybe we have a renamed package (alias = { package = "foo" })?
                    let update_renamed_dep = table.get_values().iter().any(|(_, p)| {
                        if let Value::InlineTable(table) = p {
                            if let Some(Value::String(name)) = &table.get("package") {
                                return name.value() == &package_name;
                            }
                        }

                        false
                    });

                    if update_renamed_dep {
                        table[&package_name]["version"] = version.to_string().into();
                        changed = true;
                    }
                }
                _ => {}
            }
        });

        if changed {
            log::info!(
                "  Bumping {package_name} version for package {pkg}: ({prev_version} -> {version})"
            );

            dependent.save()?;
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

fn finalize_changelog(bumped_package: &CargoToml<'_>, new_version: &semver::Version) -> Result<()> {
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

fn finalize_placeholders(
    bumped_package: &CargoToml<'_>,
    new_version: &semver::Version,
) -> Result<()> {
    const PLACEHOLDER: &str = "{{currentVersion}}";

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
        let content = fs::read_to_string(path).unwrap();
        if content.contains(PLACEHOLDER) {
            log::info!("  Replacing placeholder in {}", path.display());
            let new_content = content.replace(PLACEHOLDER, &new_version.to_string());
            fs::write(path, new_content).unwrap();
        }
    });

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
