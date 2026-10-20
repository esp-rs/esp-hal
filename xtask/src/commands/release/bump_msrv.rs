use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::Args;
use regex::{Captures, Regex};
use strum::IntoEnumIterator;
use toml_edit::value;

use crate::{Package, cargo::CargoToml};

/// Arguments for bumping the MSRV.
#[derive(Debug, Args)]
pub struct BumpMsrvArgs {
    /// The MSRV to be used
    #[arg(long)]
    pub msrv: String,

    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,

    /// Don't actually change any files
    #[arg(long)]
    pub dry_run: bool,
}

/// Bump the MSRV
///
/// This will process
/// - `Cargo.toml` for the packages (adjust (or add if not present) the "rust-version")
/// - `README.md` for the packages if it exists (adjusts the MSRV badge)
///
/// CI needs no changes: `.github/actions/setup-toolchains` reads the MSRV from
/// esp-hal's "rust-version".
///
/// Non-published packages are not touched.
///
/// If it detects a package which other packages in the repo depend on it will
/// also apply the changes there. (Can be disabled)
pub fn bump_msrv(workspace: &Path, args: BumpMsrvArgs) -> Result<()> {
    log::debug!("Bumping MSRV...");
    let new_msrv = semver::Version::parse(&args.msrv)
        .with_context(|| format!("MSRV parsing with arguments {args:?} failed!"))?;
    if !new_msrv.pre.is_empty() || !new_msrv.build.is_empty() {
        bail!("Invalid MSRV: {}", args.msrv);
    }

    let mut to_process = args.packages.clone();

    // add crates which depend on any of the packages to bump
    add_dependent_crates(workspace, &mut to_process)?;

    // don't process crates which are not published
    let to_process = {
        let mut published = Vec::new();
        for pkg in to_process
            .into_iter()
            .filter(|pkg| !pkg.contains_standalone_projects())
        {
            let cargo_toml = CargoToml::new(workspace, pkg)
                .with_context(|| format!("Could not load Cargo.toml of {pkg}"))?;
            if cargo_toml.is_published() {
                published.push(pkg);
            }
        }
        published
    };

    // process packages
    let badge_re = Regex::new(
        r"(?<prefix>https://img.shields.io/badge/MSRV-)(?<msrv>[0123456789.]*)(?<postfix>-)",
    )?;
    for package in to_process {
        println!("Processing {package}");
        let mut cargo_toml = CargoToml::new(workspace, package)?;
        let package_path = cargo_toml.package_path();

        let package_table = cargo_toml
            .manifest
            .as_table_mut()
            .get_mut("package")
            .and_then(|pkg| pkg.as_table_mut());

        if let Some(package_table) = package_table {
            let mut previous_rust_version = None;
            if let Some(rust_version) = package_table.get_mut("rust-version") {
                let rust_version = rust_version.as_str().unwrap();
                if semver::Version::parse(&rust_version)? > new_msrv {
                    bail!("Downgrading rust-version is not supported");
                }
                previous_rust_version = Some(rust_version.to_string())
            }

            package_table["rust-version"] = value(&new_msrv.to_string());
            if !args.dry_run {
                cargo_toml.save()?;
            }

            let readme_path = package_path.join("README.md");
            if readme_path.exists() {
                let readme = std::fs::read_to_string(&readme_path)?;
                let readme = badge_re.replace(&readme, |caps: &Captures| {
                    format!("{}{new_msrv}{}", &caps["prefix"], &caps["postfix"])
                });

                if !args.dry_run {
                    std::fs::write(readme_path, readme.as_bytes())?;
                }
            }

            if !args.dry_run
                && let Some(previous_rust_version) = previous_rust_version
            {
                for mention in find_mentions(&package_path, &previous_rust_version)? {
                    println!(
                        "⚠️ '{previous_rust_version}' found in file {} - might be a false positive, otherwise consider adjusting the xtask.",
                        mention.display()
                    );
                }
            }
        }
    }

    println!("\nPlease review the changes before committing.");
    Ok(())
}

/// Add all crates in the repo which depend on the given packages
fn add_dependent_crates(
    workspace: &Path,
    pkgs_to_process: &mut Vec<Package>,
) -> Result<(), anyhow::Error> {
    Ok(
        while {
            let mut added = false;

            // iterate over ALL known crates
            for package in Package::iter() {
                // Directories of standalone projects have no root Cargo.toml, so they cannot be
                // inspected as crates.
                if package.contains_standalone_projects() {
                    continue;
                }

                let mut cargo_toml = CargoToml::new(workspace, package)
                    .with_context(|| format!("Could not load Cargo.toml of {package}"))?;

                // iterate the dependencies in the repo
                for dep in cargo_toml.repo_dependencies() {
                    let dependency_should_be_processed = pkgs_to_process.contains(&dep);
                    let current_package_already_contained = pkgs_to_process.contains(&package);
                    if dependency_should_be_processed && !current_package_already_contained {
                        added = true;
                        pkgs_to_process.push(package);
                    }
                }
            }

            // break once we haven't added any more crates the to be processed list
            added
        } {},
    )
}

/// Find files in the package which mention the version string.
fn find_mentions(package_path: &Path, previous_rust_version: &str) -> Result<Vec<PathBuf>> {
    use std::ffi::OsStr;
    let disallowed_extensions = [OsStr::new("gz")];

    let mut mentions = Vec::new();

    for entry in walkdir::WalkDir::new(package_path)
        .into_iter()
        .filter_map(|entry| {
            let path = entry.unwrap().into_path();

            if !path.is_file() {
                return None;
            }

            if let Some(ext) = path.extension()
                && disallowed_extensions.contains(&ext)
            {
                return None;
            }

            if path.components().any(|c| c.as_os_str() == "target") {
                return None;
            }

            Some(path)
        })
    {
        let contents = match std::fs::read_to_string(&entry) {
            Ok(contents) => contents,
            Err(error) if error.kind() == std::io::ErrorKind::InvalidData => continue,
            Err(error) => {
                return Err(error)
                    .with_context(|| format!("Failed to read {}", entry.as_path().display()));
            }
        };
        if contents.contains(previous_rust_version) {
            mentions.push(entry);
        }
    }

    Ok(mentions)
}

#[cfg(test)]
mod tests {
    use std::io::Write as _;

    use tempfile::TempDir;

    use super::*;

    fn write(package: &TempDir, relative_path: &str, contents: impl AsRef<[u8]>) -> PathBuf {
        let path = package.path().join(relative_path);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::File::create(&path)
            .unwrap()
            .write_all(contents.as_ref())
            .unwrap();
        path
    }

    #[test]
    fn mentions_of_the_previous_version_are_reported() {
        let package = TempDir::new().unwrap();

        let manifest = write(&package, "Cargo.toml", "rust-version = \"1.95.0\"\n");
        write(&package, "src/lib.rs", "//! Needs Rust 1.94.0 or newer.\n");

        let mentions = find_mentions(package.path(), "1.95.0").unwrap();

        assert_eq!(mentions, vec![manifest]);
    }

    #[test]
    fn files_which_are_not_utf8_are_skipped() {
        let package = TempDir::new().unwrap();

        write(&package, "testdata/firmware.bin", [0xE0, 0x80, 0x80, 0xFF]);
        let readme = write(&package, "README.md", "MSRV-1.95.0-blue\n");

        let mentions = find_mentions(package.path(), "1.95.0").unwrap();

        assert_eq!(mentions, vec![readme]);
    }

    #[test]
    fn files_with_a_binary_extension_are_skipped() {
        let package = TempDir::new().unwrap();

        // Valid UTF-8, so only the extension can rule this file out.
        write(&package, "api-baseline/esp32.json.gz", "1.95.0\n");

        let mentions = find_mentions(package.path(), "1.95.0").unwrap();

        assert!(mentions.is_empty(), "{mentions:?}");
    }

    #[test]
    fn the_target_directory_is_skipped() {
        let package = TempDir::new().unwrap();

        write(&package, "target/debug/build.log", "1.95.0\n");

        let mentions = find_mentions(package.path(), "1.95.0").unwrap();

        assert!(mentions.is_empty(), "{mentions:?}");
    }

    #[test]
    fn dependent_crates_can_be_collected_from_the_real_workspace() {
        let workspace = crate::repo_root_for_tests();

        let mut packages = vec![Package::EspHal];
        add_dependent_crates(&workspace, &mut packages).unwrap();

        assert!(
            packages.len() > 1,
            "esp-hal should pull in at least one dependent crate, got {packages:?}"
        );
        assert!(
            !packages.iter().any(Package::contains_standalone_projects),
            "standalone-project collections must not be treated as crates: {packages:?}"
        );
    }
}
