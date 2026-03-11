use std::path::Path;

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
/// - IF the esp-hal package was touched: .github/workflows/ci.yml (adapts the `MSRV: "<msrv>"`
///   entry)
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
    let to_process: Vec<Package> = to_process
        .iter()
        .filter(|pkg| {
            let cargo_toml = CargoToml::new(workspace, **pkg).unwrap();
            cargo_toml.is_published()
        })
        .copied()
        .collect();

    let adjust_ci = to_process.contains(&Package::EspHal);

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

            if !args.dry_run {
                if let Some(previous_rust_version) = previous_rust_version {
                    check_mentions(&package_path, &previous_rust_version)?;
                }
            }
        }
    }

    if adjust_ci {
        // process ".github/workflows/ci.yml"
        println!("Processing .github/workflows/ci.yml");
        let ci_yml_path = workspace.join(".github/workflows/ci.yml");

        let ci_yml = std::fs::read_to_string(&ci_yml_path)?;
        let ci_yml = Regex::new("(MSRV:.*\\\")([0123456789.]*)(\\\")")?
            .replace(&ci_yml, |caps: &Captures| {
                format!("{}{new_msrv}{}", &caps[1], &caps[3])
            });
        if !args.dry_run {
            std::fs::write(ci_yml_path, ci_yml.as_bytes())?;
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
                let mut cargo_toml =
                    CargoToml::new(workspace, package.clone()).with_context(|| {
                        format!(
                            "Creating Cargo.toml in workspace {} for {package} failed!",
                            workspace.display()
                        )
                    })?;

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

/// Check files in the package and show if we find the version string in any
/// file. Most probably it will report false positives but maybe not.
fn check_mentions(package_path: &std::path::PathBuf, previous_rust_version: &str) -> Result<()> {
    use std::ffi::OsStr;
    let disallowed_extensions = [OsStr::new("gz")];
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
        let contents = std::fs::read_to_string(&entry)
            .with_context(|| format!("Failed to read {}", entry.as_path().display()))?;
        if contents.contains(previous_rust_version) {
            println!(
                "⚠️ '{previous_rust_version}' found in file {} - might be a false positive, otherwise consider adjusting the xtask.",
                entry.display()
            );
        }
    }

    Ok(())
}
