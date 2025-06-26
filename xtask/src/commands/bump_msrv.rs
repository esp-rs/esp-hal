use std::path::Path;

use anyhow::Result;
use regex::{Captures, Regex};
use strum::IntoEnumIterator;
use toml_edit::{Formatted, Item, Value};

use crate::{Package, cargo::CargoToml};

/// Bump the MSRV
///
/// This will process
/// - `Cargo.toml` for each package (adjust "rust-version")
/// - `README.md` for each package (adjusts the MSRV badge)
/// - .github/workflows/ci.yml (adapts the `MSRV: "<msrv>"` entry)
pub fn bump_msrv(workspace: &Path, msrv: &str) -> Result<()> {
    // process all packages
    let badge_re = Regex::new(
        r"(?<prefix>https://img.shields.io/badge/MSRV-)(?<msrv>[0123456789.]*)(?<postfix>-)",
    )?;
    for package in Package::iter() {
        println!("Processing {package}");
        let mut cargo_toml = CargoToml::new(workspace, package)?;
        let package_path = cargo_toml.package_path();

        let package_table = cargo_toml
            .manifest
            .as_table_mut()
            .get_mut("package")
            .and_then(|pkg| pkg.as_table_mut());

        if let Some(package_table) = package_table {
            if let Some(rust_version) = package_table.get_mut("rust-version") {
                *rust_version = Item::Value(Value::String(Formatted::new(msrv.to_string())));
            }
            cargo_toml.save()?;

            let readme_path = package_path.join("README.md");
            if readme_path.exists() {
                let readme = std::fs::read_to_string(&readme_path)?;
                let readme = badge_re.replace(&readme, |caps: &Captures| {
                    format!("{}{msrv}{}", &caps[1], &caps[3])
                });
                std::fs::write(readme_path, readme.as_bytes())?;
            }
        }
    }

    // process ".github/workflows/ci.yml"
    println!("Processing .github/workflows/ci.yml");
    let ci_yml = std::fs::read_to_string(".github/workflows/ci.yml")?;
    let ci_yml = Regex::new("(MSRV:.*\\\")([0123456789.]*)(\\\")")?
        .replace(&ci_yml, |caps: &Captures| {
            format!("{}{msrv}{}", &caps[1], &caps[3])
        });
    std::fs::write(".github/workflows/ci.yml", ci_yml.as_bytes())?;

    println!("Please review the changes before committing.");
    Ok(())
}
