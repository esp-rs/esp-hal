use std::path::Path;

use clap::{Args, Subcommand};
use esp_metadata::Chip;
use strum::IntoEnumIterator;

use crate::Package;

#[derive(Debug, Subcommand)]
pub enum SemverCheckCmd {
    GenerateBaseline,
    Check,
}

#[derive(Debug, Args)]
pub struct SemverCheckArgs {
    #[command(subcommand)]
    pub command: SemverCheckCmd,

    /// Package(s) to target.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = vec![Package::EspHal])]
    pub packages: Vec<Package>,

    /// Chip(s) to target.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    pub chips: Vec<Chip>,
}

pub fn semver_checks(workspace: &Path, args: SemverCheckArgs) -> anyhow::Result<()> {
    #[cfg(not(feature = "semver-checks"))]
    {
        let _ = workspace;
        let _ = args;

        return Err(anyhow::anyhow!(
            "Feature `semver-checks` is not enabled. Use the `xcheck` alias",
        ));
    }

    #[cfg(feature = "semver-checks")]
    match args.command {
        SemverCheckCmd::GenerateBaseline => {
            checker::generate_baseline(&workspace, args.packages, args.chips)
        }
        SemverCheckCmd::Check => {
            checker::check_for_breaking_changes(&workspace, args.packages, args.chips)
        }
    }
}

#[cfg(feature = "semver-checks")]
pub mod checker {
    use std::{
        fs,
        io::Write,
        path::{Path, PathBuf},
    };

    use cargo_semver_checks::ReleaseType;
    use esp_metadata::Chip;

    use crate::{
        Package,
        semver_check::{build_doc_json, minimum_update, remove_unstable_items},
    };

    pub fn generate_baseline(
        workspace: &Path,
        packages: Vec<Package>,
        chips: Vec<Chip>,
    ) -> Result<(), anyhow::Error> {
        for package in packages {
            log::info!("Generating API baseline for {package}");

            for chip in &chips {
                log::info!("Chip = {}", chip.to_string());
                let package_name = package.to_string();
                let package_path = crate::windows_safe_path(&workspace.join(&package_name));

                let current_path = build_doc_json(package, chip, &package_path)?;

                remove_unstable_items(&current_path)?;

                let file_name = if package.chip_features_matter() {
                    chip.to_string()
                } else {
                    "api".to_string()
                };

                let to_path = PathBuf::from(&package_path)
                    .join(format!("api-baseline/{}.json.gz", file_name));
                fs::create_dir_all(to_path.parent().unwrap())?;

                log::debug!("Compress into {current_path:?}");
                let mut encoder = flate2::write::GzEncoder::new(
                    fs::File::create(to_path)?,
                    flate2::Compression::default(),
                );
                encoder.write_all(&std::fs::read(current_path)?)?;

                if !package.chip_features_matter() {
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn min_package_update(
        workspace: &Path,
        package: Package,
        chips: &[Chip],
    ) -> anyhow::Result<ReleaseType> {
        fn stricter(a: ReleaseType, b: ReleaseType) -> ReleaseType {
            fn index_of(rt: ReleaseType) -> usize {
                match rt {
                    ReleaseType::Major => 2,
                    ReleaseType::Minor => 1,
                    ReleaseType::Patch => 0,
                    _ => unreachable!(),
                }
            }
            if index_of(b) > index_of(a) { b } else { a }
        }

        let mut highest_result = ReleaseType::Patch;
        for chip in chips {
            let result = minimum_update(workspace, package, *chip)?;

            if result == ReleaseType::Major {
                return Ok(result);
            }

            highest_result = stricter(highest_result, result);

            if !package.chip_features_matter() {
                break;
            }
        }

        Ok(highest_result)
    }

    pub fn check_for_breaking_changes(
        workspace: &Path,
        packages: Vec<Package>,
        chips: Vec<Chip>,
    ) -> anyhow::Result<()> {
        let mut semver_incompatible_packages = Vec::new();

        for package in packages {
            log::info!("Semver-check API for {package}");

            let result = min_package_update(workspace, package, &chips)?;
            log::info!("Required bump = {:?}", result);
            if result == ReleaseType::Major {
                semver_incompatible_packages.push(package.to_string());
                // no need to check other chips for this package
            }
        }

        if !semver_incompatible_packages.is_empty() {
            Err(anyhow::anyhow!(
                "Semver check failed - needs a major bump: {}",
                semver_incompatible_packages.join(", ")
            ))
        } else {
            Ok(())
        }
    }
}
