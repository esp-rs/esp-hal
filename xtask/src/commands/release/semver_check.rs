use std::path::Path;

use clap::{Args, Subcommand};
use esp_metadata::Chip;
use strum::IntoEnumIterator;

use crate::Package;

/// Commands for performing semver checks on the public API of packages.
#[derive(Debug, Subcommand)]
pub enum SemverCheckCmd {
    GenerateBaseline,
    Check,
    DownloadBaselines,
}

/// Arguments for performing semver checks on the public API of packages.
#[derive(Debug, Args)]
pub struct SemverCheckArgs {
    /// The semver check command to run.
    #[command(subcommand)]
    pub command: SemverCheckCmd,

    /// Package(s) to target.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = vec![Package::EspHal])]
    pub packages: Vec<Package>,

    /// Chip(s) to target.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    pub chips: Vec<Chip>,
}

/// Perform semver checks on the public API of packages.
pub fn semver_checks(workspace: &Path, args: SemverCheckArgs) -> anyhow::Result<()> {
    #[cfg(not(feature = "semver-checks"))]
    {
        let _ = workspace;
        let _ = args;

        Err(anyhow::anyhow!(
            "Feature `semver-checks` is not enabled. Use the `xcheck` alias",
        ))
    }

    #[cfg(feature = "semver-checks")]
    match args.command {
        SemverCheckCmd::GenerateBaseline => {
            checker::generate_baseline(&workspace, args.packages, args.chips)
        }
        SemverCheckCmd::Check => {
            checker::check_for_breaking_changes(&workspace, args.packages, args.chips)
        }
        SemverCheckCmd::DownloadBaselines => checker::download_baselines(&workspace, args.packages),
    }
}

/// Module containing functions for performing semver checks on the public API of packages.
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
        semver_check::{build_doc_json, minimum_update},
    };

    /// Generate the API baselines for the specified packages and chips.
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

    /// Determine the minimum required version bump for the specified package and chips.
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

    /// Check for breaking changes in the specified packages and chips.
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

    #[derive(Debug)]
    enum BaselineSource {
        Artifact(String), // GitHub Actions artifact name
    }

    /// Download API baselines from GitHub Actions artifacts for the specified packages.
    /// Fails with helpful error message if baselines are not available.
    pub fn download_baselines(workspace: &Path, packages: Vec<Package>) -> anyhow::Result<()> {
        for package in packages {
            log::info!("Downloading API baselines for {package}");

            let package_name = package.to_string();
            let package_path = crate::windows_safe_path(&workspace.join(&package_name));
            let baseline_dir = package_path.join("api-baseline");

            // Remove existing baselines
            if baseline_dir.exists() {
                std::fs::remove_dir_all(&baseline_dir)?;
            }

            // Try to download from GitHub Actions artifacts
            // Note: Artifacts have a 90-day retention limit for public repositories
            let baseline_sources = vec![BaselineSource::Artifact("api-baselines".to_string())];

            let mut downloaded = false;
            for baseline_source in baseline_sources {
                match &baseline_source {
                    BaselineSource::Artifact(artifact_name) => {
                        log::info!(
                            "Attempting to download from GitHub Actions artifact: {artifact_name}"
                        );
                        let repo = std::env::var("GITHUB_REPOSITORY")
                            .unwrap_or_else(|_| "esp-rs/esp-hal".to_string());
                        if try_download_from_artifact(&package_path, artifact_name, &repo)? {
                            log::info!(
                                "Successfully downloaded baselines from artifact {artifact_name}"
                            );
                            downloaded = true;
                            break;
                        }
                    }
                }
            }

            if !downloaded {
                return Err(anyhow::anyhow!("No API baselines found for {package}!"));
            }

            // Verify that baselines were extracted correctly
            if !baseline_dir.exists() {
                return Err(anyhow::anyhow!(
                    "Baseline directory not found after extraction: {}",
                    baseline_dir.display()
                ));
            }

            let baseline_files: Vec<_> = std::fs::read_dir(&baseline_dir)?
                .filter_map(|entry| entry.ok())
                .filter(|entry| {
                    entry
                        .path()
                        .extension()
                        .and_then(|ext| ext.to_str())
                        .map(|ext| ext == "gz")
                        .unwrap_or(false)
                })
                .collect();

            if baseline_files.is_empty() {
                return Err(anyhow::anyhow!(
                    "No baseline files found in extracted directory: {}",
                    baseline_dir.display()
                ));
            }

            log::info!(
                "Found {} baseline files for {package}",
                baseline_files.len()
            );
        }

        Ok(())
    }

    /// Try to download baselines from a GitHub Actions artifact
    fn try_download_from_artifact(
        package_path: &PathBuf,
        artifact_name: &str,
        repo: &str,
    ) -> anyhow::Result<bool> {
        use std::process::Command;

        // Check if GitHub CLI is available
        let gh_check = Command::new("gh").arg("--version").output();
        if gh_check.is_err() {
            log::debug!("GitHub CLI (gh) not available, skipping artifact download");
            return Ok(false);
        }

        // Check if we're in a GitHub Actions environment (or allow override for testing)
        if std::env::var("GITHUB_ACTIONS").is_err() && std::env::var("GITHUB_REPOSITORY").is_err() {
            log::debug!(
                "Not in GitHub Actions environment and no GITHUB_REPOSITORY set, skipping artifact download"
            );
            return Ok(false);
        }

        // Query repository artifacts via GitHub API and download the one matching `artifact_name`
        log::debug!("Attempting to download artifact: {artifact_name} from {repo}");

        // List artifacts for the repository, filtered by name
        let list_output = Command::new("gh")
            .args([
                "api",
                "-H",
                "Accept: application/vnd.github+json",
                "-H",
                "X-GitHub-Api-Version: 2022-11-28",
                &format!(
                    "/repos/{}/actions/artifacts?name={}",
                    repo, artifact_name
                ),
            ])
            .output();

        let artifacts: Vec<serde_json::Value> = match list_output {
            Ok(result) if result.status.success() => {
                let body = String::from_utf8_lossy(&result.stdout);
                log::debug!("Artifacts list response: {}", body);
                if let Ok(json) = serde_json::from_str::<serde_json::Value>(&body) {
                    match json.get("artifacts").and_then(|a| a.as_array()) {
                        Some(arr) => arr.clone(),
                        None => vec![],
                    }
                } else {
                    vec![]
                }
            }
            Ok(result) => {
                let stderr = String::from_utf8_lossy(&result.stderr);
                log::debug!("Failed to list artifacts. stderr: {}", stderr);
                vec![]
            }
            Err(e) => {
                log::debug!("Error invoking gh api to list artifacts: {}", e);
                vec![]
            }
        };

        if artifacts.is_empty() {
            log::debug!("No artifacts found for name {}", artifact_name);
            return Ok(false);
        }

        // Try each matching artifact until download and extraction succeeds
        for artifact in artifacts {
            let name_matches = artifact
                .get("name")
                .and_then(|v| v.as_str())
                .map(|n| n == artifact_name)
                .unwrap_or(false);
            if !name_matches {
                continue;
            }

            let expired = artifact
                .get("expired")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            if expired {
                log::debug!("Skipping expired artifact: {}", artifact_name);
                continue;
            }

            let id = match artifact.get("id").and_then(|v| v.as_u64()) {
                Some(id) => id.to_string(),
                None => continue,
            };

            log::debug!("Downloading artifact id {} (name: {})", id, artifact_name);

            // Download the artifact ZIP (API returns a redirect to the ZIP)
            let download = Command::new("gh")
                .args([
                    "api",
                    "-H",
                    "Accept: application/vnd.github+json",
                    "-H",
                    "X-GitHub-Api-Version: 2022-11-28",
                    &format!("/repos/{}/actions/artifacts/{}/zip", repo, id),
                ])
                .output();

            let Ok(result) = download else {
                log::debug!("Error invoking gh api to download artifact id {}", id);
                continue;
            };

            if !result.status.success() {
                let stderr = String::from_utf8_lossy(&result.stderr);
                log::debug!(
                    "Failed to download artifact id {} (name: {}). stderr: {}",
                    id, artifact_name, stderr
                );
                continue;
            }

            // Write ZIP to a temporary file
            let zip_path = package_path.join("artifact.zip");
            if let Err(e) = std::fs::write(&zip_path, &result.stdout) {
                log::debug!("Failed to write artifact ZIP to {}: {}", zip_path.display(), e);
                continue;
            }

            // Extract ZIP into the package directory
            let unzip = Command::new("unzip")
                .args(["-o", zip_path.to_str().unwrap(), "-d", package_path.to_str().unwrap()])
                .output();

            match unzip {
                Ok(unzip_result) if unzip_result.status.success() => {
                    // Cleanup ZIP file
                    let _ = std::fs::remove_file(&zip_path);

                    // Move extracted baseline files into `api-baseline` directory
                    let baseline_dir = package_path.join("api-baseline");
                    std::fs::create_dir_all(&baseline_dir)?;

                    if let Ok(entries) = std::fs::read_dir(package_path) {
                        for entry in entries.flatten() {
                            let path = entry.path();
                            if path.extension().and_then(|s| s.to_str()) == Some("gz")
                                && path
                                    .file_stem()
                                    .and_then(|s| s.to_str())
                                    .map(|s| s.ends_with(".json"))
                                    .unwrap_or(false)
                            {
                                let filename = path.file_name().unwrap();
                                let dest = baseline_dir.join(filename);
                                if let Err(e) = std::fs::rename(&path, &dest) {
                                    log::warn!(
                                        "Failed to move {} to baseline directory: {}",
                                        path.display(),
                                        e
                                    );
                                } else {
                                    log::debug!("Moved {} to {}", path.display(), dest.display());
                                }
                            }
                        }
                    }

                    return Ok(true);
                }
                Ok(unzip_result) => {
                    let stderr = String::from_utf8_lossy(&unzip_result.stderr);
                    log::debug!("Failed to unzip artifact ZIP: {}", stderr);
                    let _ = std::fs::remove_file(&zip_path);
                }
                Err(e) => {
                    log::debug!("Error invoking unzip: {}", e);
                    let _ = std::fs::remove_file(&zip_path);
                }
            }
        }

        log::debug!("No downloadable artifacts found for {}", artifact_name);
        Ok(false)
    }
}
