use std::{fs, process::Command};

use anyhow::{Context, Result};
use semver::Version;

use super::{PLACEHOLDER, Plan, execute_plan::make_git_changes};
use crate::commands::{VersionBump, comparison_url};

/// Perform post-release tasks such as creating migration guides for packages that have them.
pub fn post_release(workspace: &std::path::Path) -> Result<()> {
    // Read the release plan
    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let plan = Plan::from_path(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

    run_baseline_workflow(&plan)?;

    // Process packages from the plan that have migration guides
    for package_plan in plan.packages.iter() {
        let package = package_plan.package;

        if !package.has_migration_guide(workspace) {
            continue;
        }

        // Get the package's directory path
        let package_path = workspace.join(package.to_string());
        let cargo_toml_path = package_path.join("Cargo.toml");

        // Read and parse Cargo.toml
        let cargo_toml_content = fs::read_to_string(&cargo_toml_path)
            .with_context(|| format!("Failed to read from {:?}", cargo_toml_path))?;
        let cargo_toml = cargo_toml_content
            .parse::<toml_edit::DocumentMut>()
            .with_context(|| {
                format!(
                    "Failed to parse Cargo.toml at {}",
                    cargo_toml_path.display()
                )
            })?;

        // Extract version from Cargo.toml
        let version_str = cargo_toml["package"]["version"].as_str().ok_or_else(|| {
            anyhow::anyhow!(
                "Could not find version in Cargo.toml for package {:?}",
                package
            )
        })?;

        // Parse version using semver and zero out patch version
        let mut version = Version::parse(version_str)
            .with_context(|| format!("Failed to parse version {version_str:?}"))?;
        version.patch = 0;

        // Generate migration guide filename
        let migration_file_name = format!("MIGRATING-{}.md", version);
        let migration_file_path = package_path.join(&migration_file_name);

        // Create the migration guide file if it doesn't exist
        if !migration_file_path.exists() {
            // Create the title content
            let title = format!("# Migration Guide from {} to {}\n", version, PLACEHOLDER);
            fs::write(&migration_file_path, title)
                .with_context(|| format!("Failed to write to {migration_file_path:?}"))?;
            log::info!("Created migration guide: {}", migration_file_path.display());
        } else {
            log::info!(
                "Migration guide already exists: {}",
                migration_file_path.display()
            );
        }

        // Backport branch management: only for minor/major bumps on stable releases (**major** >=
        // 1), we do not have and do not maintain backport branches for `0.{minor}.x`
        // versions.
        match package_plan.bump {
            VersionBump::Minor | VersionBump::Major => {
                if version.major >= 1 {
                    let branch_name = format!("{}-{}.{}.x", package, version.major, version.minor);

                    // Check if branch already exists on origin
                    let branch_exists = Command::new("git")
                        .args([
                            "ls-remote",
                            "--exit-code",
                            "--heads",
                            "origin",
                            &branch_name,
                        ])
                        .current_dir(workspace)
                        .status()
                        .map(|status| status.success())
                        .unwrap_or(false);

                    if branch_exists {
                        log::info!(
                            "Backport branch already exists on origin: {branch_name}, skipping creation"
                        );
                    } else {
                        // We want the backport branch to start at the released tag for this
                        // package.
                        let tag_name = &package_plan.tag_name;

                        let tag_exists = Command::new("git")
                            .args(["rev-parse", "--verify", "--quiet", tag_name])
                            .current_dir(workspace)
                            .status()
                            .map(|status| status.success())
                            .unwrap_or(false);

                        let create_status = if tag_exists {
                            // Create branch from the release tag (preferred).
                            Command::new("git")
                                .arg("branch")
                                .arg(&branch_name)
                                .arg(tag_name)
                                .current_dir(workspace)
                                .status()
                        } else {
                            // Fallback: create branch from current HEAD if tag is missing for some reason.
                            log::warn!(
                                "Tag {tag_name} not found; creating backport branch {branch_name} from current HEAD"
                            );
                            Command::new("git")
                                .arg("branch")
                                .arg(&branch_name)
                                .current_dir(workspace)
                                .status()
                        }
                        .with_context(|| format!("Failed to create backport branch {branch_name}"))?;

                        if !create_status.success() {
                            return Err(anyhow::anyhow!("`git branch {}` failed", branch_name));
                        }

                        let push_status = Command::new("git")
                            .arg("push")
                            .arg("origin")
                            .arg(&branch_name)
                            .current_dir(workspace)
                            .status()
                            .with_context(|| {
                                format!("Failed to push backport branch {branch_name}")
                            })?;

                        if !push_status.success() {
                            return Err(anyhow::anyhow!(
                                "`git push origin {}` failed",
                                branch_name
                            ));
                        }

                        log::info!(
                            "Created backport branch {branch_name} for {} v{}",
                            package,
                            version_str
                        );
                    }

                    // If there was a previous minor branch (e.g. 1.1.x when creating 1.2.x),
                    // delete it on origin if it exists.
                    if version.minor > 0 {
                        let prev_branch_name =
                            format!("{}-{}.{}.x", package, version.major, version.minor - 1);

                        let prev_exists = Command::new("git")
                            .args([
                                "ls-remote",
                                "--exit-code",
                                "--heads",
                                "origin",
                                &prev_branch_name,
                            ])
                            .current_dir(workspace)
                            .status()
                            .map(|status| status.success())
                            .unwrap_or(false);

                        if prev_exists {
                            let delete_status = Command::new("git")
                                .arg("push")
                                .arg("origin")
                                .arg(format!(":{}", prev_branch_name))
                                .current_dir(workspace)
                                .status()
                                .with_context(|| {
                                    format!(
                                        "Failed to delete previous backport branch {prev_branch_name}"
                                    )
                                })?;

                            if delete_status.success() {
                                log::info!(
                                    "Deleted previous backport branch {prev_branch_name} on origin"
                                );
                            } else {
                                log::warn!(
                                    "Failed to delete previous backport branch {prev_branch_name} on origin"
                                );
                            }
                        }
                    }
                } else {
                    log::info!(
                        "Skipping backport branch creation for {} v{} (major < 1)",
                        package,
                        version_str
                    );
                }
            }
            _ => {
                log::info!(
                    "Skipping backport branch creation for {} v{} (bump is not minor/major)",
                    package,
                    version_str
                );
            }
        }
    }

    let branch = make_git_changes(false, "post-release-branch", "Post release rollover")?;

    println!("Post-release migration guides created successfully.");

    let pr_url_base = comparison_url(&plan.base, &branch.upstream, &branch.name)?;

    let open_pr_url = format!(
        "{pr_url_base}?quick_pull=1&title=Post+release+rollover&labels={labels}",
        labels = "skip-changelog",
    );

    if opener::open(&open_pr_url).is_err() {
        println!("Open the following URL to create a pull request:");
        println!("{open_pr_url}");
    }

    Ok(())
}

fn run_baseline_workflow(plan: &Plan) -> Result<()> {
    // Only include packages where semver_checked is true
    let checked_packages: Vec<(&str, &str)> = plan
        .packages
        .iter()
        .filter(|p| p.semver_checked)
        .map(|p| (p.package.as_ref(), p.tag_name.as_str()))
        .collect();

    if checked_packages.is_empty() {
        log::info!("No packages with semver_checked = true. Skipping workflow dispatch.");
        return Ok(());
    }

    // Check if GitHub CLI is available
    if Command::new("gh").arg("--version").output().is_err() {
        log::error!("GitHub CLI (gh) not available. Skipping workflow dispatch.");
        return Ok(());
    }

    log::info!("Triggering API Baseline Generation workflow...");

    for (package_name, tag_name) in checked_packages {
        println!(
            "Triggering workflow for package: {}, tag: {}",
            package_name, tag_name
        );

        // Use a main branch for `-r` so the workflow file with the expected inputs is used;
        // the target tag is passed via `tag_name` input for checkout.
        let status = Command::new("gh")
            .arg("workflow")
            .arg("run")
            .arg("api-baseline-generation.yml")
            .arg("-R")
            .arg("esp-rs/esp-hal")
            .arg("-r")
            .arg("main")
            .arg("-f")
            .arg(format!("package_name={}", package_name))
            .arg("-f")
            .arg(format!("tag_name={}", tag_name))
            .status()
            .context("Failed to run gh workflow")?;

        if !status.success() {
            anyhow::bail!(
                "gh workflow run failed for package `{}` — manual trigger required",
                package_name
            );
        }
    }

    log::info!("All workflows triggered successfully.");
    Ok(())
}
