use std::fs;

use anyhow::{Context, Result};
use semver::Version;
use super::execute_plan::make_git_changes;
use super::PLACEHOLDER;
use super::Plan;
use crate::commands::comparison_url;

pub fn post_release(workspace: &std::path::Path) -> Result<()> {
    // Read the release plan
    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let plan = Plan::from_path(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

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
        let cargo_toml_content = fs::read_to_string(&cargo_toml_path)?;
        let cargo_toml = cargo_toml_content.parse::<toml_edit::DocumentMut>()?;

        // Extract version from Cargo.toml
        let version_str = cargo_toml["package"]["version"].as_str().ok_or_else(|| {
            anyhow::anyhow!(
                "Could not find version in Cargo.toml for package {:?}",
                package
            )
        })?;

        // Parse version using semver and zero out patch version
        let mut version = Version::parse(version_str)?;
        version.patch = 0;

        // Generate migration guide filename
        let migration_file_name = format!("MIGRATING-{}.md", version);
        let migration_file_path = package_path.join(&migration_file_name);

        // Create the migration guide file if it doesn't exist
        if !migration_file_path.exists() {
            // Create the title content
            let title = format!("# Migration Guide from {} to {}\n", version, PLACEHOLDER);
            fs::write(&migration_file_path, title)?;
            log::info!("Created migration guide: {}", migration_file_path.display());
        } else {
            log::info!(
                "Migration guide already exists: {}",
                migration_file_path.display()
            );
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
