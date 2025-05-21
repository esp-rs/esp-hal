use std::{path::Path, process::Command};

use anyhow::{Context, Result, bail, ensure};
use clap::Args;

use crate::{
    cargo::CargoToml,
    commands::{release::plan::Plan, update_package},
};

#[derive(Debug, Args)]
pub struct ApplyPlanArgs {
    /// Only make code changes, do not commit or push.
    #[arg(long)]
    dry_run: bool,
}

pub fn apply_plan(workspace: &Path, args: ApplyPlanArgs) -> Result<()> {
    ensure_workspace_clean(workspace)?;

    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let plan_source = std::fs::read_to_string(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

    if plan_source.lines().any(|line| line.starts_with("//")) {
        bail!(
            "The release plan has not been finalized. Please open the plan and follow the instructions in it."
        );
    }

    let plan = serde_json::from_str::<Plan>(&plan_source)
        .with_context(|| format!("Failed to parse release plan from {}", plan_path.display()))?;

    // Make code changes
    for step in plan.packages.iter() {
        let mut package = CargoToml::new(workspace, step.package)?;

        if package.package_version() != step.current_version {
            if package.package_version() == step.new_version {
                println!(
                    "Package {} is already at version {}. Skipping.",
                    step.package, step.new_version
                );
                continue;
            }
            bail!(
                "The version of package {} has changed in an unexpected way. Cannot continue.",
                step.package
            );
        }

        update_package(&mut package, &step.bump)?;
    }

    make_git_changes(workspace, args.dry_run, &plan_source, &plan)?;

    Ok(())
}

fn ensure_workspace_clean(workspace: &Path) -> Result<()> {
    std::env::set_current_dir(workspace)
        .with_context(|| format!("Failed to change directory to {}", workspace.display()))?;

    let status = Command::new("git")
        .arg("status")
        .arg("--porcelain")
        .output()
        .context("Failed to check git status")?;

    ensure!(
        String::from_utf8_lossy(&status.stdout).trim().is_empty(),
        "The workspace is not clean. Please commit or stash your changes before running this command."
    );

    Ok(())
}

fn make_git_changes(
    _workspace: &Path,
    dry_run: bool,
    release_plan_str: &str,
    release_plan: &Plan,
) -> Result<()> {
    // Find an available branch name
    let branch_name = "release-branch"; // Placeholder for the actual branch name logic
    let branch_name = format!(
        "{}-{}",
        branch_name,
        jiff::Timestamp::now().strftime("%Y-%m-%d")
    );

    // Switch to the new branch
    if dry_run {
        println!("Dry run: would create a new branch: {}", branch_name);
    } else {
        // Create a new branch
        let status = Command::new("git")
            .arg("switch")
            .arg("-c")
            .arg(&branch_name)
            .status()
            .context("Failed to create new branch")?;

        if !status.success() {
            bail!("Failed to create new branch: {}", branch_name);
        }
    }

    // Commit the changes
    if dry_run {
        println!("Dry run: would commit changes to branch: {}", branch_name);
    } else {
        Command::new("git")
            .arg("commit")
            .arg("-am")
            .arg("Finalize crates for release")
            .status()
            .context("Failed to commit changes")?;
    }

    // Push the branch
    let url = if dry_run {
        println!("Dry run: would push branch: {}", branch_name);
        String::from("<dry_run_url>")
    } else {
        // git push origin <branch_name>
        let message = Command::new("git")
            .arg("push")
            .arg("origin")
            .arg(&branch_name)
            .output()
            .context("Failed to push branch")?;

        if !message.status.success() {
            bail!(
                "Failed to push branch: {}",
                String::from_utf8_lossy(&message.stderr)
            );
        }

        // Extract the URL from the output
        String::from_utf8_lossy(&message.stderr) // git outputs to stderr
            .lines()
            .map(|s| s.trim_start_matches("remote:"))
            .map(|s| s.trim())
            .find(|&s| s.starts_with("https://"))
            .unwrap_or("")
            .to_string()
    };

    // Open a pull request

    let packages_to_release = release_plan
        .packages
        .iter()
        .map(|step| format!("- {}: {}", step.package, step.new_version))
        .collect::<Vec<_>>()
        .join("\n");

    let body = format!(
        r#"This pull request prepares the following packages for release:

{packages_to_release}

The release plan used for this release:

```json
{release_plan_str}
```
        "#
    );

    let pr_url_base = if url.starts_with("https://github.com/esp-rs/") {
        format!("https://github.com/esp-rs/esp-hal/compare/{branch_name}")
    } else {
        let Some(user) = url.split('/').nth(3) else {
            bail!("Failed to extract user from URL: {}", url);
        };
        format!("https://github.com/esp-rs/esp-hal/compare/main...{user}:esp-hal:{branch_name}")
    };

    let open_pr_url = format!(
        "{}?quick_pull=1&title=Prepare+release&body={}",
        pr_url_base,
        urlencoding::encode(&body)
    );

    if opener::open(&open_pr_url).is_err() {
        println!("Open the following URL to create a pull request:");
        println!("{}", open_pr_url);
    }

    println!("Once you create and merge the pull request, check out current main.");
    println!("Make sure you have the release_plan.jsonc file in the root of the workspace.");
    println!("Next, run `cargo xrelease publish` to tag the release and publish the packages.");

    Ok(())
}
