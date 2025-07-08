use std::{path::Path, process::Command};

use anyhow::{Context, Result, bail, ensure};
use clap::Args;
use esp_metadata::Chip;
use strum::IntoEnumIterator;

use crate::{
    cargo::CargoToml,
    commands::{checker::generate_baseline, release::plan::Plan, update_package},
    git::{current_branch, ensure_workspace_clean, get_remote_name_for},
};

#[derive(Debug, Args)]
pub struct ApplyPlanArgs {
    /// Actually make git changes. Without this flag, the command will only
    /// update code.
    #[arg(long)]
    no_dry_run: bool,
}

pub fn execute_plan(workspace: &Path, args: ApplyPlanArgs) -> Result<()> {
    ensure_workspace_clean(workspace)?;

    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let mut plan = Plan::from_path(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

    ensure!(
        current_branch()? == plan.base,
        "The release plan must be executed on the same branch it was created on. \
        Please switch to the {} branch and try again.",
        plan.base
    );

    // Make code changes
    for step in plan.packages.iter_mut() {
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

        let new_version = update_package(&mut package, &step.bump, !args.no_dry_run)?;

        step.tag_name = package.package.tag(&new_version);
        step.new_version = new_version;

        if step.package.is_semver_checked() {
            if args.no_dry_run {
                generate_baseline(
                    workspace,
                    vec![step.package],
                    if step.package.chip_features_matter() {
                        Chip::iter()
                            .filter(|c| step.package.validate_package_chip(c).is_ok())
                            .collect::<Vec<_>>()
                    } else {
                        vec![Chip::Esp32c6]
                    },
                )?;
            } else {
                println!(
                    "Dry run: would create semver baseline for package {}",
                    step.package
                );
            }
        }
    }

    // Update release plan file
    let plan_source = serde_json::to_string_pretty(&plan).with_context(|| {
        format!(
            "Failed to serialize release plan to {}",
            plan_path.display()
        )
    })?;

    if args.no_dry_run {
        std::fs::write(&plan_path, &plan_source)
            .with_context(|| format!("Failed to write release plan to {}", plan_path.display()))?;
    } else {
        println!(
            "Dry run: would write the updated release plan to {}",
            plan_path.display()
        );
    }

    let branch = make_git_changes(!args.no_dry_run, "release-branch", "Finalize crate releases")?;

    open_pull_request(&branch, !args.no_dry_run, &plan_source, &plan)
        .with_context(|| "Failed to open pull request")?;

    if !args.no_dry_run {
        println!(
            "Dry run completed. To make changes, run `cargo xrelease execute-plan --no-dry-run`."
        );
    }

    Ok(())
}

pub(crate) struct Branch {
    pub name: String,
    pub upstream: String,
}

pub(crate) fn make_git_changes(dry_run: bool, branch_name: &str, commit: &str) -> Result<Branch> {
    // Find an available branch name
    let branch_name = format!(
        "{branch_name}-{}",
        jiff::Timestamp::now().strftime("%Y-%m-%d"),
    );

    let upstream = get_remote_name_for("esp-rs/esp-hal")?;

    // Switch to the new branch
    if dry_run {
        println!("Dry run: would create a new branch: {branch_name}");
    } else {
        // Create a new branch
        let status = Command::new("git")
            .arg("switch")
            .arg("-c")
            .arg(&branch_name)
            .status()
            .context("Failed to create new branch")?;

        if !status.success() {
            bail!("Failed to create new branch: {branch_name}");
        }
    }

    // Commit the changes
    if dry_run {
        println!("Dry run: would commit changes to branch: {branch_name}");
    } else {
        Command::new("git").arg("add").arg(".").status().context("Failed to stage changes")?;
        Command::new("git")
            .arg("commit")
            .arg("-m")
            .arg(commit)
            .status()
            .context("Failed to commit changes")?;
    }

    // Push the branch
    let url = if dry_run {
        println!("Dry run: would push branch: {branch_name}");
        String::from("https://github.com/esp-rs/esp-hal/")
    } else {
        // git push origin <branch_name>
        let message = Command::new("git")
            .arg("push")
            .arg(&upstream)
            .arg(&branch_name)
            .output()
            .context("Failed to push branch")?;

        if !message.status.success() {
            bail!(
                "Failed to push branch: {}",
                String::from_utf8_lossy(&message.stderr)
            );
        }

        log::info!("Pushed release branch: {upstream}/{branch_name}");

        // Extract the URL from the output
        extract_url_from_push(&String::from_utf8_lossy(&message.stderr)) // git outputs to stderr
    };

    Ok(Branch {
        name: branch_name,
        upstream: url,
    })
}

fn open_pull_request(
    branch: &Branch,
    dry_run: bool,
    release_plan_str: &str,
    release_plan: &Plan,
) -> Result<()> {
    // Open a pull request

    let packages_to_release = release_plan
        .packages
        .iter()
        .map(|step| format!("- {}: {}", step.package, step.new_version))
        .collect::<Vec<_>>()
        .join("\n");

    let mut body = format!(
        r#"This pull request prepares the following packages for release:

{packages_to_release}

The release plan used for this release:

```json
{release_plan_str}
```

Please review the changes and merge them into the `{base_branch}` branch.

After merging, please make sure you have this release plan in the repo root,
then run the following command on the `{base_branch}` branch to tag and publish the packages:

```
cargo xrelease publish-plan --no-dry-run
```
"#,
        base_branch = release_plan.base,
    );

    if release_plan.base != "main" {
        body = format!(
            "⚠️ This pull request was branched off from `{current_branch}`. ⚠️\n\n{body}",
            current_branch = release_plan.base
        );
    }

    // TODO: don't forget to update the PR text once we have the `publish` command
    // updated.

    let pr_url_base = comparison_url(&release_plan.base, &branch.upstream, &branch.name)?;

    // Query string options are documented at: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/using-query-parameters-to-create-a-pull-request
    let mut open_pr_url = format!(
        "{pr_url_base}?quick_pull=1&title=Prepare+release&labels={labels}&body={body}",
        body = urlencoding::encode(&body),
        labels = "release-pr,skip-changelog",
    );

    // https://stackoverflow.com/a/64565317
    if open_pr_url.len() > 8201 {
        println!();
        println!("PR description begins here.");
        println!();
        eprintln!("{body}");
        println!();

        println!("The PR description is too long to be included in the URL.");
        println!("Please copy the above text as the PR description.");
        println!();

        open_pr_url = format!(
            "{pr_url_base}?quick_pull=1&title=Prepare+release&labels={labels}",
            labels = "release-pr,skip-changelog",
        );
    }

    if dry_run {
        println!("Dry run: would open the following URL to create a pull request:");
        println!("{open_pr_url}");
    } else {
        if opener::open(&open_pr_url).is_err() {
            println!("Open the following URL to create a pull request:");
            println!("{open_pr_url}");
        }
    }

    println!("Create the release PR and follow the next steps laid out there.");
    Ok(())
}

fn extract_url_from_push(output: &str) -> String {
    output
        .lines()
        .map(|s| s.trim_start_matches("remote:"))
        .map(|s| s.trim())
        .find(|&s| s.starts_with("https://"))
        .unwrap_or("")
        .to_string()
}

pub(crate) fn comparison_url(base: &str, url: &str, branch_name: &str) -> Result<String> {
    let url = if url.starts_with("https://github.com/esp-rs/") {
        format!("https://github.com/esp-rs/esp-hal/compare/{base}...{branch_name}")
    } else {
        let Some(user) = url.split('/').nth(3) else {
            bail!("Failed to extract user from URL: {url}");
        };
        format!("https://github.com/esp-rs/esp-hal/compare/{base}...{user}:esp-hal:{branch_name}")
    };

    Ok(url)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn extract_url() {
        let message = "Enumerating objects: 20, done.
Counting objects: 100% (20/20), done.
Delta compression using up to 16 threads
Compressing objects: 100% (14/14), done.
Writing objects: 100% (14/14), 2.61 KiB | 2.61 MiB/s, done.
Total 14 (delta 13), reused 0 (delta 0), pack-reused 0 (from 0)
remote: Resolving deltas: 100% (13/13), completed with 6 local objects.
remote: 
remote: Create a pull request for 'foo' on GitHub by visiting:
remote:      https://github.com/bugadani/esp-hal/pull/new/foo
remote:
To https://github.com/bugadani/esp-hal.git
 * [new branch]          foo -> foo
branch 'foo' set up to track 'origin/foo'.
";

        let url = extract_url_from_push(message);
        assert_eq!(url, "https://github.com/bugadani/esp-hal/pull/new/foo");
    }

    #[test]
    fn create_comparison_url() {
        let cases = [
            // From forked repo
            (
                "https://github.com/bugadani/esp-hal/pull/new/foo",
                ("main", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/main...bugadani:esp-hal:foo",
            ),
            (
                "https://github.com/bugadani/esp-hal/pull/new/foo",
                ("backport", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/backport...bugadani:esp-hal:foo",
            ),
            // From upstream
            (
                "https://github.com/esp-rs/esp-hal/pull/new/foo",
                ("main", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/main...foo",
            ),
        ];

        for (input_url, (current_branch, release_branch), expected_url) in cases {
            let url = comparison_url(current_branch, input_url, release_branch)
                .expect("Failed to create PR URL");
            assert_eq!(url, expected_url);
        }
    }
}
