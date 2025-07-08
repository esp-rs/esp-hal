use std::{path::Path, process::Command};

use anyhow::{bail, ensure, Context, Result};
use clap::Args;

use crate::{
    cargo::{CargoArgsBuilder, CargoToml},
    commands::Plan,
    git::{current_branch, ensure_workspace_clean, get_remote_name_for},
};

#[derive(Debug, Args)]
pub struct PublishPlanArgs {
    /// Do not pass the `--dry-run` argument, actually try to publish.
    #[arg(long)]
    no_dry_run: bool,
}

pub fn publish_plan(workspace: &Path, args: PublishPlanArgs) -> Result<()> {
    ensure_workspace_clean(workspace)?;

    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let plan = Plan::from_path(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

    // Check that the publish command is being run on the right branch. This is
    // meant to prevent publishing from the release branch before the changes
    // are reviewed and merged.
    ensure!(
        current_branch()? == plan.base,
        "The packages must be published from the same branch the plan was created on. \
        Please switch to the {} branch and try again.",
        plan.base
    );

    let tomls = plan
        .packages
        .iter()
        .map(|step| {
            CargoToml::new(workspace, step.package)
                .with_context(|| format!("Failed to read Cargo.toml for {}", step.package))
        })
        .collect::<Result<Vec<_>>>()?;

    // Check that all packages are updated and ready to go. This is meant to prevent
    // publishing unupdated packages.
    for (step, toml) in plan.packages.iter().zip(tomls.iter()) {
        if toml.package_version() != step.new_version {
            if toml.package_version() == step.current_version {
                bail!(
                    "Package {} has not been updated yet. Run `cargo xrelease execute-plan` before publishing the packages.",
                    step.package
                );
            }
            bail!(
                "The version of package {} has changed in an unexpected way. Cannot continue.",
                step.package
            );
        }
    }

    // Actually publish the packages.
    for (step, toml) in plan.packages.iter().zip(tomls.iter()) {
        let mut publish_args =
            if step.package.has_chip_features() || step.package.has_inline_assembly(workspace) {
                vec!["--no-verify"]
            } else {
                vec![]
            };

        if !args.no_dry_run {
            publish_args.push("--dry-run");
        }

        let builder = CargoArgsBuilder::default()
            .subcommand("publish")
            .args(&publish_args);

        let args = builder.build();
        log::debug!("{args:#?}");

        // Execute `cargo publish` command from the package root:
        crate::cargo::run(&args, &toml.package_path())?;
    }

    // Tag the releases
    let mut new_tags = Vec::new();
    for (step, toml) in plan.packages.iter().zip(tomls.iter()) {
        let tag_name = toml.package.tag(&toml.package_version());
        let tag_message = format!("{} {}", step.package, toml.version());

        if args.no_dry_run {
            let output = Command::new("git")
                .arg("tag")
                .arg("-m")
                .arg(&tag_message)
                .arg(&tag_name)
                .current_dir(workspace)
                .output()
                .context("Failed to create git tag")?;

            if !output.status.success() {
                bail!(
                    "Failed to create git tag {}: {}",
                    tag_name,
                    String::from_utf8_lossy(&output.stderr)
                );
            }
            println!("Tagged {} with message: {}", tag_name, tag_message);
        } else {
            println!(
                "Dry run: would tag {} with message: {}",
                tag_name, tag_message
            );
        }

        new_tags.push(tag_name);
    }

    let upstream = get_remote_name_for("esp-rs/esp-hal")?;
    if args.no_dry_run {
        let output = Command::new("git")
            .arg("push")
            .arg(upstream)
            .args(&new_tags)
            .output()
            .context("Failed to push git tags")?;

        if !output.status.success() {
            bail!(
                "Failed to push git tags: {}",
                String::from_utf8_lossy(&output.stderr)
            );
        }
    } else {
        println!(
            "Dry run: would push the following tags to {upstream}: {}",
            new_tags.join(", ")
        );
    }

    println!("publish-plan completed successfully.");

    Ok(())
}
