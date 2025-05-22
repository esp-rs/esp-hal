use std::{path::Path, process::Command};

use anyhow::Result;
use clap::Args;
use strum::IntoEnumIterator;

use crate::{Package, package_version};

#[derive(Debug, Args)]
pub struct TagReleasesArgs {
    /// Package(s) to tag.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Actually try and create the tags
    #[arg(long)]
    no_dry_run: bool,
}

pub fn tag_releases(workspace: &Path, mut args: TagReleasesArgs) -> Result<()> {
    args.packages.sort();

    #[derive(serde::Serialize)]
    struct DocumentationItem {
        name: String,
        tag: String,
    }

    let mut created = Vec::new();
    for package in args.packages {
        // If a package does not require documentation, this also means that it is not
        // published (maybe this function needs a better name), so we can skip tagging
        // it:
        if !package.is_published(workspace) {
            continue;
        }

        let version = package_version(workspace, package)?;
        let tag = package.tag(&version);

        if args.no_dry_run {
            let output = Command::new("git")
                .arg("tag")
                .arg(&tag)
                .current_dir(workspace)
                .output()?;

            if output.stderr.is_empty() {
                log::info!("Created tag '{tag}'");
            } else {
                let err = String::from_utf8_lossy(&output.stderr);
                let err = err.trim_start_matches("fatal: ");
                log::warn!("{}", err);
            }
        } else {
            log::info!("Would create '{tag}' if `--no-dry-run` was passed.")
        }
        created.push(DocumentationItem {
            name: package.to_string(),
            tag,
        });
    }

    if args.no_dry_run {
        log::info!("Created {} tags", created.len());
        log::info!("IMPORTANT: Don't forget to push the tags to the correct remote!");
    }

    let workflow_input = serde_json::to_string(&created)?;

    log::info!("Documentation workflow input for these packages:\r\n\r\n{workflow_input}\r\n\r\n");

    // Print to stdout for GitHub Actions
    print!("{workflow_input}");

    Ok(())
}
