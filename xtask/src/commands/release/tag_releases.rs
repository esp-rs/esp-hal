use std::{collections::HashSet, path::Path, process::Command};

use anyhow::{Context, Result, ensure};
use clap::Args;
use strum::IntoEnumIterator;

use crate::{Package, cargo::related_crates, package_version};

/// Tag the releases for the specified packages.
#[derive(Debug, Args)]
pub struct TagReleasesArgs {
    /// Package(s) to tag.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Actually try and create the tags
    #[arg(long)]
    no_dry_run: bool,

    /// Only list packages whose documentation may differ from what was built at this git revision:
    /// the package or one of its in-repo dependencies changed since then.
    #[arg(long, conflicts_with = "no_dry_run")]
    changed_since: Option<String>,

    /// Skip packages whose current version has no tag on the `origin` remote yet.
    #[arg(long, conflicts_with = "no_dry_run")]
    tagged_only: bool,
}

/// Tag the releases for the specified packages.
pub fn tag_releases(workspace: &Path, mut args: TagReleasesArgs) -> Result<()> {
    args.packages.sort();

    if let Some(rev) = &args.changed_since {
        let changed = changed_paths(workspace, rev)?;
        args.packages.retain(|&package| {
            package.is_published() && docs_affected(&changed, &related_crates(workspace, package))
        });
        log::info!("Packages changed since {rev}: {:?}", args.packages);
    }

    #[derive(serde::Serialize)]
    struct DocumentationItem {
        name: String,
        tag: String,
    }

    let remote_tags = if args.tagged_only {
        Some(remote_tags(workspace)?)
    } else {
        None
    };

    let mut created = Vec::new();
    for package in args.packages {
        // If a package does not require documentation, this also means that it is not
        // published (maybe this function needs a better name), so we can skip tagging
        // it:
        if !package.is_published() {
            continue;
        }

        let version = package_version(workspace, package)?;
        let tag = package.tag(&version);

        if let Some(tags) = &remote_tags
            && !tags.contains(&tag)
        {
            log::warn!("Skipping {package}: '{tag}' is not tagged on origin");
            continue;
        }

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

fn changed_paths(workspace: &Path, rev: &str) -> Result<Vec<String>> {
    let output = Command::new("git")
        .args(["diff", "--name-only", "--no-renames", rev, "HEAD"])
        .current_dir(workspace)
        .output()
        .context("Failed to run git diff")?;
    ensure!(
        output.status.success(),
        "git diff against {rev} failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(str::to_string)
        .collect())
}

fn remote_tags(workspace: &Path) -> Result<HashSet<String>> {
    let output = Command::new("git")
        .args(["ls-remote", "--tags", "--refs", "origin"])
        .current_dir(workspace)
        .output()
        .context("Failed to run git ls-remote")?;
    ensure!(
        output.status.success(),
        "git ls-remote origin failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| line.split_once("refs/tags/"))
        .map(|(_, tag)| tag.to_string())
        .collect())
}

/// Whether any of `changed` can alter the docs of a package whose in-repo dependency closure (the
/// package included) is `related`. A path outside every package directory can, so a new shared
/// docs input never leaves docs stale.
fn docs_affected(changed: &[String], related: &[Package]) -> bool {
    changed.iter().any(|path| {
        let owner =
            Package::iter().find(|package| path.starts_with(&format!("{}/", package.directory())));
        match owner {
            // xtask reads esp-metadata while building any package's docs (doc cfgs, the esp-hal
            // peripheral table), so it changes docs even though no published package depends on it.
            None | Some(Package::EspMetadata) => true,
            Some(owner) => related.contains(&owner),
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // esp-radio's closure: itself plus esp-hal and esp-hal's dependencies.
    const ESP_RADIO: &[Package] = &[
        Package::EspRadio,
        Package::EspHal,
        Package::EspHalProcmacros,
    ];

    fn affects(paths: &[&str], related: &[Package]) -> bool {
        let paths = paths.iter().map(|p| p.to_string()).collect::<Vec<_>>();
        docs_affected(&paths, related)
    }

    #[test]
    fn change_in_the_closure_affects() {
        assert!(affects(&["esp-radio/src/lib.rs"], ESP_RADIO));
        assert!(affects(&["esp-hal-procmacros/src/lib.rs"], ESP_RADIO));
    }

    #[test]
    fn change_outside_the_closure_does_not_affect() {
        assert!(!affects(&["esp-println/src/lib.rs"], ESP_RADIO));
        assert!(!affects(
            &["examples/qa/foo/src/main.rs", "hil-test/src/lib.rs"],
            ESP_RADIO
        ));
    }

    #[test]
    fn directory_prefix_is_not_a_match() {
        // `esp-hal-host-tests/` must not count as a change to `esp-hal/`.
        assert!(!affects(
            &["esp-hal-host-tests/src/lib.rs"],
            &[Package::EspHal]
        ));
    }

    #[test]
    fn path_outside_every_package_affects_everything() {
        for path in [
            "xtask/src/documentation.rs",
            ".github/actions/setup-toolchains/action.yml",
            "Cargo.lock",
        ] {
            assert!(affects(&[path], &[Package::EspPrintln]), "{path}");
        }
    }

    #[test]
    fn esp_metadata_affects_everything() {
        assert!(affects(
            &["esp-metadata/devices/esp32c6/device.toml"],
            &[Package::EspPrintln]
        ));
        assert!(affects(&["esp-metadata/src/lib.rs"], ESP_RADIO));
    }
}
