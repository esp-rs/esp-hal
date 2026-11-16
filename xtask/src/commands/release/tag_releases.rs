use std::{path::Path, process::Command};

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
        .args(["diff", "--name-only", rev, "HEAD"])
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

/// Whether any of `changed` can alter the docs of a package whose in-repo dependency closure (the
/// package included) is `related`. A path outside every package directory can, so a new shared
/// docs input never leaves docs stale.
fn docs_affected(changed: &[String], related: &[Package]) -> bool {
    changed.iter().any(|path| {
        let owner =
            Package::iter().find(|package| path.starts_with(&format!("{}/", package.directory())));
        owner.is_none_or(|owner| related.contains(&owner))
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
}
