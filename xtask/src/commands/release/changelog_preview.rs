use std::{collections::BTreeMap, path::Path, process::Command};

use anyhow::{Context, Result, bail};
use clap::Args;
use serde::Deserialize;

use crate::{changelog::Changelog, pr_changelog::PrChangelog};

const UPSTREAM_REPO: &str = "esp-rs/esp-hal";

/// Arguments for the `release changelog-preview` subcommand.
#[derive(Debug, Args)]
pub struct ChangelogPreviewArgs {
    /// Git ref (tag, branch, commit) to use as the baseline.
    ///
    /// Only PRs merged after this ref are included.
    /// Defaults to the most recently reachable tag.
    #[arg(long)]
    pub since: Option<String>,

    /// Maximum number of PRs to fetch (newest first).
    #[arg(long, default_value_t = 500)]
    pub limit: u32,
}

/// Migration guide entries: `(area, pr_number, guide_text)`.
pub(crate) type MigrationEntries = BTreeMap<String, Vec<(Option<String>, u64, String)>>;

/// Collect PR changelog entries merged since `since_ref` into per-crate changelogs.
///
/// Returns a pair of maps:
/// - Merged changelog entries keyed by crate name.
/// - Migration guide entries keyed by crate name.
///
/// Requires the `gh` CLI to be installed and authenticated.
pub(crate) fn collect_changelogs(
    workspace: &Path,
    since_ref: &str,
    limit: u32,
) -> Result<(BTreeMap<String, Changelog>, MigrationEntries)> {
    log::info!("Collecting PRs merged since `{since_ref}`…");

    let merged_after = ref_to_timestamp(workspace, since_ref)?;
    let prs = list_merged_prs(&merged_after, limit)?;

    log::info!(
        "Found {} merged PR(s). Parsing changelog entries…",
        prs.len()
    );

    let mut pr_changelogs: Vec<PrChangelog> = Vec::new();
    let mut skipped = 0usize;

    for pr in prs {
        match PrChangelog::parse(pr.number, &pr.body) {
            Ok(Some(cl)) => pr_changelogs.push(cl),
            Ok(None) => skipped += 1,
            Err(e) => {
                log::warn!("PR #{}: parse error — {e}", pr.number);
                skipped += 1;
            }
        }
    }

    if skipped > 0 {
        log::info!("{skipped} PR(s) had no changelog entries and were skipped.");
    }

    // Load each relevant crate's CHANGELOG.md, merge in the PR entries.
    //
    // keyed by crate name (e.g. "esp-hal")
    let mut changelogs: BTreeMap<String, Changelog> = BTreeMap::new();
    // migration guide entries: crate → list of (area, pr, text)
    let mut migrations: MigrationEntries = BTreeMap::new();

    for pr_cl in &pr_changelogs {
        for section in &pr_cl.sections {
            let crate_name = &section.crate_name;

            for entry in &section.changelog {
                let changelog = changelogs
                    .entry(crate_name.clone())
                    .or_insert_with(|| load_changelog(workspace, crate_name));

                let text = match &section.area {
                    Some(area) => format!("{area}: {}", entry.text),
                    None => entry.text.clone(),
                };
                changelog.add_entry(entry.kind.as_str(), &text, pr_cl.pr_number);
            }

            if let Some(guide) = &section.migration_guide {
                migrations.entry(crate_name.clone()).or_default().push((
                    section.area.clone(),
                    pr_cl.pr_number,
                    guide.clone(),
                ));
            }
        }
    }

    Ok((changelogs, migrations))
}

/// Run the `release changelog-preview` subcommand.
pub fn changelog_preview(workspace: &Path, args: ChangelogPreviewArgs) -> Result<()> {
    let since = match args.since {
        Some(r) => r,
        None => latest_tag(workspace)?,
    };

    let (changelogs, migrations) = collect_changelogs(workspace, &since, args.limit)?;

    if changelogs.is_empty() && migrations.is_empty() {
        println!("(No changelog entries found.)");
        return Ok(());
    }

    // Print unreleased changelog sections
    for (crate_name, changelog) in &changelogs {
        if let Some(text) = changelog.format_unreleased() {
            println!("<!-- {crate_name} -->");
            println!("{text}");
        }
    }

    // Print migration guide sections
    if !migrations.is_empty() {
        println!("---");
        println!("# Migration guide\n");
        for (crate_name, entries) in &migrations {
            println!("## {crate_name}\n");
            for (area, _pr, guide) in entries {
                if let Some(area) = area {
                    println!("### {area}\n");
                }
                println!("{guide}\n");
            }
        }
    }

    Ok(())
}

/// Load and parse an existing `CHANGELOG.md` for the given crate name.
///
/// If no changelog exists (e.g. the crate name is unknown or has no file),
/// returns an empty `Changelog`.
fn load_changelog(workspace: &Path, crate_name: &str) -> Changelog {
    let path = workspace.join(crate_name).join("CHANGELOG.md");
    let Ok(text) = std::fs::read_to_string(&path) else {
        log::debug!("No CHANGELOG.md found for '{crate_name}' (checked {path:?})");
        return Changelog::empty();
    };
    match Changelog::parse(&text) {
        Ok(cl) => cl,
        Err(e) => {
            log::warn!("Failed to parse {}: {e}", path.display());
            Changelog::empty()
        }
    }
}

// ---------------------------------------------------------------------------
// GitHub / git helpers

#[derive(Debug, Deserialize)]
struct GhPr {
    number: u64,
    #[serde(default)]
    body: String,
}

/// Return the date (`YYYY-MM-DD`) when `git_ref` was committed.
///
/// GitHub's search API accepts ISO 8601 dates; using the plain date avoids
/// any timezone-offset parsing issues with the `merged:>DATE` qualifier.
pub(crate) fn ref_to_timestamp(workspace: &Path, git_ref: &str) -> Result<String> {
    let output = Command::new("git")
        .args(["log", "-1", "--format=%cs", git_ref]) // %cs = short date YYYY-MM-DD
        .current_dir(workspace)
        .output()
        .context("Failed to run `git log`")?;

    if !output.status.success() {
        bail!(
            "`git log` failed for ref '{git_ref}': {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let ts = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if ts.is_empty() {
        bail!("Could not determine timestamp for ref '{git_ref}'");
    }
    Ok(ts)
}

/// Return the most recent tag reachable from `HEAD`.
fn latest_tag(workspace: &Path) -> Result<String> {
    let output = Command::new("git")
        .args(["describe", "--tags", "--abbrev=0"])
        .current_dir(workspace)
        .output()
        .context("Failed to run `git describe`")?;

    if !output.status.success() {
        bail!(
            "`git describe` failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Fetch merged PRs from GitHub using the `gh` CLI.
fn list_merged_prs(merged_after: &str, limit: u32) -> Result<Vec<GhPr>> {
    let search = format!("repo:{UPSTREAM_REPO} is:pr is:merged merged:>{merged_after}");

    let output = Command::new("gh")
        .args([
            "pr",
            "list",
            "--repo",
            UPSTREAM_REPO,
            "--state",
            "merged",
            "--search",
            &search,
            "--limit",
            &limit.to_string(),
            "--json",
            "number,body",
        ])
        .output()
        .context("`gh pr list` failed. Is the GitHub CLI installed and authenticated?")?;

    if !output.status.success() {
        bail!(
            "`gh pr list` failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    serde_json::from_slice(&output.stdout).context("Failed to parse `gh pr list` output")
}
