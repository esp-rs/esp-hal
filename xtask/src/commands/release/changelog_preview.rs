use std::{
    collections::{BTreeMap, btree_map::Entry},
    path::Path,
    process::Command,
};

use anyhow::{Context, Result, bail};
use clap::Args;
use jiff::Timestamp;
use serde::Deserialize;

use crate::{changelog::Changelog, pr_changelog::PrChangelog};

// ---------------------------------------------------------------------------
// Baseline helpers

/// Coarse date + exact Unix timestamp derived from a git ref.
struct Baseline {
    /// `YYYY-MM-DD` used for the GitHub `merged:>=` search qualifier.
    date: String,
    /// Exact committer Unix timestamp — used for client-side filtering so
    /// PRs merged on the same calendar day as the ref are not dropped.
    unix: i64,
}

/// Derive a `Baseline` from `git_ref` by running `git log` once.
fn ref_to_baseline(workspace: &Path, git_ref: &str) -> Result<Baseline> {
    // Request both the short date and the Unix timestamp in one invocation.
    let output = Command::new("git")
        .args(["log", "-1", "--format=%cs%n%ct", git_ref])
        .current_dir(workspace)
        .output()
        .context("Failed to run `git log`")?;

    if !output.status.success() {
        bail!(
            "`git log` failed for ref '{git_ref}': {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let text = String::from_utf8_lossy(&output.stdout);
    let mut lines = text.lines();

    let date = lines
        .next()
        .filter(|s| !s.trim().is_empty())
        .context("No date in `git log` output")?
        .trim()
        .to_string();

    let unix = lines
        .next()
        .context("No Unix timestamp in `git log` output")?
        .trim()
        .parse::<i64>()
        .context("Could not parse Unix timestamp from `git log`")?;

    Ok(Baseline { date, unix })
}

use crate::UPSTREAM_REPO;

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

    let baseline = ref_to_baseline(workspace, since_ref)?;
    // Fetch with >= (inclusive date) to cast a wide enough net, then trim
    // precisely client-side so PRs merged later on the same day as the
    // baseline ref are not excluded.
    let prs = list_merged_prs(&baseline.date, limit)?;
    let prs: Vec<GhPr> = prs
        .into_iter()
        .filter(|pr| {
            pr.merged_at
                .parse::<Timestamp>()
                .map(|t| t.as_second() > baseline.unix)
                .unwrap_or(true) // keep if timestamp is unparseable
        })
        .collect();

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
                let changelog = match changelogs.entry(crate_name.clone()) {
                    Entry::Occupied(e) => e.into_mut(),
                    Entry::Vacant(e) => {
                        match load_changelog(workspace, crate_name) {
                            Ok(Some(cl)) => e.insert(cl),
                            Ok(None) => e.insert(Changelog::empty()),
                            Err(err) => {
                                log::warn!("Skipping changelog entries for '{crate_name}': {err}");
                                // Do not insert — prevents accidental overwrites.
                                continue;
                            }
                        }
                    }
                };

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
/// Returns `Ok(None)` when no `CHANGELOG.md` exists for the crate (normal for
/// new or changelog-free crates). Returns `Err` when the file exists but
/// cannot be parsed — callers **must not** write a derived changelog back to
/// disk in that case, to avoid accidentally overwriting good history with a
/// partially-constructed file.
fn load_changelog(workspace: &Path, crate_name: &str) -> Result<Option<Changelog>> {
    let path = workspace.join(crate_name).join("CHANGELOG.md");
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            log::debug!("No CHANGELOG.md found for '{crate_name}' (checked {path:?})");
            return Ok(None);
        }
        Err(e) => return Err(e).context(format!("Failed to read {}", path.display())),
    };
    Changelog::parse(&text)
        .map(Some)
        .with_context(|| format!("Failed to parse {}", path.display()))
}

// ---------------------------------------------------------------------------
// GitHub / git helpers

#[derive(Debug, Deserialize)]
struct GhPr {
    number: u64,
    #[serde(default)]
    body: String,
    /// RFC 3339 merge timestamp from GitHub (e.g. `2024-01-15T10:30:00Z`).
    #[serde(rename = "mergedAt", default)]
    merged_at: String,
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
///
/// `since_date` is a `YYYY-MM-DD` string used with `merged:>=` (inclusive) as
/// a coarse server-side filter. The caller is responsible for precise
/// client-side filtering using the exact `mergedAt` timestamp.
fn list_merged_prs(since_date: &str, limit: u32) -> Result<Vec<GhPr>> {
    let search = format!("repo:{UPSTREAM_REPO} is:pr is:merged merged:>={since_date}");

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
            "number,body,mergedAt",
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
