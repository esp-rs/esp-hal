use std::{collections::BTreeMap, process::Command};

use anyhow::{Context, Result, bail};
use clap::Args;
use serde::Deserialize;

use crate::pr_changelog::{EntryKind, PrChangelog};

const UPSTREAM_REPO: &str = "esp-rs/esp-hal";

/// Arguments for the `release changelog-preview` subcommand.
#[derive(Debug, Args)]
pub struct ChangelogPreviewArgs {
    /// Git ref (tag, branch, commit) to use as the baseline.
    ///
    /// Only PRs merged after this ref was created are included.
    /// Defaults to the most recently published tag on the `main` branch.
    #[arg(long)]
    pub since: Option<String>,

    /// Maximum number of PRs to fetch (newest first).
    #[arg(long, default_value_t = 500)]
    pub limit: u32,
}

/// Run the `release changelog-preview` subcommand.
pub fn changelog_preview(args: ChangelogPreviewArgs) -> Result<()> {
    let since = match args.since {
        Some(r) => r,
        None => latest_tag()?,
    };

    log::info!("Collecting PRs merged since `{since}`…");

    let merged_after = ref_to_timestamp(&since)?;
    let prs = list_merged_prs(&merged_after, args.limit)?;

    log::info!("Found {} merged PR(s). Parsing changelog entries…", prs.len());

    let mut changelogs: Vec<PrChangelog> = Vec::new();
    let mut skipped = 0usize;

    for pr in prs {
        match PrChangelog::parse(pr.number, &pr.body) {
            Ok(Some(cl)) => changelogs.push(cl),
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

    print_preview(&changelogs);

    Ok(())
}

// ---------------------------------------------------------------------------
// Preview renderer

/// Group and render all collected changelog entries.
fn print_preview(changelogs: &[PrChangelog]) {
    // crate → area (or "") → kind → Vec<(pr, text)>
    let mut changelog_map: BTreeMap<String, BTreeMap<String, BTreeMap<EntryKind, Vec<(u64, String)>>>> =
        BTreeMap::new();
    // crate → area (or "") → Vec<(pr, text)>
    let mut migration_map: BTreeMap<String, BTreeMap<String, Vec<(u64, String)>>> =
        BTreeMap::new();

    for cl in changelogs {
        for section in &cl.sections {
            let area_key = section.area.clone().unwrap_or_default();

            for entry in &section.changelog {
                changelog_map
                    .entry(section.crate_name.clone())
                    .or_default()
                    .entry(area_key.clone())
                    .or_default()
                    .entry(entry.kind)
                    .or_default()
                    .push((cl.pr_number, entry.text.clone()));
            }

            if let Some(guide) = &section.migration_guide {
                migration_map
                    .entry(section.crate_name.clone())
                    .or_default()
                    .entry(area_key.clone())
                    .or_default()
                    .push((cl.pr_number, guide.clone()));
            }
        }
    }

    if changelog_map.is_empty() && migration_map.is_empty() {
        println!("(No changelog entries found.)");
        return;
    }

    // --- Changelog ---
    if !changelog_map.is_empty() {
        println!("# Changelog\n");
        for (crate_name, areas) in &changelog_map {
            for (area, kinds) in areas {
                if area.is_empty() {
                    println!("## {crate_name}\n");
                } else {
                    println!("## {crate_name}/{area}\n");
                }
                for kind in [EntryKind::Added, EntryKind::Changed, EntryKind::Fixed, EntryKind::Removed] {
                    if let Some(entries) = kinds.get(&kind) {
                        for (pr, text) in entries {
                            println!("- {kind}: {text} (#{pr})");
                        }
                    }
                }
                println!();
            }
        }
    }

    // --- Migration guide ---
    if !migration_map.is_empty() {
        println!("# Migration guide\n");
        for (crate_name, areas) in &migration_map {
            for (area, entries) in areas {
                if area.is_empty() {
                    println!("## {crate_name}\n");
                } else {
                    println!("## {crate_name}/{area}\n");
                }
                for (pr, guide) in entries {
                    println!("{guide}\n");
                    println!("*(from PR #{pr})*\n");
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// GitHub helpers

#[derive(Debug, Deserialize)]
struct GhPr {
    number: u64,
    #[serde(default)]
    body: String,
}

/// Return the date-time string when `git_ref` was created (ISO 8601).
fn ref_to_timestamp(git_ref: &str) -> Result<String> {
    let output = Command::new("git")
        .args(["log", "-1", "--format=%cI", git_ref])
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
fn latest_tag() -> Result<String> {
    let output = Command::new("git")
        .args(["describe", "--tags", "--abbrev=0"])
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
