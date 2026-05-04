use std::{collections::HashSet, io::Read as _, path::Path, process::Command};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::{UPSTREAM_REPO, pr_changelog::{PrChangelog, PrSection, validate}};

const SKIP_LABEL: &str = "skip-changelog";

/// Validate the changelog entries in a PR description.
///
/// Changelog entries are optional — contributors are not required to add them.
/// When entries are present their format is checked and the referenced crate
/// names must be published workspace packages. If a published package was
/// modified without any changelog entry, the check fails; a maintainer can
/// apply the `skip-changelog` label to waive this requirement.
///
/// When `--pr` is given, the PR body, labels, and changed files are fetched
/// from GitHub. When reading from stdin the body format and crate names are
/// validated, but the per-package coverage check is skipped (no file list).
pub fn check_pr_changelog(workspace: &Path, pr_number: Option<u64>) -> Result<()> {
    match pr_number {
        Some(pr) => {
            let info = fetch_pr_info(pr)?;
            check_with_github_info(workspace, pr, &info)
        }
        None => {
            let mut body = String::new();
            std::io::stdin()
                .read_to_string(&mut body)
                .context("Failed to read PR body from stdin")?;
            check_body(workspace, &body)
        }
    }
}

fn check_with_github_info(workspace: &Path, pr_number: u64, info: &PrInfo) -> Result<()> {
    fail_on_format_errors(Some(pr_number), validate(&info.body))?;

    // If the skip label is set, nothing else to check.
    if info.labels.iter().any(|l| l == SKIP_LABEL) {
        log::info!("PR #{pr_number}: `{SKIP_LABEL}` label is set — skipping changelog check.");
        return Ok(());
    }

    // Parse once; reuse for both crate-name validation and coverage check.
    let sections = parse_and_validate_crates(workspace, &info.body)?;

    // Determine which published packages were touched.
    let modified = modified_packages(workspace, &info.files);
    if modified.is_empty() {
        log::info!("PR #{pr_number}: no published packages modified — OK.");
        return Ok(());
    }

    // A section counts as "covered" only when it contains at least one
    // changelog list item. A migration-guide-only section does not satisfy
    // the changelog requirement.
    let covered: HashSet<&str> = sections
        .iter()
        .filter(|s| !s.changelog.is_empty())
        .map(|s| s.crate_name.as_str())
        .collect();
    let missing: Vec<&str> = modified
        .iter()
        .filter(|pkg| !covered.contains(pkg.as_str()))
        .map(|s| s.as_str())
        .collect();

    if missing.is_empty() {
        log::info!("PR #{pr_number}: all modified packages have changelog entries — OK.");
        Ok(())
    } else {
        bail!(
            "PR #{pr_number} modifies the following package(s) without a changelog entry: {}.\n\
             Add entries to the `# Changelog` section of the PR description, \
             or ask a maintainer to apply the `{SKIP_LABEL}` label.",
            missing.join(", ")
        )
    }
}

/// Validate body format and crate names (used when reading from stdin).
fn check_body(workspace: &Path, body: &str) -> Result<()> {
    fail_on_format_errors(None, validate(body))?;
    parse_and_validate_crates(workspace, body)?;
    log::info!("PR description changelog format is valid.");
    Ok(())
}

/// Emit formatted errors and bail if `errors` is non-empty.
fn fail_on_format_errors(pr_number: Option<u64>, errors: Vec<String>) -> Result<()> {
    if errors.is_empty() {
        return Ok(());
    }
    for error in &errors {
        log::error!("{error}");
    }
    match pr_number {
        Some(pr) => bail!(
            "{} format error(s) found in PR #{pr} description.",
            errors.len()
        ),
        None => bail!(
            "{} format error(s) found in the PR description.",
            errors.len()
        ),
    }
}

/// Parse `body` and verify every referenced crate is a published workspace package.
///
/// Returns the parsed sections so callers can reuse them without re-parsing.
fn parse_and_validate_crates(workspace: &Path, body: &str) -> Result<Vec<PrSection>> {
    let sections = PrChangelog::parse(0, body)?
        .map(|cl| cl.sections)
        .unwrap_or_default();

    let mut bad: Vec<String> = Vec::new();
    for section in &sections {
        let name = &section.crate_name;
        let cargo_toml = workspace.join(name).join("Cargo.toml");
        if !cargo_toml.exists() {
            bad.push(format!("`{name}` (no such package in the workspace)"));
        } else if !is_published(&cargo_toml) {
            bad.push(format!("`{name}` (publish = false)"));
        }
    }

    if bad.is_empty() {
        Ok(sections)
    } else {
        bail!(
            "Changelog entries reference package(s) that are not published:\n  {}\n\
             Only published packages should have changelog entries.",
            bad.join("\n  ")
        )
    }
}

/// Return the set of published package directory names touched by `files`.
fn modified_packages(workspace: &Path, files: &[GhFile]) -> HashSet<String> {
    files
        .iter()
        .filter_map(|f| f.path.split('/').next())
        .collect::<HashSet<_>>() // deduplicate: many files share the same top-level dir
        .into_iter()
        .filter(|dir| {
            let p = workspace.join(dir).join("Cargo.toml");
            if !p.exists() {
                return false;
            }
            let published = is_published(&p);
            if !published {
                log::debug!("Skipping '{dir}': publish = false");
            }
            published
        })
        .map(|s| s.to_string())
        .collect()
}

/// Returns `true` when the `Cargo.toml` at `path` does not have `publish = false`.
fn is_published(path: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(path) else {
        return true;
    };
    let Ok(doc) = text.parse::<toml_edit::DocumentMut>() else {
        return true;
    };
    doc.get("package")
        .and_then(|p| p.get("publish"))
        .and_then(|v| v.as_bool())
        .unwrap_or(true)
}

// ---------------------------------------------------------------------------
// GitHub API types

#[derive(Debug, Deserialize)]
struct PrInfo {
    body: String,
    labels: Vec<Label>,
    files: Vec<GhFile>,
}

#[derive(Debug, Deserialize)]
struct Label {
    name: String,
}

impl PartialEq<str> for Label {
    fn eq(&self, other: &str) -> bool {
        self.name == other
    }
}

#[derive(Debug, Deserialize)]
struct GhFile {
    path: String,
}

/// Use the `gh` CLI to fetch the body, labels, and changed files of a PR.
fn fetch_pr_info(pr_number: u64) -> Result<PrInfo> {
    let output = Command::new("gh")
        .args([
            "pr",
            "view",
            &pr_number.to_string(),
            "--repo",
            UPSTREAM_REPO,
            "--json",
            "body,labels,files",
        ])
        .output()
        .context("Failed to invoke `gh`. Is the GitHub CLI installed and authenticated?")?;

    if !output.status.success() {
        bail!(
            "`gh pr view` failed for PR #{pr_number}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    serde_json::from_slice(&output.stdout).context("Failed to parse `gh pr view` output")
}
