use std::{collections::HashSet, io::Read as _, path::Path, process::Command};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::pr_changelog::{PrChangelog, validate};

const SKIP_LABEL: &str = "skip-changelog";

/// Check the changelog format of a PR description.
///
/// When `--pr` is given, the PR body, labels, and changed files are fetched
/// from GitHub. For each package that was modified in the PR, a changelog entry
/// must be present in the PR description — unless the `skip-changelog` label
/// is set.
///
/// When reading from stdin only the format of the body is validated.
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
            check_body_format_only(&body)
        }
    }
}

fn check_with_github_info(workspace: &Path, pr_number: u64, info: &FetchedPrInfo) -> Result<()> {
    // Validate format first.
    let errors = validate(&info.body);
    if !errors.is_empty() {
        for error in &errors {
            log::error!("{error}");
        }
        bail!(
            "{} format error(s) found in PR #{pr_number} description.",
            errors.len()
        );
    }

    // If the skip label is set, nothing else to check.
    if info.labels.iter().any(|l| l == SKIP_LABEL) {
        log::info!("PR #{pr_number}: `{SKIP_LABEL}` label is set — skipping changelog check.");
        return Ok(());
    }

    // Determine which published packages were touched.
    let modified = modified_packages(workspace, &info.files);
    if modified.is_empty() {
        log::info!("PR #{pr_number}: no published packages modified — OK.");
        return Ok(());
    }

    // Determine which crates have changelog entries in the PR body.
    let covered: HashSet<String> = PrChangelog::parse(pr_number, &info.body)?
        .map(|cl| {
            cl.sections
                .into_iter()
                .map(|s| s.crate_name)
                .collect::<HashSet<_>>()
        })
        .unwrap_or_default();

    let missing: Vec<&str> = modified
        .iter()
        .filter(|pkg| !covered.contains(*pkg))
        .map(|s| s.as_str())
        .collect();

    if missing.is_empty() {
        log::info!("PR #{pr_number}: all modified packages have changelog entries — OK.");
        Ok(())
    } else {
        bail!(
            "PR #{pr_number} modifies the following package(s) without a changelog entry: {}.\n\
             Add entries to the `# Changelog` section of the PR description, \
             or apply the `{SKIP_LABEL}` label.",
            missing.join(", ")
        )
    }
}

/// Validate body format only (used when reading from stdin).
fn check_body_format_only(body: &str) -> Result<()> {
    let errors = validate(body);
    if errors.is_empty() {
        log::info!("PR description changelog format is valid.");
        Ok(())
    } else {
        for error in &errors {
            log::error!("{error}");
        }
        bail!(
            "{} format error(s) found in the PR description.",
            errors.len()
        );
    }
}

/// Return the set of published package directory names touched by `files`.
///
/// A package is considered published when its `Cargo.toml` does not contain
/// `publish = false`. Packages without a `Cargo.toml` at the workspace root
/// level are ignored.
fn modified_packages(workspace: &Path, files: &[GhFile]) -> HashSet<String> {
    let mut result = HashSet::new();

    for dir in files
        .iter()
        .filter_map(|f| f.path.split('/').next())
        .collect::<HashSet<_>>()
    {
        let cargo_toml_path = workspace.join(dir).join("Cargo.toml");
        if !cargo_toml_path.exists() {
            continue;
        }
        if is_published(&cargo_toml_path) {
            result.insert(dir.to_string());
        } else {
            log::debug!("Skipping '{dir}': publish = false");
        }
    }

    result
}

/// Returns `true` when the `Cargo.toml` at `path` does not opt out of publishing.
///
/// A package is considered unpublished when it has `publish = false` in its
/// `[package]` table. Missing the key, or any parse error, defaults to `true`.
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
struct GhPrInfo {
    body: String,
    labels: Vec<GhLabel>,
    files: Vec<GhFile>,
}

#[derive(Debug, Deserialize)]
struct GhLabel {
    name: String,
}

#[derive(Debug, Deserialize)]
struct GhFile {
    path: String,
}

struct FetchedPrInfo {
    body: String,
    labels: Vec<String>,
    files: Vec<GhFile>,
}

/// Use the `gh` CLI to fetch the body, labels, and changed files of a PR.
fn fetch_pr_info(pr_number: u64) -> Result<FetchedPrInfo> {
    let output = Command::new("gh")
        .args([
            "pr",
            "view",
            &pr_number.to_string(),
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

    let info: GhPrInfo = serde_json::from_slice(&output.stdout)
        .context("Failed to parse `gh pr view` output")?;

    Ok(FetchedPrInfo {
        body: info.body,
        labels: info.labels.into_iter().map(|l| l.name).collect(),
        files: info.files,
    })
}
