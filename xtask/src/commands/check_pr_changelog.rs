use std::{io::Read as _, process::Command};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::pr_changelog::{PrChangelog, validate};

const SKIP_LABEL: &str = "skip-changelog";

/// Check the changelog format of a PR description.
///
/// When `--pr` is given the PR body and labels are fetched from GitHub. If no
/// changelog entries are present and the `skip-changelog` label is not set the
/// command fails.
///
/// When reading from stdin the label check is skipped (labels are unavailable).
pub fn check_pr_changelog(pr_number: Option<u64>) -> Result<()> {
    match pr_number {
        Some(pr) => {
            let info = fetch_pr_info(pr)?;
            check_body_with_labels(pr, &info.body, &info.labels)
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

/// Validate body format and — when no entries are found — require the
/// `skip-changelog` label.
fn check_body_with_labels(pr_number: u64, body: &str, labels: &[String]) -> Result<()> {
    let errors = validate(body);
    if !errors.is_empty() {
        for error in &errors {
            log::error!("{error}");
        }
        bail!(
            "{} format error(s) found in the PR #{pr_number} description.",
            errors.len()
        );
    }

    let has_entries = PrChangelog::parse(pr_number, body)?.is_some();
    if !has_entries {
        if labels.iter().any(|l| l == SKIP_LABEL) {
            log::info!("PR #{pr_number}: no changelog entries, but `{SKIP_LABEL}` label is set — OK.");
        } else {
            bail!(
                "PR #{pr_number} has no changelog entries and the `{SKIP_LABEL}` label is not set.\n\
                 Add changelog entries to the PR description or apply the `{SKIP_LABEL}` label."
            );
        }
    } else {
        log::info!("PR #{pr_number}: changelog format is valid.");
    }

    Ok(())
}

/// Validate body format only (no label check — used when reading from stdin).
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

#[derive(Debug, Deserialize)]
struct PrInfo {
    body: String,
    labels: Vec<GhLabel>,
}

#[derive(Debug, Deserialize)]
struct GhLabel {
    name: String,
}

impl PrInfo {
    fn labels(&self) -> Vec<String> {
        self.labels.iter().map(|l| l.name.clone()).collect()
    }
}

struct FetchedPrInfo {
    body: String,
    labels: Vec<String>,
}

/// Use the `gh` CLI to fetch the body and labels of a pull request.
fn fetch_pr_info(pr_number: u64) -> Result<FetchedPrInfo> {
    let output = Command::new("gh")
        .args([
            "pr",
            "view",
            &pr_number.to_string(),
            "--json",
            "body,labels",
        ])
        .output()
        .context("Failed to invoke `gh`. Is the GitHub CLI installed and authenticated?")?;

    if !output.status.success() {
        bail!(
            "`gh pr view` failed for PR #{pr_number}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let info: PrInfo = serde_json::from_slice(&output.stdout)
        .context("Failed to parse `gh pr view` output")?;

    Ok(FetchedPrInfo {
        labels: info.labels(),
        body: info.body,
    })
}
