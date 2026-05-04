use std::{io::Read as _, process::Command};

use anyhow::{Context, Result, bail};

use crate::pr_changelog::validate;

/// Check the changelog format of a PR description.
///
/// The body is read from stdin, or fetched from GitHub when `--pr` is given.
pub fn check_pr_changelog(pr_number: Option<u64>) -> Result<()> {
    let body = match pr_number {
        Some(pr) => fetch_pr_body(pr)?,
        None => {
            let mut buf = String::new();
            std::io::stdin()
                .read_to_string(&mut buf)
                .context("Failed to read PR body from stdin")?;
            buf
        }
    };

    let errors = validate(&body);

    if errors.is_empty() {
        log::info!("PR description changelog format is valid.");
        Ok(())
    } else {
        for error in &errors {
            log::error!("{error}");
        }
        bail!(
            "{} format error(s) found in the PR description changelog.",
            errors.len()
        );
    }
}

/// Use the `gh` CLI to fetch the body of a pull request.
fn fetch_pr_body(pr_number: u64) -> Result<String> {
    let output = Command::new("gh")
        .args(["pr", "view", &pr_number.to_string(), "--json", "body", "--jq", ".body"])
        .output()
        .context("Failed to invoke `gh`. Is the GitHub CLI installed and authenticated?")?;

    if !output.status.success() {
        bail!(
            "`gh pr view` failed for PR #{pr_number}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    Ok(String::from_utf8_lossy(&output.stdout).into_owned())
}
