use std::process::Command;

use anyhow::{Context, Result};

pub fn current_branch() -> Result<String> {
    let status = Command::new("git")
        .arg("rev-parse")
        .arg("--abbrev-ref")
        .arg("HEAD")
        .output()
        .context("Failed to get current branch")?;

    Ok(String::from_utf8_lossy(&status.stdout).trim().to_string())
}
