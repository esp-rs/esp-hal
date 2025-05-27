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

#[cfg(feature = "release")]
pub fn ensure_workspace_clean(workspace: &std::path::Path) -> Result<()> {
    std::env::set_current_dir(workspace)
        .with_context(|| format!("Failed to change directory to {}", workspace.display()))?;

    let status = Command::new("git")
        .arg("status")
        .arg("--porcelain")
        .output()
        .context("Failed to check git status")?;

    anyhow::ensure!(
        String::from_utf8_lossy(&status.stdout).trim().is_empty(),
        "The workspace is not clean. Please commit or stash your changes before running this command."
    );

    Ok(())
}

#[cfg(feature = "release")]
pub fn get_remote_name_for(repo: &str) -> Result<String> {
    let remotes = Command::new("git")
        .arg("remote")
        .arg("-v")
        .output()
        .context("Failed to get remote URL")?;

    let remotes = String::from_utf8_lossy(&remotes.stdout);

    for line in remotes.lines() {
        if line.contains(repo) {
            let parts: Vec<_> = line.split_whitespace().collect();
            if parts.len() >= 2 {
                return Ok(parts[0].to_string());
            }
        }
    }

    anyhow::bail!("Failed to find remote name for {repo}");
}
