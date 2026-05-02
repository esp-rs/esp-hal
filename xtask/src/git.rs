use std::process::Command;

use anyhow::{Context, Result};
#[cfg(feature = "release")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "release")]
use strum::IntoEnumIterator;

#[cfg(feature = "release")]
use crate::Package;

/// Parsed context from a backport branch name like `esp-hal-1.1.x`.
#[cfg(feature = "release")]
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BackportInfo {
    pub package: Package,
    pub major: u64,
    pub minor: u64,
}

/// Try to parse a branch name as a backport branch.
///
/// Recognizes the pattern `{package}-{major}.{minor}.x` where `{package}` is a
/// known workspace package name in kebab-case (e.g. `esp-hal-1.1.x`).
///
/// When multiple packages share a prefix (e.g. `esp-hal` vs
/// `esp-hal-procmacros`), the longest matching package name wins.
#[cfg(feature = "release")]
pub fn parse_backport_branch(branch: &str) -> Option<BackportInfo> {
    let mut best: Option<(Package, u64, u64, usize)> = None;

    for pkg in Package::iter() {
        let prefix = format!("{}-", pkg);
        if let Some(suffix) = branch.strip_prefix(&prefix) {
            if let Some((major, minor)) = parse_version_suffix(suffix) {
                let dominated = best
                    .as_ref()
                    .map_or(true, |&(_, _, _, len)| prefix.len() > len);
                if dominated {
                    best = Some((pkg, major, minor, prefix.len()));
                }
            }
        }
    }

    best.map(|(package, major, minor, _)| BackportInfo {
        package,
        major,
        minor,
    })
}

/// Parse `"1.2.x"` into `Some((1, 2))`.
#[cfg(feature = "release")]
fn parse_version_suffix(s: &str) -> Option<(u64, u64)> {
    let s = s.strip_suffix(".x")?;
    let (major, minor) = s.split_once('.')?;
    Some((major.parse().ok()?, minor.parse().ok()?))
}

/// Get the current git branch name.
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
/// Ensure that the git workspace is clean (no uncommitted changes).
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
/// Get the remote name for the given repository URL.
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

#[cfg(test)]
#[cfg(feature = "release")]
mod tests {
    use super::*;

    #[test]
    fn parse_esp_hal_backport_branch() {
        let info = parse_backport_branch("esp-hal-1.1.x").unwrap();
        assert_eq!(info.package, Package::EspHal);
        assert_eq!(info.major, 1);
        assert_eq!(info.minor, 1);
    }

    #[test]
    fn parse_esp_hal_procmacros_backport_branch() {
        let info = parse_backport_branch("esp-hal-procmacros-0.22.x").unwrap();
        assert_eq!(info.package, Package::EspHalProcmacros);
        assert_eq!(info.major, 0);
        assert_eq!(info.minor, 22);
    }

    #[test]
    fn main_branch_is_not_backport() {
        assert!(parse_backport_branch("main").is_none());
    }

    #[test]
    fn random_branch_is_not_backport() {
        assert!(parse_backport_branch("feature/wifi-rework").is_none());
    }

    #[test]
    fn missing_x_suffix() {
        assert!(parse_backport_branch("esp-hal-1.1").is_none());
    }
}
