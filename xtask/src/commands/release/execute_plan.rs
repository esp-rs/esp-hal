use std::{path::Path, process::Command};

use anyhow::{Context, Result, bail, ensure};
use clap::Args;
use strum::IntoEnumIterator;

use crate::{
    cargo::CargoToml,
    commands::{
        VersionBump,
        checker::generate_baseline,
        release::plan::{PackagePlan, Plan},
        update_package,
    },
    git::{current_branch, ensure_workspace_clean, get_remote_name_for},
    metadata::Chip,
};

/// Arguments for executing the release plan.
#[derive(Debug, Args)]
pub struct ApplyPlanArgs {
    /// Actually make git changes. Without this flag, the command will only
    /// update code.
    #[arg(long)]
    no_dry_run: bool,

    /// Instead of opening the pull request, just print base URL and body.
    #[arg(long)]
    manual_pull_request: bool,
}

/// Execute the release plan by making code changes, committing them to a new
pub fn execute_plan(workspace: &Path, args: ApplyPlanArgs) -> Result<()> {
    ensure_workspace_clean(workspace)
        .with_context(|| format!("Workspace {workspace:?} is not clean!"))?;

    let plan_path = workspace.join("release_plan.jsonc");
    let plan_path = crate::windows_safe_path(&plan_path);

    let mut plan = Plan::from_path(&plan_path)
        .with_context(|| format!("Failed to read release plan from {}", plan_path.display()))?;

    ensure!(
        current_branch()? == plan.base,
        "The release plan must be executed on the same branch it was created on. \
        Please switch to the {} branch and try again.",
        plan.base
    );

    if let Some(ref bp) = plan.backport {
        ensure!(
            plan.packages.len() == 1 && plan.packages[0].package == bp.package,
            "Patch release plan must contain exactly one package ({}) matching the backport branch.",
            bp.package
        );
        ensure!(
            plan.packages[0].bump == VersionBump::patch(),
            "Backport release for {} has a non-patch bump ({:?}); only Patch bumps are allowed.",
            plan.packages[0].package,
            plan.packages[0].bump
        );
    }

    // Preflight: validate every package up front, before touching any files, so
    // a mismatched version or other plan error aborts without leaving the
    // workspace half-edited.
    //
    // We deliberately do NOT reuse the manifests parsed here in the apply loop
    // below. Bumping a package rewrites the on-disk manifests of its workspace
    // dependents, so a package bumped after its dependencies must be re-read to
    // observe those rewrites. Saving a snapshot taken here would write it back
    // stale and silently revert every dependency bump an earlier step applied.
    let mut bump_decisions = Vec::with_capacity(plan.packages.len());
    for step in plan.packages.iter() {
        let package = CargoToml::new(workspace, step.package).with_context(|| {
            format!(
                "Couldn't create Cargo.toml in workspace {workspace:?} for {:?}",
                step.package
            )
        })?;

        match validate_package(&package, step)? {
            Preflight::Bump => bump_decisions.push(true),
            Preflight::AlreadyReleased => {
                println!(
                    "Package {} is already at version {}. Skipping.",
                    step.package, step.new_version
                );
                bump_decisions.push(false);
            }
        }
    }

    // Must run before the apply loop: `update_package` finalizes Unreleased into
    // the new version section, so fragments added after would land in the wrong
    // (new, empty) Unreleased.
    if args.no_dry_run {
        let modified = crate::commands::release::plan::generate_changelog_draft(workspace, &plan);
        if modified.is_empty() {
            println!(
                "Note: No changelog entries were merged. Run \
                `cargo xtask release changelog-preview` manually if needed."
            );
        } else {
            println!("Merged changelog entries into the following files:");
            for path in &modified {
                println!("  {}", path.display());
            }
        }
    } else {
        println!("Dry run: would merge PR changelog entries into CHANGELOG.md / MIGRATING-*.md");
    }

    // Make code changes. Re-read each manifest from disk instead of reusing the
    // preflight copies: earlier steps in this loop may have rewritten this
    // package's dependency versions on disk, and saving a stale in-memory copy
    // would revert them. Packages already at their target version are skipped.
    let skip_dependent_rewrites = plan.backport.is_some();
    for (step, should_bump) in plan.packages.iter_mut().zip(bump_decisions) {
        if !should_bump {
            continue;
        }

        let mut package = CargoToml::new(workspace, step.package).with_context(|| {
            format!(
                "Couldn't create Cargo.toml in workspace {workspace:?} for {:?}",
                step.package
            )
        })?;

        let new_version = update_package(
            &mut package,
            &step.bump,
            !args.no_dry_run,
            skip_dependent_rewrites,
        )?;

        step.tag_name = package.package.tag(&new_version);
        step.new_version = new_version;

        if step.package.is_semver_checked() {
            if args.no_dry_run {
                generate_baseline(
                    workspace,
                    vec![step.package],
                    if step.package.chip_features_matter() {
                        Chip::iter()
                            .filter(|c| step.package.validate_package_chip(c).is_ok())
                            .collect::<Vec<_>>()
                    } else {
                        vec![Chip::Esp32c6]
                    },
                )?;
            } else {
                println!(
                    "Dry run: would create semver baseline for package {}",
                    step.package
                );
            }
        }
    }

    // Update release plan file
    let plan_source = serde_json::to_string_pretty(&plan).with_context(|| {
        format!(
            "Failed to serialize release plan to {}",
            plan_path.display()
        )
    })?;

    if args.no_dry_run {
        std::fs::write(&plan_path, &plan_source)
            .with_context(|| format!("Failed to write release plan to {}", plan_path.display()))?;
    } else {
        println!(
            "Dry run: would write the updated release plan to {}",
            plan_path.display()
        );
    }

    let branch_name = format!("release-branch-{}", plan.slug);
    let branch = make_git_changes(!args.no_dry_run, &branch_name, &commit_message(&plan))?;

    open_pull_request(
        &branch,
        !args.no_dry_run,
        args.manual_pull_request,
        &plan_source,
        &plan,
    )
    .with_context(|| "Failed to open pull request")?;

    if !args.no_dry_run {
        println!(
            "Dry run completed. To make changes, run `cargo xrelease execute-plan --no-dry-run`."
        );
    }

    Ok(())
}

/// Outcome of validating a package against its plan entry.
enum Preflight {
    /// The package is at its expected current version and should be bumped.
    Bump,
    /// The package is already at the planned new version, so there is nothing
    /// to do (e.g. `execute-plan` is being re-run). Skip it.
    AlreadyReleased,
}

/// Validate a package against its plan entry without mutating anything.
///
/// Run for every package before any changelog or version edits happen, so that
/// a bad plan aborts cleanly instead of leaving the workspace half-edited. The
/// same checks gate the actual bump, but live here only — the apply loop reuses
/// the [`Preflight`] result instead of repeating them.
fn validate_package(package: &CargoToml, step: &PackagePlan) -> Result<Preflight> {
    let current = package.package_version();
    if current != step.current_version {
        if current == step.new_version {
            return Ok(Preflight::AlreadyReleased);
        }
        bail!(
            "The version of package {} has changed in an unexpected way. Cannot continue.",
            step.package
        );
    }

    // Special case: some packages are perma-unstable, meaning they won't ever
    // have a stable release. For these packages, we always use a patch release.
    if let Some(true) = package.espressif_metadata_bool("forever-unstable")
        && step.bump != VersionBump::patch()
    {
        bail!(
            "Cannot bump perma-unstable package {} to a non-patch version",
            step.package
        );
    }

    Ok(Preflight::Bump)
}

pub(crate) struct Branch {
    pub name: String,
    pub upstream: String,
}

pub(crate) fn make_git_changes(dry_run: bool, branch_name: &str, commit: &str) -> Result<Branch> {
    let branch_name = branch_name.to_string();
    let upstream = get_remote_name_for("esp-rs/esp-hal")?;

    // `git switch -C` creates the branch, or resets it to HEAD if it already
    // exists — carrying our uncommitted version-bump edits along. This is
    // what makes re-running `execute-plan` idempotent: the branch simply
    // snaps back to a single fresh commit off the base.
    if dry_run {
        println!("Dry run: would switch to branch: {branch_name}");
    } else {
        let status = Command::new("git")
            .arg("switch")
            .arg("-C")
            .arg(&branch_name)
            .status()
            .context("Failed to switch to release branch")?;

        if !status.success() {
            bail!("Failed to switch to release branch: {branch_name}");
        }
    }

    // Commit the changes
    if dry_run {
        println!("Dry run: would commit changes to branch: {branch_name}");
    } else {
        let status = Command::new("git")
            .arg("add")
            .arg(".")
            .status()
            .context("Failed to stage changes")?;
        if !status.success() {
            bail!("Failed to stage changes");
        }

        let status = Command::new("git")
            .arg("commit")
            .arg("-m")
            .arg(commit)
            .status()
            .context("Failed to commit changes")?;
        if !status.success() {
            bail!("Failed to commit changes to branch: {branch_name}");
        }
    }

    // Push the branch. If the remote branch already exists (e.g. this is a
    // re-run), fetch it first so the remote-tracking ref is current, then use
    // --force-with-lease so we overwrite our own prior push but bail if
    // someone else pushed in the meantime.
    let url = if dry_run {
        println!("Dry run: would push branch: {branch_name}");
        String::from("https://github.com/esp-rs/esp-hal/")
    } else {
        let remote_exists = Command::new("git")
            .args(["ls-remote", "--exit-code", "--heads"])
            .arg(&upstream)
            .arg(&branch_name)
            .status()
            .context("Failed to query remote branches")?
            .success();

        if remote_exists {
            log::info!("Remote branch {branch_name} already exists — fetching before force-push");
            let fetch_status = Command::new("git")
                .arg("fetch")
                .arg(&upstream)
                .arg(&branch_name)
                .status()
                .context("Failed to fetch existing release branch before force-push")?;
            if !fetch_status.success() {
                bail!("Failed to fetch existing release branch {branch_name} from {upstream}");
            }
        }

        let mut push = Command::new("git");
        push.arg("push").arg(&upstream).arg(&branch_name);
        if remote_exists {
            log::info!("Force-pushing {branch_name} with lease");
            push.arg("--force-with-lease");
        }
        let message = push.output().context("Failed to push branch")?;

        if !message.status.success() {
            bail!(
                "Failed to push branch: {}",
                String::from_utf8_lossy(&message.stderr)
            );
        }

        log::info!("Pushed release branch: {upstream}/{branch_name}");

        // Extract the URL from the output
        extract_url_from_push(&String::from_utf8_lossy(&message.stderr)) // git outputs to stderr
    };

    Ok(Branch {
        name: branch_name,
        upstream: url,
    })
}

fn release_subject(plan: &Plan) -> String {
    match plan.packages.len() {
        1 => {
            let step = &plan.packages[0];
            format!(
                "Release {}: {} → {}",
                step.package, step.current_version, step.new_version
            )
        }
        n => format!("Release {n} packages"),
    }
}

fn format_package_list(plan: &Plan) -> String {
    plan.packages
        .iter()
        .map(|step| {
            format!(
                "- {}: {} → {}",
                step.package, step.current_version, step.new_version
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn commit_message(plan: &Plan) -> String {
    let subject = release_subject(plan);
    let body = format_package_list(plan);
    format!("{subject}\n\n{body}\n")
}

const UPSTREAM_REPO: &str = "esp-rs/esp-hal";
// `manual-changelog` exempts the PR from the direct-CHANGELOG-edit check (the release
// tooling writes the changelog wholesale). The `release:*` labels each gate an optional,
// heavy CI workflow; all are applied by default so every check runs, and a maintainer can
// remove individual ones to skip a check that isn't relevant to the packages being released.
// `merge-freeze-exempt` lets the PR through the merge queue during a release freeze; it is a
// no-op when no freeze is active.
const PR_LABELS: &[&str] = &[
    "manual-changelog",
    "release:docs",
    "release:registry:compile-test",
    "release:registry:ci",
    "merge-freeze-exempt",
];

fn build_pr_body(plan: &Plan, release_plan_str: &str) -> String {
    let packages = format_package_list(plan);

    let mut body = format!(
        r#"This pull request prepares the following packages for release:

{packages}

<details>

<summary>Release plan (click to expand)</summary>

```jsonc
{release_plan_str}
```

</details>

Please review the changes and merge them into the `{base}` branch.

After merging, please make sure you have this release plan in the repo root,
then run the following command on the `{base}` branch to tag and publish the packages:

```
cargo xrelease publish-plan --no-dry-run
```
"#,
        base = plan.base,
    );

    if plan.base != "main" {
        body = format!(
            "⚠️ This pull request was branched off from `{}`. ⚠️\n\n{body}",
            plan.base
        );
    }

    body
}

fn open_pull_request(
    branch: &Branch,
    dry_run: bool,
    manual_pull_request: bool,
    release_plan_str: &str,
    release_plan: &Plan,
) -> Result<()> {
    let body = build_pr_body(release_plan, release_plan_str);

    if dry_run {
        println!("Dry run: would create/update the release PR with body:");
        println!("----");
        println!("{body}");
        println!("----");
        return Ok(());
    }

    if manual_pull_request || !gh_available() {
        if !manual_pull_request {
            log::warn!("`gh` CLI not available — falling back to manual PR instructions");
        }
        return print_manual_instructions(branch, release_plan, &body);
    }

    let pr_number = upsert_pull_request(branch, release_plan, &body)?;

    println!(
        "Release PR ready: https://github.com/{UPSTREAM_REPO}/pull/{pr_number} — review and merge to continue."
    );
    Ok(())
}

fn gh_available() -> bool {
    Command::new("gh")
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn head_spec(upstream_url: &str, branch_name: &str) -> Result<String> {
    if upstream_url.starts_with("https://github.com/esp-rs/") || upstream_url.is_empty() {
        Ok(branch_name.to_string())
    } else {
        let user = upstream_url
            .split('/')
            .nth(3)
            .with_context(|| format!("Failed to extract user from URL: {upstream_url}"))?;
        Ok(format!("{user}:{branch_name}"))
    }
}

/// Spawn `gh` with `args`, feed `stdin_data` to its stdin, and return the raw
/// output. A non-zero exit is *not* treated as an error here so callers can
/// inspect stderr (e.g. to detect "a pull request already exists"). Callers
/// that only care about success should use [`gh_stdin`].
fn gh_run(args: &[&str], stdin_data: &str) -> Result<std::process::Output> {
    use std::io::Write;

    let mut child = Command::new("gh")
        .args(args)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .with_context(|| format!("Failed to spawn `gh {}`", args.join(" ")))?;

    child
        .stdin
        .as_mut()
        .expect("stdin piped")
        .write_all(stdin_data.as_bytes())
        .context("Failed to write stdin to gh")?;

    child
        .wait_with_output()
        .with_context(|| format!("`gh {}` failed", args.join(" ")))
}

fn gh_stdin(args: &[&str], stdin_data: &str) -> Result<String> {
    let out = gh_run(args, stdin_data)?;
    if !out.status.success() {
        bail!(
            "`gh {}` failed: {}",
            args.join(" "),
            String::from_utf8_lossy(&out.stderr)
        );
    }
    Ok(String::from_utf8_lossy(&out.stdout).to_string())
}

fn find_existing_pr(branch_name: &str) -> Result<Option<u64>> {
    let output = Command::new("gh")
        .args([
            "pr",
            "list",
            "--state",
            "open",
            "--repo",
            UPSTREAM_REPO,
            "--head",
        ])
        .arg(branch_name)
        .args(["--json", "number"])
        .output()
        .context("`gh pr list` failed")?;
    if !output.status.success() {
        bail!(
            "`gh pr list` failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let value: serde_json::Value =
        serde_json::from_slice(&output.stdout).context("Failed to parse `gh pr list` output")?;
    Ok(value
        .as_array()
        .and_then(|a| a.first())
        .and_then(|o| o.get("number"))
        .and_then(|n| n.as_u64()))
}

/// Extract the numeric PR id from any text containing a `.../pull/<n>` URL.
///
/// Works for both the URL `gh pr create` prints on success and the
/// "...already exists: <url>" message it prints on failure. A `/pull/new/...`
/// URL (as produced by `git push`) has no number and yields `None`.
fn parse_pr_number(text: &str) -> Option<u64> {
    let start = text.rfind("/pull/")? + "/pull/".len();
    let digits: String = text[start..]
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .collect();
    digits.parse().ok()
}

/// Update the title and body of an existing release PR, leaving labels and
/// reviewers untouched.
fn edit_release_pr(number: u64, title: &str, body: &str) -> Result<()> {
    let number = number.to_string();
    gh_stdin(
        &[
            "pr",
            "edit",
            &number,
            "--repo",
            UPSTREAM_REPO,
            "--title",
            title,
            "--body-file",
            "-",
        ],
        body,
    )?;
    Ok(())
}

fn upsert_pull_request(branch: &Branch, plan: &Plan, body: &str) -> Result<u64> {
    let title = release_subject(plan);

    // Reuse an existing open release PR when we can find one, so re-running the
    // release edits the same PR instead of opening (and re-labelling) another.
    if let Some(num) = find_existing_pr(&branch.name)? {
        log::info!("Updating existing release PR #{num}");
        edit_release_pr(num, &title, body)?;
        return Ok(num);
    }

    let head = head_spec(&branch.upstream, &branch.name)?;
    log::info!("Creating release PR (head: {head}, base: {})", plan.base);
    let mut args: Vec<&str> = vec![
        "pr",
        "create",
        "--repo",
        UPSTREAM_REPO,
        "--base",
        &plan.base,
        "--head",
        &head,
        "--title",
        &title,
    ];
    // Labels are applied only when the PR is first created. A maintainer may
    // later remove one to skip its optional CI check, so the edit path above
    // deliberately never re-applies them.
    for l in PR_LABELS {
        args.push("--label");
        args.push(l);
    }
    args.push("--body-file");
    args.push("-");

    let out = gh_run(&args, body)?;
    if out.status.success() {
        let stdout = String::from_utf8_lossy(&out.stdout);
        return parse_pr_number(&stdout).with_context(|| {
            format!("Failed to parse PR number from `gh pr create` output: {stdout}")
        });
    }

    // GitHub's PR listing lags for a moment after the branch is pushed, so
    // `find_existing_pr` can miss a PR that actually exists and we land here on
    // a re-run. `gh pr create` then fails with "a pull request ... already
    // exists: <url>". Recover by editing that PR instead of bailing: letting
    // `gh pr create` run a second time is what re-applied every release label
    // and re-requested reviews, showing up as duplicated PR timeline entries.
    let stderr = String::from_utf8_lossy(&out.stderr);
    if stderr.contains("already exists")
        && let Some(num) = parse_pr_number(&stderr)
    {
        log::info!("Release PR #{num} already exists - updating it instead of opening a duplicate");
        edit_release_pr(num, &title, body)?;
        return Ok(num);
    }

    bail!("`gh pr create` failed: {stderr}");
}

fn print_manual_instructions(branch: &Branch, plan: &Plan, body: &str) -> Result<()> {
    let url = comparison_url(&plan.base, &branch.upstream, &branch.name)?;
    println!();
    println!("Open the following URL in a browser to create the PR:");
    println!("  {url}");
    println!();
    println!("Paste this as the PR description:");
    println!("----");
    println!("{body}");
    println!("----");
    Ok(())
}

fn extract_url_from_push(output: &str) -> String {
    output
        .lines()
        .map(|s| s.trim_start_matches("remote:"))
        .map(|s| s.trim())
        .find(|&s| s.starts_with("https://"))
        .unwrap_or("")
        .to_string()
}

pub(crate) fn comparison_url(base: &str, url: &str, branch_name: &str) -> Result<String> {
    let url = if url.starts_with("https://github.com/esp-rs/") {
        format!("https://github.com/esp-rs/esp-hal/compare/{base}...{branch_name}")
    } else {
        let Some(user) = url.split('/').nth(3) else {
            bail!("Failed to extract user from URL: {url}");
        };
        format!("https://github.com/esp-rs/esp-hal/compare/{base}...{user}:esp-hal:{branch_name}")
    };

    Ok(url)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_url() {
        let message = "Enumerating objects: 20, done.
Counting objects: 100% (20/20), done.
Delta compression using up to 16 threads
Compressing objects: 100% (14/14), done.
Writing objects: 100% (14/14), 2.61 KiB | 2.61 MiB/s, done.
Total 14 (delta 13), reused 0 (delta 0), pack-reused 0 (from 0)
remote: Resolving deltas: 100% (13/13), completed with 6 local objects.
remote:
remote: Create a pull request for 'foo' on GitHub by visiting:
remote:      https://github.com/bugadani/esp-hal/pull/new/foo
remote:
To https://github.com/bugadani/esp-hal.git
 * [new branch]          foo -> foo
branch 'foo' set up to track 'origin/foo'.
";

        let url = extract_url_from_push(message);
        assert_eq!(url, "https://github.com/bugadani/esp-hal/pull/new/foo");
    }

    #[test]
    fn parse_pr_number_from_create_and_already_exists() {
        // `gh pr create` success: bare PR URL on stdout.
        assert_eq!(
            parse_pr_number("https://github.com/esp-rs/esp-hal/pull/1234\n"),
            Some(1234)
        );

        // `gh pr create` failure when the PR already exists: the number must be
        // recovered from the message so we can edit instead of duplicating.
        let stderr = "a pull request for branch \"release-branch-t9telr\" into branch \"main\" \
             already exists:\nhttps://github.com/esp-rs/esp-hal/pull/6190\n";
        assert_eq!(parse_pr_number(stderr), Some(6190));

        // A `/pull/new/<branch>` URL (from `git push`) carries no PR number.
        assert_eq!(
            parse_pr_number("https://github.com/esp-rs/esp-hal/pull/new/release-branch-x"),
            None
        );

        assert_eq!(parse_pr_number("no pull url here"), None);
    }

    #[test]
    fn create_comparison_url() {
        let cases = [
            // From forked repo
            (
                "https://github.com/bugadani/esp-hal/pull/new/foo",
                ("main", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/main...bugadani:esp-hal:foo",
            ),
            (
                "https://github.com/bugadani/esp-hal/pull/new/foo",
                ("backport", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/backport...bugadani:esp-hal:foo",
            ),
            // From upstream
            (
                "https://github.com/esp-rs/esp-hal/pull/new/foo",
                ("main", "foo"),
                "https://github.com/esp-rs/esp-hal/compare/main...foo",
            ),
        ];

        for (input_url, (current_branch, release_branch), expected_url) in cases {
            let url = comparison_url(current_branch, input_url, release_branch)
                .expect("Failed to create PR URL");
            assert_eq!(url, expected_url);
        }
    }
}
