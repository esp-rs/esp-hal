
# Welcome to the `esp-hal` Contributing Guide

Thank you for considering contributing to our project! Your efforts help make `esp-hal` a better ecosystem for everyone.

This guide outlines the contribution workflow, from reporting issues and submitting pull requests, to the review process and eventual merger of contributions.

## Quick Navigation
*   [New Contributor Guide]
*   [Getting Started]
    *   [Issues: Reporting and Resolving]
    *   [Making Changes: Fork, Edit, and Pull Request]
    *   [Testing Your Contributions]
    *   [Commit Your Updates]
*   [Pull Request: From Submission to Merge]
    *   [Writing the description]
*   [AI-Assisted Contributions]
*   [Merge Freezes]
*   [Your PR is merged!]

[New Contributor Guide]: #new-contributor-guide
[Getting Started]: #getting-started
[Issues: Reporting and Resolving]: #issues-reporting-and-resolving
[Making Changes: Fork, Edit, and Pull Request]: #making-changes-fork-edit-and-pull-request
[Testing Your Contributions]: #testing-your-contributions
[Commit your updates]: #commit-your-updates
[Pull Request: From Submission to Merge]: #pull-request-from-submission-to-merge
[Writing the description]: #writing-the-description
[AI-Assisted Contributions]: #ai-assisted-contributions
[Merge Freezes]: #merge-freezes
[Your PR is merged!]: #your-pr-is-merged

## New Contributor Guide

Welcome aboard! If you're new to `esp-hal` or open-source contribution, here are some resources to get you started:

*   Intro to Open Source Contribution: [GitHub's Guide]
*   [Setting Up Git]
*   Workflow Insights: [GitHub Flow]
*   Collaborating via [Pull Requests]

Before adding or changing code, review the [esp-rs developer guidelines].

[GitHub's Guide]: https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github
[Setting Up Git]: https://docs.github.com/en/get-started/quickstart/set-up-git
[GitHub Flow]: https://docs.github.com/en/get-started/quickstart/github-flow
[Pull Requests]: https://docs.github.com/en/github/collaborating-with-pull-requests
[esp-rs developer guidelines]: ./DEVELOPER-GUIDELINES.md
[HIL guide]: https://github.com/esp-rs/esp-hal/blob/main/documentation/HIL-GUIDE.md

## Getting Started

### Issues: Reporting and Resolving

#### Reporting a New Issue

Encountered a problem or have an idea? First, [check existing issues] to avoid duplicates. If your concern is new, use our [issue form] to submit it.

Include only the information someone needs to reproduce the problem, or at least to explore it, and
leave out everything else. [REPRODUCERS.md] describes a good minimal reproducer.

If an AI tool helps you write the issue, also follow [AI-Assisted Contributions].

[check existing issues]: https://github.com/esp-rs/esp-hal/issues
[issue form]: https://github.com/esp-rs/esp-hal/issues/new/
[REPRODUCERS.md]: ./REPRODUCERS.md

#### Working on an Issue

Browse [existing issues] to find one that resonates with you. Use labels for easier filtering. If you decide to tackle an issue, it's courteous (but not mandatory) to let others know by commenting.

[existing issues]: https://github.com/esp-rs/esp-hal/issues

#### Making Changes: Fork, Edit, and Pull Request

1. **Fork**: Start by [forking the repository]. This keeps the main project safe while you make your changes.
2. **Setup**: Ensure you have the latest Rust toolchain via [rustup.rs].
3. **Branch**: Create a branch in your fork for your changes. Keep your changes focused and limited to a single issue or feature.

[forking the repository]: https://docs.github.com/en/github/getting-started-with-github/fork-a-repo
[rustup.rs]: https://rustup.rs/

#### What You Should Do:

* **API changes**: If your contribution changes the API, please adapt the driver (including module level documentation) and examples accordingly and update the [HIL] (Hardware-in-the-Loop) tests.
* **Run Related Examples**: After making changes, run any affected examples to ensure they build successfully and perform as expected.
* **Manual Testing**: For hardware-related changes, manually test your changes on the actual devices when possible. If not, please note it in the corresponding issue, and someone from our team will assist with testing. This is crucial because hardware behavior can sometimes differ from what's simulated or expected.
* **HIL Tests**: Ensure that any changes to the API or hardware interaction logic are reflected in the HIL tests located in the `hil-test` directory. This helps verify the real-world applicability of your changes.

By taking these extra steps to test your contributions, you help maintain the high quality and reliability of `esp-hal`, ensuring it remains a robust platform for everyone.

[HIL]: https://github.com/esp-rs/esp-hal/tree/main/hil-test

### Testing Your Contributions

Ensuring the quality and reliability of `esp-hal` is a shared responsibility, and testing plays a critical role in this process. Our GitHub CI automatically checks the buildability of all examples and drivers within the project. However, automated tests can't catch everything, especially when it comes to the nuanced behavior of hardware interactions. So make sure that the example affected by your change works as expected.

Further steps that can (or should) be taken in testing:

* Using [xtask], build examples for the specified chip (`cargo xtask build <name> <chip>`, or `cargo xtask build examples all <chip>`).
* When documentation or doctests change, run `cargo xtask documentation` and `cargo xtask doc-tests <CHIP>` to build the documentation and run the doctests. To reduce build/test time, use `--packages` to specify the package(s) and pass chip names to `documentation`.
* Run the [HIL] tests locally if changes have been made to them.
* Run host-side unit tests with `cargo xtask host-tests` (or `cargo xtask host-tests <package>`). When adding `#[test]` functions to a package for the first time, also add a match arm for that package in `run_host_tests` in `xtask/src/lib.rs` — see [xtask/README.md#host-tests].

For detailed instructions on how to use our HIL tests with comment commands, see the [HIL guide].

[xtask]: https://github.com/esp-rs/esp-hal/tree/main/xtask

### Commit Your Updates

Commit your changes once you're satisfied. Review your own work to streamline the review process later. Use `rustfmt` and `cargo clippy` to ensure your code adheres to Rust's conventions.

```shell
rustup component add rustfmt
rustup component add clippy
```

We _strongly_ recommend that you format your code before committing to ensure consistency throughout the project.
To format all packages in the workspace, run the following command in a terminal from the root of the repository:

```shell
cargo xtask fmt
```

We also recommend using the `lint` subcommand, which uses `cargo clippy` and will lint the entire driver in order to catch common mistakes in the code.

```shell
cargo xtask lint
```

This will use `rustfmt` to ensure that all source code is formatted correctly prior to committing.

## Pull Request: From Submission to Merge

*   Fill the pull request template so that we can review your PR. This template helps reviewers understand your changes as well as the purpose of your pull request.
*   [Link your PR] to any relevant issues it addresses.
*   [Allow edits from maintainers] so the branch can be updated for a merge. Once you submit your PR, a Docs team member will review your proposal. We may ask questions or request additional information.
*   If your change is user-visible, consider adding a brief changelog entry and/or migration guide note in the PR description using the structured sections provided by the template (see below). This is optional — if you skip it, a maintainer will either add the entries or apply the `skip-changelog` label on your behalf. Do **not** edit `CHANGELOG.md` files directly — those are updated automatically at release time.
*   If your change requires user code to be updated, add a `# Migration guide` section. Each breaking change needs a `## crate/area` heading and a `### Title` for the specific change, followed by the migration steps.
*   If your change breaks the stable API of a crate, the PR needs the `breaking-change-<crate-name>` label (e.g. `breaking-change-esp-hal`). Without it, the semver check in CI fails. Ask a maintainer to add the label. See [Breaking changes] for details.
*   We may ask for changes to be made before a PR can be merged, either using [suggested changes] or pull request comments. You can apply suggested changes directly through the UI. You can make any other changes in your fork, then commit them to your branch.
*   As you update your PR and apply changes, mark each conversation as [resolved].
*   Resolve merge conflicts if they arise, using resources like [this git tutorial] for help.

[Link your PR]: https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue
[Allow edits from maintainers]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/allowing-changes-to-a-pull-request-branch-created-from-a-fork#enabling-repository-maintainer-permissions-on-existing-pull-requests
[suggested changes]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/incorporating-feedback-in-your-pull-request
[resolved]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/commenting-on-a-pull-request#resolving-conversations
[this git tutorial]: https://github.com/skills/resolve-merge-conflicts
[Breaking changes]: ./DEVELOPER-GUIDELINES.md#breaking-changes

### Writing the description

Reviewers read the diff. The description tells them what the diff cannot: why the change is needed
and how you know it works. Include only the information a reviewer needs to review the change, and
leave out everything else. Two sentences are enough for a small fix. A timing bug needs its
measurements.

- Open with the problem and the fix, and link the issue (`Closes #1234`).
- Under **Testing**, name the chip, the board, and the example or HIL test you ran, and the affected
  chips you could not test. Leave out `fmt`, `lint`, and other checks CI runs.
- Write each changelog entry as one line describing the user-visible effect. The explanation belongs
  in the description.

The part of the template you write should look like this:

```markdown
#### Description

What was wrong or missing, and what this changes.

Closes #1234

#### Testing

Ran `<example or HIL test>` on <chip> (<board>). Not tested on <chips>.
```

If an AI tool wrote a significant part of your contribution, also follow [AI-Assisted Contributions].

## AI-Assisted Contributions

> [!NOTE]
> This section applies only when an AI tool writes a significant part of your contribution: the
> code, the pull request description, or an issue. Its requirements add to the general rules in
> [Reporting a New Issue] and [Writing the description]. If you write your contribution yourself,
> skip this section.

We follow the Rust Embedded working group's [AI tool use policy]. You are responsible for everything
you submit, so review and test it before you ask for a review.

- An AI agent must not open issues automatically. We close those issues without looking at them.
  Verify that the issue is real, and use the correct issue form.
- Say in one line of the pull request description that an AI tool generated a significant part of
  the change.
- Write pull request descriptions in Simplified Technical English (ASD-STE100), as the API
  documentation does. Follow its [Style](./API-DOC-RULES.md#style) and
  [Wording](./API-DOC-RULES.md#wording) rules: short sentences with one idea each, active voice,
  and the present tense.
- Leave out the template's greeting and submission checklist, and start the body at
  `#### Description`.
- Do not walk through the diff, restate the issue, or describe how you found the bug.
- Raise trade-offs, open questions, or follow-ups only when you need a reviewer's decision.
- Add no headings beyond the template's, no emoji, and no sign-offs.

[Reporting a New Issue]: #reporting-a-new-issue
[AI tool use policy]: https://github.com/rust-embedded/wg/blob/HEAD/CODE_OF_CONDUCT.md#ai-tool-use-policy


## Merge Freezes

Around a release we stop merging into `main`, so that what gets published is
tested in isolation from new changes.

A "freeze" is an open issue labelled `merge-freeze`, pinned so that everyone can
see it. Anyone with write access starts a freeze by opening or reopening that
issue and lifts it by closing it. Review and CI carry on as usual, only the
merge queue rejects pull requests targeting `main` while it is open.

To land a release blocker anyway, label the pull request with
`merge-freeze-exempt`. Such merge gets recorded on the freeze issue.

## Changelog and Migration Guide Entries

Changelog entries are **optional for contributors**. If you don't add them a
maintainer will either write them or apply the `skip-changelog` label before the
PR is merged.

If you do want to document your change, add entries directly in the PR description
using the structured sections in the template. Do **not** edit `CHANGELOG.md` —
those files are updated automatically at release time from the PR descriptions.

### Format

Use `# Changelog` and `# Migration guide` as top-level headings (H1). Under each
heading, group entries using H2 headings. Changelog H2 headings may use just the
crate name (e.g. `## esp-hal`), while migration guide H2 headings _must_ include
an area (e.g. `## esp-hal/SPI driver`).

Only published crates can have changelog entries. CI rejects sections for crates
with `publish = false` in their `Cargo.toml` (e.g. `esp-metadata`, `xtask`,
`hil-test`). Published crates with `changelog-exempt = true` under
`[package.metadata.espressif]` (e.g. `esp-metadata-generated`) do not need a
section.

```markdown
# Changelog

## esp-hal

- Added: Support for the Foo peripheral.
- Fixed: A bug in the Bar driver that caused incorrect output.

## esp-hal/SPI driver

- Changed: `SpiDevice::transfer` now accepts a mutable slice.

# Migration guide

## esp-hal/SPI driver

### `SpiDevice::transfer` signature changed

`SpiDevice::transfer` now takes `&mut [u8]` instead of `(&[u8], &mut [u8])`.
Update your call sites accordingly.
```

### Entry kinds

Each item in the `# Changelog` section must begin with one of:

| Kind      | When to use                                        |
| --------- | -------------------------------------------------- |
| `Added`   | New public API, feature, or peripheral support     |
| `Changed` | Behaviour or API change (non-breaking preferred)   |
| `Fixed`   | Bug fixes                                          |
| `Removed` | Removed API or feature                             |

### Skipping the changelog for a specific crate

If your PR touches a published crate but the change genuinely needs no
user-visible entry (e.g. a documentation fix, an internal refactor, or a
build-system tweak), you can exempt that crate by writing the special marker
as the **sole** item in its `# Changelog` section:

```markdown
# Changelog

## esp-hal

- No changelog necessary.
```

This tells CI that the omission is intentional.  The marker must be the only
item in the section — combining it with real entries is an error.

When the whole PR needs no changelog at all, a maintainer can apply the
`skip-changelog` label instead.

### Manually editing `CHANGELOG.md`

In rare cases — such as backports, hotfixes, or curated release notes that
cannot be expressed through the structured PR description format — a maintainer
can apply the `manual-changelog` label to a PR.  This label:

- Allows direct edits to `CHANGELOG.md` files (the automated "no direct
  CHANGELOG.md edits" check is skipped).
- Skips the per-package coverage check (the PR is not required to have
  structured entries in the description).
- Still validates any PR description entries that *are* present, as a
  safety net against formatting accidents.

Do **not** use `manual-changelog` for routine changes — the structured PR
description format exists precisely to keep changelog maintenance consistent
and automatable.

### Validation

You can validate your PR description locally before pushing:

```shell
# Pipe the body directly:
echo "# Changelog\n\n## esp-hal\n\n- Added: Something." | cargo xtask check-pr-changelog

# Or validate an open PR by number (requires the `gh` CLI):
cargo xtask check-pr-changelog --pr 1234
```

CI will also validate the format automatically on every PR.

## Your PR is Merged!

Congratulations! The `esp-rs` team thanks you for your contributions!

Contributing to open source extends beyond just code! Each contribution, regardless of size, plays a significant role. We appreciate your involvement in this collective endeavor.
