# HIL/CI Commands Guide

This document describes all comments-based CI controls available on pull requests.

By default, only `esp-rs` organization **members**s and **owners**s can trigger HIL or binary-size jobs. However, they may grant similar rights to other users, see [Trust Management Commands] section for details

[Trust Management Commands]: #trust-management-commands

## HIL Test Commands

Behind the scenes, all comment-commands are first handled by `dispatch.yml`.  
This workflow parses the comment, checks trust/permissions, and then triggers the appropriate workflow (such as `hil.yml` or `binary_size.yml`) via `workflow_dispatch`.

### `/hil quick`

Runs a **minimal** HIL matrix on a limited set of chips:

- Currently: **ESP32-S3** (Xtensa) and **ESP32-C6** (RISC-V)

The CI will:

1. Trigger appropriate workflow.
2. Post a comment with a link to the triggered HIL run.
3. Edit that comment later with a status update (succeeded / failed / cancelled / still running).

In further commands, the feedback from the bot will be identical.

### `/hil full`

Runs the **full** HIL matrix on **all supported chips**.

The full matrix run will also be executed in the merge queue and is a prerequisite for a successful merge.

### `/hil <chip> [<chip> ...]`

Runs HIL tests **only for the listed chips**.

Examples:

- `/hil esp32c3`
- `/hil esp32c3 esp32s3`
- `/hil esp32c3,esp32c6, esp32s3`

### `/test-size`

Triggers the binary size analysis workflow, which:

- Builds selected binaries for the PR and for the base branch.
- Runs `bloaty` to compare section sizes.
- Generates a report.

## Trust Management Commands

These commands are for maintainers (MEMBER/OWNER) and operate per PR.

### `/trust @username`

- Adds `@username` to the PR’s HIL trust list.
- Posts a short confirmation comment and updates/creates the “HIL trust list” comment.

Trusted users can then run all the aforementioned commands.

### `trusted-author` label

When this label is assigned, the author of this PR will receive the rights to run HIL and binary size tests, just as when using the `/trust` command.

### `/revoke @username`

- Removes `@username` from the PR’s HIL trust list.
- Updates the “HIL trust list” bot comment.
- Posts a short confirmation comment.

After revocation, the user loses access to the commands above for that PR.

## Help / Usage Hints

If you will request for `/hil help` or `/hil` without a valid variant or chips, the bot will respond with a short usage explanation, and a reminder that the requester must be a maintainer or trusted author.
