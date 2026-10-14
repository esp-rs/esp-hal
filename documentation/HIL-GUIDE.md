# HIL/CI Commands Guide

This document describes all comments-based CI controls available on pull requests.

By default, only `esp-rs` organization **members** and **owners** can trigger HIL or binary-size jobs. However, they may grant similar rights to other users, see [Trust Management Commands] section for details

[Trust Management Commands]: #trust-management-commands

## HIL Test Commands

Behind the scenes, all comment-commands are first handled by `dispatch.yml`.  
This workflow parses the comment, checks trust/permissions, and then triggers the appropriate workflow (such as `hil.yml` or `binary_size.yml`) via `workflow_dispatch`.

The words of a command go in any order, like the arguments of `cargo xtask test`: each is a chip, a package, a keyword such as `quick`, or else a test name. Separate them with spaces or commas, so `/test esp32 rmt c6 i2c` runs `rmt` and `i2c` on ESP32 and ESP32-C6. Chips can be shortened as in xtask, so `c6`, `esp32-c6` and `ESP32C6` all mean `esp32c6`. Only the first line of the comment is read, the rest is free text.

`/hil` is the previous spelling of `/test` and still works, with the same words.

### `/test quick`

Runs a **minimal** HIL matrix on a limited set of chips:

- Currently: **ESP32-S3** (Xtensa) and **ESP32-C6** (RISC-V)

The CI will:

1. Trigger appropriate workflow.
2. Post a comment with a link to the triggered HIL run.
3. Edit that comment when the run finishes with a status update (succeeded / failed / cancelled), and re-edit that same update whenever the run is re-run.

In further commands, the feedback from the bot will be identical.

### `/test full`

Runs the **full** HIL matrix on **all supported chips**. `/test all` does the same.

The full matrix run will also be executed in the merge queue and is a prerequisite for a successful merge.

### `/test <chip> [<chip> ...]`

Runs HIL tests **only for the listed chips**.

Examples:

- `/test c3`
- `/test esp32c3 s3`
- `/test c3, c6, ESP32-S3`

Chips can be added to the quick matrix too: `/test quick c3` runs on ESP32-S3, ESP32-C6 and ESP32-C3.

### `/test <matrix/chips> <test> [<test> ...]`

Runs **only chosen** HIL tests for selected chip(-s) or matrix. Every word that is not a chip, a package or a keyword is a test name, wherever it stands.

Examples:

- `/test quick rmt`
- `/test esp32 rmt c6 i2c`
- `/test c6 radio_basic::test_scan_doesnt_leak`

Use `binary::test_name` to run a single embedded-test case inside a matching HIL test binary. For example, `/test c6 radio_basic::test_scan_doesnt_leak` builds the `radio_basic` ELF and forwards `test_scan_doesnt_leak` as the embedded-test filter when CI runs the prebuilt ELF.

The `--test` and `--tests` flags of xtask are accepted as well, `/test c6 --tests rmt, i2c` is the same as `/test c6 rmt i2c`.

### Radio HIL

By default, `/test` runs the `hil-test` package. Add `radio` (or `hil-test-radio`) to run the radio HIL package instead, or both `radio` and `hil-test` to run both.

Examples:

- `/test c6 radio`
- `/test c6 radio wifi`
- `/test quick radio hil-test`

### `/test-size <example> [<example> ...] <chip> [<chip> ...]`

Triggers the binary size analysis workflow, which:

- Builds the selected example for the PR head and for the commit the PR branched from.
- Compares the size of every section that ends up in the image.
- Reports the flash and bss totals and the sections that changed in a comment on the PR.

Examples:

- `/test-size hello_world c6`
- `/test-size embassy_hello_world esp32 c3 s3`
- `/test-size hello_world embassy_hello_world c6`
- `/test-size embassy-adc qa c6`

Every word that is not a chip or a package is an example. At least one example and one chip are required.

By default, the examples come from the `examples` package, the projects under `examples/`. Add `qa` (or `qa-test`) for binaries from `examples/qa/src/bin`, named after their files. `--package qa` from before still works.

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

If you request `/test help` or `/test` alone, the bot will respond with a short usage explanation, and a reminder that the requester must be a maintainer or trusted author. A `/test` or `/test-size` command that can't be run, for example one without a chip or with an unknown chip like `c66`, gets the same explanation with what went wrong. `/hil` is answered the same way.
