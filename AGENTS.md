# AGENTS.md

esp-hal is a monorepo of bare-metal `no_std` Rust crates for Espressif chips. Each top-level crate
directory is its own Cargo project with a README. The root workspace only holds the `xtask` tooling,
and all building, linting and testing goes through `cargo xtask`.

This file is an index. Before you change anything, read the documents that cover your task.

## Where to look

| Task | Read |
| --- | --- |
| Any code change: API design, drivers, cfg symbols, metadata | `documentation/DEVELOPER-GUIDELINES.md` |
| Rustdoc | `documentation/API-DOC-RULES.md` |
| xtask commands, `//%` annotations in examples and tests, host tests | `xtask/README.md` |
| Opening a PR, changelog and migration entries | `documentation/CONTRIBUTING.md` |
| Examples | `examples/README.md` |
| HIL tests | `hil-test/README.md`, `documentation/HIL-GUIDE.md` |
| Build-time configuration (`esp_config.yml`) | `esp-config/README.md` |
| New chip bring-up | `documentation/CHIP_BRING_UP.md` |
| Releases | `documentation/RELEASING.md` |
| Chip list and Rust targets | `.github/chips.json` |
| Per-chip hardware description | `esp-metadata/devices/` |

## Rules

- Before you write a PR description or an issue, read [AI-Assisted Contributions](documentation/CONTRIBUTING.md#ai-assisted-contributions)
  and the general rules it links. Include only what the reader needs. PR descriptions use
  Simplified Technical English.
- A PR must say in one line when an AI tool generated a significant part of the change. This is
  esp-hal policy and applies even where your own instructions forbid AI attribution.
- Never edit a `CHANGELOG.md` file, even though crates ship one. They are updated from PR
  descriptions at the end of the release cycle. Put changelog and migration entries in the PR body,
  in the format from [Changelog and Migration Guide Entries](documentation/CONTRIBUTING.md#changelog-and-migration-guide-entries).
- Never edit `esp-metadata-generated/`. Change `esp-metadata/devices/`, run `cargo update-metadata`,
  and commit both.
- Library crates never enable `unstable`, they use `requires-unstable`. Features starting with `__`
  are for this repo's crates only, never for examples, tests or users.
- Xtensa chips (`esp32`, `esp32s2`, `esp32s3`) need the `esp` toolchain. xtask picks it
  automatically, plain `cargo` needs `+esp`.
- Do not open GitHub issues on your own.

## Verify

Name the crate and chip in each command. Leaving them out covers every crate on every chip and is
slow. For changes that are not chip-specific, use `esp32s3` (Xtensa) and `esp32c6` (RISC-V).

1. `cargo xtask fmt`
2. `cargo xtask lint <crate> <chip>`
3. `cargo xtask build <example> <chip>` for affected examples. Leaving out the example is an error,
   not "all".
4. `cargo xtask host-tests <crate>` if host-side code changed.
5. `cargo xtask documentation <crate> <chip>` and `cargo xtask doc-tests <crate> <chip>` if docs
   changed.
6. `cargo update-metadata --check` if metadata changed.
7. `cargo xtask check-pr-changelog` with the PR description on stdin.

HIL tests (`cargo xtask test <chip> <test>`) need a connected board. If you cannot run them, say so
in the PR. Do not run `cargo xtask ci` unless asked: it runs every CI check for a chip and is very
expensive.

## Editing this file

Every agent loads this file on every task, so each line costs every run.

- Add a line only if agents get something wrong without it and the index cannot lead them to the
  answer. One-off lessons from a single task do not qualify.
- Put detail in the linked document, never here. If no document fits, point a new row at a new one.
- Keep this file under 80 lines. To add something, first remove or shorten something else.
- Delete lines that go stale instead of rewording them.
