# AGENTS.md

esp-hal is a monorepo of bare-metal `no_std` Rust crates for Espressif chips. Each top-level crate
directory is its own Cargo project, and most have a README. The root workspace only holds the
`xtask` tooling, and all building, linting and testing goes through `cargo xtask`.

This file is an index. Read a linked document when your current step needs it, not at the start of a
task.

## Where to look

| Task | Read |
| --- | --- |
| Any code change: API design, drivers, cfg symbols, metadata | `documentation/DEVELOPER-GUIDELINES.md` |
| Rustdoc | `documentation/API-DOC-RULES.md` |
| xtask commands, `//%` annotations in examples and tests, host tests | `xtask/README.md` |
| Opening a PR or asking for review: expectations, description, changelog | `documentation/PULL-REQUESTS.md` |
| Writing an issue | `documentation/CONTRIBUTING.md`, `documentation/REPRODUCERS.md` |
| Examples | `examples/README.md` |
| HIL tests | `hil-test/README.md`, `documentation/HIL-GUIDE.md` |
| Build-time configuration (`esp_config.yml`) | `esp-config/README.md` |
| New chip bring-up | `documentation/CHIP_BRING_UP.md` |
| Releases | `documentation/RELEASING.md` |
| Chip list and Rust targets | `.github/chips.json` |
| Per-chip hardware description | `esp-metadata/devices/` |

## Rules

- When the change works, and before you open a PR or ask for review, read
  `documentation/PULL-REQUESTS.md` and apply it.
- Never edit a `CHANGELOG.md` file, even though crates ship one. They are updated from PR
  descriptions at the end of the release cycle. Put changelog and migration entries in the PR body.
- Never edit `esp-metadata-generated/`. Change `esp-metadata/devices/`, run `cargo update-metadata`,
  and commit both.
- Library crates never enable `unstable`, they use `requires-unstable`. Features starting with `__`
  are for this repo's crates only, never for examples, tests or users.
- Xtensa chips (`esp32`, `esp32s2`, `esp32s3`) need the `esp` toolchain. xtask picks it
  automatically, plain `cargo` needs `+esp`.
- Do not open GitHub issues on your own.

## Editing this file

Every agent loads this file on every task, so each line costs every run.

- Add a line only if agents get something wrong without it and the index cannot lead them to the
  answer. One-off lessons from a single task do not qualify.
- Put detail in the linked document, never here. If no document fits, point a new row at a new one.
- Keep this file under 80 lines. To add something, first remove or shorten something else.
- Delete lines that go stale instead of rewording them.
