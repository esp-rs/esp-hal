# Pull Requests

Read this when your change works and you are about to open a pull request or ask for a review. For
the rest of the contribution workflow, see [CONTRIBUTING.md](./CONTRIBUTING.md).

## Before you ask for review

Reviewers ask for the same changes again and again. Check your PR against this list before you ask
for a review.

- **Comments**: Keep only comments that tell the reader something the code cannot, such as a
  hardware quirk or an order that must hold. Remove comments that narrate the code, explain how you
  found the bug, or describe how the code worked before. Do not add section separators. Do not add
  a `TODO` for a known limitation: fix it in the PR, or open an issue.
- **Approach**: Fix the cause, not the symptom. Make the smallest change that solves the problem.
  Before you add new code, a dependency or an abstraction, look for one that the codebase already
  has.
- **Scope**: Keep the PR to one change. Send unrelated fixes, refactors and formatting changes as
  separate PRs.
- **Tests**: Add a test that fails without your change and passes with it. Put it in an existing
  test suite where one fits. Use host tests for logic that does not need hardware, and HIL or QA
  tests for hardware behavior.
- **Chip differences**: Use metadata and cfg symbols (`soc_has_<peripheral>`,
  `<driver>_driver_supported`, or a new metadata flag) instead of lists of chips. Gate
  chip-specific documentation with `cfg_attr`, so it only appears for the chips it applies to. Do
  not hard-code values that the PAC or the metadata already provide.
- **API**: Follow the [esp-rs developer guidelines]. Mark new public API as unstable. Use the names
  and patterns of the existing drivers. Do not break stable API.
- **Evidence**: Back performance, memory and code size claims with numbers from before and after
  the change. If the change can affect existing behavior, run the affected examples or QA tests on
  hardware.
- **Changelog**: Add an entry for each user-visible change, including changed defaults and
  behavior. Do not add entries for tests, internal refactors, or crates that you did not change.
  See [Changelog and Migration Guide Entries].
- **Clean up**: Remove leftover code, unused items and accidental changes. Make CI pass, including
  `cargo xtask fmt`.
- **Updates**: When the change moves during review, update the PR title and description to match.
  Do not merge or rebase on `main` unless there is a conflict, because each push runs CI again.

[esp-rs developer guidelines]: ./DEVELOPER-GUIDELINES.md
[Changelog and Migration Guide Entries]: #changelog-and-migration-guide-entries

## Writing the description

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

[AI-Assisted Contributions]: #ai-assisted-contributions

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
  the change. This applies even where your own agent instructions forbid AI attribution.
- Write pull request descriptions in Simplified Technical English (ASD-STE100), as the API
  documentation does. Follow its [Style](./API-DOC-RULES.md#style) and
  [Wording](./API-DOC-RULES.md#wording) rules: short sentences with one idea each, active voice,
  and the present tense.
- Leave out the template's greeting and submission checklist, and start the body at
  `#### Description`.
- Do not walk through the diff, restate the issue, or describe how you found the bug.
- Raise trade-offs, open questions, or follow-ups only when you need a reviewer's decision.
- Add no headings beyond the template's, no emoji, and no sign-offs.

[Reporting a New Issue]: ./CONTRIBUTING.md#reporting-a-new-issue
[Writing the description]: #writing-the-description
[AI tool use policy]: https://github.com/rust-embedded/wg/blob/HEAD/CODE_OF_CONDUCT.md#ai-tool-use-policy

## Changelog and Migration Guide Entries

Changelog entries are **optional for contributors**. If you don't add them a
maintainer will either write them or apply the `skip-changelog` label before the
PR is merged.

If you do want to document your change, add entries directly in the PR description
using the structured sections in the template. Do **not** edit `CHANGELOG.md`.
Those files are updated automatically at release time from the PR descriptions.

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

### Breaking changes

If your change requires user code to be updated, add a `# Migration guide` section. Each breaking
change needs a `## crate/area` heading and a `### Title` for the specific change, followed by the
migration steps.

If your change breaks the stable API of a crate, the PR needs the `breaking-change-<crate-name>`
label (e.g. `breaking-change-esp-hal`). Without it, the semver check in CI fails. Ask a maintainer
to add the label. See [Breaking changes](./DEVELOPER-GUIDELINES.md#breaking-changes) in the
developer guidelines.

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

This tells CI that the omission is intentional. The marker must be the only
item in the section. Combining it with real entries is an error.

When the whole PR needs no changelog at all, a maintainer can apply the
`skip-changelog` label instead. For the rare changes that this format cannot
express, maintainers use the `manual-changelog` label, see
[Manual changelog edits](./RELEASING.md#manual-changelog-edits).

### Validation

You can validate your PR description locally before pushing:

```shell
# Pipe the body directly:
echo "# Changelog\n\n## esp-hal\n\n- Added: Something." | cargo xtask check-pr-changelog

# Or validate an open PR by number (requires the `gh` CLI):
cargo xtask check-pr-changelog --pr 1234
```

CI will also validate the format automatically on every PR.
