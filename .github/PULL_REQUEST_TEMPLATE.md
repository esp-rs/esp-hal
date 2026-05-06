## Thank you for your contribution!

We appreciate the time and effort you've put into this pull request.
To help us review it efficiently, please ensure you've gone through the following checklist:

### Submission Checklist 📝
- [ ] I have updated existing examples or added new ones (if applicable).
- [ ] I have used `cargo xtask fmt-packages` command to ensure that all changed code is formatted correctly.
- [ ] I have added changelog entries and/or migration guide notes in the sections below, or I will ask a maintainer to add the `skip-changelog` or `manual-changelog` label as appropriate.
- [ ] My changes are in accordance to the [esp-rs developer guidelines](https://github.com/esp-rs/esp-hal/blob/main/documentation/DEVELOPER-GUIDELINES.md)

#### Extra:
- [ ] I have read the [CONTRIBUTING.md guide](https://github.com/esp-rs/esp-hal/blob/main/documentation/CONTRIBUTING.md) and followed its instructions.

### Pull Request Details 📖

#### Description
Please provide a clear and concise description of your changes, including the motivation behind these changes. The context is crucial for the reviewers.

#### Testing
Describe how you tested your changes.

---

<!-- ============================================================
  Changelog and migration guide entries live here in the PR body.
  They will be collected automatically at release time.
  Delete any section that doesn't apply to your PR.
  ============================================================ -->

# Changelog

<!-- Add one or more ## <crate> or ## <crate>/<area> sections below.
     Each entry must start with one of: Added, Changed, Fixed, Removed.
     To explicitly declare that a crate needs no changelog entry, write:
       - No changelog necessary.
     as the sole item under that crate's heading.

## esp-hal

- Added: Brief description of what was added.
- Changed: Brief description of what changed.
- Fixed: Brief description of what was fixed.
- Removed: Brief description of what was removed.

-->

# Migration guide

<!-- Add one or more ## <crate>/<area> sections below (area is required).
     Each breaking change needs a ### Title heading followed by the migration steps.
     Only needed when user code must be updated.

## esp-hal/SPI

### `OldType` has been renamed to `NewType`

Replace all uses of `OldType` with `NewType`.

-->
