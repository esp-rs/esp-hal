# Releasing `esp-hal`

Reference for preparing, staging, and publishing a release.

A release starts about two weeks before the intended publish date, to leave time
for testing. The date is a target, not a deadline: a release ships when it is
ready. The work runs in five phases:

| Phase          | Goal                                              |
| -------------- | ------------------------------------------------- |
| Kickoff        | Freeze `main` and publish the first pre-release   |
| Stabilization  | Soak pre-releases and fix bugs while `dev` moves  |
| Shipping       | Promote to stable, publish, and roll over         |
| Reconciliation | Merge `dev` back into `main` and unfreeze         |
| Post-mortem    | Record what to improve next time                  |

## Freeze and `dev`

During a release, `main` is frozen and development moves to a temporary `dev`
branch.

- Freeze `main` by opening and pinning an issue from the `Merge freeze` template.
  The merge queue then rejects pull requests that target `main`. See
  [Merge Freezes](./CONTRIBUTING.md#merge-freezes) for the mechanics.
- The freeze covers `main` only. Retarget open, non-blocking pull requests to
  `dev`, and keep reviewing and merging there.
- Only release pull requests and urgent fixes reach the release. Everything else
  waits on `dev`.

## Soak testing

Soak testing decides when a pre-release can be promoted, so run it for every
pre-release during the freeze.

- Update projects to the published pre-release to test against real code.
- Run QA tests that CI does not run routinely.

## Publishing

Each pre-release and the final stable release publish the same way, on `main`:

```
cargo xrelease plan <package> [more packages...]   # writes release_plan.jsonc
# edit release_plan.jsonc to set the bump, then remove the comment header
cargo xrelease execute-plan --no-dry-run           # bumps versions, opens the release PR
# review and merge the release PR into main, then:
cargo xrelease publish-plan --no-dry-run           # publishes to crates.io, tags, pushes
```

The bump in `release_plan.jsonc` has two fields: `base` (how much to raise
`major.minor.patch`) and `pre` (the pre-release identifier, such as `alpha`,
`beta`, or `rc`).

- Start a pre-release cycle by setting both: `"bump": { "base": "Minor", "pre": "beta" }`.
- Continue the cycle by leaving `base` null and keeping `pre`: `"bump": { "base": null, "pre": "beta" }`.
- Promote to stable by setting `base` and leaving `pre` null: `"bump": { "base": "Minor", "pre": null }`.

Notes:

- `execute-plan` also merges changelog and migration-guide entries from merged
  pull request descriptions, and labels the PR with `release:docs`,
  `release:registry:compile-test`, and `release:registry:ci`. Those labels drive
  the `cargo update` checks in the `pre-release-checks` workflow.
- Run `post-release` only for the final stable release.

## Kickoff

- Land toolchain changes first. If the MSRV or CI toolchain must move, run
  `cargo xrelease bump-msrv` before the freeze.
- Freeze `main` and branch `dev` off it (see [Freeze and `dev`](#freeze-and-dev)).
- Publish the first pre-release (see [Publishing](#publishing)).
- Track the release on the freeze issue. It also records exempt merges.

## Stabilization

Soak test each published pre-release (see [Soak testing](#soak-testing)).
Publish another pre-release whenever stabilization fixes accumulate on `main`.

Pre-release requirements:

- Patch: none
- Minor: at least one pre-release
- Major: a full pre-release cycle (an alpha can be skipped)

Soak rules:

- Each pre-release stays out for at least a week before promotion.
- A change larger than a small bug fix resets the clock and needs a further
  pre-release.

To land an urgent fix on frozen `main`, add the `merge-freeze-exempt` label.
Reserve it for regressions and release blockers, and remove it after merging.
Anything that is not a blocker goes to `dev`.

## Shipping

- Promote the soaked pre-release to the stable version (see [Publishing](#publishing)).
- After it is published and tagged, roll over:

```
cargo xrelease post-release --no-dry-run
```

For a stable minor or major release (major version 1 or higher), `post-release`:

- creates the next migration guide
- cuts the backport branch `<pkg>-<major>.<minor>.x` from the release tag and
  deletes the previous minor's branch
- triggers the API baseline generation workflow for semver-checked packages
- opens the rollover pull request

## Reconciliation

- Merge `dev` into `main` in a single merge. Resolve `Cargo.toml` and
  `CHANGELOG.md` conflicts in favor of the released versions.
- Close the freeze issue to lift the freeze, and delete `dev`.
- Backport later fixes by landing them on `main` with the `<pkg>-backport` label.
  The `Backport merged PR` workflow opens the cherry-pick against
  `<pkg>-<major>.<minor>.x`.

## Post-mortem

Record on the tracking issue before closing it:

- whether the release started two weeks out, and where the schedule slipped
- what broke during testing, and how many pre-releases it took
- what landed through `merge-freeze-exempt`, and whether it was worth it
- which manual step was error-prone, and whether it can be automated
- action items, each with an owner
