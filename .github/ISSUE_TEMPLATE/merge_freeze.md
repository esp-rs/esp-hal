---
name: Merge freeze
about: Hold merges into `main` while a release is prepared
title: 'Merge freeze: pre-x.y.z release'
labels: ["merge-freeze"]
assignees: ''

---

<!--
  Opening this issue starts the freeze. Closing it lifts the freeze.
  Pin it so that it shows up at the top of the issue list.

  Only maintainers should use this template.
-->

## Merge freeze is in effect

<!-- Which release this leads up to, and anything else worth knowing. -->

While this issue is open, the merge queue rejects pull requests targeting
`main`: CI's `merge-freeze-gate` job fails on them, which fails `ci-result`.
Review carries on as usual, nothing is blocked until an actual merge attempt.

If a change is urgent enough to land during the freeze, add the
`merge-freeze-exempt` label to it. Exempted merges are recorded below.
