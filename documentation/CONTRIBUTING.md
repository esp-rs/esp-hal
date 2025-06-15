
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
*   [Your PR is merged!]

[New Contributor Guide]: #new-contributor-guide
[Getting Started]: #getting-started
[Issues: Reporting and Resolving]: #issues-reporting-and-resolving
[Making Changes: Fork, Edit, and Pull Request]: #making-changes-fork-edit-and-pull-request
[Testing Your Contributions]: #testing-your-contributions
[Commit your updates]: #commit-your-updates
[Pull Request: From Submission to Merge]: #pull-request-from-submission-to-merge
[Your PR is merged!]: #your-pr-is-merged

## New Contributor Guide

Welcome aboard! If you're new to `esp-hal` or open-source contribution, here are some resources to get you started:

*   [Understanding the Project]: A high-level overview of `esp-hal`.
*   Intro to Open Source Contribution: [GitHub's Guide]
*   [Setting Up Git]
*   Workflow Insights: [GitHub Flow]
*   Collaborating via [Pull Requests]

Before adding or changing code, review the [esp-rs API guidelines].

[Understanding the Project]: README.md
[GitHub's Guide]: https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github
[Setting Up Git]: https://docs.github.com/en/get-started/quickstart/set-up-git
[GitHub Flow]: https://docs.github.com/en/get-started/quickstart/github-flow
[Pull Requests]: https://docs.github.com/en/github/collaborating-with-pull-requests
[esp-rs developer guidelines]: ./documentation/DEVELOPER-GUIDELINES.md

## Getting Started

### Issues: Reporting and Resolving

#### Reporting a New Issue

Encountered a problem or have an idea? First, [check existing issues] to avoid duplicates. If your concern is new, use our [issue form] to submit it.

[check existing issues]: https://github.com/esp-rs/esp-hal/issues
[issue form]: https://github.com/esp-rs/esp-hal/issues/new/

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

* Using [xtask], build examples for the specified chip.
* Build the documentation and run the doctests if they have been modified using the `build-documentation` and  `run-doc-test` commands in [xtask].
* Run the [HIL] tests locally if changes have been made to them.

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
cargo xtask fmt-packages
```

We also recommend using the `lint-packages` subcommand, which uses `cargo clippy` and will lint the entire driver in order to catch common mistakes in the code.

```shell
cargo xtask lint-packages
```

This will use `rustfmt` to ensure that all source code is formatted correctly prior to committing.

## Pull Request: From Submission to Merge

*   Fill the pull request template so that we can review your PR. This template helps reviewers understand your changes as well as the purpose of your pull request.
*   [Link your PR] to any relevant issues it addresses.
*   [Allow edits from maintainers] so the branch can be updated for a merge. Once you submit your PR, a Docs team member will review your proposal. We may ask questions or request additional information.
*   Make sure you add an entry with your changes to the [Changelog]. Also make sure that it is in the appropriate section of the document.
*   Make sure you add your changes to the current [migration guide].
*   We may ask for changes to be made before a PR can be merged, either using [suggested changes] or pull request comments. You can apply suggested changes directly through the UI. You can make any other changes in your fork, then commit them to your branch.
*   As you update your PR and apply changes, mark each conversation as [resolved].
*   Resolve merge conflicts if they arise, using resources like [this git tutorial] for help.

[Link your PR]: https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue
[Allow edits from maintainers]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/allowing-changes-to-a-pull-request-branch-created-from-a-forkmember
[Changelog]: esp-hal/CHANGELOG.md
[migration guide]: esp-hal/MIGRATING-0.20.md
[suggested changes]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/incorporating-feedback-in-your-pull-request
[resolved]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/commenting-on-a-pull-request#resolving-conversations
[this git tutorial]: https://github.com/skills/resolve-merge-conflicts


## Your PR is Merged!

Congratulations! The `esp-rs` team thanks you for your contributions!

Contributing to open source extends beyond just code! Each contribution, regardless of size, plays a significant role. We appreciate your involvement in this collective endeavor.
