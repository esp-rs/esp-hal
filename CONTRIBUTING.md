
Welcome to the `esp-hal` Contributing Guide
===========================================

Thank you for considering contributing to our project! Your efforts help make `esp-hal` a better ecosystem for everyone.

This guide outlines the contribution workflow, from reporting issues and submitting pull requests, to the review process and eventual merger of contributions.

Quick Navigation
----------------

*   [New Contributor Guide](#new-contributor-guide)
*   [Getting Started](#getting-started)
    *   [Issues: Reporting and Resolving](#issues-reporting-and-resolving)
    *   [Making Changes: Fork, Edit, and Pull Request](#making-changes-fork-edit-and-pull-request)
*   [Your Pull Request: From Submission to Merge](#your-pull-request-from-submission-to-merge)
*   [Community and Communication](#community-and-communication)

New Contributor Guide
---------------------

Welcome aboard! If you're new to `esp-hal` or open-source contribution, here are some resources to get you started:

*   [Understanding the Project](README.md): A high-level overview of `esp-hal`.
*   Intro to Open Source Contribution: [GitHub's Guide](https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github)
*   [Setting Up Git](https://docs.github.com/en/get-started/quickstart/set-up-git)
*   Workflow Insights: [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)
*   Collaborating via [Pull Requests](https://docs.github.com/en/github/collaborating-with-pull-requests)

Getting Started
---------------

### Issues: Reporting and Resolving

#### Report a New Issue

Encountered a problem or have an idea? First, [check existing issues](https://github.com/esp-rs/esp-hal/issues) to avoid duplicates. If your concern is new, use our [issue form](https://github.com/esp-rs/esp-hal/issues/new/) to submit it.

#### Work on an Issue

Browse [existing issues](https://github.com/esp-rs/esp-hal/issues) to find one that resonates with you. Use labels for easier filtering. If you decide to tackle an issue, it's courteous (but not mandatory) to let others know by commenting.

### Making Changes: Fork, Edit, and Pull Request

1.  **Fork**: Start by [forking the repository](https://docs.github.com/en/github/getting-started-with-github/fork-a-repo). This keeps the main project safe while you make your changes.
2.  **Setup**: Ensure you have the latest Rust toolchain via [rustup.rs](https://rustup.rs/).
3.  **Branch**: Create a branch in your fork for your changes. Keep your changes focused and limited to a single issue or feature.

### Testing Your Contributions

Ensuring the quality and reliability of `esp-hal` is a shared responsibility, and testing plays a critical role in this process. Our GitHub CI automatically checks the buildability of all examples and drivers within the project. However, automated tests can't catch everything, especially when it comes to the nuanced behavior of hardware interactions.

#### What You Should Do:

*   **API changes**: If your contribution changes the API, please take care to adapt the driver and examples to these changes.
*   **Run Related Examples**: After making changes, please run any examples that are affected by your contributions. This helps verify that not only do they build successfully, but they also perform as expected.
*   **Manual Testing**: For hardware-related changes, manually test your changes on the actual devices when possible. If not - please write about it in the corresponding issue, someone from our team will definitely take the time to do it! This is crucial because hardware behavior can sometimes differ from what's simulated or expected.

By taking these extra steps to test your contributions, you help maintain the high quality and reliability of `esp-hal`, ensuring it remains a robust platform for everyone

### Commit Your Updates

Commit your changes once you're satisfied. Review your own work to streamline the review process later. Use `rustfmt` and `cargo clippy` to ensure your code adheres to Rust's conventions. 

```shell
rustup component add rustfmt
rustup component add clippy
```

Consider using the `pre-commit` hook to automate formatting checks.

```shell
cp pre-commit .git/hooks/pre-commit
```


### Pull Request: From Submission to Merge

*   Fill the pull request template so that we can review your PR. This template helps reviewers understand your changes as well as the purpose of your pull request.
*   [Link your PR](https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue) to any relevant issues it addresses.
*   [Allow edits from maintainers](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/allowing-changes-to-a-pull-request-branch-created-from-a-fork) so the branch can be updated for a merge. Once you submit your PR, a Docs team member will review your proposal. We may ask questions or request additional information.
*   We may ask for changes to be made before a PR can be merged, either using [suggested changes](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/incorporating-feedback-in-your-pull-request) or pull request comments. You can apply suggested changes directly through the UI. You can make any other changes in your fork, then commit them to your branch.
* As you update your PR and apply changes, mark each conversation as [resolved](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/commenting-on-a-pull-request#resolving-conversations).
*   Resolve merge conflicts if they arise, using resources like [this git tutorial](https://github.com/skills/resolve-merge-conflicts) for help.


### Your PR is Merged!
------------

Congratulations! The `esp-rs` team thanks you for your contributions!

Contributing to open source extends beyond just code! Each contribution, regardless of size, plays a significant role. We appreciate your involvement in this collective endeavor.