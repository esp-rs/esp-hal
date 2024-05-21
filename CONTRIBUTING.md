# Welcome to the `esp-hal` contributing guide

Thank you for investing your time in contributing to our project!

In this guide you will get an overview of the contribution workflow from opening an issue, creating a PR, reviewing, and merging the PR.

Use the table of contents icon (<img src="resources/table-of-contents.png" width="24" height="24" />) in the top right corner of this document to get to a specific section of this guide quickly.

## New Contributor Guide

To get an overview of the project, please read the [README]. Here are some resources to help you get started with open source contributions:

- [Finding ways to contribute to open source on GitHub]
- [Set up Git]
- [GitHub flow]
- [Collaborating with pull requests]

[README]: README.md
[Finding ways to contribute to open source on GitHub]: https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github
[Set up Git]: https://docs.github.com/en/get-started/quickstart/set-up-git
[GitHub flow]: https://docs.github.com/en/get-started/quickstart/github-flow
[Collaborating with pull requests]: https://docs.github.com/en/github/collaborating-with-pull-requests

## Getting Started

Before adding or changing code you might want to review the [esp-rs API guidelines](./API-GUIDELINES.md)

### Issues

#### Create a New Issue

If you spot a problem with the docs, [search if an issue already exists]. If a related issue doesn't exist, you can open a new issue using the [issue form].

[search if an issue already exists]: https://docs.github.com/en/github/searching-for-information-on-github/searching-on-github/searching-issues-and-pull-requests#search-by-the-title-body-or-comments
[issue form]: https://github.com/esp-rs/esp-hal/issues/new/

#### Solve an Issue

Scan through our [existing issues] to find one that interests you. You can narrow down the search using labels as filters. If you find an issue to work on, you are welcome to open a PR with a fix.

It's recommended that you comment in the relevant issue, mentioning that you are actively working on it, however this is not a requirement.

If somebody is already assigned to an issue, this does not necessarily mean they are actively working on it; don't be afraid to comment in these issues asking if you can take over the work if you're interested.

[existing issues]: https://github.com/esp-rs/esp-hal/issues

### Make Changes

1. Fork the repository.
   - Using GitHub Desktop:
     - [Getting started with GitHub Desktop] will guide you through setting up Desktop.
     - Once Desktop is set up, you can use it to [fork the repo!]
   - Using the command line:
     - [Fork the repo] so that you can make your changes without affecting the original project until you're ready to merge them.
2. Install or update to the latest version of Rust. See [rustup.rs] for more information.
3. Create a working branch and start with your changes!

[Getting started with GitHub Desktop]: https://docs.github.com/en/desktop/installing-and-configuring-github-desktop/getting-started-with-github-desktop
[fork the repo!]: https://docs.github.com/en/desktop/contributing-and-collaborating-using-github-desktop/cloning-and-forking-repositories-from-github-desktop
[Fork the repo]: https://docs.github.com/en/github/getting-started-with-github/fork-a-repo#fork-an-example-repository
[rustup.rs]: https://rustup.rs/

### Commit Your Update

Commit the changes once you are happy with them. Don't forget to self-review to speed up the review process.

We ask that you ensure all source code files has been properly formatted with `rustfmt`, and that you have linted your changes by running `cargo clippy`. These tools can be installed by running the following commands:

```shell
rustup component add rustfmt
rustup component add clippy
```

We _strongly_ recommend that you format your code before committing to ensure consistency throughout the project.
To format all packages in the workspace, run the following command in a terminal from the root of the repository:

```shell
cargo xtask fmt-workspace
```

This will use `rustfmt` to ensure that all source code is formatted correctly prior to committing.

### Pull Request

When you're finished with the changes, create a pull request, also known as a PR.

- Fill the pull request template so that we can review your PR. This template helps reviewers understand your changes as well as the purpose of your pull request.
- Don't forget to [link PR to issue] if you are solving one.
- Enable the checkbox to [allow maintainer edits] so the branch can be updated for a merge. Once you submit your PR, a Docs team member will review your proposal. We may ask questions or request additional information.
- We may ask for changes to be made before a PR can be merged, either using [suggested changes] or pull request comments. You can apply suggested changes directly through the UI. You can make any other changes in your fork, then commit them to your branch.
- As you update your PR and apply changes, mark each conversation as [resolved].
- If you run into any merge issues, checkout this [git tutorial] to help you resolve merge conflicts and other issues.

[link PR to issue]: https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue
[allow maintainer edits]: https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/allowing-changes-to-a-pull-request-branch-created-from-a-fork
[suggested changes]: https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/incorporating-feedback-in-your-pull-request
[resolved]: https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/commenting-on-a-pull-request#resolving-conversations
[git tutorial]: https://github.com/skills/resolve-merge-conflicts

### Your PR is Merged!

Congratulations! The esp-rs team thanks you for your contributions!
