# xtask

Automation using [cargo-xtask](https://github.com/matklad/cargo-xtask).

## Usage

```text
Usage: xtask <COMMAND>

Commands:
  build-documentation    Build documentation for the specified chip
  build-examples         Build all examples for the specified chip
  build-package          Build the specified package with the given options
  build-tests            Build all applicable tests or the specified test for a specified chip
  bump-version           Bump the version of the specified package(s)
  fmt-packages           Format all packages in the workspace with rustfmt
  generate-efuse-fields  Generate the eFuse fields source file from a CSV
  lint-packages          Lint all packages in the workspace with clippy
  run-example            Run the given example for the specified chip
  run-doc-test           Run doctests for specified chip and package
  run-tests              Run all applicable tests or the specified test for a specified chip
  run-elfs               Run all ELFs in a folder
  help                   Print this message or the help of the given subcommand(s)

Options:
  -h, --help  Print help
```
