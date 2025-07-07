# xtask

Automation using [cargo-xtask](https://github.com/matklad/cargo-xtask).

## Usage

```text
Usage: xtask <COMMAND>

Commands:
  build                      Build-related subcommands
  run                        Run-related subcommands
  release                    Release-related subcommands
  ci                         Perform (parts of) the checks done in CI
  fmt-packages               Format all packages in the workspace with rustfmt
  clean                      Run cargo clean
  lint-packages              Lint all packages in the workspace with clippy
  semver-check               Semver Checks
  check-changelog            Check the changelog for packages
  update-chip-support-table  Re-generate the chip support table in the esp-hal README
  help                       Print this message or the help of the given subcommand(s)

Options:
  -h, --help  Print help
```

You can get help for subcommands, too!

```text
cargo xtask build examples --help
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.11s
     Running `[...]/target/debug/xtask build examples --help`

Build all examples for the specified chip

Usage: xtask build examples [OPTIONS] <CHIP> <PACKAGE>

[...]
```

## Releasing crates

To start the release process, run `cargo xrelease plan` to prepare all crates, or `cargo xrelease plan <a space-separated list of crates>` to release a specific crate and its dependencies.

For example, to release `esp-println`, run `cargo xrelease plan esp-println`.

The release is a multi-step process. Each step in the process will tell you what to do next.

## Test/example metadata use

Each test and example can specify metadata. This metadata is read, interpreted and used by the
`xtask`. It affects how a particular test or example is built, but it does not otherwise modify
the user's system. A test or example can specify one or more sets of metadata, called
"configurations".

Metadata lines come in the form of:

- `//% METADATA_KEY: value` - applies to all configuration.
- `//% METADATA_KEY(config_name_1, config_name_2, ...): value` - applies to specific configurations.

The following metadata keys can be used:

### `//% CHIPS`

A space-separated list of chip names. The test or example will be built for these chips. If the line
is missing, the file is built for all known chips.

```
//% CHIPS: esp32c6 esp32s3
```

This key is a filter. If a named configuration contains a list of chips, the named list overwrites
the unnamed list for that configuration. If multiple lines specify the same configuration, the
latter one overwrites the earlier one.

`CHIPS` can be used to set a specific feature for a specific chip, like this:

```
//% CHIPS: esp32c3 esp32c6
//% CHIPS(xtensa): esp32s3
//% FEATURES(riscv):
//% FEATURES(xtensa): psram
```

Here we need to specify an empty `FEATURES(riscv)` otherwise the xtask would only create the
`xtensa` configuration, ignoring the other chips.

### `//% TAG`

Used to sort examples, when running `run-example` without naming a specific example.

This key is a filter. If a named configuration contains a tag, the named line overwrites
the unnamed line for that configuration. If multiple lines specify the same configuration, the
latter one overwrites the earlier one. In general, you probably don't want to use `//% TAG(config)`.

### `//% FEATURES`

A space-separated list of cargo features that will be enabled automatically when
building the test or example. If you need to specify a feature of a dependency,
you can use the `crate-name/feature-name` format.

This key is additive. The unnamed list is added to named lists, and multiple lists with the
same name are merged.

### `//% ENV`

Environmental variables to be set, when building the test or example. This is
mainly intended, but not limited to setting esp-config configuration.

One environment variable is specified in a single line. The name and value are separated by `=`.

```
//% ENV(generic_queue): ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = generic
//% ENV(generic_queue): ESP_HAL_EMBASSY_CONFIG_GENERIC_QUEUE_SIZE = 16
```

This key is additive. The unnamed list is added to named lists, and multiple lists with the
same name are merged.

### `//% CARGO-CONFIG`

The value of this key will be passed as a `--config` argument to `cargo`. Any amount
of configuration can be specfied this way.

```
//% CARGO-CONFIG: target.'cfg(target_arch = "riscv32")'.rustflags = [ "-Z", "stack-protector=all" ]
//% CARGO-CONFIG: target.'cfg(target_arch = "xtensa")'.rustflags = [ "-Z", "stack-protector=all" ]
```

This key is additive. The unnamed list is added to named lists, and multiple lists with the
same name are merged.

### Working with multiple metadata configurations

Processing a file will create one configuration, or however many names (that is, the list of
different parenthesized words next to the metadata key) the xtask encounters.

For example, the following list creates two configurations (`single_integrated` and
`multiple_integrated`). Both configurations will be compiled for all listed chips, with the
`unstable` and `embassy` features enabled. One will select the `single-integrated` timer queue
option, while the other will select `multiple-integrated`.

```
// This is a hypothetical "embassy_test.rs"
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy
//% ENV(single_integrated):   ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = single-integrated
//% ENV(multiple_integrated): ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = multiple-integrated
```

You can specifiy a test or example by file, or by configuration. If the
parameters match multiple files, they will be built or executed in sucession.

For example, running `cargo xtask run-tests esp32 embassy_test` will run both
`embassy_test_single_integrated` and `embassy_test_multiple_integrated`, but you can also
run `cargo xtask run-tests esp32 embassy_test_multiple_integrated` to select only one.

In this example, running the `cargo xtask build-tests esp32 embassy_test` command creates an
`embassy_test_single_integrated` and an `embassy_test_multiple_integrated` binary.
