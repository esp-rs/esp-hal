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
  host-tests                 Run host tests for packages with registered instructions (see below)
  check-global-symbols       Check global symbols in the compiled `.rlib`
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

Usage: xtask build examples [OPTIONS] <EXAMPLE>

[...]
```

## Releasing crates

To start the release process, run `cargo xrelease plan` to prepare all crates, or `cargo xrelease plan <a space-separated list of crates>` to release a specific crate and its dependencies.

For example, to release `esp-println`, run `cargo xrelease plan esp-println`.

The release is a multi-step process. Each step in the process will tell you what to do next.

### Patch releases

Patch releases are made from backport branches (e.g. `esp-hal-1.1.x`).
Ensure bug-fix PRs are cherry-picked to the backport branch first (label with
`{package}-backport` on `main` to trigger automatic cherry-pick PRs).

The standard `plan` → `execute-plan` → `publish-plan` → `post-release` flow
works from a backport branch — the tooling auto-detects it, scopes to the
single backport package, and forces Patch bumps. No `--allow-non-main` needed.

## Host tests

`cargo xtask host-tests` runs host-side unit tests. CI invokes it for every
package where `Package::has_host_tests` finds a `#[test]` function under
`src/**/*.rs`.

**Detection is not enough.** Each package with host tests also needs a match arm
in `run_host_tests` (`xtask/src/lib.rs`). Without it, xtask fails with
`Instructions for host testing were not provided for: '<package>'` even though
the tests compile with `cargo test -p <package>`.

When adding host tests to a package:

1. Add the tests (`#[test]` in `src/`, and/or `tests/*.rs` integration tests).
2. Add a `Package::<Name> => { ... }` arm to `run_host_tests` with the right
   `cargo test` invocation (features, `--lib`, `--tests`, Miri, etc.). Copy a
   similar package if unsure.
3. Verify with `cargo xtask host-tests <package>`.

## Package metadata: `check-configs`, `clippy-configs`, `doc-config`, `semver-config`

Published crates can list CI check, clippy, documentation, and semver cases under
`[package.metadata.espressif]` in their `Cargo.toml`. Each case is one inline table; `features`
and `env` are both optional:

```toml
[package.metadata.espressif]
check-configs = [
    { features = [] },
    {
        features = ["unstable", "rt"],
        env = {
            ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = "generic"
        },
    },
    { features = ["unstable", "rt"], append = [
        { if = 'usb_otg_driver_supported', features = ["__usb_otg"] },
    ] },
]
clippy-configs = [
    { features = ["unstable", "rt"] },
]
doc-config = {
    features = ["unstable", "rt"],
    append = [
        { if = 'usb_otg_driver_supported', features = ["__usb_otg"] },
    ],
}
semver-config = {
    features = ["unstable", "rt"],
    append = [
        { if = 'wifi_driver_supported', features = ["wifi"] },
    ],
}
```

- `features` — cargo features to enable for that case (same as before).
- `env` — optional inline table of environment variables passed to the cargo invocation.
  Mainly used for `esp-config` options (`ESP_*` variables). Omit when no overrides are needed.
- `if` — optional somni expression; the whole case is skipped when the condition is false.
- `append` — optional list of partial inline tables. Matching rows add more `features` and/or
  `env` entries. Append rows may specify only `features` or only `env`.

`check-configs` and `clippy-configs` are arrays of cases. `doc-config` and `semver-config` are
single inline tables.

`cargo xtask check-packages` reads `check-configs`; `cargo xtask lint-packages` reads
`clippy-configs`; documentation builds read `doc-config`; semver checks read `semver-config`. If `check-configs` is absent, a single
default case with no features and no env overrides is used. `CI`, `DEFMT_LOG`, and `ESP_LOG`
(check only) are always set by xtask and override duplicate keys from metadata. Documentation
builds and doc-tests always set `RUSTDOCFLAGS` (and `ESP_HAL_DOCTEST` for doc-tests) and override
duplicate keys from `doc-config`. Semver checks always set `RUSTDOCFLAGS` and override duplicate
keys from `semver-config`.

## Test/example metadata use

Each test and example can specify metadata. This metadata is read, interpreted and used by the
`xtask`. It affects how a particular test or example is built, but it does not otherwise modify
the user's system. A test or example can specify one or more sets of metadata, called
"configurations".

Metadata lines come in the form of:

- `//% METADATA_KEY: value` - applies to all configuration.
- `//% METADATA_KEY(config_name_1, config_name_2, ...): value` - applies to specific configurations.

The following metadata keys can be used:

### `//% CHIP_FILTER`

A boolean expression that selects which chips the test or example is built for. If the line is
missing, the file is built for all known chips. A symbol in the expression is either a cfg name
(exactly as it appears in `#[cfg(...)]`, e.g. `i2c_master_driver_supported` or
`dma_can_access_psram`), a key-value cfg name (e.g. `interrupt_controller`), or a chip name
(e.g. `esp32`).

Prefer a capability cfg flag over an explicit chip list: it describes *why* a chip is supported and
automatically picks up new chips that gain the capability. Only fall back to chip names when no cfg
flag captures the requirement.

```
//% CHIP_FILTER: wifi_driver_supported
```

Symbols can be combined with the logical operators `&&`, `||`, `!` and parentheses; an operator is
required between symbols (`foo bar` is invalid, write `foo && bar`). Use `!` to exclude a chip that is
otherwise capable but incompatible with the test/example, and `||` to list specific chips only when a
capability flag does not exist:

```
//% CHIP_FILTER: spi_slave_supports_dma && !esp32
//% CHIP_FILTER: lp_core || ulp_riscv_core
//% CHIP_FILTER: esp32c6 || esp32h2
```

Some cfg symbols carry a string value rather than being simply present or absent (for example,
`#[cfg(interrupt_controller = "clic")]`). These can be compared with `==` and `!=` using quoted
string literals:

```
//% CHIP_FILTER: riscv && interrupt_controller != "clic"
//% CHIP_FILTER: interrupt_controller == "plic"
```

Chips that do not define a key-value symbol are treated as having an empty string value.

The expression is evaluated with [`somni`](https://crates.io/crates/somni-expr). `xtask` turns each
boolean cfg symbol into a `true`/`false` variable and each key-value cfg symbol into a string
variable, then evaluates the expression per chip. If a symbol does not exist, it causes an
evaluation error.

This key is a filter. If a named configuration contains an expression, the named line overwrites the
unnamed line for that configuration. If multiple lines specify the same configuration, the latter one
overwrites the earlier one. This can be used to build different chips with different features:

```
//% CHIP_FILTER: esp32c3 || esp32c6
//% CHIP_FILTER(xtensa): esp32s3
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
of configuration can be specified this way.

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
//% CHIP_FILTER: esp32 || esp32c2 || esp32c3 || esp32c6 || esp32h2 || esp32s2 || esp32s3
//% FEATURES: unstable embassy
//% ENV(single_integrated):   ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = single-integrated
//% ENV(multiple_integrated): ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE = multiple-integrated
```

You can specify a test or example by file, or by configuration. If the
parameters match multiple files, they will be built or executed in succession.

For example, running `cargo xtask run tests esp32 embassy_test` will run both
`embassy_test_single_integrated` and `embassy_test_multiple_integrated`, but you can also
run `cargo xtask run tests esp32 embassy_test_multiple_integrated` to select only one.

In this example, running the `cargo xtask build tests esp32 embassy_test` command creates an
`embassy_test_single_integrated` and an `embassy_test_multiple_integrated` binary.

## MCP server

The xtask includes an [MCP](https://modelcontextprotocol.io/) server that exposes xtask commands as tools for AI agents. It runs over stdio and is built with the `mcp` cargo feature.

### Claude Code

Create `.mcp.json` in the repository root (it is gitignored):

```json
{
  "mcpServers": {
    "esp-hal-xtask": {
      "command": "cargo",
      "args": ["xmcp"]
    }
  }
}
```

### VS Code (GitHub Copilot / Copilot Chat)

Add to `.vscode/mcp.json` (create if it doesn't exist):

```json
{
  "servers": {
    "esp-hal-xtask": {
      "command": "cargo",
      "args": ["xmcp"]
    }
  }
}
```

### Zed

Add to Zed's settings (`settings.json`):

```json
{
  "context_servers": {
    "esp-hal-xtask": {
      "command": {
        "path": "cargo",
        "args": ["xmcp"]
      }
    }
  }
}
```

### Gemini Code Assist (CLI)

Run with `--mcp-server` flag:

```bash
gemini --mcp-server="cargo xmcp"
```

Or add to `~/.gemini/settings.json`:

```json
{
  "mcpServers": {
    "esp-hal-xtask": {
      "command": "cargo",
      "args": ["xmcp"]
    }
  }
}
```

### Other MCP clients

The server is a standard stdio MCP server. Launch it with:

```bash
cargo xmcp
```

Point any MCP-compatible client at this command.
