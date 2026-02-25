# esp-hal agent instructions

Bare-metal `no_std` Rust HAL for Espressif SoCs. MSRV: **1.88.0** (source: `MSRV` env in `.github/workflows/ci.yml`).

## Chip reference

| Chip | Arch | Target | Toolchain flag |
|------|------|--------|----------------|
| esp32 | Xtensa | `xtensa-esp32-none-elf` | `--toolchain esp` |
| esp32s2 | Xtensa | `xtensa-esp32s2-none-elf` | `--toolchain esp` |
| esp32s3 | Xtensa | `xtensa-esp32s3-none-elf` | `--toolchain esp` |
| esp32c2 | RISC-V | `riscv32imc-unknown-none-elf` | `--toolchain stable` |
| esp32c3 | RISC-V | `riscv32imc-unknown-none-elf` | `--toolchain stable` |
| esp32c5 | RISC-V | `riscv32imac-unknown-none-elf` | `--toolchain stable` |
| esp32c6 | RISC-V | `riscv32imac-unknown-none-elf` | `--toolchain stable` |
| esp32h2 | RISC-V | `riscv32imac-unknown-none-elf` | `--toolchain stable` |

## Commands

All automation goes through `cargo xtask`. Use `--packages` and `--chips` to scope.

| Task | Command |
|------|---------|
| Format (required before PR) | `cargo xtask fmt-packages` |
| Lint | `cargo xtask lint-packages [--chips X --packages Y]` |
| Full CI check for one chip | `cargo xtask ci <chip>` |
| Host-side unit tests | `cargo xtask host-tests` |
| HIL tests (needs hardware) | `cargo xtask run tests <chip> [--test name]` |
| Run an example | `cargo xtask run example [name] --chip <chip>` |
| Build docs | `cargo xtask build documentation --chips <list>` |
| Validate metadata | `cargo xtask update-metadata --check` |
| Validate changelog | `cargo xtask check-changelog` |

## Test & example metadata

Tests and examples use `//% KEY: value` comments to control xtask builds (full docs: `xtask/README.md`).

| Annotation | Purpose | Example |
|------------|---------|---------|
| `//% CHIPS:` | Restrict to specific chips | `//% CHIPS: esp32c6 esp32s3` |
| `//% FEATURES:` | Enable cargo features | `//% FEATURES: unstable embassy` |
| `//% ENV:` | Set environment variables | `//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4` |
| `//% CARGO-CONFIG:` | Cargo `--config` passthrough | `//% CARGO-CONFIG: ...` |

Named configs build the same file with different settings:
```
//% CHIPS(wifi): esp32 esp32c3 esp32c6 esp32s3
//% CHIPS(ble_only): esp32h2
//% FEATURES(wifi): esp-radio/wifi
//% FEATURES(ble_only): esp-radio/ble
```

## The `unstable` feature

- **Libraries** must never enable `unstable`. Use `requires-unstable` instead.
- **Applications** (examples, tests) may enable `unstable` freely.
- Mark new unstable APIs with `#[instability::unstable]`.
- `unstable_module!` macro: makes a module `pub` when `unstable` is enabled, `pub(crate)` otherwise.
- Features prefixed with `__` (e.g. `__bluetooth`) are private — never enable directly.

## esp-config

Build-time configuration defined in `esp_config.yml` per crate. Override via environment variables.

```rust
esp_config::esp_config_bool!("ESP_HAL_CONFIG_PLACE_SPI_MASTER_DRIVER_IN_RAM");
esp_config::esp_config_int!(u32, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET");
esp_config::esp_config_str!("ESP_HAL_CONFIG_PSRAM_MODE");
```

Config options must not alter the public API surface.

## Driver conventions

From `documentation/DEVELOPER-GUIDELINES.md` (read it for full details):

- **Pins**: `fn with_signal_name(self, pin: impl PeripheralInput) -> Self`
- **Constructor**: takes `impl Instance + 'd`, returns `Result<Self, ConfigError>`
- **Config**: `Config` struct with `procmacros::BuilderLite` derive + separate `ConfigError` enum
- **Modes**: `DriverMode` type param; default `Blocking`; provide `into_async`/`into_blocking`
- **Drop**: must reset peripheral to idle state
- **Derives**: drivers: `Debug`; configs: `Default, Debug, PartialEq, Eq, Clone, Copy, Hash`
- **Doc examples**: use `#[doc = crate::before_snippet!()]`, `no_run`, `?` operator (not unwrap)

## Conditional compilation

Chip properties come from `esp-metadata/devices/*.toml`, generated into `esp-metadata-generated/`.

Key symbols: `riscv` / `xtensa`, `single_core` / `multi_core`, `soc_has_<peripheral>`, `<driver>_driver_supported`.

Prefer these over `#[cfg(feature = "esp32c3")]` where possible.

## Changelog & migration

- Add entries to `CHANGELOG.md` under `## [Unreleased]` > `### Added/Changed/Fixed/Removed`
- Format: `- Description. (#PR_NUMBER)` — amend existing entries when modifying same feature: `(#789, #1234)`
- Breaking changes: add migration steps to `MIGRATING-*.md` in the affected crate root
- Breaking changes to stable API require a `breaking-change-<crate-name>` PR label

## PR checklist

1. `cargo xtask fmt-packages`
2. `cargo xtask lint-packages --chips <affected>` — fix all warnings
3. `cargo xtask update-metadata --check` — if metadata changed
4. `cargo xtask check-changelog` — add changelog entry if API changed
5. Build affected examples/tests for relevant chips
6. `cargo xtask host-tests` — if host-side code changed

## Key references

- `documentation/DEVELOPER-GUIDELINES.md` — full API design rules
- `documentation/CONTRIBUTING.md` — contribution workflow
- `xtask/README.md` — metadata annotations and xtask usage
- `.github/workflows/ci.yml` — CI steps and MSRV
- `esp-metadata/devices/*.toml` — per-chip peripheral definitions
