# esp-hal Agent Instructions

Trust this document first — search the repo only when needed.

## Overview

- **esp-hal**: Rust bare-metal (`no_std`) HAL for Espressif SoCs (ESP32 family)
- **Automation**: `cargo xtask <command>` for builds, tests, lints, formatting
- **CI**: GitHub Actions in `.github/workflows/`

## Supported Chips

esp32, esp32c2, esp32c3, esp32c5, esp32c6, esp32h2, esp32s2, esp32s3

## Key Packages

| Package | Description |
|---------|-------------|
| esp-hal | Main HAL crate |
| esp-radio | WiFi, BLE, IEEE 802.15.4 |
| esp-alloc, esp-backtrace, esp-println | Support crates |
| esp-lp-hal | Low-power coprocessor HAL |

## Before Every PR

```bash
cargo xtask fmt-packages       # Required: format all code
cargo xtask lint-packages      # Check for issues
cargo xtask check-changelog    # Verify changelog entries
cargo xtask build examples --chip <chip>  # Build affected examples
```

If package metadata changed:
```bash
cargo xtask update-metadata --check
```

## Common Commands

| Task | Command |
|------|---------|
| Format | `cargo xtask fmt-packages` |
| Lint | `cargo xtask lint-packages [--packages pkg1,pkg2]` |
| Build example | `cargo xtask build examples <name> --chip <chip>` |
| Build all examples | `cargo xtask build examples --chip <chip>` |
| Build tests | `cargo xtask build tests <chip>` |
| Run tests | `cargo xtask run tests <chip>` |
| Run example | `cargo xtask run example <name> --chip <chip>` |
| Host tests | `cargo xtask host-tests` |
| Run CI locally | `cargo xtask ci <chip>` |
| Build docs | `cargo xtask build documentation --chips <chip>` |
| Clean | `cargo xtask clean` |

## Environment Setup

1. Read MSRV from `.github/workflows/ci.yml` (`MSRV:` env var)
2. Install toolchains:
   ```bash
   rustup toolchain install <MSRV>
   rustup component add rustfmt clippy rust-src --toolchain <MSRV>
   ```
3. For Xtensa chips (esp32, esp32s2, esp32s3):
   ```bash
   cargo install espup && espup install
   ```

## Project Structure

```
esp-hal/           # Main HAL
esp-radio/         # Wireless support
esp-alloc/         # Allocator
examples/          # Usage examples (CI builds these)
hil-test/          # Hardware-in-loop tests
qa-test/           # QA test harness
xtask/             # Build automation
documentation/     # CONTRIBUTING.md, DEVELOPER-GUIDELINES.md
.github/workflows/ # CI definitions
```

## Agent Workflow

1. Trust this file; avoid full repo searches
2. Run pre-PR commands (format, lint, changelog, build)
3. On failures, read error output first, then search targeted files
4. Keep changes focused; include changelog entries for API changes
5. Don't hand-edit formatting — let `fmt-packages` handle it

## MCP Server

The MCP server exposes xtask commands for programmatic access:

```bash
cargo xmcp  # Start MCP server (uses cargo alias)
```

Configure in VS Code (`.vscode/mcp.json`):
```json
{
  "servers": {
    "esp-hal-xtask": {
      "type": "stdio",
      "command": "cargo",
      "args": ["xmcp"],
      "cwd": "${workspaceFolder}"
    }
  }
}
```

### MCP Tool Reference

| Tool | Description |
|------|-------------|
| `fmt_packages` | Format code |
| `lint_packages` | Run clippy |
| `check_changelog` | Verify changelog |
| `build_examples` | Build examples for chip |
| `build_tests` | Build tests for chip |
| `run_tests` | Run tests on chip |
| `run_example` | Run specific example |
| `ci` | Run full CI for chip |
| `list_chips` | List supported chips |
| `list_packages` | List workspace packages |
| `help` | Get xtask help |

## PR Checklist

- [ ] `cargo xtask fmt-packages`
- [ ] `cargo xtask lint-packages` passes
- [ ] Changelog updated (if API changed)
- [ ] `cargo xtask update-metadata --check` (if metadata changed)
- [ ] Examples build: `cargo xtask build examples --chip <chip>`
- [ ] Host tests pass: `cargo xtask host-tests`

## Key Files

- `xtask/README.md` — xtask usage details
- `documentation/CONTRIBUTING.md` — contribution guidelines
- `.github/workflows/ci.yml` — CI config, MSRV source
- `.cargo/config.toml` — cargo aliases
