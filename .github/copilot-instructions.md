# Agent instructions for `esp-hal`

`esp-hal` is a Rust bare-metal (`no_std`) HAL for Espressif SoCs (ESP32 family). The repo contains many crates plus an `xtask` automation crate. All repository-level tasks go through `cargo xtask`.

## MCP server

xtask ships an MCP server that exposes safe build/lint/check commands as tools. **Use it instead of running shell commands when available.**

### Start manually

```sh
cargo xmcp
```

### Register in VS Code (`.vscode/mcp.json`)

```json
{
  "servers": {
    "xtask": {
      "type": "stdio",
      "command": "cargo",
      "args": ["xmcp"]
    }
  }
}
```

### Register in Claude Code (`.claude/claude_code_config.json` or `~/.claude/claude_code_config.json`)

```json
{
  "mcpServers": {
    "xtask": {
      "command": "cargo",
      "args": ["xmcp"]
    }
  }
}
```

### Register in Gemini CLI (`.gemini/settings.json` or `~/.gemini/settings.json`)

```json
{
  "mcpServers": {
    "esp-hal-xtask": {
      "command": "cargo xmcp"
    }
  }
}
```

The MCP server exposes only safe commands (build, lint, check, format, test). Release commands are intentionally excluded. The allow-list lives in `build_mcp_registry()` in `xtask/src/main.rs`.

## Important: command performance

**Many commands are slow** when run against all chips and packages. Always filter to the minimum scope needed:
- Use `chips` to target a single chip (e.g. `esp32c3`) instead of all.
- Use `packages` to target only the crate you changed.
- Use `example` or `test` to build/run a single item instead of `all`.

**The `ci` command is a full CI simulation and is very slow.** Only run it as a final pre-PR validation step after all other work is complete and individually tested. Do not use it during iterative development.

## Key commands

| Task | Command |
|------|---------|
| Format (required before PR) | `cargo xtask fmt-packages` |
| Lint | `cargo xtask lint-packages` |
| Check packages | `cargo xtask check-packages` |
| Build examples | `cargo xtask build examples --chip <chip> <EXAMPLE NAME \| all>` |
| Build tests | `cargo xtask build tests --chip <chip> --test <TEST NAME>` |
| Run host tests | `cargo xtask host-tests` |
| Check changelog | `cargo xtask check-changelog` |
| Check metadata | `cargo xtask update-metadata --check` |
| Build docs | `cargo xtask build documentation --chips <chip>` |

Use `--packages` and `--chips` to scope work and reduce build time.

## MSRV

The canonical Minimum Supported Rust Version is in `.github/workflows/ci.yml` (the `MSRV:` env var). Read it from there:

```sh
grep -E '^\s*MSRV:\s*' .github/workflows/ci.yml
```

## Project layout

- `xtask/` — automation (build, lint, test, format, CI)
- `esp-hal/` — main HAL crate
- `esp-metadata/` — chip metadata
- `examples/` — usage examples (CI builds these)
- `hil-test/`, `qa-test/` — hardware-in-the-loop tests
- `documentation/` — `CONTRIBUTING.md`, `DEVELOPER-GUIDELINES.md`
- `.github/workflows/ci.yml` — CI definitions and toolchain versions

## PR checklist

1. Run `cargo xtask lint-packages` (targeted: `--packages <name>`) and fix issues
2. Add/update changelog entry if API behavior changed
3. Run `cargo xtask update-metadata --check` if package metadata changed
4. Build relevant examples: `cargo xtask build examples --chip <chip> <EXAMPLE NAME | all>`
5. Run `cargo xtask host-tests` if host-side code changed
6. Do not hand-edit formatting diffs — let `fmt-packages` handle them - Run `cargo xtask fmt-packages`
7. **Final step only**: run `ci` for the relevant chip(s) once everything above passes
