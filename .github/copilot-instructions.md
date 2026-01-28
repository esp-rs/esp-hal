# Copilot agent onboarding for `esp-hal`

Purpose
- Give a coding agent the essential context, commands, and checks so PRs build cleanly and validate locally before proposing changes.
- Trust these instructions first — search the repo only when an instruction is missing or fails.

High-level summary
- `esp-hal` is a Rust, bare-metal (`no_std`) hardware abstraction layer for Espressif SoCs (ESP32 family, RISC-V/XTensa variants). The repository contains many crates (HALs, drivers, examples, tests) plus an `xtask` automation crate used to build, test, lint, format, and run CI-like workflows locally.
- Languages / runtimes: Rust (embedded, no_std), some host tests run on x86_64. Primary tooling is `cargo` + `cargo xtask`. CI uses GitHub Actions.

Important facts (quick)
- Primary automation hub: `xtask/` (see `xtask/README.md`). Use `cargo xtask <command>` for repository-level tasks.
- GitHub workflows live in `.github/workflows/*.yml`. The canonical source-of-truth for the Minimum Supported Rust Version (MSRV) is the `MSRV` environment variable in `.github/workflows/ci.yml`. Always read it from that file before selecting which toolchain to use for MSRV validation.
  - Example: to inspect the workflow for the MSRV value, check `.github/workflows/ci.yml` (look for the `MSRV:` env). You can extract it with a quick grep locally:
```esp-hal/.github/workflows/ci.yml#L1-200
grep -E '^\s*MSRV:\s*' .github/workflows/ci.yml || true
```
- `xtask`'s `main()` will set `CARGO_TARGET_DIR` to `<workspace>/target` if that env var is not already set — do not rely on a different target dir unless intentional.

Top-level files and directories (priority)
- `Cargo.toml` (workspace)
- `README.md` (project overview)
- `xtask/` (automation; primary place to run builds, lint, tests)
- `.github/` (workflows and PR template)
- `.cargo/config.toml` (useful aliases, e.g. `cargo xtask`)
- `documentation/` (contrib guides like `CONTRIBUTING.md`, `DEVELOPER-GUIDELINES.md`)
- Crate directories: `esp-hal`, `esp-alloc`, `esp-println`, `esp-lp-hal`, `esp-phy`, `esp-radio`, `esp-rtos`, `esp-storage`, `esp-metadata*`, `examples`, `extras`, `hil-test`, `qa-test`
- Configs: `rustfmt.toml`
- Licenses: `LICENSE-APACHE`, `LICENSE-MIT`

Developer / contributor expectations (must-dos before a PR)
- Always run formatting across changed crates: `cargo xtask fmt-packages` (this is required by the PR template). Reviewer agent: do not comment on formatting issues in Rust source code.
- Add/adjust changelog entries where relevant (changelog checks are automated).
- Follow `documentation/CONTRIBUTING.md` and `documentation/DEVELOPER-GUIDELINES.md` for API changes, HIL updates, and release guidance.
- Make sure examples affected by changes still build.

Recommended local environment bootstrap (what to install)
- Install Rust toolchains (determine MSRV by reading `.github/workflows/ci.yml` first):
  - Read MSRV from the workflow and install it. Example (inspect file first):
```esp-hal/.github/workflows/ci.yml#L1-200
# Look for a line like: MSRV: "1.88.0" in the workflow and then install that toolchain
grep -E '^\s*MSRV:\s*' .github/workflows/ci.yml || true
```
  - Then install the MSRV toolchain (example shown uses the discovered version):
```/dev/null/commands.md#L1-2
rustup toolchain install <MSRV_VERSION>
rustup component add rustfmt clippy --toolchain <MSRV_VERSION>
```
- Set up the Espressif `esp` toolchain using `espup`. Prefer the `esp` toolchain version documented in CI or project docs when available. Install it like this (replace <VERSION> with the desired esp toolchain version):
```/dev/null/commands.md#L3-3
cargo install espup
espup install # optionally -v <VERSION>
```
  - Note: the `esp` toolchain may be required to reproduce Xtensa builds locally. If the CI workflow uses an `esp` toolchain, mirror that version.
- Nightly (optional) for some doc/tests/format checks if required: `rustup toolchain install nightly` and `rustup component add rustfmt miri --toolchain nightly`
- Add common components:
  - For MSRV: `rustup component add rustfmt clippy --toolchain <MSRV>`
  - For nightly checks (if you use them): `rustup component add rustfmt miri --toolchain nightly`
- Recommended: make sure `rust-src` is available for cross-target builds where CI uses it:
  - `rustup component add rust-src --toolchain <MSRV>`

Primary commands (bootstrap → build → test → lint → docs)
- Bootstrap (one-time / ensure toolchain and components installed)
  - Set up rust toolchains and components as above.
  - Optional: create a consistent `CARGO_TARGET_DIR` if you have multi-repo workflows. By default `xtask` sets it to `<workspace>/target`.
- Formatting (always before commit)
  - `cargo xtask fmt-packages`
- Linting (recommended before PR)
  - `cargo xtask lint-packages`
  - For targeted linting: `cargo xtask lint-packages --packages <comma-separated>`
- Build examples / tests / packages
  - See `cargo xtask build --help` or `cargo xtask build examples --help`. Typical patterns:
    - Build an example: `cargo xtask build examples <EXAMPLE> --chips <chip>` (use `--chips` to limit targets)
    - Build tests: `cargo xtask build tests <package-or-test> --chips <chip>`
  - To run host-side tests: `cargo xtask host-tests` (some host tests are executed inside `xtask` itself; in CI they `cd xtask && cargo test --features release`).
- Running examples/tests
  - `cargo xtask run tests <chip> <testname>` or `cargo xtask run example <chip> <example-name>`
- Documentation
  - Build docs (heavy): `cargo xtask build documentation --chips <chip-list>`
  - Run doctests: `cargo xtask run doc-tests <CHIP>`
  - Use `--packages` and `--chips` to reduce scope and build time during development.
- Metadata & changelog checks (CI enforces)
  - `cargo xtask check-changelog`
  - `cargo xtask update-metadata` / `cargo xtask update-metadata --check` (CI runs metadata checks)
- Quick clean
  - `cargo xtask clean`
  - There's also alias support in `.cargo/config.toml` (e.g. `cargo xclean` if installed).

CI / validation notes (what GH Actions runs)
- CI workflows are in `.github/workflows/` and include `ci.yml`, `ci_nightly.yml`, `hil.yml`, and various helper workflows that:
  - Run builds for multiple chips and targets (XTENSA / RISCV combinations).
  - Use MSRV (read from `.github/workflows/ci.yml`) for many jobs; some jobs run on `nightly` (format/miri).
  - Run `cargo xtask` subcommands to format, lint, run host-tests, build documentation, generate reports, and run HIL tests.
- To mirror CI locally, run the same `cargo xtask` commands the workflows run:
  - Format check: `cargo xtask fmt-packages --check`
  - Metadata check: `cargo xtask update-metadata --check`
  - Host tests: `cargo xtask host-tests`
  - Changelog check: `cargo xtask check-changelog`

Common pitfalls and tips
- Long-running builds: documentation builds and building all examples for all chips is time-consuming. Use `--packages` and `--chips` to scope work.
- `CARGO_TARGET_DIR`: `xtask` will set the target dir to `<workspace>/target` if unset. If you set `CARGO_TARGET_DIR` externally, `xtask` will not override it — prefer leaving it unset unless you know why.
- Formatting & linting: maintainers expect `cargo xtask fmt-packages` and `cargo xtask lint-packages` to pass; include formatting changes in the PR.
- Changelog & migration guide: if your change modifies public APIs, update changelogs and migration documentation. CI checks will reject PRs missing required changelog entries.
- HIL and hardware-related changes: hardware behavior can differ from CI — prefer testing on actual hardware for critical changes and mark in PR if hardware testing is pending.

Where to make code changes (architecture overview)
- Crates are organized under root directories named after crates (e.g. `esp-hal/`, `esp-alloc/`, `esp-println/`). Many crates implement chip-specific HALs and drivers.
- Examples: `examples/` contains usage examples that CI builds for compatibility verification.
- Tests & hardware-in-the-loop: `hil-test/` and `qa-test/` contain HIL tests and QA harnesses.
- Automation: `xtask/` is the authoritative entrypoint for repository-wide tasks; prefer using or extending `xtask` for CI-like operations or new automation.

Agent workflow recommendations (how you should act)
1. Read this file and trust it. Do not run a full repo search unless an instruction is missing or an executed command fails.
2. Before producing a PR:
   - Run `cargo xtask fmt-packages`
   - Run `cargo xtask lint-packages` (or targeted linting)
   - Run `cargo xtask check-changelog`
   - Run any impacted example builds: `cargo xtask build examples <EXAMPLE> --chips <chip>` (use `--chips` to limit)
   - If you modify metadata, run `cargo xtask update-metadata --check`
3. If any of the above commands fail, surface the exact error and only then search or open files to locate the cause.
4. Keep changes small and focused (one logical change per PR), include changelog entries, and respect style changes produced by `fmt-packages` (do not hand-edit formatting diffs).

If something in these instructions is incorrect or incomplete
- Only then perform targeted searches: prefer `xtask/` and `.github/workflows/ci.yml` first, then `documentation/` files.
- When uncertain about toolchain constraints, prefer the MSRV value read from `.github/workflows/ci.yml`.

Short checklist for PRs (agent-friendly)
- [ ] Ran `cargo xtask fmt-packages`
- [ ] Ran `cargo xtask lint-packages` (or targeted equivalent) and fixed issues
- [ ] Added/updated changelog entry if API behavior changed
- [ ] Ran `cargo xtask update-metadata --check` if package metadata changed
- [ ] Built relevant examples/tests with `cargo xtask build ... --chips ...`
- [ ] Verified host-tests if you changed host-side code: `cargo xtask host-tests`

Key reference paths (where to look first)
- `xtask/README.md` — xtask usage and metadata annotations (examples/tests)
- `documentation/CONTRIBUTING.md` and `documentation/DEVELOPER-GUIDELINES.md` — contribution rules
- `.github/workflows/ci.yml` — canonical CI steps and toolchain versions
- `.cargo/config.toml` — helpful cargo aliases (`xtask`, `xfmt`, etc.)
- `README.md` — project overview and links

Final note to the agent
- Follow this file as your primary orientation. It condenses the commands and checks CI runs and reduces unnecessary repository exploration. Only search further when a commanded step fails or when the task requires additional context not documented here.
