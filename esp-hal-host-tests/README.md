# esp-hal-host-tests

Host tests of esp-hal code that does not need hardware. The crate includes source files of other
crates with `#[path]`, and gives them stand-ins for the modules and `esp_hal` items they use
(`src/rtos/` and `esp-hal-shim/`).

## Tests

```sh
cargo xtask host-tests esp-hal-host-tests
```

CI runs the tests natively and under Miri.

## Kani proofs

The `proofs` modules hold [Kani](https://model-checking.github.io/kani/) harnesses. They build only
under Kani (`#[cfg(kani)]`), and CI does not run them. Each harness has a doc comment that explains
what it proves and how.

Kani runs on Linux and macOS. On Windows, use WSL.

### Linux and macOS

Install Kani once:

```sh
cargo install --locked kani-verifier
cargo kani setup
```

Run every proof, or one harness, from this directory:

```sh
cargo kani --tests
cargo kani --tests --harness max_priority_ready_gives_the_highest_marked_level
```

`--tests` is needed because the crate builds only as a test crate. The run queue proof takes about
7 minutes. The other proofs take seconds.

### Windows

Install WSL with Ubuntu once, from PowerShell:

```powershell
wsl --install -d Ubuntu
```

Then install Rust and a linker in Ubuntu, and Kani as on Linux:

```sh
sudo apt install build-essential
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install --locked kani-verifier
cargo kani setup
```

Run the proofs in Ubuntu, from the Windows checkout under `/mnt/c`. Put the target directory in the
WSL file system: a build under `/mnt/c` is much slower, and it would mix Linux build output into the
Windows target directory.

```sh
cd /mnt/c/path/to/esp-hal/esp-hal-host-tests
CARGO_TARGET_DIR=~/kani-target cargo kani --tests
```
