use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context as _, Result, bail, ensure};
use clap::{Args, Subcommand};
use esp_metadata::Chip;

use super::{ExamplesArgs, TestsArgs};
use crate::{
    cargo::{CargoAction, CargoArgsBuilder},
    firmware::Metadata,
};

// ----------------------------------------------------------------------------
// Subcommands

#[derive(Debug, Subcommand)]
pub enum Run {
    /// Run doctests for specified chip and package.
    DocTests(ExamplesArgs),
    /// Run all ELFs in a folder.
    Elfs(RunElfsArgs),
    /// Run the given example for the specified chip.
    Example(ExamplesArgs),
    /// Run all applicable tests or the specified test for a specified chip.
    Tests(TestsArgs),
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

#[derive(Debug, Args)]
pub struct RunElfsArgs {
    /// Which chip to run the tests for.
    #[arg(value_enum)]
    pub chip: Chip,
    /// Path to the ELFs.
    pub path: PathBuf,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

pub fn run_doc_tests(workspace: &Path, args: ExamplesArgs) -> Result<()> {
    let chip = args.chip;

    let package_name = args.package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    // Determine the appropriate build target, and cargo features for the given
    // package and chip:
    let target = args.package.target_triple(&chip)?;
    let features = vec![chip.to_string(), "unstable".to_string()];

    // We need `nightly` for building the doc tests, unfortunately:
    let toolchain = if chip.is_xtensa() { "esp" } else { "nightly" };

    // Build up an array of command-line arguments to pass to `cargo`:
    let builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("test")
        .arg("--doc")
        .arg("-Zdoctest-xcompile")
        .arg("-Zbuild-std=core,panic_abort")
        .target(target)
        .features(&features)
        .arg("--release");

    let args = builder.build();
    log::debug!("{args:#?}");

    // Execute `cargo doc` from the package root:
    crate::cargo::run(&args, &package_path)?;

    Ok(())
}

pub fn run_elfs(args: RunElfsArgs) -> Result<()> {
    let mut failed: Vec<String> = Vec::new();
    for elf in fs::read_dir(&args.path)? {
        let entry = elf?;

        let elf_path = entry.path();
        let elf_name = elf_path
            .with_extension("")
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string();

        log::info!("Running test '{}' for '{}'", elf_name, args.chip);

        let mut command = Command::new("probe-rs");
        command.arg("run").arg(elf_path);

        if args.chip == Chip::Esp32c2 {
            command.arg("--speed").arg("15000");
        };

        command.arg("--verify");

        let mut command = command.spawn().context("Failed to execute probe-rs")?;
        let status = command
            .wait()
            .context("Error while waiting for probe-rs to exit")?;

        log::info!("'{elf_name}' done");

        if !status.success() {
            failed.push(elf_name);
        }
    }

    if !failed.is_empty() {
        bail!("Failed tests: {:?}", failed);
    }

    Ok(())
}

pub fn run_example(args: ExamplesArgs, examples: Vec<Metadata>, package_path: &Path) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = args.package.target_triple(&args.chip)?;

    // Filter the examples down to only the binary we're interested in, assuming it
    // actually supports the specified chip:
    let mut found_one = false;
    for example in examples.iter().filter(|ex| ex.matches(&args.example)) {
        found_one = true;
        crate::execute_app(
            package_path,
            args.chip,
            target,
            example,
            CargoAction::Run,
            1,
            args.debug,
        )?;
    }

    ensure!(
        found_one,
        "Example not found or unsupported for {}",
        args.chip
    );

    Ok(())
}

pub fn run_examples(
    args: ExamplesArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = args.package.target_triple(&args.chip)?;

    // Filter the examples down to only the binaries we're interested in
    let mut examples: Vec<Metadata> = examples
        .iter()
        .filter(|ex| ex.supports_chip(args.chip))
        .cloned()
        .collect();
    examples.sort_by_key(|ex| ex.tag());

    let console = console::Term::stdout();

    for example in examples {
        let mut skip = false;

        log::info!("Running example '{}'", example.output_file_name());
        if let Some(description) = example.description() {
            log::info!(
                "\n\n{}\n\nPress ENTER to run example, `s` to skip",
                description.trim()
            );
        } else {
            log::info!("\n\nPress ENTER to run example, `s` to skip");
        }

        loop {
            let key = console.read_key();

            match key {
                Ok(console::Key::Enter) => break,
                Ok(console::Key::Char('s')) => {
                    skip = true;
                    break;
                }
                _ => (),
            }
        }

        if !skip {
            while !skip
                && crate::execute_app(
                    package_path,
                    args.chip,
                    target,
                    &example,
                    CargoAction::Run,
                    1,
                    args.debug,
                )
                .is_err()
            {
                log::info!("Failed to run example. Retry or skip? (r/s)");
                loop {
                    let key = console.read_key();

                    match key {
                        Ok(console::Key::Char('r')) => break,
                        Ok(console::Key::Char('s')) => {
                            skip = true;
                            break;
                        }
                        _ => (),
                    }
                }
            }
        }
    }

    Ok(())
}
