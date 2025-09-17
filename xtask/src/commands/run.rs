use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context as _, Result, bail};
use clap::{Args, Subcommand};
use esp_metadata::Chip;

use super::{DocTestArgs, ExamplesArgs, TestsArgs};
use crate::{
    cargo::{CargoAction, CargoArgsBuilder},
    firmware::Metadata,
};

// ----------------------------------------------------------------------------
// Subcommands

#[derive(Debug, Subcommand)]
pub enum Run {
    /// Run doctests for specified chip and package.
    DocTests(DocTestArgs),
    /// Run all ELFs in a folder.
    Elfs(RunElfsArgs),
    /// Run the given example for the specified chip.
    Example(ExamplesArgs),
    /// Run all applicable tests or the specified test for a specified chip.
    Tests(TestsArgs),
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

/// Arguments for running ELFs.
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

/// Run doc tests for the specified package and chip.
pub fn run_doc_tests(workspace: &Path, args: DocTestArgs) -> Result<()> {
    log::debug!(
        "Running doc tests for '{}' on '{}'",
        args.package,
        args.chip
    );
    let chip = args.chip;

    let package_name = args.package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    // Determine the appropriate build target, and cargo features for the given
    // package and chip:
    let target = args.package.target_triple(&chip)?;
    let mut features = args
        .package
        .doc_feature_rules(&esp_metadata::Config::for_chip(&chip));
    features.push(chip.to_string());

    // We need `nightly` for building the doc tests, unfortunately:
    let toolchain = if chip.is_xtensa() { "esp" } else { "nightly" };

    // Build up an array of command-line arguments to pass to `cargo`:
    let builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("test")
        .arg("--doc")
        .arg("-Zbuild-std=core,panic_abort")
        .target(target)
        .features(&features)
        .arg("--release");

    let args = builder.build();
    log::debug!("{args:#?}");

    let envs = vec![
        ("RUSTDOCFLAGS", "--cfg docsrs --cfg not_really_docsrs"),
        ("ESP_HAL_DOCTEST", "1"),
    ];

    // Execute `cargo doc` from the package root:
    crate::cargo::run_with_env(&args, &package_path, envs, false)?;

    Ok(())
}

/// Run all ELFs in the specified folder using `probe-rs`.
pub fn run_elfs(args: RunElfsArgs) -> Result<()> {
    let mut failed: Vec<String> = Vec::new();
    for elf in fs::read_dir(&args.path)
        .with_context(|| format!("Failed to read {}", args.path.display()))?
    {
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

/// Run the specified examples for the given chip.
pub fn run_examples(
    args: ExamplesArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
) -> Result<()> {
    let mut examples = examples;

    // At this point, chip can never be `None`, so we can safely unwrap it.
    let chip = args.chip.unwrap();
    let target = args.package.target_triple(&chip)?;

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

        while !skip {
            let result = crate::execute_app(
                package_path,
                chip,
                &target,
                &example,
                CargoAction::Run,
                args.debug,
                args.toolchain.as_deref(),
                args.timings,
                &[],
            );

            if let Err(error) = result {
                log::error!("Failed to run example: {}", error);
                log::info!("Retry or skip? (r/s)");
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
            } else {
                break;
            }
        }
    }

    Ok(())
}
