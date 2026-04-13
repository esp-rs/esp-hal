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
    Package,
    cargo::{CargoAction, CargoArgsBuilder},
    firmware::Metadata,
    radio_hil_runner::{extract_radio_tests, get_test_pair, run_radio_test},
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
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run all ELFs in a folder using probe-rs",
        command = "run elfs"
    )
)]
#[derive(Debug, Args)]
pub struct RunElfsArgs {
    /// Which chip to run the tests for.
    #[arg(value_enum)]
    pub chip: Chip,
    /// Path to the ELFs.
    pub path: PathBuf,
    /// Optional list of elfs to execute
    #[arg(long, value_delimiter = ',')]
    pub elfs: Vec<String>,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

/// Run doc tests for the specified package and chip.
pub fn run_doc_tests(workspace: &Path, args: DocTestArgs) -> Result<()> {
    let mut success = true;
    for package in args.packages {
        success &= run_doc_tests_for_package(workspace, package, args.chip)?;
    }
    anyhow::ensure!(success, "One or more doc tests failed");
    Ok(())
}

pub fn run_doc_tests_for_package(workspace: &Path, package: Package, chip: Chip) -> Result<bool> {
    log::info!("Running doc tests for '{}' on '{}'", package, chip);

    // FIXME: this list can and should slowly be expanded, and eventually the check be removed as
    // the docs are fixed up.
    let temporary_package_list = [
        Package::EspHal,
        Package::EspRadio,
        Package::EspBootloaderEspIdf,
    ];
    if !temporary_package_list.contains(&package) {
        log::info!("Package {} is temporarily not doctested", package);
        return Ok(true);
    }

    // Ensure that the package/chip combination provided are valid:
    if let Err(err) = package.validate_package_chip(&chip) {
        log::warn!("{err}");
        return Ok(true);
    }

    // Packages that have doc features are documented. We run doc-tests for these, and only these.
    let Some(mut features) = package.doc_feature_rules(&esp_metadata::Config::for_chip(&chip))
    else {
        log::info!("Skipping undocumented package {package}.");
        return Ok(true);
    };

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    if package.has_chip_features() {
        features.push(chip.to_string());
    }

    // Determine the appropriate build target, and cargo features for the given
    // package and chip:
    let Ok(target) = package.target_triple(&chip) else {
        log::warn!("Package {package} is not applicable for {chip}");
        return Ok(true);
    };

    // We need `nightly` for building the doc tests, unfortunately:
    let toolchain = if chip.is_xtensa() { "esp" } else { "nightly" };

    // Build up an array of command-line arguments to pass to `cargo`:
    let builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("test")
        .arg("--doc")
        .arg("-Zbuild-std=core,alloc,panic_abort")
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
    let success = crate::cargo::run_with_env(&args, &package_path, envs, false).is_ok();
    Ok(success)
}

/// Run all (or filtered) ELFs in the specified folder using `probe-rs`.
pub fn run_elfs(args: RunElfsArgs) -> Result<()> {
    let mut failed: Vec<String> = Vec::new();
    let filters: Vec<String> = args
        .elfs
        .iter()
        .map(|s| s.trim().to_lowercase())
        .filter(|s| !s.is_empty())
        .collect();

    for elf in fs::read_dir(&args.path)
        .with_context(|| format!("Failed to read {}", args.path.display()))?
    {
        let entry = elf?;
        let elf_path = entry.path();
        if !elf_path.is_file() {
            continue;
        }

        let elf_name = elf_path
            .with_extension("")
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string();

        if !filters.is_empty() && !filters.iter().any(|f| elf_name.to_lowercase().contains(f)) {
            log::info!(
                "Skipping test '{}' for '{}' (does not match filters: {:?})",
                elf_name,
                args.chip,
                filters,
            );
            continue;
        }

        let test_names = extract_radio_tests(&elf_path)?;
        let (ap_test, sta_test) = get_test_pair(&test_names)?;

        log::info!("Running test '{}' for '{}'", elf_name, args.chip);

        let is_radio_test = elf_name.contains("esp_radio");

        if is_radio_test {
            if let Err(e) = run_radio_test(&elf_path, &ap_test, &sta_test, 120, None) {
                failed.push(elf_name.clone());
                log::error!("Radio test '{}' failed: {}", elf_name, e);
            } else {
                log::info!("'{}' passed", elf_name);
            }
        } else {
            // Use standard probe-rs runner for HAL tests
            let status = Command::new("probe-rs")
                .arg("run")
                .arg(&elf_path)
                .arg("--verify")
                .status()
                .context("Failed to execute probe-rs")?;

            log::info!("'{}' done", elf_name);

            if !status.success() {
                failed.push(elf_name);
            }
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
    let target = args.package.as_package().target_triple(&chip)?;

    examples.sort_by_key(|ex| ex.tag());

    let console = console::Term::stdout();
    let interactive = console.is_term();

    for example in examples {
        let mut skip = false;

        log::info!("Running example '{}'", example.output_file_name());

        if interactive {
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
                if interactive {
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
                    return Err(error);
                }
            } else {
                break;
            }
        }
    }

    Ok(())
}
