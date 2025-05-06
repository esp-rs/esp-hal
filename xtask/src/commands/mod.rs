use std::path::Path;

use anyhow::{Result, bail};
use clap::Args;
use esp_metadata::Chip;

pub use self::{build::*, run::*};
use crate::{Package, cargo::CargoAction};

mod build;
mod run;

// ----------------------------------------------------------------------------
// Subcommand Arguments

#[derive(Debug, Args)]
pub struct ExamplesArgs {
    /// Package whose examples we which to act on.
    #[arg(value_enum)]
    pub package: Package,
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,

    /// Build examples in debug mode only
    #[arg(long)]
    pub debug: bool,
    /// Optional example to act on (all examples used if omitted)
    #[arg(long)]
    pub example: Option<String>,
}

#[derive(Debug, Args)]
pub struct TestsArgs {
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,

    /// Repeat the tests for a specific number of times.
    #[arg(long, default_value_t = 1)]
    pub repeat: usize,
    /// Optional test to act on (all tests used if omitted)
    #[arg(long, short = 't')]
    pub test: Option<String>,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

pub fn examples(workspace: &Path, mut args: ExamplesArgs, action: CargoAction) -> Result<()> {
    // Ensure that the package/chip combination provided are valid:
    args.package.validate_package_chip(&args.chip)?;

    // If the 'esp-hal' package is specified, what we *really* want is the
    // 'examples' package instead:
    if args.package == Package::EspHal {
        log::warn!(
            "Package '{}' specified, using '{}' instead",
            Package::EspHal,
            Package::Examples
        );
        args.package = Package::Examples;
    }

    // Absolute path of the package's root:
    let package_path = crate::windows_safe_path(&workspace.join(args.package.to_string()));

    let example_path = match args.package {
        Package::Examples | Package::QaTest => package_path.join("src").join("bin"),
        Package::HilTest => package_path.join("tests"),
        _ => package_path.join("examples"),
    };

    // Load all examples which support the specified chip and parse their metadata:
    let mut examples = crate::firmware::load(&example_path)?
        .into_iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    // Sort all examples by name:
    examples.sort_by_key(|a| a.binary_name());

    // Execute the specified action:
    match action {
        CargoAction::Build(out_path) => build_examples(args, examples, &package_path, &out_path),
        CargoAction::Run if args.example.is_some() => run_example(args, examples, &package_path),
        CargoAction::Run => run_examples(args, examples, &package_path),
    }
}

pub fn tests(workspace: &Path, args: TestsArgs, action: CargoAction) -> Result<()> {
    // Absolute path of the 'hil-test' package's root:
    let package_path = crate::windows_safe_path(&workspace.join("hil-test"));

    // Determine the appropriate build target for the given package and chip:
    let target = Package::HilTest.target_triple(&args.chip)?;

    // Load all tests which support the specified chip and parse their metadata:
    let mut tests = crate::firmware::load(&package_path.join("tests"))?
        .into_iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    // Sort all tests by name:
    tests.sort_by_key(|a| a.binary_name());

    // Execute the specified action:
    if tests.iter().any(|test| test.matches(&args.test)) {
        for test in tests.iter().filter(|test| test.matches(&args.test)) {
            crate::execute_app(
                &package_path,
                args.chip,
                target,
                test,
                action.clone(),
                args.repeat,
                false,
            )?;
        }
        Ok(())
    } else if args.test.is_some() {
        bail!("Test not found or unsupported for the given chip")
    } else {
        let mut failed = Vec::new();
        for test in tests {
            if crate::execute_app(
                &package_path,
                args.chip,
                target,
                &test,
                action.clone(),
                args.repeat,
                false,
            )
            .is_err()
            {
                failed.push(test.name_with_configuration());
            }
        }

        if !failed.is_empty() {
            bail!("Failed tests: {:#?}", failed);
        }

        Ok(())
    }
}
