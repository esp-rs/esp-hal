use std::path::Path;

use anyhow::{Result, bail};
use clap::Args;
use esp_metadata::Chip;
use inquire::Select;
use strum::IntoEnumIterator;

pub use self::{build::*, check_changelog::*, release::*, run::*};
use crate::{Package, cargo::CargoAction};
mod build;
mod check_changelog;
mod release;
mod run;

// ----------------------------------------------------------------------------
// Subcommand Arguments

#[derive(Debug, Args)]
pub struct ExamplesArgs {
    /// Example to act on ("all" will execute every example).
    pub example: Option<String>,
    /// Chip to target.
    #[arg(value_enum, long)]
    pub chip: Option<Chip>,
    /// Package whose examples we wish to act on.
    #[arg(value_enum, long, default_value_t = Package::Examples)]
    pub package: Package,
    /// Build examples in debug mode only
    #[arg(long)]
    pub debug: bool,

    /// The toolchain used to build the examples
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Emit crate build timings
    #[arg(long)]
    pub timings: bool,
}

#[derive(Debug, Args)]
pub struct DocTestArgs {
    /// Package where we wish to run doc tests.
    #[arg(value_enum)]
    pub package: Package,
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,
}

#[derive(Debug, Args)]
pub struct TestsArgs {
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,

    /// Repeat the tests for a specific number of times.
    #[arg(long, default_value_t = 1)]
    pub repeat: usize,
    /// Optional test to act on (all tests used if omitted).
    ///
    /// The `test_suite::test_name` syntax allows running a single specific test.
    #[arg(long, short = 't')]
    pub test: Option<String>,

    /// The toolchain used to build the tests
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Emit crate build timings
    #[arg(long)]
    pub timings: bool,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

pub fn examples(workspace: &Path, mut args: ExamplesArgs, action: CargoAction) -> Result<()> {
    if args.chip.is_none() {
        let chip_variants = Chip::iter().collect::<Vec<_>>();

        let chip = Select::new("Select your target chip:", chip_variants).prompt()?;

        args.chip = Some(chip);
    }

    let chip = args.chip.unwrap();

    // Ensure that the package/chip combination provided are valid:
    args.package.validate_package_chip(&chip)?;

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

    // Load all examples which support the specified chip and parse their metadata.
    //
    // The `examples` directory contains a number of individual projects, and does not rely on
    // metadata comments in the source files. As such, it needs to load its metadata differently
    // than other packages.
    let examples = if args.package == Package::Examples {
        crate::firmware::load_cargo_toml(&package_path)?
    } else {
        let example_path = match args.package {
            Package::QaTest => package_path.join("src").join("bin"),
            Package::HilTest => package_path.join("tests"),
            _ => package_path.join("examples"),
        };

        crate::firmware::load(&example_path)?
    };

    let mut examples = examples
        .into_iter()
        .filter(|example| example.supports_chip(chip))
        .collect::<Vec<_>>();

    // At this point, chip can never be `None`, so we can safely unwrap it.
    let chip = args.chip.unwrap();

    // Filter the examples down to only the binaries supported by the given chip
    examples.retain(|ex| ex.supports_chip(chip));

    // Sort all examples by name:
    examples.sort_by_key(|a| a.binary_name());

    let mut filtered = examples.clone();

    if let Some(example) = args.example.as_deref() {
        if !example.eq_ignore_ascii_case("all") {
            // Only keep the example the user wants
            filtered.retain(|ex| ex.matches_name(example));

            if filtered.is_empty() {
                log::warn!(
                    "Example '{example}' not found or unsupported for the given chip. Please select one of the existing examples in the desired package."
                );

                let example_name = inquire::Select::new(
                    "Select the example:",
                    examples.iter().map(|ex| ex.binary_name()).collect(),
                )
                .prompt()?;

                if let Some(selected) = examples.iter().find(|ex| ex.binary_name() == example_name)
                {
                    filtered.push(selected.clone());
                }
            }
        }
    } else {
        let example_name = inquire::Select::new(
            "Select an example:",
            examples.iter().map(|ex| ex.binary_name()).collect(),
        )
        .prompt()?;

        if let Some(selected) = examples.iter().find(|ex| ex.binary_name() == example_name) {
            filtered.push(selected.clone());
        }
    }

    // Execute the specified action:
    match action {
        CargoAction::Build(out_path) => build_examples(
            args,
            filtered,
            &package_path,
            out_path.as_ref().map(|p| p.as_path()),
        ),
        CargoAction::Run => run_examples(args, filtered, &package_path),
    }
}

pub fn tests(workspace: &Path, args: TestsArgs, action: CargoAction) -> Result<()> {
    let (test_arg, filter) = if let Some(test_arg) = args.test.as_deref() {
        match test_arg.split_once("::") {
            Some((test, filter)) => (Some(test), Some(filter)),
            None => (Some(test_arg), None),
        }
    } else {
        (None, None)
    };

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

    let run_test_extra_args = (action == CargoAction::Run)
        .then(|| filter.as_ref().map(|f| std::slice::from_ref(f)))
        .flatten()
        .unwrap_or(&[]);

    // Execute the specified action:
    if tests.iter().any(|test| test.matches(test_arg.as_deref())) {
        for test in tests
            .iter()
            .filter(|test| test.matches(test_arg.as_deref()))
        {
            crate::execute_app(
                &package_path,
                args.chip,
                &target,
                test,
                action.clone(),
                args.repeat,
                false,
                args.toolchain.as_deref(),
                args.timings,
                run_test_extra_args,
            )?;
        }
        Ok(())
    } else if test_arg.is_some() {
        bail!("Test not found or unsupported for the given chip")
    } else {
        let mut failed = Vec::new();
        for test in tests {
            if crate::execute_app(
                &package_path,
                args.chip,
                &target,
                &test,
                action.clone(),
                args.repeat,
                false,
                args.toolchain.as_deref(),
                args.timings,
                run_test_extra_args,
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
