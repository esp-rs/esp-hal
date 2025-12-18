use std::path::Path;

use anyhow::{Context, Result, bail};
use clap::Args;
use esp_metadata::Chip;
use inquire::Select;
use strum::IntoEnumIterator;

pub use self::{build::*, check_changelog::*, release::*, run::*};
use crate::{
    Package,
    cargo::{CargoAction, CargoCommandBatcher},
};
mod build;
mod check_changelog;
#[cfg(feature = "report")]
pub mod generate_report;
#[cfg(feature = "semver-checks")]
pub(crate) mod generate_rom_symbols;
mod release;
mod run;

// ----------------------------------------------------------------------------
// Subcommand Arguments

/// Arguments common to commands which act on examples.
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

/// Arguments common to commands which act on doctests.
#[derive(Debug, Args)]
pub struct DocTestArgs {
    /// Package(s) where we wish to run doc tests.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    pub packages: Vec<Package>,
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,
}

/// Arguments common to commands which act on tests.
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
    /// Multiple tests may be selected via a comma-separated list, e.g. `--test rmt,i2c,uart`.
    /// The `test_suite::test_name` syntax allows selecting a specific test (and may be combined
    /// with commas).
    #[arg(long, short = 't', alias = "tests", value_delimiter = ',', num_args = 1..)]
    pub test: Option<Vec<String>>,

    /// The toolchain used to build the tests
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Emit crate build timings
    #[arg(long)]
    pub timings: bool,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

/// Execute the given action on the specified examples.
pub fn examples(workspace: &Path, mut args: ExamplesArgs, action: CargoAction) -> Result<()> {
    log::debug!(
        "Running examples for '{}' on '{:?}'",
        args.package,
        args.chip
    );
    if args.chip.is_none() {
        let chip_variants = Chip::iter().collect::<Vec<_>>();

        let chip = Select::new("Select your target chip:", chip_variants).prompt()?;

        args.chip = Some(chip);
    }

    let chip = args.chip.unwrap();

    // Ensure that the package/chip combination provided are valid:
    args.package.validate_package_chip(&chip).with_context(|| {
        format!(
            "The package '{0}' does not support the chip '{chip:?}'",
            args.package
        )
    })?;

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
        crate::firmware::load_cargo_toml(&package_path).with_context(|| {
            format!(
                "Failed to load specified examples from {}",
                package_path.display()
            )
        })?
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

    let mut filtered = vec![];

    if let Some(example) = args.example.as_deref() {
        filtered.clone_from(&examples);
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

/// Execute the given action on the specified HIL tests.
pub fn tests(workspace: &Path, args: TestsArgs, action: CargoAction) -> Result<()> {
    // Absolute path of the 'hil-test' package's root:
    let package_path = crate::windows_safe_path(&workspace.join("hil-test"));

    // Determine the appropriate build target for the given package and chip:
    let target = Package::HilTest.target_triple(&args.chip)?;

    // Load all tests which support the specified chip and parse their metadata:
    let mut tests = crate::firmware::load(&package_path.join("src").join("bin"))?
        .into_iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    // Sort all tests by name:
    tests.sort_by_key(|a| a.binary_name());

    if let CargoAction::Build(Some(out_dir)) = &action {
        // Make sure the tmp directory has no garbage for us.
        let tmp_dir = out_dir.join("tmp");
        _ = std::fs::remove_dir_all(&tmp_dir);
        std::fs::create_dir_all(&tmp_dir).unwrap();
    }

    let mut commands = CargoCommandBatcher::new();
    // Execute the specified action:
    // If user sets some specific test(s)
    if let Some(test_arg) = args.test.as_deref() {
        let trimmed: Vec<&str> = test_arg.iter().map(|s| s.trim()).collect();

        // Reject `--test ""` / `--test " "`
        if trimmed.iter().all(|s| s.is_empty()) {
            bail!("Empty test selector is not allowed.");
        }

        let mut selected_failed: Vec<String> = Vec::new();

        for selected in trimmed.into_iter().filter(|s| !s.is_empty()) {
            let (test_arg, filter) = match selected.split_once("::") {
                Some((test, filter)) => (Some(test), Some(filter)),
                None => (Some(selected), None),
            };

            let run_test_extra_args = (action == CargoAction::Run)
                .then(|| filter.as_ref().map(|f| std::slice::from_ref(f)))
                .flatten()
                .unwrap_or(&[]);

            let matched: Vec<_> = tests
                .iter()
                .filter(|t| t.matches(test_arg.as_deref()))
                .collect();

            if matched.is_empty() {
                selected_failed.push(selected.to_string());
            } else {
                for test in matched {
                    let command = crate::generate_build_command(
                        &package_path,
                        args.chip,
                        &target,
                        test,
                        action.clone(),
                        false,
                        args.toolchain.as_deref(),
                        args.timings,
                        run_test_extra_args,
                    )?;
                    commands.push(command);
                }
            }
        }

        if !selected_failed.is_empty() {
            bail!(
                "Test not found or unsupported for the given chip: {}",
                selected_failed.join(", ")
            )
        }
    } else {
        for test in tests {
            let command = crate::generate_build_command(
                &package_path,
                args.chip,
                &target,
                &test,
                action.clone(),
                false,
                args.toolchain.as_deref(),
                args.timings,
                &[],
            )?;
            commands.push(command);
        }
    }

    let mut failed = Vec::new();

    for c in commands.build(false) {
        let repeat = if matches!(action, CargoAction::Run) {
            args.repeat
        } else {
            1
        };

        println!(
            "Command: cargo {}",
            c.command.join(" ").replace("---", "\n    ---")
        );
        for i in 0..repeat {
            if repeat != 1 {
                log::info!("Run {}/{}", i + 1, repeat);
            }
            if c.run(false).is_err() {
                failed.push(c.artifact_name.clone());
            }
        }
    }

    move_artifacts(args.chip, &action);

    if !failed.is_empty() {
        bail!("Failed tests: {:#?}", failed);
    }

    Ok(())
}

fn move_artifacts(chip: Chip, action: &CargoAction) {
    if let CargoAction::Build(Some(out_dir)) = action {
        // Move binaries
        let from = out_dir.join("tmp");
        let to = out_dir.join(chip.to_string());
        std::fs::create_dir_all(&to).unwrap();

        // Binaries are in nested folders. There is one file in each folder. The name of the
        // final binary should be the name of the source binary's parent folder.
        for dir_entry in std::fs::read_dir(&from).unwrap() {
            let dir = dir_entry.unwrap();
            let mut bin_folder = std::fs::read_dir(dir.path()).unwrap();
            let file = bin_folder
                .next()
                .expect("No binary found")
                .expect("Failed to read entry");
            assert!(
                bin_folder.next().is_none(),
                "Only one binary should be present in each folder"
            );
            let source_file = file.path();
            let dest = to.join(dir.path().file_name().unwrap().to_string_lossy().as_ref());
            std::fs::rename(source_file, dest).unwrap();
        }
        // Clean up
        std::fs::remove_dir_all(from).unwrap();
    }
}
