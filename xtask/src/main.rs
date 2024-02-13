use std::path::{Path, PathBuf};

use anyhow::{bail, Result};
use clap::{Args, Parser};
use xtask::{Chip, Package};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build documentation for the specified chip.
    BuildDocumentation(BuildDocumentationArgs),
    /// Build all examples for the specified chip.
    BuildExamples(BuildExamplesArgs),
}

#[derive(Debug, Args)]
struct BuildDocumentationArgs {
    /// Package to build documentation for.
    #[arg(value_enum)]
    package: Package,
    /// Which chip to build the documentation for.
    #[arg(value_enum)]
    chip: Chip,
    /// Open the documentation in the default browser once built.
    #[arg(long)]
    open: bool,
}

#[derive(Debug, Args)]
struct BuildExamplesArgs {
    /// Package to build examples for.
    #[arg(value_enum)]
    package: Package,
    /// Which chip to build the examples for.
    #[arg(value_enum)]
    chip: Chip,
}

// ----------------------------------------------------------------------------
// Application

fn main() -> Result<()> {
    env_logger::Builder::new()
        .filter_module("xtask", log::LevelFilter::Info)
        .init();

    let workspace = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = workspace.parent().unwrap().canonicalize()?;

    match Cli::parse() {
        Cli::BuildDocumentation(args) => build_documentation(&workspace, args),
        Cli::BuildExamples(args) => build_examples(&workspace, args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn build_documentation(workspace: &Path, args: BuildDocumentationArgs) -> Result<()> {
    // Ensure that the package/chip combination provided are valid:
    validate_package_chip(&args.package, &args.chip)?;

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;

    // Simply build the documentation for the specified package, targeting the
    // specified chip:
    xtask::build_documentation(workspace, args.package, args.chip, target, args.open)
}

fn build_examples(workspace: &Path, mut args: BuildExamplesArgs) -> Result<()> {
    // Ensure that the package/chip combination provided are valid:
    validate_package_chip(&args.package, &args.chip)?;

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
    let package_path = workspace.join(args.package.to_string());

    // Absolute path to the directory containing the examples:
    let example_path = if args.package == Package::Examples {
        package_path.join("src").join("bin")
    } else {
        package_path.join("examples")
    };

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;

    // Load all examples and parse their metadata:
    xtask::load_examples(&example_path)?
        .iter()
        // Filter down the examples to only those for which the specified chip is supported:
        .filter(|example| example.supports_chip(args.chip))
        // Attempt to build each supported example, with all required features enabled:
        .try_for_each(|example| xtask::build_example(&package_path, args.chip, target, example))
}

// ----------------------------------------------------------------------------
// Helper Functions

fn target_triple<'a>(package: &'a Package, chip: &'a Chip) -> Result<&'a str> {
    if *package == Package::EspLpHal {
        chip.lp_target()
    } else {
        Ok(chip.target())
    }
}

fn validate_package_chip(package: &Package, chip: &Chip) -> Result<()> {
    if *package == Package::EspLpHal && !chip.has_lp_core() {
        bail!(
            "Invalid chip provided for package '{}': '{}'",
            package,
            chip
        );
    }

    Ok(())
}
