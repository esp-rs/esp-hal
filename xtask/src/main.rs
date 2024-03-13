use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{bail, Result};
use clap::{Args, Parser};
use strum::IntoEnumIterator;
use xtask::{Chip, Package, Version};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build documentation for the specified chip.
    BuildDocumentation(BuildDocumentationArgs),
    /// Build all examples for the specified chip.
    BuildExamples(BuildExamplesArgs),
    /// Build the specified package with the given options.
    BuildPackage(BuildPackageArgs),
    /// Bump the version of the specified package(s).
    BumpVersion(BumpVersionArgs),
    /// Generate the eFuse fields source file from a CSV.
    GenerateEfuseFields(GenerateEfuseFieldsArgs),
    /// Run the given example for the specified chip.
    RunExample(RunExampleArgs),
}

#[derive(Debug, Args)]
struct BuildDocumentationArgs {
    /// Open the documentation in the default browser once built.
    #[arg(long)]
    open: bool,
    /// Package to build documentation for.
    #[arg(value_enum)]
    package: Package,
    /// Which chip to build the documentation for.
    #[arg(value_enum, default_values_t = Chip::iter())]
    chips: Vec<Chip>,
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

#[derive(Debug, Args)]
struct BuildPackageArgs {
    /// Package to build.
    #[arg(value_enum)]
    package: Package,
    /// Target to build for.
    #[arg(long)]
    target: Option<String>,
    /// Features to build with.
    #[arg(long, value_delimiter = ',')]
    features: Vec<String>,
    /// Toolchain to build with.
    #[arg(long)]
    toolchain: Option<String>,
}

#[derive(Debug, Args)]
struct GenerateEfuseFieldsArgs {
    /// Path to the local ESP-IDF repository.
    idf_path: PathBuf,
    /// Chip to build eFuse fields table for.
    #[arg(value_enum)]
    chip: Chip,
}

#[derive(Debug, Args)]
struct BumpVersionArgs {
    /// How much to bump the version by.
    #[arg(value_enum)]
    amount: Version,
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

#[derive(Debug, Args)]
struct RunExampleArgs {
    /// Package to run example from.
    #[arg(value_enum)]
    package: Package,
    /// Which chip to run the examples for.
    #[arg(value_enum)]
    chip: Chip,
    /// Which example to run
    example: String,
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
        Cli::BuildPackage(args) => build_package(&workspace, args),
        Cli::BumpVersion(args) => bump_version(&workspace, args),
        Cli::GenerateEfuseFields(args) => generate_efuse_src(&workspace, args),
        Cli::RunExample(args) => run_example(&workspace, args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn build_documentation(workspace: &Path, args: BuildDocumentationArgs) -> Result<()> {
    let output_path = workspace.join("docs");
    let resources = workspace.join("resources");

    let package = args.package.to_string();
    let version = xtask::package_version(&workspace, args.package)?;

    let mut crates = Vec::new();

    for chip in args.chips {
        // Ensure that the package/chip combination provided are valid:
        validate_package_chip(&args.package, &chip)?;

        // Determine the appropriate build target for the given package and chip:
        let target = target_triple(&args.package, &chip)?;

        // Build the documentation for the specified package, targeting the
        // specified chip:
        xtask::build_documentation(workspace, args.package, chip, target, args.open)?;

        let docs_path = xtask::windows_safe_path(
            &workspace
                .join(package.clone())
                .join("target")
                .join(target)
                .join("doc"),
        );

        let output_path = output_path
            .join(package.clone())
            .join(version.to_string())
            .join(chip.to_string());
        let output_path = xtask::windows_safe_path(&output_path);

        // Create the output directory, and copy the built documentation into it:
        fs::create_dir_all(&output_path)?;
        copy_dir_all(&docs_path, &output_path)?;

        // Build the context object required for rendering this particular build's
        // information on the documentation index:
        crates.push(minijinja::context! {
            name => package,
            version => version,
            chip => chip.to_string(),
            chip_pretty => chip.pretty_name(),
            package => package.replace('-', "_"),
            description => format!("{} (targeting {})", package, chip.pretty_name()),
        });
    }

    // Copy any additional assets to the documentation's output path:
    fs::copy(
        &resources.join("esp-rs.svg"),
        &output_path.join("esp-rs.svg"),
    )?;

    // Render the index and write it out to the documentaiton's output path:
    let source = fs::read_to_string(resources.join("index.html.jinja"))?;

    let mut env = minijinja::Environment::new();
    env.add_template("index", &source)?;

    let tmpl = env.get_template("index")?;
    let html = tmpl.render(minijinja::context! { crates => crates })?;

    fs::write(output_path.join("index.html"), html)?;

    Ok(())
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
    let package_path = xtask::windows_safe_path(&workspace.join(args.package.to_string()));

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

fn build_package(workspace: &Path, args: BuildPackageArgs) -> Result<()> {
    // Absolute path of the package's root:
    let package_path = xtask::windows_safe_path(&workspace.join(args.package.to_string()));

    // Build the package using the provided features and/or target, if any:
    xtask::build_package(&package_path, args.features, args.toolchain, args.target)
}

fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    // Bump the version by the specified amount for each given package:
    for package in args.packages {
        xtask::bump_version(workspace, package, args.amount)?;
    }

    Ok(())
}

fn generate_efuse_src(workspace: &Path, args: GenerateEfuseFieldsArgs) -> Result<()> {
    let idf_path = args.idf_path.canonicalize()?;

    // Build the path for the generated source file, for the specified chip:
    let esp_hal = workspace.join("esp-hal");
    let out_path = esp_hal
        .join("src")
        .join("soc")
        .join(args.chip.to_string())
        .join("efuse")
        .join("fields.rs");

    // Generate the Rust source file from the CSV file, and write it out to
    // the appropriate path:
    xtask::generate_efuse_table(&args.chip, &idf_path, out_path)?;

    // Format the generated code:
    xtask::cargo::run(&["fmt".into()], &esp_hal)?;

    Ok(())
}

fn run_example(workspace: &Path, mut args: RunExampleArgs) -> Result<()> {
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
    let package_path = xtask::windows_safe_path(&workspace.join(args.package.to_string()));

    // Absolute path to the directory containing the examples:
    let example_path = if args.package == Package::Examples {
        package_path.join("src").join("bin")
    } else {
        package_path.join("examples")
    };

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;

    // Load all examples and parse their metadata:
    let example = xtask::load_examples(&example_path)?
        .iter()
        // Filter down the examples to only those for which the specified chip is supported:
        .filter(|example| example.supports_chip(args.chip))
        .find_map(|example| {
            if example.name() == args.example {
                Some(example.clone())
            } else {
                None
            }
        });

    if let Some(example) = example {
        xtask::run_example(&package_path, args.chip, target, &example)?;
    } else {
        log::error!("Example not found or unsupported for the given chip");
    }

    Ok(())
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

// https://stackoverflow.com/a/65192210
fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> Result<()> {
    fs::create_dir_all(&dst)?;

    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;

        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
        }
    }

    Ok(())
}
