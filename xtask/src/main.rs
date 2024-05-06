use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{bail, Result};
use clap::{Args, Parser};
use strum::IntoEnumIterator;
use xtask::{cargo::CargoAction, Chip, Metadata, Package, Version};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build documentation for the specified chip.
    BuildDocumentation(BuildDocumentationArgs),
    /// Build all examples for the specified chip.
    BuildExamples(ExampleArgs),
    /// Build the specified package with the given options.
    BuildPackage(BuildPackageArgs),
    /// Build all applicable tests or the specified test for a specified chip.
    BuildTests(TestArgs),
    /// Bump the version of the specified package(s).
    BumpVersion(BumpVersionArgs),
    /// Generate the eFuse fields source file from a CSV.
    GenerateEfuseFields(GenerateEfuseFieldsArgs),
    /// Run the given example for the specified chip.
    RunExample(ExampleArgs),
    /// Run all applicable tests or the specified test for a specified chip.
    RunTests(TestArgs),
    /// Run all ELFs in a folder.
    RunElfs(RunElfArgs),
}

#[derive(Debug, Args)]
struct ExampleArgs {
    /// Package whose examples we which to act on.
    #[arg(value_enum)]
    package: Package,
    /// Chip to target.
    #[arg(value_enum)]
    chip: Chip,
    /// Optional example to act on (all examples used if omitted)
    example: Option<String>,
}

#[derive(Debug, Args)]
struct TestArgs {
    /// Chip to target.
    #[arg(value_enum)]
    chip: Chip,
    /// Optional test to act on (all tests used if omitted)
    #[arg(short = 't', long)]
    test: Option<String>,
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
    /// Don't enabled the default features.
    #[arg(long)]
    no_default_features: bool,
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
struct GenerateEfuseFieldsArgs {
    /// Path to the local ESP-IDF repository.
    idf_path: PathBuf,
    /// Chip to build eFuse fields table for.
    #[arg(value_enum)]
    chip: Chip,
}

#[derive(Debug, Args)]
struct RunElfArgs {
    /// Which chip to run the tests for.
    #[arg(value_enum)]
    chip: Chip,
    /// Path to the ELFs.
    path: PathBuf,
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
        Cli::BuildExamples(args) => examples(&workspace, args, CargoAction::Build),
        Cli::BuildPackage(args) => build_package(&workspace, args),
        Cli::BuildTests(args) => tests(&workspace, args, CargoAction::Build),
        Cli::BumpVersion(args) => bump_version(&workspace, args),
        Cli::GenerateEfuseFields(args) => generate_efuse_src(&workspace, args),
        Cli::RunExample(args) => examples(&workspace, args, CargoAction::Run),
        Cli::RunTests(args) => tests(&workspace, args, CargoAction::Run),
        Cli::RunElfs(args) => run_elfs(args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn examples(workspace: &Path, mut args: ExampleArgs, action: CargoAction) -> Result<()> {
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

    let example_path = match args.package {
        Package::Examples => package_path.join("src").join("bin"),
        Package::HilTest => package_path.join("tests"),
        _ => package_path.join("examples"),
    };

    // Load all examples which support the specified chip and parse their metadata:
    let mut examples = xtask::load_examples(&example_path)?
        .iter()
        .filter_map(|example| {
            if example.supports_chip(args.chip) {
                Some(example.clone())
            } else {
                None
            }
        })
        .collect::<Vec<_>>();

    // Sort all examples by name:
    examples.sort_by(|a, b| a.name().cmp(&b.name()));

    // Execute the specified action:
    match action {
        CargoAction::Build => build_examples(args, examples, &package_path),
        CargoAction::Run => run_example(args, examples, &package_path),
    }
}

fn build_examples(args: ExampleArgs, examples: Vec<Metadata>, package_path: &Path) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;

    if let Some(example) = examples.iter().find(|ex| Some(ex.name()) == args.example) {
        // Attempt to build only the specified example:
        xtask::execute_app(
            &package_path,
            args.chip,
            target,
            example,
            &CargoAction::Build,
        )
    } else if args.example.is_some() {
        // An invalid argument was provided:
        bail!("Example not found or unsupported for the given chip")
    } else {
        // Attempt to build each supported example, with all required features enabled:
        examples.iter().try_for_each(|example| {
            xtask::execute_app(
                &package_path,
                args.chip,
                target,
                example,
                &CargoAction::Build,
            )
        })
    }
}

fn run_example(args: ExampleArgs, examples: Vec<Metadata>, package_path: &Path) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;

    // Filter the examples down to only the binary we're interested in, assuming it
    // actually supports the specified chip:
    if let Some(example) = examples.iter().find(|ex| Some(ex.name()) == args.example) {
        xtask::execute_app(
            &package_path,
            args.chip,
            target,
            &example,
            &CargoAction::Run,
        )
    } else {
        bail!("Example not found or unsupported for the given chip")
    }
}

fn tests(workspace: &Path, args: TestArgs, action: CargoAction) -> Result<()> {
    // Absolute path of the 'hil-test' package's root:
    let package_path = xtask::windows_safe_path(&workspace.join("hil-test"));

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&Package::HilTest, &args.chip)?;

    // Load all tests which support the specified chip and parse their metadata:
    let mut tests = xtask::load_examples(&package_path.join("tests"))?
        .into_iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    // Sort all tests by name:
    tests.sort_by(|a, b| a.name().cmp(&b.name()));

    // Execute the specified action:
    if let Some(test) = tests.iter().find(|test| Some(test.name()) == args.test) {
        xtask::execute_app(&package_path, args.chip, target, &test, &action)
    } else if args.test.is_some() {
        bail!("Test not found or unsupported for the given chip")
    } else {
        let mut failed = Vec::new();
        for test in tests {
            if xtask::execute_app(&package_path, args.chip, target, &test, &action).is_err() {
                failed.push(test.name());
            }
        }

        if !failed.is_empty() {
            bail!("Failed tests: {:?}", failed);
        }

        Ok(())
    }
}

fn build_documentation(workspace: &Path, args: BuildDocumentationArgs) -> Result<()> {
    let output_path = workspace.join("docs");
    let resources = workspace.join("resources");

    let package = args.package.to_string();
    let version = xtask::package_version(workspace, args.package)?;

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
    fs::copy(resources.join("esp-rs.svg"), output_path.join("esp-rs.svg"))?;

    // Render the index and write it out to the documentaiton's output path:
    let source = fs::read_to_string(resources.join("index.html.jinja"))?;

    let mut env = minijinja::Environment::new();
    env.add_template("index", &source)?;

    let tmpl = env.get_template("index")?;
    let html = tmpl.render(minijinja::context! { crates => crates })?;

    fs::write(output_path.join("index.html"), html)?;

    Ok(())
}

fn build_package(workspace: &Path, args: BuildPackageArgs) -> Result<()> {
    // Absolute path of the package's root:
    let package_path = xtask::windows_safe_path(&workspace.join(args.package.to_string()));

    // Build the package using the provided features and/or target, if any:
    xtask::build_package(
        &package_path,
        args.features,
        args.no_default_features,
        args.toolchain,
        args.target,
    )
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
    xtask::generate_efuse_table(&args.chip, idf_path, out_path)?;

    // Format the generated code:
    xtask::cargo::run(&["fmt".into()], &esp_hal)?;

    Ok(())
}

fn run_elfs(args: RunElfArgs) -> Result<(), anyhow::Error> {
    let elfs = fs::read_dir(&args.path)?;
    let mut failed_elfs: Vec<String> = Vec::new();
    for elf in elfs {
        let elf = elf?;
        let elf_path = elf.path();
        let elf_name = elf_path.file_name().unwrap().to_str().unwrap();
        let elf_name = elf_name.split('.').next().unwrap();
        let elf_name = elf_name.to_string();
        println!("Running '{}' test", elf_name);

        let command = Command::new("probe-rs")
            .arg("run")
            .arg("--chip")
            .arg(args.chip.to_string())
            .arg(elf_path)
            .output()
            .expect("Failed to execute probe-rs run command");
        let stdout = String::from_utf8_lossy(&command.stdout);
        let stderr = String::from_utf8_lossy(&command.stderr);
        println!("{}\n{}", stderr, stdout);
        if !command.status.success() {
            failed_elfs.push(elf_name);
        }
    }

    if !failed_elfs.is_empty() {
        bail!("Failed tests: {:?}", failed_elfs);
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
