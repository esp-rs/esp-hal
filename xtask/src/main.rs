use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{bail, Result};
use clap::{Args, Parser};
use esp_metadata::{Chip, Config};
use minijinja::Value;
use strum::IntoEnumIterator;
use xtask::{
    cargo::{CargoAction, CargoArgsBuilder},
    Metadata,
    Package,
    Version,
};

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
    /// Format all packages in the workspace with rustfmt
    FmtPackages(FmtPackagesArgs),
    /// Generate the eFuse fields source file from a CSV.
    GenerateEfuseFields(GenerateEfuseFieldsArgs),
    /// Lint all packages in the workspace with clippy
    LintPackages(LintPackagesArgs),
    /// Run doctests for specified chip and package.
    RunDocTest(ExampleArgs),
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
    /// Package to build documentation for.
    #[arg(long, value_enum, value_delimiter(','))]
    packages: Vec<Package>,
    /// Which chip to build the documentation for.
    #[arg(long, value_enum, value_delimiter(','), default_values_t = Chip::iter())]
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
struct FmtPackagesArgs {
    /// Run in 'check' mode; exists with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    check: bool,
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
struct LintPackagesArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
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
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::GenerateEfuseFields(args) => generate_efuse_src(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::RunDocTest(args) => run_doctests(&workspace, args),
        Cli::RunElfs(args) => run_elfs(args),
        Cli::RunExample(args) => examples(&workspace, args, CargoAction::Run),
        Cli::RunTests(args) => tests(&workspace, args, CargoAction::Run),
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

    let mut packages = HashMap::new();
    for package in args.packages {
        packages.insert(
            package,
            build_documentation_for_package(workspace, package, &args.chips)?,
        );
    }

    // Copy any additional assets to the documentation's output path:
    fs::copy(resources.join("esp-rs.svg"), output_path.join("esp-rs.svg"))?;

    // Render the index and write it out to the documentaiton's output path:
    let source = fs::read_to_string(resources.join("index.html.jinja"))?;

    let mut env = minijinja::Environment::new();
    env.add_template("index", &source)?;

    let tmpl = env.get_template("index")?;
    let html = tmpl.render(minijinja::context! { packages => packages })?;

    fs::write(output_path.join("index.html"), html)?;

    Ok(())
}

fn build_documentation_for_package(
    workspace: &Path,
    package: Package,
    chips: &[Chip],
) -> Result<Vec<Value>> {
    let output_path = workspace.join("docs");

    let version = xtask::package_version(workspace, package)?;

    let mut metadata = Vec::new();

    for chip in chips {
        // Ensure that the package/chip combination provided are valid:
        validate_package_chip(&package, chip)?;

        // Determine the appropriate build target for the given package and chip:
        let target = target_triple(&package, &chip)?;

        // Build the documentation for the specified package, targeting the
        // specified chip:
        xtask::build_documentation(workspace, package, *chip, target)?;

        let docs_path = xtask::windows_safe_path(
            &workspace
                .join(package.to_string())
                .join("target")
                .join(target)
                .join("doc"),
        );

        let output_path = output_path
            .join(package.to_string())
            .join(version.to_string())
            .join(chip.to_string());
        let output_path = xtask::windows_safe_path(&output_path);

        // Create the output directory, and copy the built documentation into it:
        fs::create_dir_all(&output_path)?;
        copy_dir_all(&docs_path, &output_path)?;

        // Build the context object required for rendering this particular build's
        // information on the documentation index:
        metadata.push(minijinja::context! {
            name => package,
            version => version,
            chip => chip.to_string(),
            chip_pretty => chip.pretty_name(),
            package => package.to_string().replace('-', "_"),
            description => format!("{} (targeting {})", package, chip.pretty_name()),
        });
    }

    Ok(metadata)
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

fn fmt_packages(workspace: &Path, args: FmtPackagesArgs) -> Result<()> {
    for path in xtask::package_paths(workspace)? {
        log::info!("Formatting package: {}", path.display());

        let mut cargo_args = CargoArgsBuilder::default()
            .toolchain("nightly")
            .subcommand("fmt")
            .arg("--all")
            .build();

        if args.check {
            cargo_args.push("--".into());
            cargo_args.push("--check".into());
        }

        xtask::cargo::run(&cargo_args, &path)?;
    }

    Ok(())
}

fn lint_packages(workspace: &Path, args: LintPackagesArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        let path = workspace.join(package.to_string());

        // Unfortunately each package has its own unique requirements for
        // building, so we need to handle each individually (though there
        // is *some* overlap)

        match package {
            Package::EspBacktrace => lint_package(
                &path,
                &[
                    "-Zbuild-std=core",
                    "--no-default-features",
                    "--target=riscv32imc-unknown-none-elf",
                    "--features=esp32c6,defmt",
                ],
            )?,

            Package::EspHal => {
                // Since different files/modules can be included/excluded
                // depending on the target, we must lint *all* targets:
                for chip in Chip::iter() {
                    let device = Config::for_chip(&chip);
                    let mut features = format!("--features={chip},ci");

                    // Cover all esp-hal features where a device is supported
                    if device.contains(&"usb0".to_owned()) {
                        features.push_str(",usb-otg")
                    }
                    if device.contains(&"psram".to_owned()) {
                        // TODO this doesn't test octal psram as it would require a separate build
                        features.push_str(",psram-4m,psram-80mhz")
                    }
                    if matches!(chip, Chip::Esp32c6 | Chip::Esp32h2) {
                        features.push_str(",flip-link")
                    }

                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &features,
                        ],
                    )?;
                }
            }

            Package::EspHalEmbassy => {
                // Since different files/modules can be included/excluded
                // depending on the target, we must lint *all* targets:
                for chip in Chip::iter() {
                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip},executors,defmt,integrated-timers"),
                        ],
                    )?;
                }
            }

            Package::EspHalProcmacros | Package::EspRiscvRt => lint_package(
                &path,
                &["-Zbuild-std=core", "--target=riscv32imc-unknown-none-elf"],
            )?,

            Package::EspIeee802154 => {
                for chip in Chip::iter()
                    .filter(|chip| Config::for_chip(chip).contains(&"ieee802154".to_owned()))
                {
                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip}"),
                        ],
                    )?;
                }
            }
            Package::EspLpHal => {
                for chip in Chip::iter()
                    .filter(|chip| Config::for_chip(chip).contains(&"lp_core".to_owned()))
                {
                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.lp_target().unwrap()),
                            &format!("--features={chip},embedded-io,embedded-hal-02"),
                        ],
                    )?;
                }
            }
            Package::EspHalSmartled => lint_package(
                &path,
                &[
                    "-Zbuild-std=core",
                    "--target=riscv32imac-unknown-none-elf",
                    "--features=esp32c6",
                ],
            )?,

            Package::EspPrintln => lint_package(
                &path,
                &[
                    "-Zbuild-std=core",
                    "--target=riscv32imc-unknown-none-elf",
                    "--features=esp32c6",
                ],
            )?,

            Package::EspStorage => lint_package(
                &path,
                &[
                    "-Zbuild-std=core",
                    "--target=riscv32imc-unknown-none-elf",
                    "--features=esp32c6",
                ],
            )?,

            Package::EspWifi => {
                // Since different files/modules can be included/excluded
                // depending on the target, we must lint *all* targets:
                for chip in Chip::iter() {
                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!(
                                "--features={chip},wifi-default,ble,esp-now,async,embassy-net,embedded-svc,coex,ps-min-modem,dump-packets"
                            ),
                        ],
                    )?;
                }
            }

            Package::XtensaLxRt => {
                for chip in [Chip::Esp32, Chip::Esp32s2, Chip::Esp32s3] {
                    lint_package(
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target=xtensa-{chip}-none-elf"),
                            &format!("--features={chip}"),
                        ],
                    )?
                }
            }

            // We will *not* check the following packages with `clippy`; this
            // may or may not change in the future:
            Package::Examples | Package::HilTest => {}

            // By default, no `clippy` arguments are required:
            _ => lint_package(&path, &[])?,
        }
    }

    Ok(())
}

fn lint_package(path: &Path, args: &[&str]) -> Result<()> {
    log::info!("Linting package: {}", path.display());

    let mut builder = CargoArgsBuilder::default()
        .toolchain("esp")
        .subcommand("clippy"); // TODO: Is this still actually required?

    for arg in args {
        builder = builder.arg(arg.to_string());
    }

    let cargo_args = builder.arg("--").arg("-D").arg("warnings").build();

    xtask::cargo::run(&cargo_args, &path)
}

fn run_elfs(args: RunElfArgs) -> Result<()> {
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

        let command = if args.chip == Chip::Esp32 {
            Command::new("probe-rs")
                .arg("run")
                .arg("--chip")
                .arg("esp32-3.3v")
                .arg(elf_path)
                .output()?
        } else if args.chip == Chip::Esp32c2 {
            Command::new("probe-rs")
                .arg("run")
                .arg("--chip")
                .arg(args.chip.to_string())
                .arg("--speed")
                .arg("15000")
                .arg(elf_path)
                .output()?
        } else {
            Command::new("probe-rs")
                .arg("run")
                .arg("--chip")
                .arg(args.chip.to_string())
                .arg(elf_path)
                .output()?
        };

        println!(
            "{}\n{}",
            String::from_utf8_lossy(&command.stderr),
            String::from_utf8_lossy(&command.stdout)
        );

        if !command.status.success() {
            failed.push(elf_name);
        }
    }

    if !failed.is_empty() {
        bail!("Failed tests: {:?}", failed);
    }

    Ok(())
}

fn run_doctests(workspace: &Path, args: ExampleArgs) -> Result<()> {
    let package_name = args.package.to_string();
    let package_path = xtask::windows_safe_path(&workspace.join(&package_name));

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(&args.package, &args.chip)?;
    let features = vec![args.chip.to_string()];

    // Build up an array of command-line arguments to pass to `cargo`:
    let builder = CargoArgsBuilder::default()
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
    xtask::cargo::run(&args, &package_path)?;

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
