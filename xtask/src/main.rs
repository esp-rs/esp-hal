use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{bail, ensure, Context as _, Result};
use clap::{Args, Parser};
use esp_metadata::{Arch, Chip, Config};
use minijinja::Value;
use strum::IntoEnumIterator;
use xtask::{
    cargo::{CargoAction, CargoArgsBuilder},
    firmware::Metadata,
    target_triple,
    Package,
    Version,
};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build documentation for the specified chip.
    BuildDocumentationIndex(BuildDocumentationArgs),
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
    #[clap(alias = "format-packages")]
    FmtPackages(FmtPackagesArgs),
    /// Generate the eFuse fields source file from a CSV.
    GenerateEfuseFields(GenerateEfuseFieldsArgs),
    /// Lint all packages in the workspace with clippy
    LintPackages(LintPackagesArgs),
    /// Attempt to publish the specified package.
    Publish(PublishArgs),
    /// Run doctests for specified chip and package.
    #[clap(alias = "run-doc-test")]
    RunDocTests(ExampleArgs),
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
    /// Build examples in debug mode only
    #[arg(long)]
    debug: bool,
}

#[derive(Debug, Args)]
struct TestArgs {
    /// Chip to target.
    #[arg(value_enum)]
    chip: Chip,
    /// Optional test to act on (all tests used if omitted)
    #[arg(short = 't', long)]
    test: Option<String>,
    /// Repeat the tests for a specific number of times.
    #[arg(long)]
    repeat: Option<usize>,
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

    /// Lint for a specific chip
    #[arg(long, value_enum, default_values_t = Chip::iter())]
    chips: Vec<Chip>,

    /// Automatically apply fixes
    #[arg(long)]
    fix: bool,
}

#[derive(Debug, Args)]
struct PublishArgs {
    /// Package to publish (performs a dry-run by default).
    #[arg(value_enum)]
    package: Package,

    /// Do not pass the `--dry-run` argument, actually try to publish.
    #[arg(long)]
    no_dry_run: bool,
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
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let workspace = std::env::current_dir()?;

    let out_path = Path::new("target");

    match Cli::parse() {
        Cli::BuildDocumentation(args) => build_documentation(&workspace, args),
        Cli::BuildDocumentationIndex(args) => build_documentation_index(&workspace, args),
        Cli::BuildExamples(args) => examples(
            &workspace,
            args,
            CargoAction::Build(out_path.join("examples")),
        ),
        Cli::BuildPackage(args) => build_package(&workspace, args),
        Cli::BuildTests(args) => {
            tests(&workspace, args, CargoAction::Build(out_path.join("tests")))
        }
        Cli::BumpVersion(args) => bump_version(&workspace, args),
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::GenerateEfuseFields(args) => generate_efuse_src(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::Publish(args) => publish(&workspace, args),
        Cli::RunDocTests(args) => run_doc_tests(&workspace, args),
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
        Package::Examples | Package::QaTest => package_path.join("src").join("bin"),
        Package::HilTest => package_path.join("tests"),
        _ => package_path.join("examples"),
    };

    // Load all examples which support the specified chip and parse their metadata:
    let mut examples = xtask::firmware::load(&example_path)?
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
    examples.sort_by_key(|a| a.binary_name());

    // Execute the specified action:
    match action {
        CargoAction::Build(out_path) => build_examples(args, examples, &package_path, out_path),
        CargoAction::Run if args.example.is_some() => run_example(args, examples, &package_path),
        CargoAction::Run => run_examples(args, examples, &package_path),
    }
}

fn build_examples(
    args: ExampleArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
    out_path: PathBuf,
) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(args.package, &args.chip)?;

    if examples
        .iter()
        .find(|ex| ex.matches(&args.example))
        .is_some()
    {
        // Attempt to build only the specified example:
        for example in examples.iter().filter(|ex| ex.matches(&args.example)) {
            xtask::execute_app(
                package_path,
                args.chip,
                target,
                example,
                CargoAction::Build(out_path.clone()),
                1,
                args.debug,
            )?;
        }
        Ok(())
    } else if args.example.is_some() {
        // An invalid argument was provided:
        bail!("Example not found or unsupported for the given chip")
    } else {
        // Attempt to build each supported example, with all required features enabled:
        examples.iter().try_for_each(|example| {
            xtask::execute_app(
                package_path,
                args.chip,
                target,
                example,
                CargoAction::Build(out_path.clone()),
                1,
                args.debug,
            )
        })
    }
}

fn run_example(args: ExampleArgs, examples: Vec<Metadata>, package_path: &Path) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(args.package, &args.chip)?;

    // Filter the examples down to only the binary we're interested in, assuming it
    // actually supports the specified chip:
    let mut found_one = false;
    for example in examples.iter().filter(|ex| ex.matches(&args.example)) {
        found_one = true;
        xtask::execute_app(
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

fn run_examples(args: ExampleArgs, examples: Vec<Metadata>, package_path: &Path) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(args.package, &args.chip)?;

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
                && xtask::execute_app(
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

fn tests(workspace: &Path, args: TestArgs, action: CargoAction) -> Result<()> {
    // Absolute path of the 'hil-test' package's root:
    let package_path = xtask::windows_safe_path(&workspace.join("hil-test"));

    // Determine the appropriate build target for the given package and chip:
    let target = target_triple(Package::HilTest, &args.chip)?;

    // Load all tests which support the specified chip and parse their metadata:
    let mut tests = xtask::firmware::load(&package_path.join("tests"))?
        .into_iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    // Sort all tests by name:
    tests.sort_by_key(|a| a.binary_name());

    // Execute the specified action:
    if tests.iter().find(|test| test.matches(&args.test)).is_some() {
        for test in tests.iter().filter(|test| test.matches(&args.test)) {
            xtask::execute_app(
                &package_path,
                args.chip,
                target,
                test,
                action.clone(),
                args.repeat.unwrap_or(1),
                false,
            )?;
        }
        Ok(())
    } else if args.test.is_some() {
        bail!("Test not found or unsupported for the given chip")
    } else {
        let mut failed = Vec::new();
        for test in tests {
            if xtask::execute_app(
                &package_path,
                args.chip,
                target,
                &test,
                action.clone(),
                args.repeat.unwrap_or(1),
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

fn build_documentation(workspace: &Path, args: BuildDocumentationArgs) -> Result<()> {
    let output_path = workspace.join("docs");

    fs::create_dir_all(&output_path)
        .with_context(|| format!("Failed to create {}", output_path.display()))?;

    let mut packages = HashMap::new();
    for package in args.packages {
        packages.insert(
            package,
            build_documentation_for_package(workspace, package, &args.chips)?,
        );
    }

    generate_index(workspace, &packages)?;

    Ok(())
}

fn build_documentation_index(workspace: &Path, args: BuildDocumentationArgs) -> Result<()> {
    let mut packages = HashMap::new();
    for package in args.packages {
        packages.insert(
            package,
            generate_documentation_meta_for_package(workspace, package, &args.chips)?,
        );
    }

    generate_index(workspace, &packages)?;

    Ok(())
}

fn generate_index(workspace: &Path, packages: &HashMap<Package, Vec<Value>>) -> Result<()> {
    let output_path = workspace.join("docs");
    let resources = workspace.join("resources");

    fs::create_dir_all(&output_path)
        .with_context(|| format!("Failed to create {}", output_path.display()))?;
    // Copy any additional assets to the documentation's output path:
    fs::copy(resources.join("esp-rs.svg"), output_path.join("esp-rs.svg"))
        .context("Failed to copy esp-rs.svg")?;

    // Render the index and write it out to the documentaiton's output path:
    let source = fs::read_to_string(resources.join("index.html.jinja"))
        .context("Failed to read index.html.jinja")?;

    let mut env = minijinja::Environment::new();
    env.add_template("index", &source)?;

    let tmpl = env.get_template("index")?;
    let html = tmpl.render(minijinja::context! { packages => packages })?;

    fs::write(output_path.join("index.html"), html).context("Failed to write index.html")?;

    Ok(())
}

fn build_documentation_for_package(
    workspace: &Path,
    package: Package,
    chips: &[Chip],
) -> Result<Vec<Value>> {
    let output_path = workspace.join("docs");

    let version = xtask::package_version(workspace, package)?;

    for chip in chips {
        // Ensure that the package/chip combination provided are valid:
        validate_package_chip(&package, chip)?;

        // Build the documentation for the specified package, targeting the
        // specified chip:
        let docs_path = xtask::build_documentation(workspace, package, *chip)?;

        ensure!(
            docs_path.exists(),
            "Documentation not found at {}",
            docs_path.display()
        );

        let output_path = output_path
            .join(package.to_string())
            .join(version.to_string())
            .join(chip.to_string());
        let output_path = xtask::windows_safe_path(&output_path);

        // Create the output directory, and copy the built documentation into it:
        fs::create_dir_all(&output_path)
            .with_context(|| format!("Failed to create {}", output_path.display()))?;

        copy_dir_all(&docs_path, &output_path).with_context(|| {
            format!(
                "Failed to copy {} to {}",
                docs_path.display(),
                output_path.display()
            )
        })?;
    }

    Ok(generate_documentation_meta_for_package(
        workspace, package, chips,
    )?)
}

fn generate_documentation_meta_for_package(
    workspace: &Path,
    package: Package,
    chips: &[Chip],
) -> Result<Vec<Value>> {
    let version = xtask::package_version(workspace, package)?;

    let mut metadata = Vec::new();

    for chip in chips {
        // Ensure that the package/chip combination provided are valid:
        validate_package_chip(&package, chip)?;

        // Build the context object required for rendering this particular build's
        // information on the documentation index:
        metadata.push(minijinja::context! {
            name => package,
            version => version,
            chip => chip.to_string(),
            chip_pretty => chip.pretty_name(),
            package => package.to_string().replace('-', "_"),
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

        for chip in &args.chips {
            let device = Config::for_chip(chip);

            match package {
                Package::EspBacktrace => {
                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core",
                            "--no-default-features",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip},defmt"),
                        ],
                        args.fix,
                    )?;
                }

                Package::EspHal => {
                    let mut features = format!("--features={chip},ci,unstable");

                    // Cover all esp-hal features where a device is supported
                    if device.contains("usb0") {
                        features.push_str(",usb-otg")
                    }
                    if device.contains("bt") {
                        features.push_str(",bluetooth")
                    }
                    if device.contains("psram") {
                        // TODO this doesn't test octal psram (since `ESP_HAL_CONFIG_PSRAM_MODE`
                        // defaults to `quad`) as it would require a separate build
                        features.push_str(",psram")
                    }

                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &features,
                        ],
                        args.fix,
                    )?;
                }

                Package::EspHalEmbassy => {
                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip},executors,defmt,esp-hal/unstable"),
                        ],
                        args.fix,
                    )?;
                }

                Package::EspIeee802154 => {
                    if device.contains("ieee802154") {
                        let features = format!("--features={chip},sys-logs,esp-hal/unstable");
                        lint_package(
                            chip,
                            &path,
                            &[
                                "-Zbuild-std=core",
                                &format!("--target={}", chip.target()),
                                &features,
                            ],
                            args.fix,
                        )?;
                    }
                }
                Package::EspLpHal => {
                    if device.contains("lp_core") {
                        lint_package(
                            chip,
                            &path,
                            &[
                                "-Zbuild-std=core",
                                &format!("--target={}", chip.lp_target().unwrap()),
                                &format!("--features={chip},embedded-io"),
                            ],
                            args.fix,
                        )?;
                    }
                }

                Package::EspPrintln => {
                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip},defmt-espflash"),
                        ],
                        args.fix,
                    )?;
                }

                Package::EspRiscvRt => {
                    if matches!(device.arch(), Arch::RiscV) {
                        lint_package(
                            chip,
                            &path,
                            &["-Zbuild-std=core", &format!("--target={}", chip.target())],
                            args.fix,
                        )?;
                    }
                }

                Package::EspStorage => {
                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core",
                            &format!("--target={}", chip.target()),
                            &format!("--features={chip},storage,nor-flash,low-level"),
                        ],
                        args.fix,
                    )?;
                }

                Package::EspWifi => {
                    let mut features = format!("--features={chip},defmt,sys-logs,esp-hal/unstable");

                    if device.contains("wifi") {
                        features.push_str(",esp-now,sniffer")
                    }
                    if device.contains("bt") {
                        features.push_str(",ble")
                    }
                    if device.contains("coex") {
                        features.push_str(",coex")
                    }
                    lint_package(
                        chip,
                        &path,
                        &[
                            "-Zbuild-std=core,alloc",
                            &format!("--target={}", chip.target()),
                            "--no-default-features",
                            &features,
                        ],
                        args.fix,
                    )?;
                }

                Package::XtensaLxRt => {
                    if matches!(device.arch(), Arch::Xtensa) {
                        lint_package(
                            chip,
                            &path,
                            &[
                                "-Zbuild-std=core",
                                &format!("--target={}", chip.target()),
                                &format!("--features={chip}"),
                            ],
                            args.fix,
                        )?
                    }
                }

                // We will *not* check the following packages with `clippy`; this
                // may or may not change in the future:
                Package::Examples | Package::HilTest | Package::QaTest => {}

                // By default, no `clippy` arguments are required:
                _ => lint_package(chip, &path, &[], args.fix)?,
            }
        }
    }

    Ok(())
}

fn lint_package(chip: &Chip, path: &Path, args: &[&str], fix: bool) -> Result<()> {
    log::info!("Linting package: {} ({})", path.display(), chip);

    let builder = CargoArgsBuilder::default().subcommand("clippy");

    let mut builder = if chip.is_xtensa() {
        // We only overwrite Xtensas so that externally set nightly/stable toolchains
        // are not overwritten.
        builder.toolchain("esp")
    } else {
        builder
    };

    for arg in args {
        builder = builder.arg(arg.to_string());
    }

    // build in release to reuse example artifacts
    let cargo_args = builder.arg("--release");
    let cargo_args = if fix {
        cargo_args.arg("--fix").arg("--lib").arg("--allow-dirty")
    } else {
        cargo_args.arg("--").arg("-D").arg("warnings")
    };

    let cargo_args = cargo_args.build();

    xtask::cargo::run(&cargo_args, path)
}

fn publish(workspace: &Path, args: PublishArgs) -> Result<()> {
    let package_name = args.package.to_string();
    let package_path = xtask::windows_safe_path(&workspace.join(&package_name));

    use Package::*;
    let mut publish_args = match args.package {
        Examples | HilTest | QaTest => {
            bail!(
                "Invalid package '{}' specified, this package should not be published!",
                args.package
            )
        }

        EspBacktrace | EspHal | EspHalEmbassy | EspIeee802154 | EspLpHal | EspPrintln
        | EspRiscvRt | EspStorage | EspWifi | XtensaLxRt => vec!["--no-verify"],

        _ => vec![],
    };

    if !args.no_dry_run {
        publish_args.push("--dry-run");
    }

    let builder = CargoArgsBuilder::default()
        .subcommand("publish")
        .args(&publish_args);

    let args = builder.build();
    log::debug!("{args:#?}");

    // Execute `cargo publish` command from the package root:
    xtask::cargo::run(&args, &package_path)?;

    Ok(())
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

fn run_doc_tests(workspace: &Path, args: ExampleArgs) -> Result<()> {
    let chip = args.chip;

    let package_name = args.package.to_string();
    let package_path = xtask::windows_safe_path(&workspace.join(&package_name));

    // Determine the appropriate build target, and cargo features for the given
    // package and chip:
    let target = target_triple(args.package, &chip)?;
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
    xtask::cargo::run(&args, &package_path)?;

    Ok(())
}

// ----------------------------------------------------------------------------
// Helper Functions

fn validate_package_chip(package: &Package, chip: &Chip) -> Result<()> {
    ensure!(
        *package != Package::EspLpHal || chip.has_lp_core(),
        "Invalid chip provided for package '{}': '{}'",
        package,
        chip
    );

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
