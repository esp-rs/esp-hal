use std::{
    collections::{HashMap, HashSet},
    path::Path,
};

use anyhow::{Context, Result, bail};
use clap::Args;
use esp_metadata::Chip;
use inquire::Select;
use strum::IntoEnumIterator;

pub use self::{build::*, check_changelog::*, check_pr_changelog::*, release::*, run::*};
use crate::{
    Package,
    cargo::{CargoAction, CargoCommandBatcher},
    firmware,
    radio_hil_runner::run_radio_test,
};
mod build;
mod check_changelog;
mod check_pr_changelog;
#[cfg(feature = "report")]
pub mod generate_report;
#[cfg(feature = "semver-checks")]
pub(crate) mod generate_rom_symbols;
#[cfg(feature = "mcp")]
pub mod mcp;
#[cfg(feature = "rel-check")]
pub mod relcheck;
mod release;
mod run;

// ----------------------------------------------------------------------------
// Top level command and subcommand definitions

/// Arguments for the `ci` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Perform (parts of) the checks done in CI for a given chip",
        command = "ci"
    )
)]
#[derive(Debug, Args)]
pub struct CiArgs {
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,

    /// The toolchain used to run the lints
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Steps to run in the CI pipeline.
    #[arg(long, value_delimiter = ',')]
    pub steps: Vec<String>,

    /// Whether to skip running lints
    #[arg(long)]
    pub no_lint: bool,

    /// Whether to skip building documentation
    #[arg(long)]
    pub no_docs: bool,

    /// Whether to skip checking the crates itself
    #[arg(long)]
    pub no_check_crates: bool,
}

/// Arguments for the `fmt-packages` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Format all packages in the workspace with rustfmt",
        command = "fmt-packages"
    )
)]
#[derive(Debug, Args)]
pub struct FmtPackagesArgs {
    /// Run in 'check' mode; exits with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    pub check: bool,

    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Arguments for the `clean` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run cargo clean for the specified packages",
        command = "clean"
    )
)]
#[derive(Debug, Args)]
pub struct CleanArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Arguments for the `host-tests` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run host-tests in the workspace with cargo test",
        command = "host-tests"
    )
)]
#[derive(Debug, Args)]
pub struct HostTestsArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,
}

/// Arguments for the `check-packages` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Check all packages in the workspace with cargo check",
        command = "check-packages"
    )
)]
#[derive(Debug, Args)]
pub struct CheckPackagesArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,

    /// Check for a specific chip
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    pub chips: Vec<Chip>,

    /// The toolchain used to run the checks
    #[arg(long)]
    pub toolchain: Option<String>,
}

/// Arguments for the `lint-packages` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Lint all packages in the workspace with clippy",
        command = "lint-packages"
    )
)]
#[derive(Debug, Args)]
pub struct LintPackagesArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    pub packages: Vec<Package>,

    /// Lint for a specific chip
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    pub chips: Vec<Chip>,

    /// Automatically apply fixes
    #[arg(long)]
    pub fix: bool,

    /// The toolchain used to run the lints
    #[arg(long)]
    pub toolchain: Option<String>,
}

/// Arguments for the `update-metadata` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Re-generate metadata and the chip support table in the esp-hal README",
        command = "update-metadata"
    )
)]
#[derive(Debug, Args)]
pub struct UpdateMetadataArgs {
    /// Run in 'check' mode; exits with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    pub check: bool,
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

/// Arguments common to commands which act on examples.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run examples for the specified chip and package",
        command = "run example"
    )
)]
#[derive(Debug, Args)]
pub struct ExamplesArgs {
    /// Example to act on ("all" will execute every example).
    pub example: Option<String>,
    /// Chip to target.
    #[arg(value_enum, long)]
    pub chip: Option<Chip>,
    /// Package whose examples we wish to act on.
    #[arg(value_enum, long, default_value_t = ExamplesPackage::Examples)]
    pub package: ExamplesPackage,
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

/// The different packages which contain examples, and which the `examples` subcommand can act on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum ExamplesPackage {
    Examples,
    QaTest,
    EspLpHal,
    CompileTests,
}

impl From<ExamplesPackage> for Package {
    fn from(ep: ExamplesPackage) -> Self {
        match ep {
            ExamplesPackage::Examples => Package::Examples,
            ExamplesPackage::QaTest => Package::QaTest,
            ExamplesPackage::EspLpHal => Package::EspLpHal,
            ExamplesPackage::CompileTests => Package::CompileTests,
        }
    }
}

impl ExamplesPackage {
    /// Get the underlying Package
    pub fn as_package(self) -> Package {
        Package::from(self)
    }

    fn single_project_examples(&self) -> bool {
        match self {
            ExamplesPackage::Examples | ExamplesPackage::CompileTests => true,
            _ => false,
        }
    }
}

/// Arguments common to commands which act on doctests.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run doc tests for the specified chip and packages",
        command = "run doc-tests"
    )
)]
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
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Build or run HIL tests for the specified chip",
        command = "run tests"
    )
)]
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

    /// "hil-test" or "hil-test-radio"
    pub package: Option<String>,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

/// Execute the given action on the specified examples.
pub fn examples(workspace: &Path, mut args: ExamplesArgs, action: CargoAction) -> Result<()> {
    log::debug!(
        "Running examples for '{}' on '{:?}'",
        args.package.as_package(),
        args.chip
    );
    if args.chip.is_none() {
        let chip_variants = Chip::iter().collect::<Vec<_>>();

        let chip = Select::new("Select your target chip:", chip_variants).prompt()?;

        args.chip = Some(chip);
    }

    let chip = args.chip.unwrap();

    // Ensure that the package/chip combination provided are valid:
    args.package
        .as_package()
        .validate_package_chip(&chip)
        .with_context(|| {
            format!(
                "The package '{0}' does not support the chip '{chip:?}'",
                args.package.as_package()
            )
        })?;

    // Absolute path of the package's root:
    let package_path =
        crate::windows_safe_path(&workspace.join(args.package.as_package().to_string()));

    // Load all examples which support the specified chip and parse their metadata.
    //
    // Directories might contain a number of individual projects, and don't not rely on
    // metadata comments in the source files. As such, it needs to load its metadata differently
    // than other packages.
    let examples = if args.package.single_project_examples() {
        crate::firmware::load_cargo_toml(&package_path).with_context(|| {
            format!(
                "Failed to load specified examples from {}",
                package_path.display()
            )
        })?
    } else {
        let example_path = match args.package.as_package() {
            Package::QaTest => package_path.join("src").join("bin"),
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
    let package_name = args.package.as_deref().unwrap_or("hil-test");

    let package_path = match package_name {
        "hil-test" => crate::windows_safe_path(&workspace.join("hil-test")),
        "hil-test-radio" => crate::windows_safe_path(&workspace.join("hil-test-radio")),
        other => bail!(
            "Unknown package: {}. Use 'hil-test' or 'hil-test-radio'",
            other
        ),
    };

    // Determine the appropriate build target for the given package and chip:
    let target = Package::HilTest.target_triple(&args.chip)?;

    // Load all test metadata first so we can differentiate between:
    // - unknown test selectors (typos)
    // - valid tests that are unsupported for the selected chip.
    let mut all_tests = crate::firmware::load(&package_path.join("src").join("bin"))?;
    all_tests.sort_by_key(|a| a.binary_name());
    let tests_for_chip = all_tests
        .iter()
        .filter(|example| example.supports_chip(args.chip))
        .collect::<Vec<_>>();

    let is_radio_package = package_name == "hil-test-radio";
    // Radio runs are orchestrated by `run_radio_test`, so cargo is only used to build binaries.
    let command_action = if is_radio_package && matches!(action, CargoAction::Run) {
        CargoAction::Build(None)
    } else {
        action.clone()
    };

    if let CargoAction::Build(Some(out_dir)) = &action {
        // Make sure the tmp directory has no garbage for us.
        let tmp_dir = out_dir.join("tmp");
        _ = std::fs::remove_dir_all(&tmp_dir);
        std::fs::create_dir_all(&tmp_dir).unwrap();
    }

    let mut commands = CargoCommandBatcher::new();
    let mut artifact_meta = HashMap::<String, firmware::Metadata>::new();
    // Execute the specified action:
    // If user sets some specific test(s)
    if let Some(test_arg) = args.test.as_deref() {
        let trimmed: Vec<&str> = test_arg.iter().map(|s| s.trim()).collect();

        // Reject `--test ""` / `--test " "`
        if trimmed.iter().all(|s| s.is_empty()) {
            bail!("Empty test selector is not allowed.");
        }

        let mut unknown_selected: Vec<String> = Vec::new();
        let mut unsupported_for_chip: Vec<String> = Vec::new();

        for selected in trimmed.into_iter().filter(|s| !s.is_empty()) {
            let (test_arg, filter) = match selected.split_once("::") {
                Some((test, filter)) => (Some(test), Some(filter)),
                None => (Some(selected), None),
            };

            let run_test_extra_args = if is_radio_package && action == CargoAction::Run {
                &[]
            } else {
                (action == CargoAction::Run)
                    .then(|| filter.as_ref().map(|f| std::slice::from_ref(f)))
                    .flatten()
                    .unwrap_or(&[])
            };

            let matched: Vec<_> = all_tests
                .iter()
                .filter(|t| t.matches(test_arg.as_deref()))
                .collect();

            if matched.is_empty() {
                if is_radio_package
                    && let Some(radio_meta) =
                        resolve_radio_module_alias(&package_path, &all_tests, args.chip, selected)
                {
                    let command = crate::generate_build_command(
                        &package_path,
                        args.chip,
                        &target,
                        &radio_meta,
                        command_action.clone(),
                        false,
                        args.toolchain.as_deref(),
                        args.timings,
                        &[],
                    )?;
                    artifact_meta.insert(command.artifact_name.clone(), radio_meta);
                    commands.push(command);
                    continue;
                }
                unknown_selected.push(selected.to_string());
            } else {
                let matched_for_chip: Vec<_> = matched
                    .into_iter()
                    .filter(|t| t.supports_chip(args.chip))
                    .collect();

                if matched_for_chip.is_empty() {
                    unsupported_for_chip.push(selected.to_string());
                    continue;
                }

                for test in matched_for_chip {
                    let command = crate::generate_build_command(
                        &package_path,
                        args.chip,
                        &target,
                        test,
                        command_action.clone(),
                        false,
                        args.toolchain.as_deref(),
                        args.timings,
                        run_test_extra_args,
                    )?;
                    artifact_meta.insert(command.artifact_name.clone(), test.clone());
                    commands.push(command);
                }
            }
        }

        if !unsupported_for_chip.is_empty() {
            log::warn!(
                "Skipping unsupported tests for '{}': {}",
                args.chip,
                unsupported_for_chip.join(", ")
            );
        }

        if !unknown_selected.is_empty() {
            bail!("Test not found: {}", unknown_selected.join(", "))
        }
    } else {
        for test in tests_for_chip {
            let command = crate::generate_build_command(
                &package_path,
                args.chip,
                &target,
                test,
                command_action.clone(),
                false,
                args.toolchain.as_deref(),
                args.timings,
                &[],
            )?;
            artifact_meta.insert(command.artifact_name.clone(), test.clone());
            commands.push(command);
        }
    }

    if is_radio_package {
        let mut selected_artifacts: HashSet<String> = artifact_meta.keys().cloned().collect();
        let mut required_harness = Vec::<firmware::Metadata>::new();

        for test in artifact_meta.values() {
            if test.is_support_firmware() {
                continue;
            }
            let Some(harness_name) = test.harness_firmware() else {
                continue;
            };

            let Some(harness_meta) = all_tests.iter().find(|candidate| {
                candidate.supports_chip(args.chip)
                    && (candidate.binary_name() == harness_name
                        || candidate.output_file_name() == harness_name)
            }) else {
                bail!(
                    "Missing harness firmware '{harness_name}' required by '{}'",
                    test.output_file_name()
                );
            };

            required_harness.push(harness_meta.clone());
        }

        for harness in required_harness {
            let artifact_name = harness.output_file_name();
            if selected_artifacts.contains(&artifact_name) {
                continue;
            }

            let command = crate::generate_build_command(
                &package_path,
                args.chip,
                &target,
                &harness,
                command_action.clone(),
                false,
                args.toolchain.as_deref(),
                args.timings,
                &[],
            )?;
            selected_artifacts.insert(command.artifact_name.clone());
            artifact_meta.insert(command.artifact_name.clone(), harness);
            commands.push(command);
        }
    }

    let mut failed = Vec::new();
    // Radio run orchestration depends on per-artifact metadata, so avoid
    // cargo-batch collapsing commands into a single synthetic "batch" artifact.
    let no_batch = is_radio_package && matches!(action, CargoAction::Run);
    let built_commands = commands.build(no_batch);

    if is_radio_package && matches!(action, CargoAction::Run) {
        // Build all selected radio artifacts first to ensure support firmware is
        // available before starting DUT runs.
        let mut successfully_built = HashSet::<String>::new();
        for c in &built_commands {
            let mut build_cmd = c.clone();
            if let Some(pos) = build_cmd.command.iter().position(|x| x == "run") {
                build_cmd.command[pos] = "build".to_string();
            }
            if !build_cmd.command.iter().any(|x| x == "--release") {
                build_cmd.command.push("--release".to_string());
            }

            log::info!(
                "Building radio artifact with command: {}",
                build_cmd.command.join(" ")
            );

            if build_cmd.run(false).is_err() {
                failed.push(c.artifact_name.clone());
            } else {
                successfully_built.insert(c.artifact_name.clone());
            }
        }

        for c in &built_commands {
            if !successfully_built.contains(&c.artifact_name) {
                log::warn!(
                    "Skipping '{}' run phase because build failed",
                    c.artifact_name
                );
                continue;
            }

            let Some(meta) = artifact_meta.get(&c.artifact_name) else {
                continue;
            };

            if meta.is_support_firmware() {
                log::info!("Skipping support firmware '{}' as DUT", c.artifact_name);
                continue;
            }

            let dut_binary_path = resolve_radio_binary_path(workspace, &target, &c.artifact_name);
            if !dut_binary_path.exists() {
                failed.push(c.artifact_name.clone());
                log::error!(
                    "Radio DUT binary not found for '{}': {}",
                    c.artifact_name,
                    dut_binary_path.display()
                );
                continue;
            }

            let harness_binary_path = meta.harness_firmware().map(|name| {
                let harness_artifact = artifact_meta
                    .iter()
                    .find_map(|(artifact, candidate)| {
                        if candidate.supports_chip(args.chip)
                            && (candidate.output_file_name() == name
                                || candidate.binary_name() == name)
                        {
                            Some(artifact.as_str())
                        } else {
                            None
                        }
                    })
                    .unwrap_or(name);
                resolve_radio_binary_path(workspace, &target, harness_artifact)
            });

            if let Some(path) = &harness_binary_path
                && !path.exists()
            {
                failed.push(c.artifact_name.clone());
                log::error!(
                    "Harness firmware '{}' required by '{}' is missing at {}",
                    meta.harness_firmware().unwrap_or_default(),
                    c.artifact_name,
                    path.display()
                );
                continue;
            }

            for i in 0..args.repeat {
                if args.repeat != 1 {
                    log::info!("Run {}/{}", i + 1, args.repeat);
                }

                if let Err(e) =
                    run_radio_test(&dut_binary_path, harness_binary_path.as_deref(), 120, None)
                {
                    failed.push(c.artifact_name.clone());
                    log::error!("Radio test '{}' failed: {}", c.artifact_name, e);
                    break;
                }
            }
        }
    } else {
        for c in built_commands {
            let repeat = if matches!(action, CargoAction::Run) {
                args.repeat
            } else {
                1
            };

            println!(
                "Command: cargo {}",
                c.command.join(" ").replace("---", "\n    ---")
            );

            // For HAL tests or build action, use standard probe-rs runner.
            for i in 0..repeat {
                if repeat != 1 {
                    log::info!("Run {}/{}", i + 1, repeat);
                }
                if c.run(false).is_err() {
                    failed.push(c.artifact_name.clone());
                }
            }
        }
    }

    move_artifacts(args.chip, &action);

    if !failed.is_empty() {
        bail!("Failed tests: {:#?}", failed);
    }

    Ok(())
}

fn resolve_radio_binary_path(
    workspace: &Path,
    target: &str,
    artifact_name: &str,
) -> std::path::PathBuf {
    let release_dir = workspace.join("target").join(target).join("release");
    let exact = release_dir.join(artifact_name);
    if exact.exists() {
        return exact;
    }

    let base_name = artifact_name
        .split('_')
        .take_while(|part| *part != "has" && *part != "no")
        .collect::<Vec<_>>()
        .join("_");
    release_dir.join(base_name)
}

fn resolve_radio_module_alias(
    package_path: &Path,
    all_tests: &[firmware::Metadata],
    chip: Chip,
    selected: &str,
) -> Option<firmware::Metadata> {
    let bins_root = package_path.join("src").join("bin");
    let entries = std::fs::read_dir(bins_root).ok()?;

    for entry in entries.flatten() {
        let subdir = entry.path();
        if !subdir.is_dir() {
            continue;
        }

        if !subdir.join(format!("{selected}.rs")).exists() {
            continue;
        }

        let binary_name = entry.file_name().to_string_lossy().to_string();
        if let Some(meta) = all_tests
            .iter()
            .find(|test| test.supports_chip(chip) && test.binary_name() == binary_name)
        {
            return Some(meta.clone());
        }
    }

    None
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
