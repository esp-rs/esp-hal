use std::{path::Path, time::Instant};

use anyhow::{Context, Result, bail};
use clap::{Args, Parser};
use esp_metadata::{Chip, Config};
use strum::IntoEnumIterator;
use xtask::{
    Package,
    cargo::{CargoAction, CargoArgsBuilder, CargoCommandBatcher},
    commands::*,
    update_metadata,
};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build-related subcommands
    #[clap(subcommand)]
    Build(Build),
    /// Run-related subcommands
    #[clap(subcommand)]
    Run(Run),
    /// Release-related subcommands
    #[clap(subcommand)]
    Release(Release),

    /// Perform (parts of) the checks done in CI
    Ci(CiArgs),
    /// Format all packages in the workspace with rustfmt
    #[clap(alias = "format-packages")]
    FmtPackages(FmtPackagesArgs),
    /// Run cargo clean
    Clean(CleanArgs),
    /// Check all packages in the workspace with cargo check
    CheckPackages(CheckPackagesArgs),
    /// Lint all packages in the workspace with clippy
    LintPackages(LintPackagesArgs),
    /// Semver Checks
    SemverCheck(SemverCheckArgs),
    /// Check the changelog for packages.
    CheckChangelog(CheckChangelogArgs),
    /// Re-generate metadata and the chip support table in the esp-hal README.
    UpdateMetadata(UpdateMetadataArgs),
    /// Run host-tests in the workspace with `cargo test`
    HostTests(HostTestsArgs),
}

#[derive(Debug, Args)]
struct CiArgs {
    /// Chip to target.
    #[arg(value_enum)]
    chip: Chip,

    /// The toolchain used to run the lints
    #[arg(long)]
    toolchain: Option<String>,

    /// Whether to skip running lints
    #[arg(long)]
    no_lint: bool,

    /// Whether to skip building documentation
    #[arg(long)]
    no_docs: bool,
}

#[derive(Debug, Args)]
struct FmtPackagesArgs {
    /// Run in 'check' mode; exists with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    check: bool,

    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

#[derive(Debug, Args)]
struct CleanArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

#[derive(Debug, Args)]
struct HostTestsArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

#[derive(Debug, Args)]
struct CheckPackagesArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Check for a specific chip
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    chips: Vec<Chip>,

    /// The toolchain used to run the checks
    #[arg(long)]
    toolchain: Option<String>,
}

#[derive(Debug, Args)]
struct LintPackagesArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Lint for a specific chip
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    chips: Vec<Chip>,

    /// Automatically apply fixes
    #[arg(long)]
    fix: bool,

    /// The toolchain used to run the lints
    #[arg(long)]
    toolchain: Option<String>,
}

#[derive(Debug, Args)]
struct CheckChangelogArgs {
    /// Package(s) to tag.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Re-generate the changelog with consistent formatting.
    #[arg(long)]
    normalize: bool,
}

#[derive(Debug, Args)]
struct UpdateMetadataArgs {
    /// Run in 'check' mode; exists with 0 if formatted correctly, 1 otherwise
    #[arg(long)]
    check: bool,
}

// ----------------------------------------------------------------------------
// Application

fn main() -> Result<()> {
    let mut builder =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"));
    builder.target(env_logger::Target::Stdout);
    builder.init();

    let workspace =
        std::env::current_dir().with_context(|| format!("Failed to get the current dir!"))?;
    let target_path = workspace.join("target");

    if std::env::var("CARGO_TARGET_DIR").is_err() {
        unsafe { std::env::set_var("CARGO_TARGET_DIR", target_path.to_str().unwrap()) };
    }

    match Cli::parse() {
        // Build-related subcommands:
        Cli::Build(build) => match build {
            Build::Documentation(args) => build_documentation(&workspace, args),
            #[cfg(feature = "deploy-docs")]
            Build::DocumentationIndex => build_documentation_index(&workspace),
            Build::Examples(args) => examples(&workspace, args, CargoAction::Build(None)),
            Build::Package(args) => build_package(&workspace, args),
            Build::Tests(args) => tests(
                &workspace,
                args,
                CargoAction::Build(Some(target_path.join("tests"))),
            ),
        },

        // Run-related subcommands:
        Cli::Run(run) => match run {
            Run::DocTests(args) => run_doc_tests(&workspace, args),
            Run::Elfs(args) => run_elfs(args),
            Run::Example(args) => examples(&workspace, args, CargoAction::Run),
            Run::Tests(args) => tests(&workspace, args, CargoAction::Run),
        },

        // Release-related subcommands:
        Cli::Release(release) => match release {
            Release::BumpVersion(args) => bump_version(&workspace, args),
            Release::TagReleases(args) => tag_releases(&workspace, args),
            Release::Publish(args) => publish(&workspace, args),
            #[cfg(feature = "release")]
            Release::Plan(args) => plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::ExecutePlan(args) => execute_plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::PublishPlan(args) => publish_plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::PostRelease => post_release(&workspace),
            #[cfg(feature = "release")]
            Release::BumpMsrv(args) => bump_msrv::bump_msrv(&workspace, args),
        },

        Cli::Ci(args) => run_ci_checks(&workspace, args),
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::Clean(args) => clean(&workspace, args),
        Cli::CheckPackages(args) => check_packages(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::SemverCheck(args) => semver_checks(&workspace, args),
        Cli::CheckChangelog(args) => check_changelog(&workspace, &args.packages, args.normalize),
        Cli::UpdateMetadata(args) => update_metadata(&workspace, args.check),
        Cli::HostTests(args) => host_tests(&workspace, args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn fmt_packages(workspace: &Path, args: FmtPackagesArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        xtask::format_package(workspace, package, args.check)?;
    }

    Ok(())
}

fn clean(workspace: &Path, args: CleanArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        let path = workspace.join(package.to_string());
        for dir in walkdir::WalkDir::new(path) {
            if let Ok(dir) = dir
                && let path = dir.path()
                && path.join("Cargo.toml").exists()
            {
                log::info!("Cleaning folder: {}", path.display());
                let cargo_args = CargoArgsBuilder::default()
                    .subcommand("clean")
                    .arg("--target-dir")
                    .arg(path.join("target").display().to_string())
                    .build();

                xtask::cargo::run(&cargo_args, &path).with_context(|| {
                    format!(
                        "Failed to run `cargo run` with {cargo_args:?} in {}",
                        path.display()
                    )
                })?;
            }
        }
    }

    Ok(())
}

fn check_packages(workspace: &Path, args: CheckPackagesArgs) -> Result<()> {
    log::debug!("Checking packages: {:?}", args.packages);
    let mut packages = args.packages;
    packages.sort();

    let mut commands = CargoCommandBatcher::new();

    for package in packages.iter().filter(|p| p.is_published()) {
        // Unfortunately each package has its own unique requirements for
        // building, so we need to handle each individually (though there
        // is *some* overlap)
        for chip in &args.chips {
            log::debug!("  for chip: {}", chip);
            let device = Config::for_chip(chip);

            if let Err(e) = package.validate_package_chip(chip) {
                log::warn!("{e}. Skipping");
                continue;
            }

            for mut features in package.check_feature_rules(device) {
                if package.has_chip_features() {
                    features.push(device.name())
                }

                commands.push(build_check_package_command(
                    workspace,
                    *package,
                    chip,
                    &["--no-default-features"],
                    &features,
                    args.toolchain.as_deref(),
                )?);
            }
        }
    }

    for c in commands.build(false) {
        println!(
            "Command: cargo {}",
            c.command.join(" ").replace("---", "\n    ---")
        );
        c.run(false)?;
    }

    Ok(())
}

fn build_check_package_command(
    workspace: &Path,
    package: Package,
    chip: &Chip,
    args: &[&str],
    features: &[String],
    mut toolchain: Option<&str>,
) -> Result<CargoArgsBuilder> {
    log::info!(
        "Linting package: {} ({}, features: {:?})",
        package,
        chip,
        features
    );

    let path = workspace.join(package.to_string());

    let mut builder = CargoArgsBuilder::default()
        .subcommand("check")
        .manifest_path(path.join("Cargo.toml"));

    if !package.build_on_host(features) {
        if chip.is_xtensa() {
            // In case the user doesn't specify a toolchain, make sure we use +esp
            toolchain.get_or_insert("esp");
        }
        builder = builder.target(package.target_triple(chip)?);
    }

    if let Some(toolchain) = toolchain {
        if !package.build_on_host(features) && toolchain.starts_with("esp") {
            builder = builder.config("-Zbuild-std=core,alloc");
        }
        builder = builder.toolchain(toolchain);
    }

    builder = builder.args(&args);

    if !features.is_empty() {
        builder = builder.arg(format!("--features={}", features.join(",")));
    }

    // TODO: these should come from the outside
    builder.add_env_var("CI", "1");
    builder.add_env_var("DEFMT_LOG", "trace");
    builder.add_env_var("ESP_LOG", "trace");

    Ok(builder)
}

fn lint_packages(workspace: &Path, args: LintPackagesArgs) -> Result<()> {
    log::debug!("Linting packages: {:?}", args.packages);
    let mut packages = args.packages;
    packages.sort();

    for package in packages.iter().filter(|p| p.is_published()) {
        // Unfortunately each package has its own unique requirements for
        // building, so we need to handle each individually (though there
        // is *some* overlap)
        for chip in &args.chips {
            log::debug!("  for chip: {}", chip);
            let device = Config::for_chip(chip);

            if let Err(e) = package.validate_package_chip(chip) {
                log::warn!("{e}. Skipping");
                continue;
            }

            for mut features in package.lint_feature_rules(device) {
                if package.has_chip_features() {
                    features.push(device.name())
                }

                lint_package(
                    workspace,
                    *package,
                    chip,
                    &["--no-default-features"],
                    &features,
                    args.fix,
                    args.toolchain.as_deref(),
                )?;
            }
        }
    }

    Ok(())
}

fn lint_package(
    workspace: &Path,
    package: Package,
    chip: &Chip,
    args: &[&str],
    features: &[String],
    fix: bool,
    mut toolchain: Option<&str>,
) -> Result<()> {
    log::info!(
        "Linting package: {} ({}, features: {:?})",
        package,
        chip,
        features
    );

    let path = workspace.join(package.to_string());

    let mut builder = CargoArgsBuilder::default().subcommand("clippy");

    if !package.build_on_host(features) {
        if chip.is_xtensa() {
            // In case the user doesn't specify a toolchain, make sure we use +esp
            toolchain.get_or_insert("esp");
        }
        builder = builder.target(package.target_triple(chip)?);
    }

    if let Some(toolchain) = toolchain {
        if !package.build_on_host(features) && toolchain.starts_with("esp") {
            builder = builder.config("-Zbuild-std=core,alloc");
        }
        builder = builder.toolchain(toolchain);
    }

    for arg in args {
        builder = builder.arg(arg.to_string());
    }

    if !features.is_empty() {
        builder = builder.arg(format!("--features={}", features.join(",")));
    }

    let builder = if fix {
        builder.arg("--fix").arg("--lib").arg("--allow-dirty")
    } else {
        builder.arg("--").arg("-D").arg("warnings").arg("--no-deps")
    };

    let cargo_args = builder.build();

    xtask::cargo::run_with_env(
        &cargo_args,
        &path,
        [("CI", "1"), ("DEFMT_LOG", "trace")],
        false,
    )
    .with_context(|| {
        format!(
            "Failed to run `cargo run` with {args:?} `CI, `1`, `DEFMT_LOG`, and `trace` envs in {}",
            path.display()
        )
    })?;

    Ok(())
}

struct Runner {
    failed: Vec<&'static str>,
    started_at: Instant,
}

impl Runner {
    fn new() -> Self {
        Self {
            failed: Vec::new(),
            started_at: Instant::now(),
        }
    }

    fn run(&mut self, group: &'static str, op: impl FnOnce() -> Result<()>) {
        // Output grouped logs
        // https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#grouping-log-lines
        println!("::group::{group}");
        if op().is_err() {
            self.failed.push(group);
        }
        println!("::endgroup::");
    }

    fn finish(self) -> Result<()> {
        fn write_summary(message: &str) {
            if let Some(summary_file) = std::env::var_os("GITHUB_STEP_SUMMARY") {
                std::fs::write(summary_file, message).unwrap();
            }
        }

        log::info!("CI checks completed in {:?}", self.started_at.elapsed());

        if !self.failed.is_empty() {
            let mut summary = String::new();
            summary.push_str("# Summary of failed CI checks\n");
            for failed_check in self.failed {
                summary.push_str(&format!("* {failed_check}\n"));
            }
            println!("{summary}");
            write_summary(&summary);
            bail!("CI checks failed");
        }

        Ok(())
    }
}

fn run_ci_checks(workspace: &Path, args: CiArgs) -> Result<()> {
    log::info!("Running CI checks for chip: {}", args.chip);
    println!("::add-matcher::.github/rust-matchers.json");

    let mut runner = Runner::new();

    unsafe {
        std::env::set_var("CI", "true");
    }

    runner.run("Check crates", || {
        check_packages(
            workspace,
            CheckPackagesArgs {
                packages: Package::iter().collect(),
                chips: vec![args.chip],
                toolchain: args.toolchain.clone(),
            },
        )
    });

    if !args.no_lint {
        runner.run("Lint", || {
            lint_packages(
                workspace,
                LintPackagesArgs {
                    packages: Package::iter().collect(),
                    chips: vec![args.chip],
                    fix: false,
                    toolchain: args.toolchain.clone(),
                },
            )
        });
    }

    runner.run("Run Doc Test", || {
        run_doc_tests(
            workspace,
            DocTestArgs {
                package: Package::EspHal,
                chip: args.chip,
            },
        )
    });

    if !args.no_docs {
        runner.run("Build Docs", || {
            build_documentation(
                workspace,
                BuildDocumentationArgs {
                    packages: vec![Package::EspHal, Package::EspRadio, Package::EspRtos],
                    chips: vec![args.chip],
                    ..Default::default()
                },
            )
        });
    }

    // for chips with esp-lp-hal: Build all supported examples for the low-power
    // core first
    if args.chip.has_lp_core() {
        // Build prerequisite examples (esp-lp-hal)
        // `examples` copies the examples to a folder with the chip name as the last
        // path element then we copy it to the place where the HP core example
        // expects it
        runner.run("Build LP-HAL Examples", || {
            // The LP examples aren't really that demanding, but they need to be at a certain place.
            // Instead of trying to figure out where the results are, let's just make sure the
            // target folder is set up as expected.
            let original_target_dir = std::env::var("CARGO_TARGET_DIR");

            unsafe {
                std::env::set_var(
                    "CARGO_TARGET_DIR",
                    workspace.join("esp-lp-hal").join("target"),
                );
            }
            let result = examples(
                workspace,
                ExamplesArgs {
                    package: Package::EspLpHal,
                    chip: Some(args.chip),
                    example: Some("all".to_string()),
                    debug: false,
                    toolchain: args.toolchain.clone(),
                    timings: false,
                },
                CargoAction::Build(None),
            );

            // Still need to rename examples to remove the fingerprint off of their names:
            let dir = workspace
                .join("esp-lp-hal")
                .join("target")
                .join(Package::EspLpHal.target_triple(&args.chip)?)
                .join("release")
                .join("examples");
            let examples = dir
                .read_dir()
                .with_context(|| format!("Failed to read examples directory: {}", dir.display()))?;
            for example in examples {
                let example = example.context("Failed to read example")?;
                if example
                    .file_type()
                    .with_context(|| {
                        format!(
                            "Failed to get file type for example: {}",
                            example.path().display()
                        )
                    })?
                    .is_file()
                    && example.path().extension().is_none()
                {
                    let example_name = example.file_name().to_string_lossy().to_string();
                    let without_fingerprint = example_name
                        .rsplit_once('-')
                        .map(|(a, _)| a)
                        .unwrap_or(&example_name);
                    // Copy so we don't trigger a rebuild unnecessarily by deleting the original
                    std::fs::copy(example.path(), dir.join(without_fingerprint)).with_context(
                        || {
                            format!(
                                "Failed to copy example: {} to {}",
                                example.path().display(),
                                dir.join(without_fingerprint).display()
                            )
                        },
                    )?;
                }
            }

            // Restore the original target directory
            unsafe {
                if let Ok(target) = original_target_dir {
                    std::env::set_var("CARGO_TARGET_DIR", target);
                } else {
                    std::env::remove_var("CARGO_TARGET_DIR");
                }
            }

            result
        });

        if !args.no_docs {
            // Check documentation
            runner.run("Build LP-HAL docs", || {
                build_documentation(
                    workspace,
                    BuildDocumentationArgs {
                        packages: vec![Package::EspLpHal],
                        chips: vec![args.chip],
                        ..Default::default()
                    },
                )
            });
        }
    }

    runner.run("Build examples", || {
        // The `ota_example` expects a file named `examples/target/ota_image` - it
        // doesn't care about the contents however
        std::fs::create_dir_all("./examples/target")
            .with_context(|| format!("Failed to create `./examples/target`"))?;
        std::fs::write("./examples/target/ota_image", "DUMMY")
            .with_context(|| format!("Failed to create a dummy file required by ota example!"))?;

        examples(
            workspace,
            ExamplesArgs {
                package: Package::Examples,
                chip: Some(args.chip),
                example: Some("all".to_string()),
                debug: true,
                toolchain: args.toolchain.clone(),
                timings: false,
            },
            CargoAction::Build(None),
        )
    });

    runner.run("Build qa-test", || {
        examples(
            workspace,
            ExamplesArgs {
                package: Package::QaTest,
                chip: Some(args.chip),
                example: Some("all".to_string()),
                debug: true,
                toolchain: args.toolchain.clone(),
                timings: false,
            },
            CargoAction::Build(None),
        )
    });

    runner.finish()
}

fn host_tests(workspace: &Path, args: HostTestsArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        log::debug!("Running host-tests for package: {}", package);
        if package.has_host_tests(workspace) {
            xtask::run_host_tests(workspace, package)?;
        }
    }

    Ok(())
}
