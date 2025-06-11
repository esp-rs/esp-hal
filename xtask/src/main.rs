use std::{
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

use anyhow::{bail, Result};
use clap::{Args, Parser};
use esp_metadata::{Chip, Config};
use strum::IntoEnumIterator;
use xtask::{
    cargo::{CargoAction, CargoArgsBuilder},
    commands::*,
    Package,
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
    /// Lint all packages in the workspace with clippy
    LintPackages(LintPackagesArgs),
    /// Semver Checks
    SemverCheck(SemverCheckArgs),
    /// Check the changelog for packages.
    CheckChangelog(CheckChangelogArgs),
    /// Re-generate the chip support table in the esp-hal README.
    UpdateChipSupportTable,
}

#[derive(Debug, Args)]
struct CiArgs {
    /// Chip to target.
    #[arg(value_enum)]
    chip: Chip,
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

// ----------------------------------------------------------------------------
// Application

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let workspace = std::env::current_dir()?;
    let target_path = Path::new("target");

    match Cli::parse() {
        // Build-related subcommands:
        Cli::Build(build) => match build {
            Build::Documentation(args) => build_documentation(&workspace, args),
            #[cfg(feature = "deploy-docs")]
            Build::DocumentationIndex => build_documentation_index(&workspace),
            Build::Examples(args) => examples(
                &workspace,
                args,
                CargoAction::Build(target_path.join("examples")),
            ),
            Build::Package(args) => build_package(&workspace, args),
            Build::Tests(args) => tests(
                &workspace,
                args,
                CargoAction::Build(target_path.join("tests")),
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
        },

        Cli::Ci(args) => run_ci_checks(&workspace, args),
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::SemverCheck(args) => semver_checks(&workspace, args),
        Cli::CheckChangelog(args) => check_changelog(&workspace, &args.packages, args.normalize),
        Cli::UpdateChipSupportTable => {
            // Re-generate the chip support table in the esp-hal README.
            // This is a no-op if the table is already up-to-date.
            xtask::update_chip_support_table(&workspace)?;
            Ok(())
        }
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn fmt_packages(workspace: &Path, args: FmtPackagesArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages {
        log::info!("Formatting package: {}", package);
        let path = workspace.join(package.to_string());

        // we need to list all source files since modules in `unstable_module!` macros
        // won't get picked up otherwise
        let source_files: Vec<String> = walkdir::WalkDir::new(path.join("src"))
            .into_iter()
            .filter_map(|entry| {
                let path = entry.unwrap().into_path();
                if let Some("rs") = path.extension().unwrap_or_default().to_str() {
                    Some(String::from(path.to_str().unwrap()))
                } else {
                    None
                }
            })
            .collect();

        let mut cargo_args = CargoArgsBuilder::default()
            .toolchain("nightly")
            .subcommand("fmt")
            .arg("--all")
            .build();

        if args.check {
            cargo_args.push("--".into());
            cargo_args.push("--check".into());
        }

        cargo_args.push("--".into());
        cargo_args.extend_from_slice(&source_files);

        xtask::cargo::run(&cargo_args, &path)?;
    }

    Ok(())
}

fn lint_packages(workspace: &Path, args: LintPackagesArgs) -> Result<()> {
    let mut packages = args.packages;
    packages.sort();

    for package in packages.iter().filter(|p| p.is_published(workspace)) {
        // Unfortunately each package has its own unique requirements for
        // building, so we need to handle each individually (though there
        // is *some* overlap)
        for chip in &args.chips {
            let device = Config::for_chip(chip);

            if package.validate_package_chip(chip).is_err() {
                continue;
            }

            let feature_sets = [
                vec![package.feature_rules(device)], // initially test all features
                package.lint_feature_rules(device),  // add separate test cases
            ]
            .concat();

            for mut features in feature_sets {
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
) -> Result<()> {
    log::info!(
        "Linting package: {} ({}, features: {:?})",
        package,
        chip,
        features
    );

    let path = workspace.join(package.to_string());

    let mut builder = CargoArgsBuilder::default().subcommand("clippy");

    let mut builder = if !package.build_on_host() {
        if chip.is_xtensa() {
            // We only overwrite Xtensas so that externally set nightly/stable toolchains
            // are not overwritten.
            builder = builder.arg("-Zbuild-std=core,alloc");
            builder = builder.toolchain("esp");
        }

        builder.target(package.target_triple(chip)?)
    } else {
        builder
    };

    for arg in args {
        builder = builder.arg(arg.to_string());
    }

    builder = builder.arg(format!("--features={}", features.join(",")));

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
    )?;

    Ok(())
}

fn run_ci_checks(workspace: &Path, args: CiArgs) -> Result<()> {
    fn write_summary(message: &str) {
        if let Some(summary_file) = std::env::var_os("GITHUB_STEP_SUMMARY") {
            std::fs::write(summary_file, message).unwrap();
        }
    }

    println!("::add-matcher::.github/rust-matchers.json");

    let mut failed = Vec::new();
    let started_at = Instant::now();

    // Clippy and docs checks

    // Clippy
    println!("::group::Lint");
    lint_packages(
        workspace,
        LintPackagesArgs {
            packages: Package::iter().collect(),
            chips: vec![args.chip],
            fix: false,
        },
    )
    .inspect_err(|_| failed.push("Lint"))
    .ok();
    println!("::endgroup");

    // Check doc-tests
    println!("::group::Doc Test");
    run_doc_tests(
        workspace,
        ExamplesArgs {
            package: Package::EspHal,
            chip: args.chip,
            example: None,
            debug: true,
        },
    )
    .inspect_err(|_| failed.push("Doc Test"))
    .ok();
    println!("::endgroup");

    // Check documentation
    println!("::group::Build Docs");
    build_documentation(
        workspace,
        BuildDocumentationArgs {
            packages: vec![Package::EspHal, Package::EspWifi, Package::EspHalEmbassy],
            chips: vec![args.chip],
            ..Default::default()
        },
    )
    .inspect_err(|_| failed.push("Build Docs"))
    .ok();
    println!("::endgroup");

    // for chips with esp-lp-hal: Build all supported examples for the low-power
    // core first
    if args.chip.has_lp_core() {
        // Build prerequisite examples (esp-lp-hal)
        // `examples` copies the examples to a folder with the chip name as the last
        // path element then we copy it to the place where the HP core example
        // expects it
        println!("::group::Build LP-HAL Examples");
        examples(
            workspace,
            ExamplesArgs {
                package: Package::EspLpHal,
                chip: args.chip,
                example: None,
                debug: false,
            },
            CargoAction::Build(PathBuf::from(format!(
                "./esp-lp-hal/target/{}/release/examples",
                args.chip.target()
            ))),
        )
        .inspect_err(|_| failed.push("Build LP-HAL Examples"))
        .and_then(|_| {
            let from_dir = PathBuf::from(format!(
                "./esp-lp-hal/target/{}/release/examples/{}",
                args.chip.target(),
                args.chip
            ));
            let to_dir = PathBuf::from(format!(
                "./esp-lp-hal/target/{}/release/examples",
                args.chip.target()
            ));
            from_dir.read_dir()?.for_each(|entry| {
                let entry = entry.unwrap();
                let path = entry.path();
                let to = to_dir.join(entry.file_name());
                fs::copy(path, to).expect("Failed to copy file");
            });
            Ok(())
        })
        .ok();
        println!("::endgroup");

        // Check documentation
        println!("::group::Build LP-HAL docs");
        build_documentation(
            workspace,
            BuildDocumentationArgs {
                packages: vec![Package::EspLpHal],
                chips: vec![args.chip],
                ..Default::default()
            },
        )
        .inspect_err(|_| failed.push("Build LP-HAL docs"))
        .ok();
        println!("::endgroup");
    }

    // Make sure we're able to build the HAL without the default features enabled
    println!("::group::Build HAL");
    build_package(
        workspace,
        BuildPackageArgs {
            package: Package::EspHal,
            target: Some(args.chip.target().to_string()),
            features: vec![args.chip.to_string()],
            toolchain: None,
            no_default_features: true,
        },
    )
    .inspect_err(|_| failed.push("Build HAL"))
    .ok();
    println!("::endgroup");

    // Build (examples)
    println!("::group::Build examples");

    // The `ota_example` expects a file named `examples/target/ota_image` - it doesn't care about the contents however
    std::fs::create_dir_all("./examples/target")?;
    std::fs::write("./examples/target/ota_image", "DUMMY")?;

    examples(
        workspace,
        ExamplesArgs {
            package: Package::Examples,
            chip: args.chip,
            example: None,
            debug: true,
        },
        CargoAction::Build(PathBuf::from("./examples/target/")),
    )
    .inspect_err(|_| failed.push("Build examples"))
    .ok();
    println!("::endgroup");

    // Build (qa-test)
    println!("::group::Build qa-test");
    examples(
        workspace,
        ExamplesArgs {
            package: Package::QaTest,
            chip: args.chip,
            example: None,
            debug: true,
        },
        CargoAction::Build(PathBuf::from("./qa-test/target/")),
    )
    .inspect_err(|_| failed.push("Build qa-test"))
    .ok();
    println!("::endgroup");

    let completed_at = Instant::now();
    log::info!("CI checks completed in {:?}", completed_at - started_at);

    if !failed.is_empty() {
        let mut summary = String::new();
        summary.push_str("# Summary of failed CI checks\n");
        for failed_check in failed {
            summary.push_str(&format!("* {failed_check}\n"));
        }
        println!("{summary}");
        write_summary(&summary);
        bail!("CI checks failed");
    }

    Ok(())
}
