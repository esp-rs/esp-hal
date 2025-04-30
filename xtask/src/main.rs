use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};

use anyhow::{Result, bail};
use clap::{Args, Parser};
use esp_metadata::{Chip, Config};
use strum::IntoEnumIterator;
use xtask::{
    Package,
    Version,
    cargo::{CargoAction, CargoArgsBuilder},
    commands::*,
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

    /// Bump the version of the specified package(s).
    BumpVersion(BumpVersionArgs),
    /// Perform (parts of) the checks done in CI
    Ci(CiArgs),
    /// Format all packages in the workspace with rustfmt
    #[clap(alias = "format-packages")]
    FmtPackages(FmtPackagesArgs),
    /// Lint all packages in the workspace with clippy
    LintPackages(LintPackagesArgs),
    /// Attempt to publish the specified package.
    Publish(PublishArgs),
    /// Generate git tags for all new package releases.
    TagReleases(TagReleasesArgs),
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
struct PublishArgs {
    /// Package to publish (performs a dry-run by default).
    #[arg(value_enum)]
    package: Package,

    /// Do not pass the `--dry-run` argument, actually try to publish.
    #[arg(long)]
    no_dry_run: bool,
}

#[derive(Debug, Args)]
struct TagReleasesArgs {
    /// Package(s) to tag.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Actually try and create the tags
    #[arg(long)]
    no_dry_run: bool,
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

        Cli::BumpVersion(args) => bump_version(&workspace, args),
        Cli::Ci(args) => run_ci_checks(&workspace, args),
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::Publish(args) => publish(&workspace, args),
        Cli::TagReleases(args) => tag_releases(&workspace, args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn bump_version(workspace: &Path, args: BumpVersionArgs) -> Result<()> {
    // Bump the version by the specified amount for each given package:
    for package in args.packages {
        xtask::bump_version(workspace, package, args.amount)?;
    }

    Ok(())
}

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

    for package in packages.iter().filter(|p| p.is_published()) {
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

    xtask::cargo::run_with_env(&cargo_args, &path, [("CI", "1")], false)?;

    Ok(())
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

fn run_ci_checks(workspace: &Path, args: CiArgs) -> Result<()> {
    let mut failure = false;
    let started_at = Instant::now();

    // Clippy and docs checks

    // Clippy
    lint_packages(
        workspace,
        LintPackagesArgs {
            packages: Package::iter().collect(),
            chips: vec![args.chip],
            fix: false,
        },
    )
    .inspect_err(|_| failure = true)
    .ok();

    // Check doc-tests
    run_doc_tests(
        workspace,
        ExamplesArgs {
            package: Package::EspHal,
            chip: args.chip,
            example: None,
            debug: true,
        },
    )
    .inspect_err(|_| failure = true)
    .ok();

    // Check documentation
    build_documentation(
        workspace,
        BuildDocumentationArgs {
            packages: vec![Package::EspHal, Package::EspWifi, Package::EspHalEmbassy],
            chips: vec![args.chip],
            ..Default::default()
        },
    )
    .inspect_err(|_| failure = true)
    .ok();

    // for chips with esp-lp-hal: Build all supported examples for the low-power
    // core first
    if args.chip.has_lp_core() {
        // Build prerequisite examples (esp-lp-hal)
        // `examples` copies the examples to a folder with the chip name as the last
        // path element then we copy it to the place where the HP core example
        // expects it
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
        .inspect_err(|_| failure = true)
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

        // Check documentation
        build_documentation(
            workspace,
            BuildDocumentationArgs {
                packages: vec![Package::EspLpHal],
                chips: vec![args.chip],
                ..Default::default()
            },
        )
        .inspect_err(|_| failure = true)
        .ok();
    }

    // Make sure we're able to build the HAL without the default features enabled
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
    .inspect_err(|_| failure = true)
    .ok();

    // Build (examples)
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
    .inspect_err(|_| failure = true)
    .ok();

    // Build (qa-test)
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
    .inspect_err(|_| failure = true)
    .ok();

    let completed_at = Instant::now();
    log::info!("CI checks completed in {:?}", completed_at - started_at);

    if failure {
        bail!("CI checks failed");
    }

    Ok(())
}

fn tag_releases(workspace: &Path, mut args: TagReleasesArgs) -> Result<()> {
    args.packages.sort();

    #[derive(serde::Serialize)]
    struct DocumentationItem {
        name: String,
        tag: String,
    }

    let mut created = Vec::new();
    for package in args.packages {
        // If a package does not require documentation, this also means that it is not
        // published (maybe this function needs a better name), so we can skip tagging
        // it:
        if !package.is_published() {
            continue;
        }

        let version = xtask::package_version(workspace, package)?;
        let tag = format!("{package}-v{version}");

        if args.no_dry_run {
            let output = Command::new("git")
                .arg("tag")
                .arg(&tag)
                .current_dir(workspace)
                .output()?;

            if output.stderr.is_empty() {
                log::info!("Created tag '{tag}'");
            } else {
                let err = String::from_utf8_lossy(&output.stderr);
                let err = err.trim_start_matches("fatal: ");
                log::warn!("{}", err);
            }
        } else {
            log::info!("Would create '{tag}' if `--no-dry-run` was passed.")
        }
        created.push(DocumentationItem {
            name: package.to_string(),
            tag,
        });
    }

    if args.no_dry_run {
        log::info!("Created {} tags", created.len());
        log::info!("IMPORTANT: Don't forget to push the tags to the correct remote!");
    }

    log::info!(
        "Documentation workflow input for these packages:\r\n\r\n {:#}",
        serde_json::to_string(&created)?
    );

    Ok(())
}
