use std::path::Path;

use anyhow::{Result, bail};
use clap::{Args, Subcommand};
use esp_metadata::Chip;
use strum::IntoEnumIterator as _;

use super::{ExamplesArgs, TestsArgs};
use crate::{
    Package,
    cargo::{self, CargoAction, CargoArgsBuilder},
    firmware::Metadata,
};

// ----------------------------------------------------------------------------
// Subcommands

#[derive(Debug, Subcommand)]
pub enum Build {
    /// Build documentation for the specified chip.
    Documentation(BuildDocumentationArgs),
    /// Build all examples for the specified chip.
    Examples(ExamplesArgs),
    /// Build the specified package with the given options.
    Package(BuildPackageArgs),
    /// Build all applicable tests or the specified test for a specified chip.
    Tests(TestsArgs),
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

#[derive(Debug, Default, Args)]
pub struct BuildDocumentationArgs {
    /// Package(s) to document.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    pub packages: Vec<Package>,
    /// Chip(s) to build documentation for.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    pub chips: Vec<Chip>,
    /// Base URL of the deployed documentation.
    #[arg(long)]
    pub base_url: Option<String>,
    #[cfg(feature = "preview-docs")]
    #[arg(long)]
    pub serve: bool,
}

#[derive(Debug, Args)]
pub struct BuildPackageArgs {
    /// Package to build.
    #[arg(value_enum)]
    pub package: Package,
    /// Target to build for.
    #[arg(long)]
    pub target: Option<String>,
    /// Features to build with.
    #[arg(long, value_delimiter = ',')]
    pub features: Vec<String>,
    /// Toolchain to build with.
    #[arg(long)]
    pub toolchain: Option<String>,
    /// Don't enabled the default features.
    #[arg(long)]
    pub no_default_features: bool,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

pub fn build_documentation(workspace: &Path, mut args: BuildDocumentationArgs) -> Result<()> {
    crate::documentation::build_documentation(
        workspace,
        &mut args.packages,
        &mut args.chips,
        args.base_url,
    )?;

    crate::documentation::build_documentation_index(workspace, &mut args.packages)?;

    #[cfg(feature = "preview-docs")]
    if args.serve {
        use std::{
            thread::{sleep, spawn},
            time::Duration,
        };

        use rocket::fs::{FileServer, Options};

        spawn(|| {
            sleep(Duration::from_millis(1000));
            opener::open_browser("http://127.0.0.1:8000/").ok();
        });

        rocket::async_main(
            {
                rocket::build().mount(
                    "/",
                    FileServer::new(
                        "docs",
                        Options::Index | Options::IndexFile | Options::DotFiles,
                    ),
                )
            }
            .launch(),
        )?;
    }

    Ok(())
}

pub fn build_examples(
    args: ExamplesArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
    out_path: &Path,
) -> Result<()> {
    // Determine the appropriate build target for the given package and chip:
    let target = args.package.target_triple(&args.chip)?;

    if examples.iter().any(|ex| ex.matches(&args.example)) {
        // Attempt to build only the specified example:
        for example in examples.iter().filter(|ex| ex.matches(&args.example)) {
            crate::execute_app(
                package_path,
                args.chip,
                target,
                example,
                CargoAction::Build(out_path.to_path_buf()),
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
            crate::execute_app(
                package_path,
                args.chip,
                target,
                example,
                CargoAction::Build(out_path.to_path_buf()),
                1,
                args.debug,
            )
        })
    }
}

pub fn build_package(workspace: &Path, args: BuildPackageArgs) -> Result<()> {
    // Absolute path of the package's root:
    let package_path = crate::windows_safe_path(&workspace.join(args.package.to_string()));

    // Build the package using the provided features and/or target, if any:

    log::info!("Building package '{}'", package_path.display());
    if !args.features.is_empty() {
        log::info!("  Features: {}", args.features.join(","));
    }
    if let Some(ref target) = args.target {
        log::info!("  Target:   {}", target);
    }

    let mut builder = CargoArgsBuilder::default()
        .subcommand("build")
        .arg("--release");

    if let Some(toolchain) = args.toolchain {
        builder = builder.toolchain(toolchain);
    }

    if let Some(target) = args.target {
        // If targeting an Xtensa device, we must use the '+esp' toolchain modifier:
        if target.starts_with("xtensa") {
            builder = builder.toolchain("esp");
            builder = builder.arg("-Zbuild-std=core,alloc")
        }
        builder = builder.target(target);
    }

    if !args.features.is_empty() {
        builder = builder.features(&args.features);
    }

    if args.no_default_features {
        builder = builder.arg("--no-default-features");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run(&args, &package_path)?;

    Ok(())
}
