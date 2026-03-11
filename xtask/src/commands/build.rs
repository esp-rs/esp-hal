use std::path::Path;

use anyhow::Result;
use clap::{Args, Subcommand};
use esp_metadata::Chip;
use strum::IntoEnumIterator as _;

use super::{ExamplesArgs, TestsArgs};
use crate::{
    Package,
    cargo::{self, CargoAction, CargoArgsBuilder, CargoCommandBatcher},
    commands::move_artifacts,
    firmware::Metadata,
};

// ----------------------------------------------------------------------------
// Subcommands

/// Build subcommands and their arguments.
#[derive(Debug, Subcommand)]
pub enum Build {
    /// Build documentation for the specified chip.
    Documentation(BuildDocumentationArgs),
    /// Build documentation index.
    #[cfg(feature = "deploy-docs")]
    DocumentationIndex,
    /// Build all examples for the specified chip.
    Examples(ExamplesArgs),
    /// Build the specified package with the given options.
    Package(BuildPackageArgs),
    /// Build all applicable tests or the specified test for a specified chip.
    Tests(TestsArgs),
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

/// Arguments for building documentation.
#[cfg_attr(feature = "mcp", xtask_mcp_macros::mcp_tool(
    description = "Build documentation for the specified packages and chips",
    command = "build documentation"
))]
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

/// Arguments for building a package.
#[cfg_attr(feature = "mcp", xtask_mcp_macros::mcp_tool(
    description = "Build the specified package with the given options",
    command = "build package"
))]
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

/// Build documentation for the specified packages and chips.
pub fn build_documentation(workspace: &Path, mut args: BuildDocumentationArgs) -> Result<()> {
    log::debug!(
        "Building documentation for packages {:?} on chips {:?}",
        args.packages,
        args.chips
    );
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

/// Build the documentation index for all packages.
#[cfg(feature = "deploy-docs")]
pub fn build_documentation_index(workspace: &Path) -> Result<()> {
    let mut packages = Package::iter().collect::<Vec<_>>();
    crate::documentation::build_documentation_index(workspace, &mut packages)?;

    Ok(())
}

/// Build all examples for the specified package and chip.
pub fn build_examples(
    args: ExamplesArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
    out_path: Option<&Path>,
) -> Result<()> {
    let chip = args.chip.unwrap();

    // Determine the appropriate build target for the given package and chip:
    let target = args.package.target_triple(&chip)?;

    // Attempt to build each supported example, with all required features enabled:

    let action = CargoAction::Build(out_path.map(|p| p.to_path_buf()));
    let mut commands = CargoCommandBatcher::new();
    // Build command list
    for example in examples.iter() {
        let command = crate::generate_build_command(
            &package_path,
            chip,
            &target,
            example,
            action.clone(),
            args.debug,
            args.toolchain.as_deref(),
            args.timings,
            &[],
        )?;
        commands.push(command);
    }
    // Execute the specified action:
    for c in commands.build(false) {
        println!(
            "Command: cargo {}",
            c.command.join(" ").replace("---", "\n    ---")
        );
        c.run(false)?;
    }
    move_artifacts(chip, &action);

    Ok(())
}

/// Build the specified package with the given options.
pub fn build_package(workspace: &Path, args: BuildPackageArgs) -> Result<()> {
    log::debug!(
        "Building package '{}' with target '{:?}' and features {:?}",
        args.package,
        args.target,
        args.features
    );
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
