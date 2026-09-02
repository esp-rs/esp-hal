use std::path::Path;

use anyhow::Result;
use clap::Args;
use strum::IntoEnumIterator as _;

use crate::{
    Package,
    cargo::{self, CargoAction, CargoArgsBuilder, CargoCommandBatcher},
    commands::move_artifacts,
    firmware::Metadata,
    metadata::Chip,
};

// ----------------------------------------------------------------------------
// Command Arguments

/// Arguments for building documentation.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Build documentation for the specified packages and chips",
        command = "documentation"
    )
)]
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
    /// Documentation channel name (for example `main`).
    ///
    /// When set, documentation is written under this name instead of the crate
    /// version, so a local or CI build cannot overwrite a released docs tree.
    #[arg(long)]
    pub channel: Option<String>,
    #[cfg(feature = "preview-docs")]
    #[arg(long)]
    pub serve: bool,
}

/// Arguments for building the documentation index.
#[cfg(feature = "deploy-docs")]
#[derive(Debug, Default, Args)]
pub struct BuildDocumentationIndexArgs {
    /// Base URL of the deployed documentation.
    #[arg(long)]
    pub base_url: Option<String>,
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
        args.base_url.clone(),
        args.channel,
    )?;

    crate::documentation::build_documentation_index(workspace, &mut args.packages, args.base_url)?;

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
pub fn build_documentation_index(
    workspace: &Path,
    args: BuildDocumentationIndexArgs,
) -> Result<()> {
    let mut packages = Package::iter().collect::<Vec<_>>();
    crate::documentation::build_documentation_index(workspace, &mut packages, args.base_url)?;

    Ok(())
}

/// Build all examples for the specified package and chip.
pub fn build_examples(
    package: Package,
    chip: Chip,
    debug: bool,
    toolchain: Option<&str>,
    timings: bool,
    examples: Vec<Metadata>,
    package_path: &Path,
    out_path: Option<&Path>,
) -> Result<()> {
    let target = package.target_triple(&chip)?;

    // Attempt to build each supported example, with all required features enabled:

    let action = CargoAction::Build(out_path.map(|p| p.to_path_buf()));
    let mut commands = CargoCommandBatcher::new();
    // Build command list
    for example in examples.iter() {
        let command = crate::generate_build_command(
            package_path,
            chip,
            &target,
            example,
            action.clone(),
            debug,
            toolchain,
            timings,
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

/// Build the specified package.
pub fn build_package(workspace: &Path, package: Package, toolchain: Option<&str>) -> Result<()> {
    let package_path = crate::windows_safe_path(&workspace.join(package.to_string()));

    log::info!("Building package '{}'", package_path.display());

    let mut builder = CargoArgsBuilder::default()
        .subcommand("build")
        .arg("--release");

    if let Some(toolchain) = toolchain {
        builder = builder.toolchain(toolchain);
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    cargo::run(&args, &package_path)?;

    Ok(())
}
