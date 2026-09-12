use std::path::Path;

use anyhow::{Context, Result, bail};
use clap::Args;
use strum::IntoEnumIterator;

use crate::{
    Package,
    cargo::{CargoArgsBuilder, CargoCommandBatcher},
    metadata::{Chip, Config},
    resolve::{ResolveInput, resolve},
};

/// Arguments for the `lint` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Lint all packages in the workspace with clippy",
        command = "lint"
    )
)]
#[derive(Debug, Args)]
pub struct LintPackagesArgs {
    /// Chip and/or package, in any order. Omitted means every published crate on every chip.
    pub tokens: Vec<String>,

    /// Automatically apply fixes
    #[arg(long)]
    pub fix: bool,

    /// The toolchain used to run the lints
    #[arg(long)]
    pub toolchain: Option<String>,
}

/// Lint published packages with clippy.
pub fn lint_packages(workspace: &Path, args: LintPackagesArgs) -> Result<()> {
    let resolution = resolve(ResolveInput::from_tokens(args.tokens));
    if !resolution.names.is_empty() {
        bail!("Unknown argument: {}", resolution.names.join(", "));
    }
    let mut packages = if resolution.packages.is_empty() {
        Package::iter().collect()
    } else {
        resolution.packages
    };
    let chips = if resolution.chips.is_empty() {
        Chip::iter().collect()
    } else {
        resolution.chips
    };

    log::debug!("Linting packages: {packages:?}");
    packages.sort();

    for package in packages.iter().filter(|p| p.is_published()) {
        // Unfortunately each package has its own unique requirements for
        // building, so we need to handle each individually (though there
        // is *some* overlap)
        for chip in &chips {
            log::debug!("  for chip: {}", chip);
            let device = Config::for_chip(chip);

            if let Err(e) = package.validate_package_chip(chip) {
                log::warn!("{e}. Skipping");
                continue;
            }

            for mut check_config in package.lint_config_rules(device) {
                if package.has_chip_features() {
                    check_config.features.push(device.name.clone());
                }

                lint_package(
                    workspace,
                    *package,
                    chip,
                    &["--no-default-features"],
                    &check_config,
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
    check_config: &crate::CheckConfig,
    fix: bool,
    mut toolchain: Option<&str>,
) -> Result<()> {
    log::info!(
        "Linting package: {} ({}, features: {:?}, env: {:?})",
        package,
        chip,
        check_config.features,
        check_config.env
    );

    let path = workspace.join(package.directory());
    let features = &check_config.features;

    let mut builder = CargoArgsBuilder::default()
        .subcommand("clippy")
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

    for arg in args {
        builder = builder.arg(arg.to_string());
    }

    if !features.is_empty() {
        builder = builder.arg(format!("--features={}", features.join(",")));
    }

    let mut builder = if fix {
        builder.arg("--fix").arg("--lib").arg("--allow-dirty")
    } else {
        builder.arg("--").arg("-D").arg("warnings").arg("--no-deps")
    };

    for (key, value) in &check_config.env {
        builder.add_env_var(key, value);
    }
    builder.add_env_var("CI", "1");
    builder.add_env_var("DEFMT_LOG", "trace");

    let command = CargoCommandBatcher::build_one_for_cargo(&builder);
    crate::cargo::run_with_env(&command.command, &path, command.env_vars, false)
        .with_context(|| format!("Failed to run `cargo clippy` in {}", path.display()))?;

    Ok(())
}
