use std::{
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result};
use clap::Args;
use object::{Object, ObjectSymbol, SymbolKind, read::archive::ArchiveFile};
use rustc_demangle::try_demangle;
use strum::IntoEnumIterator;

use crate::{
    Package,
    cargo::{CargoArgsBuilder, CargoCommandBatcher},
    metadata::{Chip, Config},
};

/// Arguments for checking packages (`check` without an example/test name, and
/// `check-global-symbols`).
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

/// Check published packages with `cargo check`.
pub fn check_packages(workspace: &Path, args: CheckPackagesArgs) -> Result<()> {
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

            for mut check_config in package.check_config_rules(device) {
                if package.has_chip_features() {
                    check_config.features.push(device.name.clone());
                }

                commands.push(build_check_package_command(
                    workspace,
                    *package,
                    chip,
                    &["--no-default-features"],
                    &check_config,
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
    check_config: &crate::CheckConfig,
    mut toolchain: Option<&str>,
) -> Result<CargoArgsBuilder> {
    log::info!(
        "Checking package: {} ({}, features: {:?}, env: {:?})",
        package,
        chip,
        check_config.features,
        check_config.env
    );

    let path = workspace.join(package.directory());
    let features = &check_config.features;

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

    builder = builder.args(args);

    if !features.is_empty() {
        builder = builder.arg(format!("--features={}", features.join(",")));
    }

    for (key, value) in &check_config.env {
        builder.add_env_var(key, value);
    }
    // TODO: these should come from the outside
    builder.add_env_var("CI", "1");
    builder.add_env_var("DEFMT_LOG", "trace");
    builder.add_env_var("ESP_LOG", "trace");

    Ok(builder)
}

/// Build the given package for the given chip and return its `.rlib path` with `esp` toolchain.
fn build_rlib(package: &str, chip: &str, target: &str) -> Result<PathBuf> {
    let workspace = std::env::current_dir().with_context(|| "Failed to get the current dir!")?;
    Command::new("cargo")
        .args([
            "+esp",
            "build",
            "--no-default-features",
            "--features",
            chip,
            "--target",
            target,
            "-Zbuild-std=core",
        ])
        .current_dir(workspace.join(package))
        .status()
        .context("Failed to run cargo build")?;

    let lib_name = format!("lib{}.rlib", package.replace('-', "_"));
    let rlib_path = Path::new("target")
        .join(target)
        .join("debug")
        .join(&lib_name);

    if !rlib_path.exists() {
        anyhow::bail!("Expected rlib not found: {}", rlib_path.display());
    }

    Ok(rlib_path)
}

/// Check global symbols in the compiled rlib of the specified packages for the
/// specified chips. Reports any unmangled global symbols that may pollute the
/// global namespace.
pub fn check_global_symbols(chips: &[Chip]) -> Result<()> {
    let mut total_problematic = vec![];

    let package = Package::EspHal; // Only esp-hal for now

    for chip in chips {
        let target = package.target_triple(chip)?;

        let rlib_path = match build_rlib(package.as_ref(), chip.as_ref(), &target) {
            Ok(path) => path,
            Err(e) => {
                println!(
                    "Failed to build/find rlib for {} on {}: {}",
                    package, chip, e
                );
                continue;
            }
        };

        log::info!("Checking global symbols for {} on {}:\n", package, chip);

        let data = std::fs::read(&rlib_path)
            .with_context(|| format!("Failed to read {}!", rlib_path.display()))?;
        let archive = ArchiveFile::parse(data.as_slice())
            .with_context(|| "Failed to create archive!".to_string())?;

        let mut problematic_symbols: Vec<(String, SymbolKind, usize)> = Vec::new();

        for member in archive.members().flatten() {
            let obj = object::File::parse(
                member
                    .data(data.as_slice())
                    .with_context(|| "Failed to parse archive!")?,
            )?;

            for symbol in obj.symbols().filter(|s| s.is_global() && s.is_definition()) {
                if let Ok(name) = symbol.name()
                    && try_demangle(name).is_err()
                {
                    let section = symbol.section_index().map(|i| i.0).unwrap_or(0);
                    problematic_symbols.push((name.to_string(), symbol.kind(), section));
                }
            }
        }

        if problematic_symbols.is_empty() {
            println!("All global symbols are properly mangled Rust symbols\n");
        } else {
            println!(
                "Found {} potentially problematic global symbols:",
                problematic_symbols.len(),
            );

            for (name, kind, _) in &problematic_symbols {
                println!("{:?} {}", kind, name);
            }

            total_problematic.extend(
                problematic_symbols
                    .into_iter()
                    .map(|(name, kind, _)| (chip, name, kind)),
            );
        }
    }

    if !total_problematic.is_empty() {
        for (chip, name, kind) in total_problematic.iter() {
            println!("{}: {} ({:?})", chip, name, kind);
        }
        Err(anyhow::anyhow!(
            "Found {count} unmangled global symbols across all packages/chips",
            count = total_problematic.len()
        ))
    } else {
        Ok(())
    }
}
