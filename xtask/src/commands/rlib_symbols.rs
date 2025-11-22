use std::{
    collections::HashSet,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, anyhow};
use clap::Args;
use object::{Object, ObjectSymbol, read::archive::ArchiveFile};
use rustc_demangle::try_demangle;
use strum::IntoEnumIterator;

use crate::{Chip, Package};

/// Arguments for checking rlib symbols in esp-rom-sys.
#[derive(Debug, Args)]
pub struct CheckRlibArgs {
    /// Package(s) to target.
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Check for a specific chip.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
    chips: Vec<Chip>,

    /// The toolchain used to run the checks.
    #[arg(long)]
    toolchain: Option<String>,

    /// Regenerate the baseline file(s) for the checked packages/chips.
    #[arg(long)]
    regenerate_baseline: bool,
}

/// Build the given package for the given chip and target, then extract relevant global symbols.
fn build_and_extract_symbols(
    package: &str,
    chip: &str,
    target: &str,
    for_abi_check: bool,
) -> Result<HashSet<String>> {
    let workspace = std::env::current_dir().with_context(|| "Failed to get the current dir!")?;

    Command::new("cargo")
        .args(&[
            "+esp",
            "build",
            "--no-default-features",
            "--features",
            chip,
            "--target",
            target,
            "-Zbuild-std=core",
        ])
        .current_dir(workspace.join(package.to_string()))
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

    let data = std::fs::read(&rlib_path)
        .with_context(|| format!("Failed to read {}!", rlib_path.display()))?;
    let archive =
        ArchiveFile::parse(data.as_slice()).with_context(|| "Failed to create archive!")?;

    let mut current_symbols: HashSet<String> = HashSet::new();

    for member in archive.members().flatten() {
        let obj = object::File::parse(
            member
                .data(data.as_slice())
                .with_context(|| "Failed to parse archive!")?,
        )?;

        // Filter for global definitions
        for symbol in obj.symbols().filter(|s| s.is_global() && s.is_definition()) {
            if let Ok(name) = symbol.name() {
                current_symbols.insert(name.to_string());
            }
        }
    }

    // For ABI checks, filter to only symbols that matter for public API
    if for_abi_check {
        current_symbols = filter_abi_relevant_symbols(current_symbols);
    }

    Ok(current_symbols)
}

/// Filter symbols to only those relevant for ABI stability:
/// - C symbols (unmangled)
/// - Rust symbols with #[no_mangle]
/// - Exclude regular Rust symbols (internal, mangled)
fn filter_abi_relevant_symbols(symbols: HashSet<String>) -> HashSet<String> {
    symbols
        .into_iter()
        .filter(|sym| {
            // Keep all C symbols (they fail demangling)
            if try_demangle(sym).is_err() {
                return true;
            }

            // For Rust symbols, only keep ones that are explicitly #[no_mangle]
            // These will have simple names without complex mangling
            is_no_mangle_symbol(sym)
        })
        .collect()
}

/// Determine if a symbol is a #[no_mangle] Rust symbol
fn is_no_mangle_symbol(symbol: &str) -> bool {
    // If it starts with Rust mangling pattern, it's a regular Rust symbol, not #[no_mangle]
    if symbol.starts_with("_ZN") {
        return false;
    }

    // If it can be demangled but doesn't start with mangling pattern,
    // it might be a #[no_mangle] symbol
    if let Ok(demangled) = try_demangle(symbol) {
        let demangled_str = demangled.to_string();
        // #[no_mangle] symbols typically don't have namespace paths like core::
        // and don't have hash suffixes
        !demangled_str.contains("::") && !demangled_str.contains("::h") && demangled_str == symbol
    } else {
        false
    }
}

/// Check global symbols in the compiled rlib of the specified packages for the
/// specified chips. Reports any unmangled global symbols that may pollute the
/// global namespace.
pub fn check_global_symbols(chips: &[Chip]) -> Result<()> {
    let mut total_problematic = 0;
    let package = Package::EspHal; // Only esp-hal for now

    for chip in chips {
        let target = package.target_triple(chip)?;
        let package_str = package.to_string();
        let chip_str = chip.to_string();

        let all_symbols = match build_and_extract_symbols(&package_str, &chip_str, &target, false) {
            Ok(symbols) => symbols,
            Err(e) => {
                println!(
                    "Failed to build/extract symbols for {} on {}: {}",
                    package, chip, e
                );
                continue;
            }
        };

        log::info!("Checking global symbols for {} on {}:\n", package, chip);

        let mut problematic_symbols: Vec<String> = Vec::new();

        for name in all_symbols.iter() {
            // The core logic for this check: symbol is global and unmangled
            if try_demangle(name).is_err() {
                problematic_symbols.push(name.clone());
            }
        }

        if problematic_symbols.is_empty() {
            println!("All global symbols are properly mangled Rust symbols\n");
        } else {
            println!(
                "Found {} potentially problematic global symbols:",
                problematic_symbols.len(),
            );

            for name in &problematic_symbols {
                println!("{}", name);
            }

            total_problematic += problematic_symbols.len();
        }
    }

    if total_problematic > 0 {
        Err(anyhow::anyhow!(
            "Found {total_problematic} unmangled global symbols across all packages/chips"
        ))
    } else {
        Ok(())
    }
}

/// Check rlib symbols in the compiled rlib of esp-rom-sys for the specified chips.
pub fn check_rom_sys_symbols(args: &CheckRlibArgs) -> Result<()> {
    let mut total_problems = 0;
    let package = Package::EspRomSys;

    let git_rev = git_rev()?;

    // Aggregators for the final report
    let mut all_missing_symbols: Vec<(String, String)> = Vec::new();
    let mut all_new_symbols: Vec<(String, String)> = Vec::new();

    for chip in args.chips.clone() {
        let chip_name_str = chip.to_string();
        let target_triple = package.target_triple(&chip)?;
        let baseline_path =
            PathBuf::from(format!("./esp-rom-sys/baselines/{}.txt", &chip_name_str));

        // For esp-rom-sys ABI check, we only want symbols that matter for public API
        let current_symbols = match build_and_extract_symbols(
            &package.to_string(),
            &chip_name_str,
            &target_triple,
            true,
        ) {
            Ok(symbols) => symbols,
            Err(e) => {
                println!(
                    "Failed to build/extract symbols for {}: {}",
                    chip_name_str, e
                );
                continue;
            }
        };

        let should_write_baseline = args.regenerate_baseline || !baseline_path.exists();
        if should_write_baseline {
            if args.regenerate_baseline && baseline_path.exists() {
                log::warn!(
                    "Regenerating existing baseline file for {}: {}",
                    chip_name_str,
                    baseline_path.display()
                );
            } else {
                log::warn!(
                    "Baseline file not found for {}. Creating baseline file: {}",
                    chip_name_str,
                    baseline_path.display()
                );
            }

            if let Some(parent_dir) = baseline_path.parent() {
                fs::create_dir_all(parent_dir).with_context(|| {
                    format!(
                        "Failed to create directory structure for {}",
                        parent_dir.display()
                    )
                })?;
            }

            // Just use the filtered symbols directly - no need to process further
            let mut abi_symbols_vec: Vec<String> = current_symbols.into_iter().collect();
            abi_symbols_vec.sort();

            let mut content = format!("GIT_REV: {}\n", git_rev);
            content.push_str(&abi_symbols_vec.join("\n"));

            fs::write(&baseline_path, content).with_context(|| {
                format!("Failed to write baseline to {}", baseline_path.display())
            })?;

            continue;
        }

        // Load existing baseline
        let baseline_content = fs::read_to_string(&baseline_path)
            .with_context(|| format!("Failed to read baseline from {}", baseline_path.display()))?;

        let baseline_symbols: HashSet<String> = baseline_content
            .lines()
            .skip(1) // Skip the first line (GIT_REV: ...)
            .filter(|s| !s.trim().is_empty())
            .map(|s| s.to_string())
            .collect();

        // Current symbols are already filtered to only ABI-relevant ones
        let abi_current = current_symbols;

        // Missing Symbols (Must be present - ABI Break)
        let missing_current: Vec<String> = baseline_symbols
            .iter()
            .filter(|name| !abi_current.contains(*name))
            .cloned()
            .collect();

        // New Symbols
        let new_current: Vec<String> = abi_current
            .iter()
            .filter(|name| !baseline_symbols.contains(*name))
            .cloned()
            .collect();

        let chip_problems = missing_current.len() + new_current.len();

        if chip_problems != 0 {
            // Collect results for the final report
            for symbol in missing_current {
                all_missing_symbols.push((chip_name_str.clone(), symbol));
            }
            for symbol in new_current {
                all_new_symbols.push((chip_name_str.clone(), symbol));
            }

            total_problems += chip_problems;
        }
    }

    if total_problems > 0 {
        if !all_missing_symbols.is_empty() {
            println!(
                "MISSING SYMBOLS (ABI BREAK) ({})",
                all_missing_symbols.len()
            );
            all_missing_symbols.sort_by(|a, b| a.0.cmp(&b.0));
            for (chip, symbol) in &all_missing_symbols {
                println!("  - Chip: {:<10} | Symbol: {}", chip, symbol);
            }
        }

        if !all_new_symbols.is_empty() {
            println!("NEW SYMBOLS ({})", all_new_symbols.len());
            all_new_symbols.sort_by(|a, b| a.0.cmp(&b.0));
            for (chip, symbol) in &all_new_symbols {
                println!("  - Chip: {:<10} | Symbol: {}", chip, symbol);
            }
        }

        Err(anyhow!(
            "Found {total_problems} symbol problems across all chips. Baselines were NOT updated."
        ))
    } else {
        println!("All checked symbols match the baselines.\n");
        Ok(())
    }
}

fn git_rev() -> Result<String> {
    let output = Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
        .context("Failed to run git rev-parse")?;

    if !output.status.success() {
        anyhow::bail!(
            "git command failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}
