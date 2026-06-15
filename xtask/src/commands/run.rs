use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context as _, Result, bail};
use clap::{Args, Subcommand};
use esp_metadata::Chip;
use serde::Deserialize;

use super::{DocTestArgs, ExamplesArgs, TestsArgs};
use crate::{
    Package,
    cargo::{CargoAction, CargoArgsBuilder},
    firmware::{self, Metadata},
    radio_hil_runner::run_radio_test_elf,
};

// ----------------------------------------------------------------------------
// Subcommands

#[derive(Debug, Subcommand)]
pub enum Run {
    /// Run doctests for specified chip and package.
    DocTests(DocTestArgs),
    /// Run all ELFs in a folder.
    Elfs(RunElfsArgs),
    /// Run the given example for the specified chip.
    Example(ExamplesArgs),
    /// Run all applicable tests or the specified test for a specified chip.
    Tests(TestsArgs),
}

// ----------------------------------------------------------------------------
// Subcommand Arguments

/// Arguments for running ELFs.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Run all ELFs in a folder using probe-rs. Use --elfs binary or binary::test_name to filter which ELFs/tests run.",
        command = "run elfs"
    )
)]
#[derive(Debug, Args)]
pub struct RunElfsArgs {
    /// Which chip to run the tests for.
    #[arg(value_enum)]
    pub chip: Chip,
    /// Path to the ELFs.
    pub path: PathBuf,
    /// Optional list of ELFs to execute. Use `binary::test_name` to run a single
    /// embedded-test case inside a matching ELF.
    #[arg(long, value_delimiter = ',')]
    pub elfs: Vec<String>,
}

// ----------------------------------------------------------------------------
// Subcommand Actions

/// Run doc tests for the specified package and chip.
pub fn run_doc_tests(workspace: &Path, args: DocTestArgs) -> Result<()> {
    let mut success = true;
    for package in args.packages {
        success &= run_doc_tests_for_package(workspace, package, args.chip)?;
    }
    anyhow::ensure!(success, "One or more doc tests failed");
    Ok(())
}

pub fn run_doc_tests_for_package(workspace: &Path, package: Package, chip: Chip) -> Result<bool> {
    log::info!("Running doc tests for '{}' on '{}'", package, chip);

    // FIXME: this list can and should slowly be expanded, and eventually the check be removed as
    // the docs are fixed up.
    let temporary_package_list = [
        Package::EspHal,
        Package::EspRadio,
        Package::EspBootloaderEspIdf,
    ];
    if !temporary_package_list.contains(&package) {
        log::info!("Package {} is temporarily not doctested", package);
        return Ok(true);
    }

    // Ensure that the package/chip combination provided are valid:
    if let Err(err) = package.validate_package_chip(&chip) {
        log::warn!("{err}");
        return Ok(true);
    }

    // Packages that have doc features are documented. We run doc-tests for these, and only these.
    let Some(mut doc_config) = package.doc_config_rules(&esp_metadata::Config::for_chip(&chip))
    else {
        log::info!("Skipping undocumented package {package}.");
        return Ok(true);
    };

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    if package.has_chip_features() {
        doc_config.features.push(chip.to_string());
    }
    let features = &doc_config.features;

    // Determine the appropriate build target, and cargo features for the given
    // package and chip:
    let Ok(target) = package.target_triple(&chip) else {
        log::warn!("Package {package} is not applicable for {chip}");
        return Ok(true);
    };

    // We need `nightly` for building the doc tests, unfortunately:
    let toolchain = if chip.is_xtensa() { "esp" } else { "nightly" };

    // Build up an array of command-line arguments to pass to `cargo`:
    let mut builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("test")
        .arg("--doc")
        .arg("-Zbuild-std=core,alloc,panic_abort")
        .target(target)
        .features(features)
        .arg("--release");

    for (key, value) in &doc_config.env {
        builder.add_env_var(key, value);
    }
    builder.add_env_var("RUSTDOCFLAGS", "--cfg docsrs --cfg not_really_docsrs");
    builder.add_env_var("ESP_HAL_DOCTEST", "1");

    let command = crate::cargo::CargoCommandBatcher::build_one_for_cargo(&builder);
    log::debug!("{command:#?}");

    let success =
        crate::cargo::run_with_env(&command.command, &package_path, command.env_vars, false)
            .is_ok();
    Ok(success)
}

/// Run all (or filtered) ELFs in the specified folder using `probe-rs`.
pub fn run_elfs(args: RunElfsArgs) -> Result<()> {
    let mut failed: Vec<String> = Vec::new();
    let filters = parse_elf_filters(&args.elfs)?;
    let mut filter_matched = vec![false; filters.len()];

    let mut elfs = Vec::<(String, PathBuf)>::new();
    for entry in fs::read_dir(&args.path)
        .with_context(|| format!("Failed to read {}", args.path.display()))?
    {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        if path
            .file_name()
            .is_some_and(|name| name == std::ffi::OsStr::new(RADIO_ELF_MANIFEST_FILENAME))
        {
            continue;
        }

        let elf_name = path
            .with_extension("")
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string();
        elfs.push((elf_name, path));
    }

    let radio_manifest = load_radio_test_manifest(&args.path)?;
    let radio_tests = if radio_manifest.is_some() {
        Vec::new()
    } else {
        load_radio_tests_for_chip(args.chip)?
    };

    for (elf_name, elf_path) in &elfs {
        let elf_name = elf_name.clone();
        let elf_path = elf_path.clone();

        // Match the ELF against the requested selectors. The part before `::`
        // selects the binary, the part after is forwarded to the device as an
        // embedded-test filter (first matching selector wins).
        let elf_name_lower = elf_name.to_lowercase();
        let mut matched = false;
        let mut test_filter = None;
        for (filter, used) in filters.iter().zip(filter_matched.iter_mut()) {
            if elf_name_lower.contains(&filter.binary) {
                *used = true;
                matched = true;
                test_filter = test_filter.or(filter.test_filter.as_deref());
            }
        }

        if !filters.is_empty() && !matched {
            log::info!(
                "Skipping test '{}' for '{}' (does not match filters: {:?})",
                elf_name,
                args.chip,
                filters.iter().map(|f| f.raw.as_str()).collect::<Vec<_>>(),
            );
            continue;
        }

        log::info!("Running test '{}' for '{}'", elf_name, args.chip);

        if let Some(entry) = radio_manifest.as_ref().and_then(|m| m.get(&elf_name)) {
            if entry.support_firmware {
                log::info!("Skipping support firmware '{}'", elf_name);
                continue;
            }

            let harness_path = entry
                .harness
                .as_deref()
                .and_then(|name| resolve_harness_binary_path(&elfs, name));

            if entry.harness.is_some() && harness_path.is_none() {
                failed.push(elf_name.clone());
                log::error!(
                    "Radio test '{}' requires harness firmware '{}' but no matching ELF was found",
                    elf_name,
                    entry.harness.as_deref().unwrap_or_default(),
                );
                continue;
            }

            if let Err(e) =
                run_radio_test_elf(&elf_path, harness_path.as_deref(), 120, None, test_filter)
            {
                failed.push(elf_name.clone());
                log::error!("Radio test '{}' failed: {}", elf_name, e);
            } else {
                log::info!("'{}' passed", elf_name);
            }
            continue;
        }

        if let Some(meta) = firmware::find_test_by_name(&radio_tests, &elf_name) {
            if meta.is_support_firmware() {
                log::info!("Skipping support firmware '{}' (metadata)", elf_name);
                continue;
            }

            let harness_path = meta
                .harness_firmware()
                .and_then(|name| resolve_harness_binary_path(&elfs, name));

            if meta.harness_firmware().is_some() && harness_path.is_none() {
                failed.push(elf_name.clone());
                log::error!(
                    "Radio test '{}' requires harness firmware '{}' but no matching ELF was found",
                    elf_name,
                    meta.harness_firmware().unwrap_or_default(),
                );
                continue;
            }

            if let Err(e) =
                run_radio_test_elf(&elf_path, harness_path.as_deref(), 120, None, test_filter)
            {
                failed.push(elf_name.clone());
                log::error!("Radio test '{}' failed: {}", elf_name, e);
            } else {
                log::info!("'{}' passed", elf_name);
            }
            continue;
        }

        // Use standard probe-rs runner for non-radio tests.
        let mut command = Command::new("probe-rs");
        command.arg("run").arg(&elf_path).arg("--verify");
        if let Some(test_filter) = test_filter {
            command.arg(test_filter);
        }

        let status = command.status().context("Failed to execute probe-rs")?;

        log::info!("'{}' done", elf_name);

        if !status.success() {
            failed.push(elf_name);
        }
    }

    let unmatched = filters
        .iter()
        .zip(&filter_matched)
        .filter(|(_, matched)| !**matched)
        .map(|(filter, _)| filter.raw.as_str())
        .collect::<Vec<_>>();
    if !unmatched.is_empty() {
        bail!(
            "ELF selector did not match any artifact: {}",
            unmatched.join(", ")
        );
    }

    if !failed.is_empty() {
        bail!("Failed tests: {:?}", failed);
    }

    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ElfFilter {
    raw: String,
    binary: String,
    test_filter: Option<String>,
}

fn parse_elf_filters(raw_filters: &[String]) -> Result<Vec<ElfFilter>> {
    let mut filters = Vec::new();
    for raw in raw_filters {
        let raw = raw.trim();
        if raw.is_empty() {
            continue;
        }

        let (binary, test_filter) = match raw.split_once("::") {
            Some((binary, test_filter)) => {
                let test_filter = test_filter.trim();
                if test_filter.is_empty() {
                    bail!("Empty test filter in ELF selector '{raw}' is not allowed");
                }
                (binary, Some(test_filter.to_string()))
            }
            None => (raw, None),
        };

        let binary = binary.trim().to_lowercase();
        if binary.is_empty() {
            bail!("Empty binary filter in ELF selector '{raw}' is not allowed");
        }

        filters.push(ElfFilter {
            raw: raw.to_string(),
            binary,
            test_filter,
        });
    }
    Ok(filters)
}

fn load_radio_tests_for_chip(chip: Chip) -> Result<Vec<Metadata>> {
    let radio_path = Path::new("hil-test-radio/src/bin");
    if !radio_path.exists() {
        return Ok(Vec::new());
    }

    let mut tests = firmware::load(radio_path)?;
    for subdir in ["tests", "support"] {
        let path = radio_path.join(subdir);
        if path.exists() {
            tests.extend(firmware::load(&path)?);
        }
    }

    let tests = tests
        .into_iter()
        .filter(|test| test.supports_chip(chip))
        .collect();
    Ok(tests)
}

const RADIO_ELF_MANIFEST_FILENAME: &str = "radio-elfs.json";

#[derive(Debug, Deserialize)]
struct RadioElfManifestEntry {
    artifact: String,
    harness: Option<String>,
    support_firmware: bool,
}

fn load_radio_test_manifest(path: &Path) -> Result<Option<HashMap<String, RadioElfManifestEntry>>> {
    let manifest_path = path.join(RADIO_ELF_MANIFEST_FILENAME);
    if !manifest_path.exists() {
        return Ok(None);
    }

    let content = fs::read_to_string(&manifest_path)
        .with_context(|| format!("Failed to read {}", manifest_path.display()))?;
    let entries: Vec<RadioElfManifestEntry> = serde_json::from_str(&content)
        .with_context(|| format!("Failed to parse {}", manifest_path.display()))?;

    let by_artifact = entries
        .into_iter()
        .map(|entry| (entry.artifact.clone(), entry))
        .collect::<HashMap<_, _>>();
    Ok(Some(by_artifact))
}

fn resolve_harness_binary_path(elfs: &[(String, PathBuf)], harness_name: &str) -> Option<PathBuf> {
    if let Some((_, path)) = elfs.iter().find(|(name, _)| name == harness_name) {
        return Some(path.clone());
    }

    let prefix = format!("{harness_name}_");
    elfs.iter()
        .find(|(name, _)| name.starts_with(&prefix))
        .map(|(_, path)| path.clone())
}

/// Run the specified examples for the given chip.
pub fn run_examples(
    args: ExamplesArgs,
    examples: Vec<Metadata>,
    package_path: &Path,
) -> Result<()> {
    let mut examples = examples;

    // At this point, chip can never be `None`, so we can safely unwrap it.
    let chip = args.chip.unwrap();
    let target = args.package.as_package().target_triple(&chip)?;

    examples.sort_by_key(|ex| ex.tag());

    let console = console::Term::stdout();
    let interactive = console.is_term();

    for example in examples {
        let mut skip = false;

        log::info!("Running example '{}'", example.output_file_name());

        if interactive {
            if let Some(description) = example.description() {
                log::info!(
                    "\n\n{}\n\nPress ENTER to run example, `s` to skip",
                    description.trim()
                );
            } else {
                log::info!("\n\nPress ENTER to run example, `s` to skip");
            }

            loop {
                let key = console.read_key();

                match key {
                    Ok(console::Key::Enter) => break,
                    Ok(console::Key::Char('s')) => {
                        skip = true;
                        break;
                    }
                    _ => (),
                }
            }
        }

        while !skip {
            let result = crate::execute_app(
                package_path,
                chip,
                &target,
                &example,
                CargoAction::Run,
                args.debug,
                args.toolchain.as_deref(),
                args.timings,
                &[],
            );

            if let Err(error) = result {
                log::error!("Failed to run example: {}", error);
                if interactive {
                    log::info!("Retry or skip? (r/s)");
                    loop {
                        let key = console.read_key();

                        match key {
                            Ok(console::Key::Char('r')) => break,
                            Ok(console::Key::Char('s')) => {
                                skip = true;
                                break;
                            }
                            _ => (),
                        }
                    }
                } else {
                    return Err(error);
                }
            } else {
                break;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_string()).collect()
    }

    #[test]
    fn parses_binary_filter() {
        let filters = parse_elf_filters(&strings(&[" misc_non_drivers "])).unwrap();

        assert_eq!(
            filters,
            vec![ElfFilter {
                raw: "misc_non_drivers".to_string(),
                binary: "misc_non_drivers".to_string(),
                test_filter: None,
            }]
        );
    }

    #[test]
    fn parses_binary_test_filter() {
        let filters = parse_elf_filters(&strings(&["radio_basic::test_scan_doesnt_leak"])).unwrap();

        assert_eq!(
            filters,
            vec![ElfFilter {
                raw: "radio_basic::test_scan_doesnt_leak".to_string(),
                binary: "radio_basic".to_string(),
                test_filter: Some("test_scan_doesnt_leak".to_string()),
            }]
        );
    }

    #[test]
    fn rejects_empty_binary_or_test_filters() {
        assert!(parse_elf_filters(&strings(&["::test_name"])).is_err());
        assert!(parse_elf_filters(&strings(&["misc_non_drivers::"])).is_err());
    }
}
