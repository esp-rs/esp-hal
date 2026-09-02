use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail};
use serde::Serialize;

use super::move_artifacts;
use crate::{
    Package,
    cargo::{CargoAction, CargoCommandBatcher},
    firmware,
    metadata::Chip,
};

/// Execute the given action on the specified HIL tests.
pub fn tests(
    workspace: &Path,
    package: Package,
    chip: Chip,
    test: Option<&[String]>,
    action: CargoAction,
    repeat: usize,
    toolchain: Option<&str>,
    timings: bool,
) -> Result<()> {
    if !matches!(package, Package::HilTest | Package::HilTestRadio) {
        bail!("Unknown test package: {package}. Use 'hil-test' or 'hil-test-radio'");
    }

    let package_path = crate::windows_safe_path(&workspace.join(package.directory()));
    let is_radio_package = package == Package::HilTestRadio;

    // Determine the appropriate build target for the given package and chip:
    let target = Package::HilTest.target_triple(&chip)?;

    // Load all test metadata first so we can differentiate between:
    // - unknown test selectors (typos)
    // - valid tests that are unsupported for the selected chip.
    let bins_root = package_path.join("src").join("bin");
    let mut all_tests = crate::firmware::load(&bins_root)?;
    if is_radio_package {
        let tests_root = bins_root.join("tests");
        if tests_root.exists() {
            all_tests.extend(crate::firmware::load(&tests_root)?);
        }
        let support_root = bins_root.join("support");
        if support_root.exists() {
            all_tests.extend(crate::firmware::load(&support_root)?);
        }
    }
    all_tests.sort_by_key(|a| a.binary_name());
    let tests_for_chip = all_tests
        .iter()
        .filter(|example| example.supports_chip(chip))
        .collect::<Vec<_>>();

    // Radio tests reuse the normal HIL flow; the only difference is a pre-step
    // that builds and flashes the support firmware on a secondary probe.
    if let CargoAction::Build(Some(out_dir)) = &action {
        // Make sure the tmp directory has no garbage for us.
        let tmp_dir = out_dir.join("tmp");
        _ = std::fs::remove_dir_all(&tmp_dir);
        std::fs::create_dir_all(&tmp_dir).unwrap();
    }

    let mut commands = CargoCommandBatcher::new();
    let mut artifact_meta = HashMap::<String, firmware::Metadata>::new();
    // Execute the specified action:
    // If user sets some specific test(s)
    if let Some(test_arg) = test {
        let trimmed: Vec<&str> = test_arg.iter().map(|s| s.trim()).collect();

        // Reject `--test ""` / `--test " "`
        if trimmed.iter().all(|s| s.is_empty()) {
            bail!("Empty test selector is not allowed.");
        }

        let mut unknown_selected: Vec<String> = Vec::new();
        let mut unsupported_for_chip: Vec<String> = Vec::new();

        for selected in trimmed.into_iter().filter(|s| !s.is_empty()) {
            let (test_arg, filter) = match selected.split_once("::") {
                Some((test, filter)) => (Some(test), Some(filter)),
                None => (Some(selected), None),
            };

            let run_test_extra_args = if is_radio_package && action == CargoAction::Run {
                &[]
            } else {
                (action == CargoAction::Run)
                    .then(|| filter.as_ref().map(std::slice::from_ref))
                    .flatten()
                    .unwrap_or(&[])
            };

            let matched: Vec<_> = all_tests.iter().filter(|t| t.matches(test_arg)).collect();

            if matched.is_empty() {
                if is_radio_package
                    && let Some(radio_meta) = resolve_radio_module_alias(&all_tests, chip, selected)
                {
                    let command = crate::generate_build_command(
                        &package_path,
                        chip,
                        &target,
                        &radio_meta,
                        action.clone(),
                        false,
                        toolchain,
                        timings,
                        &[],
                    )?;
                    artifact_meta.insert(command.artifact_name.clone(), radio_meta);
                    commands.push(command);
                    continue;
                }
                unknown_selected.push(selected.to_string());
            } else {
                let matched_for_chip: Vec<_> = matched
                    .into_iter()
                    .filter(|t| t.supports_chip(chip))
                    .collect();

                if matched_for_chip.is_empty() {
                    unsupported_for_chip.push(selected.to_string());
                    continue;
                }

                for test in matched_for_chip {
                    let command = crate::generate_build_command(
                        &package_path,
                        chip,
                        &target,
                        test,
                        action.clone(),
                        false,
                        toolchain,
                        timings,
                        run_test_extra_args,
                    )?;
                    artifact_meta.insert(command.artifact_name.clone(), test.clone());
                    commands.push(command);
                }
            }
        }

        if !unsupported_for_chip.is_empty() {
            log::warn!(
                "Skipping unsupported tests for '{}': {}",
                chip,
                unsupported_for_chip.join(", ")
            );
        }

        if !unknown_selected.is_empty() {
            bail!("Test not found: {}", unknown_selected.join(", "))
        }
    } else {
        for test in tests_for_chip {
            let command = crate::generate_build_command(
                &package_path,
                chip,
                &target,
                test,
                action.clone(),
                false,
                toolchain,
                timings,
                &[],
            )?;
            artifact_meta.insert(command.artifact_name.clone(), test.clone());
            commands.push(command);
        }
    }

    let mut harness_for_artifact = HashMap::<String, firmware::Metadata>::new();
    if is_radio_package {
        for test in artifact_meta.values() {
            if is_support_firmware(test) {
                continue;
            }
            let Some(harness_name) = test.harness_firmware() else {
                continue;
            };

            let Some(harness_meta) = all_tests.iter().find(|candidate| {
                if !candidate.supports_chip(chip) {
                    return false;
                }

                let harness_basename = harness_name.rsplit('/').next().unwrap_or(harness_name);
                candidate.binary_name() == harness_name
                    || candidate.output_file_name() == harness_name
                    || candidate.binary_name() == harness_basename
                    || candidate.output_file_name() == harness_basename
            }) else {
                bail!(
                    "Missing harness firmware '{harness_name}' required by '{}'",
                    test.output_file_name()
                );
            };

            harness_for_artifact.insert(test.output_file_name(), harness_meta.clone());
        }
    }

    // Ensure required harness firmware is built so the artifact is
    // self-contained for `run elfs` on the HIL runner. Not needed on Run:
    // `build_radio_harness` builds harnesses lazily per test.
    if is_radio_package && matches!(action, CargoAction::Build(_)) {
        use std::collections::HashSet;

        let mut queued: HashSet<String> = artifact_meta
            .values()
            .map(firmware::Metadata::output_file_name)
            .collect();

        let mut harnesses_to_add: Vec<firmware::Metadata> = Vec::new();
        for harness in harness_for_artifact.values() {
            if queued.insert(harness.output_file_name()) {
                harnesses_to_add.push(harness.clone());
            }
        }

        for harness in harnesses_to_add {
            let command = crate::generate_build_command(
                &package_path,
                chip,
                &target,
                &harness,
                action.clone(),
                false,
                toolchain,
                timings,
                &[],
            )?;
            artifact_meta.insert(command.artifact_name.clone(), harness);
            commands.push(command);
        }
    }

    let mut failed = Vec::new();
    let built_commands = commands.build(false);
    let built_artifacts = artifact_meta.keys().cloned().collect::<Vec<_>>();

    let mut built_harness = HashMap::<String, PathBuf>::new();

    for c in built_commands {
        let repeat = if matches!(action, CargoAction::Run) {
            repeat
        } else {
            1
        };

        let radio_meta = artifact_meta.get(&c.artifact_name);

        // Support firmware never runs as a DUT; it's used only as a pre-step
        // for whichever DUT requests it via `HARNESS-FIRMWARE`.
        if matches!(action, CargoAction::Run) && radio_meta.is_some_and(is_support_firmware) {
            log::info!("Skipping support firmware '{}' as DUT", c.artifact_name);
            continue;
        }

        println!(
            "Command: cargo {}",
            c.command.join(" ").replace("---", "\n    ---")
        );

        for i in 0..repeat {
            if repeat != 1 {
                log::info!("Run {}/{}", i + 1, repeat);
            }

            if matches!(action, CargoAction::Run)
                && let Some(harness_meta) = harness_for_artifact.get(&c.artifact_name)
            {
                let harness_path = match build_radio_harness(
                    workspace,
                    &package_path,
                    chip,
                    &target,
                    toolchain,
                    timings,
                    harness_meta,
                    &mut built_harness,
                ) {
                    Ok(path) => path,
                    Err(e) => {
                        failed.push(c.artifact_name.clone());
                        log::error!(
                            "Failed to build harness firmware '{}' for '{}': {e}",
                            harness_meta.output_file_name(),
                            c.artifact_name
                        );
                        break;
                    }
                };

                let harness_spec = crate::radio_hil_runner::HarnessSpec {
                    binary_path: &harness_path,
                    target: &target,
                };
                if let Err(e) =
                    crate::radio_hil_runner::run_radio_test(&c, Some(harness_spec), None)
                {
                    failed.push(c.artifact_name.clone());
                    log::error!("Radio test '{}' failed: {}", c.artifact_name, e);
                    break;
                }
            } else if c.run(false).is_err() {
                failed.push(c.artifact_name.clone());
            }
        }
    }

    move_artifacts(chip, &action);

    if !failed.is_empty() {
        bail!("Failed tests: {:#?}", failed);
    }

    if is_radio_package && let CargoAction::Build(Some(out_dir)) = &action {
        write_radio_elf_manifest(
            out_dir,
            chip,
            &built_artifacts,
            &artifact_meta,
            &harness_for_artifact,
        )?;
    }

    Ok(())
}

const RADIO_ELF_MANIFEST_FILENAME: &str = "radio-elfs.json";

#[derive(Debug, Serialize)]
struct RadioElfManifestEntry {
    artifact: String,
    harness: Option<String>,
    support_firmware: bool,
}

fn write_radio_elf_manifest(
    out_dir: &Path,
    chip: Chip,
    built_artifacts: &[String],
    artifact_meta: &HashMap<String, firmware::Metadata>,
    harness_for_artifact: &HashMap<String, firmware::Metadata>,
) -> Result<()> {
    let mut entries = Vec::new();
    for artifact in built_artifacts {
        let Some(meta) = artifact_meta.get(artifact) else {
            continue;
        };
        entries.push(RadioElfManifestEntry {
            artifact: artifact.clone(),
            harness: harness_for_artifact
                .get(artifact)
                .map(firmware::Metadata::output_file_name),
            support_firmware: is_support_firmware(meta),
        });
    }

    if entries.is_empty() {
        return Ok(());
    }

    let manifest_path = out_dir
        .join(chip.to_string())
        .join(RADIO_ELF_MANIFEST_FILENAME);
    let payload = serde_json::to_string_pretty(&entries)?;
    std::fs::write(&manifest_path, payload)
        .with_context(|| format!("Failed to write {}", manifest_path.display()))?;
    Ok(())
}

/// Build the harness firmware required by a radio HIL test (if not already
/// built) and return the resulting ELF path. Results are cached in
/// `built_harness` so repeated tests sharing a harness only build it once.
fn build_radio_harness(
    workspace: &Path,
    package_path: &Path,
    chip: Chip,
    target: &str,
    toolchain: Option<&str>,
    timings: bool,
    harness: &firmware::Metadata,
    built_harness: &mut HashMap<String, PathBuf>,
) -> Result<PathBuf> {
    let artifact_name = harness.output_file_name();
    if let Some(path) = built_harness.get(&artifact_name) {
        return Ok(path.clone());
    }

    let builder = crate::generate_build_command(
        package_path,
        chip,
        target,
        harness,
        CargoAction::Build(None),
        false,
        toolchain,
        timings,
        &[],
    )?;
    let built = CargoCommandBatcher::build_one_for_cargo(&builder);
    log::info!(
        "Building harness firmware: cargo {}",
        built.command.join(" ")
    );
    built
        .run(false)
        .with_context(|| format!("Failed to build harness firmware '{artifact_name}'"))?;

    let path = crate::radio_hil_runner::resolve_release_binary(workspace, target, &artifact_name);
    if !path.exists() {
        bail!(
            "Harness firmware '{artifact_name}' built successfully but binary not found at {}",
            path.display()
        );
    }

    built_harness.insert(artifact_name, path.clone());
    Ok(path)
}

fn resolve_radio_module_alias(
    all_tests: &[firmware::Metadata],
    chip: Chip,
    selected: &str,
) -> Option<firmware::Metadata> {
    let module_file = format!("{selected}.rs");
    let module_decl = format!("mod {selected};");

    all_tests
        .iter()
        .find(|test| {
            test.supports_chip(chip)
                && test.harness_firmware().is_none()
                && std::fs::read_to_string(test.example_path()).is_ok_and(|source| {
                    source.contains(&module_file) || source.contains(&module_decl)
                })
        })
        .cloned()
}

fn is_support_firmware(meta: &firmware::Metadata) -> bool {
    meta.is_support_firmware()
}
