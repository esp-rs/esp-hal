use std::path::Path;

use anyhow::{Context, Result};

use super::{build::build_examples, run::run_examples, select};
use crate::{Package, cargo::CargoAction, metadata::Chip};

const EXAMPLE_ARGUMENT_HINT: &str =
    "the example name as a token, or `all` to act on every example of the package";

/// Execute the given action on the specified examples.
pub fn examples(
    workspace: &Path,
    package: Package,
    chip: Chip,
    example: Option<&str>,
    action: CargoAction,
    debug: bool,
    toolchain: Option<&str>,
    timings: bool,
) -> Result<()> {
    log::debug!("Running examples for '{package}' on '{chip:?}'");

    package
        .validate_package_chip(&chip)
        .with_context(|| format!("The package '{package}' does not support the chip '{chip:?}'"))?;

    let package_path = crate::windows_safe_path(&workspace.join(package.directory()));

    // Load all examples which support the specified chip and parse their metadata.
    //
    // Directories might contain a number of individual projects, and don't not rely on
    // metadata comments in the source files. As such, it needs to load its metadata differently
    // than other packages.
    let examples = if package.contains_standalone_projects() {
        crate::firmware::load_cargo_toml(&package_path).with_context(|| {
            format!(
                "Failed to load specified examples from {}",
                package_path.display()
            )
        })?
    } else {
        let example_path = match package {
            Package::QaTest => package_path.join("src").join("bin"),
            _ => package_path.join("examples"),
        };

        crate::firmware::load(&example_path)?
    };

    let mut examples = examples
        .into_iter()
        .filter(|example| example.supports_chip(chip))
        .collect::<Vec<_>>();

    examples.sort_by_key(|a| a.binary_name());

    let mut filtered = vec![];

    if let Some(example) = example {
        filtered.clone_from(&examples);
        if !example.eq_ignore_ascii_case("all") {
            filtered.retain(|ex| ex.matches_name(example));

            if filtered.is_empty() {
                log::warn!(
                    "Example '{example}' not found or unsupported for the given chip. Please select one of the existing examples in the desired package."
                );

                let example_name = select(
                    "Select the example:",
                    examples
                        .iter()
                        .map(|ex| ex.lookup_name(&package_path))
                        .collect(),
                    EXAMPLE_ARGUMENT_HINT,
                )?;

                if let Some(selected) = examples.iter().find(|ex| ex.matches_name(&example_name)) {
                    filtered.push(selected.clone());
                }
            }
        }
    } else {
        let example_name = select(
            "Select an example:",
            examples
                .iter()
                .map(|ex| ex.lookup_name(&package_path))
                .collect(),
            EXAMPLE_ARGUMENT_HINT,
        )?;

        if let Some(selected) = examples.iter().find(|ex| ex.matches_name(&example_name)) {
            filtered.push(selected.clone());
        }
    }

    match action {
        CargoAction::Build(out_path) => build_examples(
            package,
            chip,
            debug,
            toolchain,
            timings,
            filtered,
            &package_path,
            out_path.as_deref(),
        ),
        CargoAction::Run => run_examples(
            package,
            chip,
            debug,
            toolchain,
            timings,
            filtered,
            &package_path,
        ),
    }
}
