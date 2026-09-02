//! Dispatch `build` / `run` / `check` / `test` after token resolution.

use std::path::Path;

use anyhow::{Result, bail};
use strum::IntoEnumIterator as _;

use super::{CheckPackagesArgs, DispatchArgs, examples, tests};
use crate::{
    Package,
    cargo::CargoAction,
    firmware,
    metadata::Chip,
    resolve::{ResolveInput, Verb, resolve},
};

/// Resolve tokens and run the matching existing action.
pub fn dispatch(
    workspace: &Path,
    verb: Verb,
    args: DispatchArgs,
    check_libs: impl FnOnce(CheckPackagesArgs) -> Result<()>,
) -> Result<()> {
    let mut input = ResolveInput::from_tokens(args.tokens.iter().cloned());
    input.chips = args.chip.clone();
    input.packages = args.package.clone();
    input.infer_chip = true;

    let mut resolution = resolve(verb, input);
    match verb {
        Verb::Run => {
            if args.test.is_some() {
                bail!("HIL tests use `test`, not `run`.");
            }
            for package in &resolution.packages {
                if !is_example_package(*package) {
                    bail!("`run` only flashes examples or qa, use `test` for HIL tests");
                }
            }
            for name in &resolution.names {
                if is_test_package(source_package(workspace, name)) {
                    log::error!(
                        "`{name}` is a HIL test. You probably want to use `test` instead of `run`."
                    );
                    std::process::exit(1);
                }
            }
        }
        Verb::Test => {
            for package in &resolution.packages {
                if !is_test_package(*package) {
                    bail!("`test` only runs HIL tests. Use `run` for examples.");
                }
            }
        }
        _ => {}
    }
    if let Some(tests) = &args.test {
        for name in tests {
            if !resolution.names.iter().any(|existing| existing == name) {
                resolution.names.push(name.clone());
            }
        }
    }

    if resolution.packages.is_empty() {
        return dispatch_defaults(workspace, verb, &resolution, &args, check_libs);
    }

    let mut lib_packages = Vec::new();

    for package in &resolution.packages {
        if is_test_package(*package) {
            dispatch_tests(
                workspace,
                verb,
                &resolution.chips,
                &resolution.names,
                *package,
                &args,
            )?;
        } else if is_example_package(*package) {
            dispatch_examples(
                workspace,
                verb,
                &resolution.chips,
                &resolution.names,
                *package,
                &args,
            )?;
        } else if verb == Verb::Check {
            lib_packages.push(*package);
        } else if verb == Verb::Build {
            crate::commands::build_package(workspace, *package, args.toolchain.as_deref())?;
        } else {
            bail!(
                "Cannot {verb:?} crate '{package}'. Pass an example or test name, or use `check`."
            );
        }
    }

    if !lib_packages.is_empty() {
        check_libs(CheckPackagesArgs {
            packages: lib_packages,
            chips: chips_or_all(&resolution.chips),
            toolchain: args.toolchain.clone(),
        })?;
    }

    Ok(())
}

fn dispatch_defaults(
    workspace: &Path,
    verb: Verb,
    resolution: &crate::resolve::Resolution,
    args: &DispatchArgs,
    check_libs: impl FnOnce(CheckPackagesArgs) -> Result<()>,
) -> Result<()> {
    match verb {
        Verb::Test => dispatch_tests(
            workspace,
            verb,
            &resolution.chips,
            &resolution.names,
            Package::HilTest,
            args,
        ),
        Verb::Check if resolution.names.is_empty() => check_libs(CheckPackagesArgs {
            packages: Package::iter().collect(),
            chips: chips_or_all(&resolution.chips),
            toolchain: args.toolchain.clone(),
        }),
        Verb::Check | Verb::Build | Verb::Run => {
            if resolution.names.is_empty() {
                return dispatch_examples(
                    workspace,
                    verb,
                    &resolution.chips,
                    &resolution.names,
                    Package::Examples,
                    args,
                );
            }
            for name in &resolution.names {
                let package = source_package(workspace, name);
                if is_test_package(package) {
                    dispatch_tests(
                        workspace,
                        verb,
                        &resolution.chips,
                        std::slice::from_ref(name),
                        package,
                        args,
                    )?;
                } else {
                    dispatch_examples(
                        workspace,
                        verb,
                        &resolution.chips,
                        std::slice::from_ref(name),
                        package,
                        args,
                    )?;
                }
            }
            Ok(())
        }
    }
}

fn chips_or_all(chips: &[Chip]) -> Vec<Chip> {
    if chips.is_empty() {
        Chip::iter().collect()
    } else {
        chips.to_vec()
    }
}

fn is_example_package(package: Package) -> bool {
    matches!(
        package,
        Package::Examples | Package::QaTest | Package::EspLpHal | Package::CompileTests
    )
}

fn is_test_package(package: Package) -> bool {
    matches!(package, Package::HilTest | Package::HilTestRadio)
}

/// Decide which package owns `name`: a standalone example, a qa bin, or a HIL test.
/// Unknown names default to [`Package::Examples`].
fn source_package(workspace: &Path, name: &str) -> Package {
    if name.eq_ignore_ascii_case("all") {
        return Package::Examples;
    }

    let examples = firmware::load_cargo_toml(&workspace.join(Package::Examples.to_string()));
    if examples.is_ok_and(|examples| examples.iter().any(|example| example.matches_name(name))) {
        return Package::Examples;
    }

    let qa_bins = workspace
        .join(Package::QaTest.directory())
        .join("src")
        .join("bin");
    if firmware::load(&qa_bins).is_ok_and(|qa| qa.iter().any(|example| example.matches_name(name)))
    {
        return Package::QaTest;
    }

    let hil = workspace
        .join(Package::HilTest.directory())
        .join("src")
        .join("bin");
    if firmware::load(&hil).is_ok_and(|tests| tests.iter().any(|test| test.matches_name(name))) {
        return Package::HilTest;
    }

    let radio = workspace
        .join(Package::HilTestRadio.directory())
        .join("src")
        .join("bin");
    for path in [&radio, &radio.join("tests"), &radio.join("support")] {
        if path.exists()
            && firmware::load(path)
                .is_ok_and(|tests| tests.iter().any(|test| test.matches_name(name)))
        {
            return Package::HilTestRadio;
        }
    }

    Package::Examples
}

fn dispatch_examples(
    workspace: &Path,
    verb: Verb,
    chips: &[Chip],
    names: &[String],
    package: Package,
    args: &DispatchArgs,
) -> Result<()> {
    let action = match verb {
        Verb::Run => CargoAction::Run,
        Verb::Build | Verb::Check => CargoAction::Build(None),
        Verb::Test => unreachable!("`test` does not dispatch examples, use `run` instead"),
    };
    let names: Vec<Option<String>> = if names.is_empty() {
        vec![None]
    } else {
        names.iter().cloned().map(Some).collect()
    };

    for chip in required_chips(verb, chips)? {
        for name in &names {
            examples(
                workspace,
                package,
                chip,
                name.as_deref(),
                action.clone(),
                args.debug,
                args.toolchain.as_deref(),
                args.timings,
            )?;
        }
    }
    Ok(())
}

fn dispatch_tests(
    workspace: &Path,
    verb: Verb,
    chips: &[Chip],
    names: &[String],
    package: Package,
    args: &DispatchArgs,
) -> Result<()> {
    let action = match verb {
        Verb::Build | Verb::Check => {
            CargoAction::Build(Some(workspace.join("target").join("tests")))
        }
        Verb::Test => CargoAction::Run,
        Verb::Run => unreachable!("`run` does not dispatch HIL tests, use `test` instead"),
    };
    let test = if names.is_empty() {
        None
    } else {
        Some(names.to_vec())
    };

    for chip in required_chips(verb, chips)? {
        tests(
            workspace,
            package,
            chip,
            test.as_deref(),
            action.clone(),
            args.repeat,
            args.toolchain.as_deref(),
            args.timings,
        )?;
    }
    Ok(())
}

fn required_chips(verb: Verb, chips: &[Chip]) -> Result<Vec<Chip>> {
    if !chips.is_empty() {
        return Ok(chips.to_vec());
    }

    let inferred = if verb == Verb::Run {
        crate::detect::with_espflash()
    } else {
        crate::detect::with_probe_rs()
    };
    if let Some(chip) = inferred {
        return Ok(vec![chip]);
    }

    Ok(vec![super::select(
        "Select your target chip:",
        Chip::iter().collect(),
        "`--chip <CHIP>`",
    )?])
}
