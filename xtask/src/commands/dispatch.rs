//! Dispatch `build` / `run` / `check` / `test` after token resolution.

use std::{io::IsTerminal as _, path::Path};

use anyhow::{Result, bail};
use clap::Args;
use inquire::Select;
use strum::IntoEnumIterator as _;

use super::{check::CheckPackagesArgs, examples::examples, tests::tests};
use crate::{
    Package,
    cargo::CargoAction,
    firmware,
    metadata::Chip,
    resolve::{ResolveInput, resolve},
};

/// The verb being dispatched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    Build,
    Run,
    Check,
    Test,
}

/// Flags and free-form tokens shared by `build` / `run` / `check` / `test`.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(verbs(
        build = "Build examples or HIL tests. Tokens: chip, example, test, or package alias (examples, tests, qa), in any order. Crates are not built, use `check` for those. Always pass the chip, building detects no device.",
        run = "Flash and run an example or qa binary. Tokens: chip, example name, or package alias (examples, qa). Not for HIL tests — use `test`. Chip is inferred from a connected device when omitted.",
        check = "Check crates with cargo check, or try-build examples and tests. Tokens: chip, crate, example, test, or package alias. No tokens checks all published crates on all chips. Pass the chip when naming an example or test, checking detects no device.",
        test = "Run HIL tests only. Tokens: chip and/or test name. Not for examples — use `run`. Chip is inferred from probe-rs when omitted.",
    ))
)]
#[derive(Debug, Args)]
pub struct DispatchArgs {
    /// Package(s) to act on.
    #[arg(long, alias = "packages", value_enum, value_delimiter = ',')]
    pub package: Vec<Package>,

    /// Build examples in debug mode.
    #[arg(long)]
    pub debug: bool,

    /// Toolchain used to build.
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Emit crate build timings.
    #[arg(long)]
    pub timings: bool,

    /// Repeat HIL tests this many times. Used by `test`, ignored by `run`.
    #[arg(long, default_value_t = 1)]
    pub repeat: usize,

    /// HIL test selector(s). Also accepted as free-form tokens. Used by `test` / `build` /
    /// `check`, `run --test` is an error.
    #[arg(long, short = 't', alias = "tests", value_delimiter = ',', num_args = 1..)]
    pub test: Option<Vec<String>>,

    /// Chip, crate, example, or test names, in any order.
    pub tokens: Vec<String>,
}

/// Resolve tokens and run the matching existing action.
pub fn dispatch(
    workspace: &Path,
    verb: Verb,
    args: DispatchArgs,
    check_libs: impl FnOnce(CheckPackagesArgs) -> Result<()>,
) -> Result<()> {
    let mut input = ResolveInput::from_tokens(args.tokens.iter().cloned());
    input.packages = args.package.clone();

    let mut resolution = resolve(input);
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

    for name in &mut resolution.names {
        *name = expand_name(workspace, verb, &resolution.packages, name)?;
    }

    if resolution.packages.is_empty() {
        return dispatch_defaults(workspace, verb, &resolution, &args, check_libs);
    }

    let mut lib_packages = Vec::new();

    for package in &resolution.packages {
        // `esp-lp-hal` holds examples but is a crate as well, so checking it without naming an
        // example checks the crate.
        if verb == Verb::Check && resolution.names.is_empty() && package.is_published() {
            lib_packages.push(*package);
        } else if is_test_package(*package) {
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
        } else {
            bail!(
                "Cannot {} crate '{package}'. Pass an example or test name, or use `check`.",
                format!("{verb:?}").to_lowercase()
            );
        }
    }

    if !lib_packages.is_empty() {
        check_libs(CheckPackagesArgs {
            packages: lib_packages,
            chips: chips_or_all(&resolution.chips)?,
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
            chips: chips_or_all(&resolution.chips)?,
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

/// The chips to check crates for: the ones that were named, the connected one, or all of them.
fn chips_or_all(chips: &[Chip]) -> Result<Vec<Chip>> {
    if !chips.is_empty() {
        Ok(chips.to_vec())
    } else if let Some(chip) = connected_chip()? {
        Ok(vec![chip])
    } else {
        Ok(Chip::iter().collect())
    }
}

/// The connected chip, for `check`, which drives none itself. Only looked for from a terminal:
/// scripts and CI jobs keep checking every chip.
fn connected_chip() -> Result<Option<Chip>> {
    if !std::io::stdin().is_terminal() {
        return Ok(None);
    }
    if let Some(chip) = crate::detect::with_probe_rs()? {
        return Ok(Some(chip));
    }
    crate::detect::with_espflash()
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

/// The packages that hold examples or tests.
const FIRMWARE_PACKAGES: [Package; 4] = [
    Package::Examples,
    Package::QaTest,
    Package::HilTest,
    Package::HilTestRadio,
];

/// Decide which package owns `name`: a standalone example, a qa bin, or a HIL test.
/// Unknown names default to [`Package::Examples`].
fn source_package(workspace: &Path, name: &str) -> Package {
    if name.eq_ignore_ascii_case("all") {
        return Package::Examples;
    }

    FIRMWARE_PACKAGES
        .into_iter()
        .find(|package| {
            firmware::load_package(workspace, *package)
                .is_ok_and(|firmware| firmware.iter().any(|meta| meta.matches_name(name)))
        })
        .unwrap_or(Package::Examples)
}

/// Complete a partial name, so `sdmmc` becomes `sdmmc_sd_async`.
///
/// `-` and `_` are interchangeable, so `sleep_timer` finds `sleep-timer`. A name that fits several
/// binaries, or none, gets a selector in a terminal, with the name typed into its filter.
fn expand_name(workspace: &Path, verb: Verb, packages: &[Package], name: &str) -> Result<String> {
    if name.eq_ignore_ascii_case("all") || name.contains("::") {
        return Ok(name.to_owned());
    }

    // Look in the packages that were named, or in the ones this verb can act on.
    let searched: Vec<Package> = if !packages.is_empty() {
        packages.to_vec()
    } else {
        FIRMWARE_PACKAGES
            .into_iter()
            .filter(|package| match verb {
                Verb::Run => is_example_package(*package),
                Verb::Test => is_test_package(*package),
                Verb::Build | Verb::Check => true,
            })
            .collect()
    };

    let mut candidates: Vec<String> = Vec::new();

    for package in searched {
        let Ok(firmware) = firmware::load_package(workspace, package) else {
            continue;
        };
        if firmware.iter().any(|meta| meta.matches_name(name)) {
            return Ok(name.to_owned());
        }
        // One entry per supported chip, so the same name turns up more than once.
        for candidate in firmware.iter().map(|meta| meta.binary_name()) {
            if !candidates.contains(&candidate) {
                candidates.push(candidate);
            }
        }
    }

    let wanted = normalize_name(name);
    let fits: Vec<&String> = match candidates
        .iter()
        .find(|candidate| normalize_name(candidate) == wanted)
    {
        // `spi` names a test of its own, so it is not the `spi_slave` a fragment would find.
        Some(named) => vec![named],
        None => candidates
            .iter()
            .filter(|candidate| normalize_name(candidate).contains(&wanted))
            .collect(),
    };

    if let [fit] = fits[..] {
        log::info!("'{name}' selected '{fit}'");
        return Ok(fit.clone());
    }

    if !std::io::stdin().is_terminal() {
        if fits.is_empty() {
            // Nothing fits, so leave the name alone and let the package have its say about it.
            return Ok(name.to_owned());
        }
        let fits = fits.iter().map(|fit| fit.as_str()).collect::<Vec<_>>();
        bail!("'{name}' fits several: {}", fits.join(", "));
    }

    let nothing_fits = fits.is_empty();
    let message = if nothing_fits {
        format!("Nothing is called '{name}', select one:")
    } else {
        format!("'{name}' fits several, select one:")
    };
    candidates.sort();

    // Not `select`: the filter narrows by the same rule the fits above do, and starts out with the
    // name in it, unless that name is one that narrows to nothing.
    let mut prompt = Select::new(&message, candidates).with_scorer(&|input, _, candidate, _| {
        normalize_name(candidate)
            .contains(&normalize_name(input))
            .then_some(0)
    });
    if !nothing_fits {
        prompt = prompt.with_starting_filter_input(name);
    }
    Ok(prompt.prompt()?)
}

/// Names that differ only in how they separate words are the same name.
fn normalize_name(name: &str) -> String {
    name.to_lowercase().replace('-', "_")
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
        Verb::Build => CargoAction::Build(Some(workspace.join("target").join("tests"))),
        // A check only has to compile, collecting ELFs would clobber what `build` left there.
        Verb::Check => CargoAction::Build(None),
        Verb::Test => CargoAction::Run,
        Verb::Run => unreachable!("`run` does not dispatch HIL tests, use `test` instead"),
    };
    // `tests` builds every test when given no selector, which is what `all` asks for.
    let test = if names.is_empty() || names.iter().any(|name| name.eq_ignore_ascii_case("all")) {
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

    // The verbs that drive a device detect it with the tool they are about to use. `build` never
    // touches hardware, so it asks instead of poking every serial port.
    let inferred = match verb {
        Verb::Run => crate::detect::with_espflash()?,
        Verb::Test => crate::detect::with_probe_rs()?,
        Verb::Check => connected_chip()?,
        Verb::Build => None,
    };
    if let Some(chip) = inferred {
        return Ok(vec![chip]);
    }

    Ok(vec![super::select(
        "Select your target chip:",
        Chip::iter().collect(),
        "a chip name",
    )?])
}
