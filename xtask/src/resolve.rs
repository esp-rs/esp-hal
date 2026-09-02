//! Free-form token resolution for `build` / `run` / `check` / `test`.
//!
//! Tokens after those verbs can be a chip, a crate, an example or test name, or
//! a package alias (`example`, `tests`, `qa`), in any order. Wired via
//! [`crate::commands::dispatch`].

use std::str::FromStr;

use strum::IntoEnumIterator as _;

use crate::{Package, detect, metadata::Chip};

/// The verb that requested resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    Build,
    Run,
    Check,
    Test,
}

/// Tokens and any chips/packages already known (e.g. from flags).
#[derive(Debug, Clone, Default)]
pub struct ResolveInput {
    pub tokens: Vec<String>,
    pub chips: Vec<Chip>,
    pub packages: Vec<Package>,
    /// Infer a connected chip when none was given.
    pub infer_chip: bool,
}

impl ResolveInput {
    pub fn from_tokens(tokens: impl IntoIterator<Item = impl Into<String>>) -> Self {
        Self {
            tokens: tokens.into_iter().map(Into::into).collect(),
            ..Self::default()
        }
    }
}

/// Classified tokens.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolution {
    pub verb: Verb,
    pub chips: Vec<Chip>,
    pub packages: Vec<Package>,
    pub names: Vec<String>,
}

/// Classify `input.tokens`. Does not look up examples on disk.
pub fn resolve(verb: Verb, mut input: ResolveInput) -> Resolution {
    let mut chips = input.chips;
    let mut packages = input.packages;
    let mut names = Vec::new();

    for token in input.tokens.drain(..) {
        if let Some(chip) = chip_from_token(&token) {
            if !chips.contains(&chip) {
                chips.push(chip);
            }
        } else if let Some(package) = package_from_token(&token) {
            if !packages.contains(&package) {
                packages.push(package);
            }
        } else {
            names.push(token);
        }
    }

    if chips.is_empty() && input.infer_chip {
        let tests = packages
            .iter()
            .any(|package| matches!(package, Package::HilTest | Package::HilTestRadio));
        let chip = if verb == Verb::Test || tests {
            detect::with_probe_rs()
        } else if verb == Verb::Run {
            detect::with_espflash()
        } else {
            detect::with_probe_rs()
        };
        if let Some(chip) = chip {
            chips.push(chip);
        }
    }

    Resolution {
        verb,
        chips,
        packages,
        names,
    }
}

fn chip_from_token(token: &str) -> Option<Chip> {
    let mut normalized = token.trim().to_ascii_lowercase();
    normalized.retain(|c| c != '-');
    Chip::from_str(&normalized).ok()
}

fn package_from_token(token: &str) -> Option<Package> {
    match token.trim().to_ascii_lowercase().as_str() {
        "example" | "examples" => return Some(Package::Examples),
        "test" | "tests" => return Some(Package::HilTest),
        "qa" | "qa-test" => return Some(Package::QaTest),
        _ => {}
    }

    Package::iter().find(|package| package.to_string().eq_ignore_ascii_case(token.trim()))
}
