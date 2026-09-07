//! Free-form token resolution for `build` / `run` / `check` / `test`.
//!
//! Tokens after those verbs can be a chip, a crate, an example or test name, or
//! a package alias (`example`, `tests`, `qa`), in any order. Wired via
//! [`crate::commands::dispatch`].

use clap::ValueEnum as _;

use crate::{Package, metadata::Chip};

/// Tokens and any chips/packages already known (e.g. from flags).
#[derive(Debug, Clone, Default)]
pub struct ResolveInput {
    pub tokens: Vec<String>,
    pub chips: Vec<Chip>,
    pub packages: Vec<Package>,
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
    pub chips: Vec<Chip>,
    pub packages: Vec<Package>,
    pub names: Vec<String>,
}

/// Classify `input.tokens`. Does not look up examples on disk.
pub fn resolve(mut input: ResolveInput) -> Resolution {
    let mut chips = input.chips;
    let mut packages = input.packages;
    let mut names = Vec::new();

    for token in input.tokens.drain(..) {
        if let Some(chip) = chip_from_token(&token) {
            if !chips.contains(&chip) {
                chips.push(chip);
            }
        } else if let Ok(package) = Package::from_str(token.trim(), true) {
            if !packages.contains(&package) {
                packages.push(package);
            }
        } else {
            names.push(token);
        }
    }

    Resolution {
        chips,
        packages,
        names,
    }
}

/// `esp32-c6` and `ESP32C6` name the same chip as `esp32c6`.
fn chip_from_token(token: &str) -> Option<Chip> {
    let mut normalized = token.trim().to_owned();
    normalized.retain(|c| c != '-');
    Chip::from_str(&normalized, true).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn package_aliases_and_names_both_work() {
        let packages = resolve(ResolveInput::from_tokens(["qa", "hil-test-radio"])).packages;
        assert_eq!(packages, vec![Package::QaTest, Package::HilTestRadio]);
    }

    #[test]
    fn everything_else_is_a_name() {
        let names = resolve(ResolveInput::from_tokens(["sleep_timer"])).names;
        assert_eq!(names, vec!["sleep_timer"]);
    }
}
