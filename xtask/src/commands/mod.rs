use anyhow::{Result, bail};
use inquire::Select;

pub use self::{
    build::*,
    check::*,
    check_changelog::*,
    check_pr_changelog::*,
    ci::*,
    clean::*,
    dispatch::DispatchArgs,
    fmt::*,
    host_tests::*,
    lint::*,
    release::*,
    run::*,
};
use crate::{cargo::CargoAction, metadata::Chip};

mod build;
mod check;
mod check_changelog;
mod check_pr_changelog;
mod ci;
mod clean;
pub mod dispatch;
mod examples;
mod fmt;
#[cfg(feature = "report")]
pub mod generate_report;
#[cfg(feature = "semver-checks")]
pub(crate) mod generate_rom_symbols;
mod host_tests;
mod lint;
#[cfg(feature = "mcp")]
pub mod mcp;
#[cfg(feature = "rel-check")]
pub mod relcheck;
mod release;
mod run;
mod tests;

pub use examples::examples;
pub use tests::tests;

/// Asks the caller to pick one of `options`.
///
/// A prompt needs someone to answer it, and a caller that is not a person — a script, a CI job, an
/// agent — would wait for input that never arrives. Say what to pass instead, and stop.
pub(crate) fn select<T: std::fmt::Display>(
    message: &str,
    options: Vec<T>,
    hint: &str,
) -> Result<T> {
    use std::io::IsTerminal;
    if !std::io::stdin().is_terminal() {
        bail!("\"{message}\" needs a terminal to answer it. Pass {hint}.");
    }

    Ok(Select::new(message, options).prompt()?)
}

pub(crate) fn move_artifacts(chip: Chip, action: &CargoAction) {
    if let CargoAction::Build(Some(out_dir)) = action {
        // Move binaries
        let from = out_dir.join("tmp");
        let to = out_dir.join(chip.to_string());
        std::fs::create_dir_all(&to).unwrap();

        // Binaries are in nested folders. There is one file in each folder. The name of the
        // final binary should be the name of the source binary's parent folder.
        for dir_entry in std::fs::read_dir(&from).unwrap() {
            let dir = dir_entry.unwrap();
            let mut bin_folder = std::fs::read_dir(dir.path()).unwrap();
            let file = bin_folder
                .next()
                .expect("No binary found")
                .expect("Failed to read entry");
            assert!(
                bin_folder.next().is_none(),
                "Only one binary should be present in each folder"
            );
            let source_file = file.path();
            let dest = to.join(dir.path().file_name().unwrap().to_string_lossy().as_ref());
            std::fs::rename(source_file, dest).unwrap();
        }
        // Clean up
        std::fs::remove_dir_all(from).unwrap();
    }
}
