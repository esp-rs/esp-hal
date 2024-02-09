use std::path::{Path, PathBuf};

use anyhow::Result;
use clap::{Args, Parser};
use xtask::Chip;

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build all examples for the specified chip.
    BuildExamples(BuildExamplesArgs),
}

#[derive(Debug, Args)]
struct BuildExamplesArgs {
    /// Which chip to build the examples for.
    #[clap(value_enum)]
    chip: Chip,
}

// ----------------------------------------------------------------------------
// Application

fn main() -> Result<()> {
    env_logger::Builder::new()
        .filter_module("xtask", log::LevelFilter::Info)
        .init();

    let workspace = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = workspace.parent().unwrap().canonicalize()?;

    match Cli::parse() {
        Cli::BuildExamples(args) => build_examples(&workspace, args),
    }
}

// ----------------------------------------------------------------------------
// Subcommands

fn build_examples(workspace: &Path, args: BuildExamplesArgs) -> Result<()> {
    // Load all examples and parse their metadata. Filter down the examples to only
    // those for which our chip is supported, and then attempt to build each
    // remaining example, with the required features enabled:
    xtask::load_examples(workspace)?
        .iter()
        .filter(|example| example.supports_chip(args.chip))
        .try_for_each(|example| xtask::build_example(workspace, args.chip, example))
}
