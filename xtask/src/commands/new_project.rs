use std::{
    io::ErrorKind,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail};
use clap::Args;

pub(crate) const INSTALL_HINT: &str = "`--template` needs a newer esp-generate: \
     `cargo install --git https://github.com/esp-rs/esp-generate --locked`.";

/// Arguments for the `new-project` subcommand.
#[derive(Debug, Args)]
pub struct NewProjectArgs {
    /// Name of the project to generate.
    pub name: String,

    /// Directory to generate into. Defaults to the current directory.
    #[arg(short = 'O', long)]
    pub output_path: Option<PathBuf>,

    /// Generation option, passed through to esp-generate. Repeatable.
    #[arg(short = 'o', long = "option")]
    pub options: Vec<String>,

    /// Pick options non-interactively instead of opening the TUI.
    #[arg(long)]
    pub headless: bool,
}

/// Generate a project from `template/`, wired to the crates in this checkout.
pub fn new_project(workspace: &Path, args: NewProjectArgs) -> Result<()> {
    let template = workspace.join("template");
    if !template.join("metadata.toml").exists() {
        bail!(
            "no template at {} — run this from the repository root",
            template.display()
        );
    }

    let output_path = args.output_path.unwrap_or_else(|| PathBuf::from("."));

    let mut command = Command::new("esp-generate");
    command.arg("--template").arg(&template);
    if args.headless {
        command.arg("--headless");
    }
    for option in &args.options {
        command.args(["-o", option]);
    }
    command.arg("--output-path").arg(&output_path);
    command.arg(&args.name);

    let status = match command.status() {
        Ok(status) => status,
        Err(e) if e.kind() == ErrorKind::NotFound => {
            bail!("esp-generate was not found on PATH. {INSTALL_HINT}")
        }
        Err(e) => return Err(e).context("failed to run esp-generate"),
    };
    if !status.success() {
        bail!("esp-generate failed. If it rejected `--template`: {INSTALL_HINT}");
    }

    log::info!(
        "{} depends on the crates in {}",
        args.name,
        workspace.display()
    );

    Ok(())
}
