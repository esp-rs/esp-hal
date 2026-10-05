use std::{
    io::ErrorKind,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail};
use clap::Args;

const INSTALL_COMMAND: &str =
    "cargo install --git https://github.com/esp-rs/esp-generate --locked esp-generate";

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

/// Returns the path of the esp-generate template in `workspace`.
pub(crate) fn template_path(workspace: &Path) -> Result<PathBuf> {
    let template = workspace.join("template");
    if !template.join("metadata.toml").exists() {
        bail!(
            "no esp-generate template at {} (expected a `metadata.toml` there)",
            template.display()
        );
    }
    Ok(template)
}

/// Makes sure an esp-generate that supports `--template` is on `PATH`.
pub(crate) fn ensure_esp_generate() -> Result<()> {
    let output = match Command::new("esp-generate").arg("--help").output() {
        Ok(output) => output,
        Err(e) if e.kind() == ErrorKind::NotFound => {
            bail!("esp-generate was not found on PATH. Install it with `{INSTALL_COMMAND}`.")
        }
        Err(e) => return Err(e).context("failed to run `esp-generate --help`"),
    };

    if !String::from_utf8_lossy(&output.stdout).contains("--template") {
        bail!(
            "the installed esp-generate does not support `--template`. \
             Update it with `{INSTALL_COMMAND}`."
        );
    }

    Ok(())
}

/// Generate a project from `template/`, wired to the crates in this checkout.
pub fn new_project(workspace: &Path, args: NewProjectArgs) -> Result<()> {
    let template = template_path(workspace)?;
    ensure_esp_generate()?;

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

    let status = command.status().context("failed to run esp-generate")?;
    if !status.success() {
        bail!("esp-generate failed ({status}), see its output above");
    }

    log::info!(
        "{} depends on the crates in {}",
        args.name,
        workspace.display()
    );

    Ok(())
}
