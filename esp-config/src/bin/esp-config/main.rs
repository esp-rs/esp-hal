use std::{
    collections::HashMap,
    error::Error,
    path::{Path, PathBuf},
};

use clap::Parser;
use env_logger::{Builder, Env};
use esp_config::{ConfigOption, Value};
use serde::Deserialize;
use toml_edit::{DocumentMut, Formatted, Item, Table};
use walkdir::WalkDir;

mod tui;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Root of the project
    #[arg(short = 'P', long)]
    path: Option<PathBuf>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CrateConfig {
    name: String,
    options: Vec<ConfigItem>,
}

#[derive(Deserialize, Debug, Clone, PartialEq, Eq)]
struct ConfigItem {
    option: ConfigOption,
    actual_value: Value,
}

fn main() -> Result<(), Box<dyn Error>> {
    Builder::from_env(Env::default().default_filter_or(log::LevelFilter::Info.as_str()))
        .format_target(false)
        .init();

    let args = Args::parse();

    let work_dir = args.path.clone().unwrap_or(".".into());

    ensure_fresh_build(&work_dir)?;

    let mut configs = parse_configs(&work_dir)?;
    let initial_configs = configs.clone();
    let mut previous_config = initial_configs.clone();

    let mut errors_to_show = None;

    loop {
        let repository = tui::Repository::new(configs.clone());

        // TUI stuff ahead
        let terminal = tui::init_terminal()?;

        // create app and run it
        let updated_cfg = tui::App::new(errors_to_show, repository).run(terminal)?;

        tui::restore_terminal()?;

        // done with the TUI
        if let Some(updated_cfg) = updated_cfg {
            configs = updated_cfg.clone();
            apply_config(&work_dir, updated_cfg.clone(), previous_config.clone())?;
            previous_config = updated_cfg;
        } else {
            println!("Reverted configuration...");
            apply_config(&work_dir, initial_configs, vec![])?;
            break;
        }

        if let Some(errors) = check_build_after_changes(&work_dir) {
            errors_to_show = Some(errors);
        } else {
            println!("Updated configuration...");
            break;
        }
    }

    Ok(())
}

fn apply_config(
    path: &Path,
    updated_cfg: Vec<CrateConfig>,
    previous_cfg: Vec<CrateConfig>,
) -> Result<(), Box<dyn Error>> {
    let config_toml = path.join(".cargo/config.toml");

    let mut config = std::fs::read_to_string(&config_toml)?
        .as_str()
        .parse::<DocumentMut>()?;

    if !config.contains_key("env") {
        config.insert("env", Item::Table(Table::new()));
    }

    let envs = config.get_mut("env").unwrap().as_table_mut().unwrap();

    for cfg in updated_cfg {
        let prefix = cfg.name.to_ascii_uppercase().replace("-", "_");
        let previous_crate_cfg = previous_cfg.iter().find(|c| c.name == cfg.name);

        for option in cfg.options {
            let previous_option = previous_crate_cfg.and_then(|c| {
                c.options
                    .iter()
                    .find(|o| o.option.name == option.option.name)
            });

            let key = format!(
                "{prefix}_CONFIG_{}",
                option.option.name.to_ascii_uppercase().replace("-", "_")
            );

            // avoid updating unchanged options to keep the comments (if any)
            if Some(&option.actual_value) != previous_option.map(|option| &option.actual_value) {
                if option.actual_value != option.option.default_value {
                    let value = toml_edit::Value::String(Formatted::new(format!(
                        "{}",
                        option.actual_value
                    )));

                    envs.insert(&key, Item::Value(value));
                } else {
                    envs.remove(&key);
                }
            }
        }
    }

    std::fs::write(&config_toml, config.to_string().as_bytes())?;

    Ok(())
}

fn parse_configs(path: &Path) -> Result<Vec<CrateConfig>, Box<dyn Error>> {
    // we cheat by just trying to find the latest version of the config files
    // this should be fine since we force a fresh build before
    let mut candidates: Vec<_> = WalkDir::new(path.join("target"))
        .into_iter()
        .filter_entry(|entry| {
            entry.file_type().is_dir() || {
                if let Some(name) = entry.file_name().to_str() {
                    name.ends_with("_config_data.json")
                } else {
                    false
                }
            }
        })
        .filter(|entry| !entry.as_ref().unwrap().file_type().is_dir())
        .map(|entry| entry.unwrap())
        .collect();
    candidates.sort_by_key(|entry| entry.metadata().unwrap().modified().unwrap());

    let mut crate_config_table_to_json: HashMap<String, PathBuf> = HashMap::new();

    for e in candidates {
        if e.file_name()
            .to_str()
            .unwrap()
            .ends_with("_config_data.json")
        {
            let crate_name = e
                .file_name()
                .to_str()
                .unwrap()
                .replace("_config_data.json", "")
                .replace("_", "-");
            crate_config_table_to_json.insert(crate_name.clone(), e.path().to_path_buf());
        }
    }

    let mut configs = Vec::new();

    for (crate_name, path) in crate_config_table_to_json {
        let options =
            serde_json::from_str::<Vec<ConfigItem>>(std::fs::read_to_string(&path)?.as_str())
                .map_err(|e| {
                    format!(
                        "Unable to read config file {:?} - try `cargo clean` first ({e:?})",
                        path
                    )
                })?
                .iter()
                .filter(|option| option.option.active)
                .cloned()
                .collect();

        configs.push(CrateConfig {
            name: crate_name,
            options,
        });
    }
    configs.sort_by_key(|entry| entry.name.clone());

    if configs.is_empty() {
        return Err("No config files found.".into());
    }

    Ok(configs)
}

fn ensure_fresh_build(path: &PathBuf) -> Result<(), Box<dyn Error>> {
    let status = std::process::Command::new("cargo")
        .arg("build")
        .current_dir(path)
        .status()?;

    if !status.success() {
        return Err("Your project doesn't build. Fix the errors first.".into());
    }

    Ok(())
}

fn check_build_after_changes(path: &PathBuf) -> Option<String> {
    println!("Check configuration...");

    let status = std::process::Command::new("cargo")
        .arg("build")
        .current_dir(path)
        .stdout(std::process::Stdio::inherit())
        .output();

    if let Ok(status) = &status {
        if status.status.success() {
            return None;
        }
    }

    let mut errors = String::new();

    for line in String::from_utf8(status.unwrap().stderr)
        .unwrap_or_default()
        .lines()
    {
        if line.contains("the evaluated program panicked at '") {
            let error = line[line.find('\'').unwrap() + 1..].to_string();
            let error = error[..error.find("',").unwrap_or(error.len())].to_string();
            errors.push_str(&format!("{error}\n"));
        }
    }

    Some(errors)
}
