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

mod tui;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Root of the project
    #[arg(short = 'P', long)]
    path: Option<PathBuf>,

    /// Chip
    #[arg(short = 'C', long)]
    chip: Option<esp_metadata::Chip>,
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

        if let Some(errors) = check_after_changes(&work_dir) {
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
    let config_toml = std::fs::read_to_string(path.join(".cargo/config.toml")).unwrap();
    let config_toml = config_toml.as_str().parse::<DocumentMut>()?;

    let envs = config_toml.get("env").unwrap().as_table().unwrap();
    let envs: HashMap<String, String> = envs
        .iter()
        .map(|(k, v)| (k.into(), v.as_str().unwrap().into()))
        .collect();

    let meta = cargo_metadata::MetadataCommand::new()
        .current_dir(path)
        .exec()
        .unwrap();

    let chip_from_meta = {
        let mut chip = None;
        for pkg in &meta.root_package().unwrap().dependencies {
            if pkg.name == "esp-hal" {
                if pkg.features.contains(&"esp32c2".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32c2);
                } else if pkg.features.contains(&"esp32c3".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32c3);
                } else if pkg.features.contains(&"esp32c6".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32c6);
                } else if pkg.features.contains(&"esp32h2".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32h2);
                } else if pkg.features.contains(&"esp32".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32);
                } else if pkg.features.contains(&"esp32s2".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32s2);
                } else if pkg.features.contains(&"esp32s3".to_string()) {
                    chip = Some(esp_metadata::Chip::Esp32s3);
                }
            }
        }

        chip
    };

    let mut configs = Vec::new();

    // TODO "guess" the chip
    // - if given as a parameter, use it
    // - if there is a hint in the config.toml, use it
    // - if we can infer it from metadata, use it (done)
    let chip = chip_from_meta.unwrap();
    let chip = esp_metadata::Config::for_chip(&chip);
    let features = vec![];
    for krate in meta.packages {
        let maybe_cfg = krate.manifest_path.parent().unwrap().join("esp_config.yml");
        if maybe_cfg.exists() {
            let yaml = std::fs::read_to_string(&maybe_cfg)?;
            let (cfg, options) =
                esp_config::evaluate_yaml_config(&yaml, Some(chip.clone()), features.clone(), true)
                    .unwrap();

            let crate_name = cfg.krate.clone();
            configs.push(CrateConfig {
                name: crate_name.clone(),
                options: options
                    .iter()
                    .map(|cfg| ConfigItem {
                        option: cfg.clone(),
                        actual_value: {
                            let key = format!(
                                "{}_CONFIG_{}",
                                crate_name.clone().to_ascii_uppercase().replace("-", "_"),
                                cfg.name.to_ascii_uppercase().replace("-", "_")
                            );
                            let def_val = &cfg.default_value.to_string();
                            let val = envs.get(&key).unwrap_or(def_val);

                            let val = match cfg.default_value {
                                Value::Bool(_) => Value::Bool(val.parse().unwrap()),
                                Value::Integer(_) => Value::Integer(parse_i128(val).unwrap()),
                                Value::String(_) => Value::String(val.into()),
                            };
                            val
                        },
                    })
                    .collect(),
            });
        }
    }

    if configs.is_empty() {
        return Err("No config files found.".into());
    }

    Ok(configs)
}

fn check_after_changes(path: &PathBuf) -> Option<String> {
    println!("Check configuration...");

    // TODO check the config by evaluating the `checks` - in case of an error return
    // Some(message)
    None
}

fn parse_i128(str: &str) -> Result<i128, std::num::ParseIntError> {
    let str = str.trim();

    if let Some(stripped) = str.strip_prefix("0x") {
        return i128::from_str_radix(stripped, 16);
    }

    if let Some(stripped) = str.strip_prefix("0b") {
        return i128::from_str_radix(stripped, 2);
    }

    str.parse::<i128>()
}
