use std::{
    collections::HashMap,
    error::Error,
    path::{Path, PathBuf},
    str::FromStr,
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

    /// Config file - using `config.toml` by default
    #[arg(short = 'c', long)]
    config_file: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CrateConfig {
    name: String,
    options: Vec<ConfigItem>,
    checks: Option<Vec<String>>,
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

    let config_file = if args.config_file.is_none() {
        // if there are multiple config files and none is selected via the command line option
        // let the user choose (but don't offer the base config.toml)

        let mut config_file = None;
        let cargo_dir = work_dir.join(".cargo");

        if cargo_dir.exists() {
            let files: Vec<String> = cargo_dir
                .read_dir()?
                .filter_map(|e| e.ok())
                .filter(|entry| {
                    entry.path().is_file()
                        && entry.path().extension().unwrap_or_default() == "toml"
                        && entry.file_name().to_string_lossy() != "config.toml"
                })
                .map(|entry| entry.file_name().to_string_lossy().to_string())
                .collect();

            if !files.is_empty() {
                let terminal = tui::init_terminal()?;
                let mut chooser = tui::ConfigChooser::new(files);
                config_file = chooser.run(terminal)?;
                tui::restore_terminal()?;
            } else {
                config_file = Some("config.toml".to_string());
            }
        }

        config_file
    } else {
        Some(
            args.config_file
                .as_deref()
                .unwrap_or("config.toml")
                .to_string(),
        )
    };

    if config_file.is_none() {
        return Ok(());
    }

    let config_file = config_file.unwrap();

    let config_file_path = work_dir.join(".cargo").join(config_file);
    if !config_file_path.exists() {
        return Err(format!(
            "Config file {} does not exist or is not readable.",
            config_file_path.display()
        )
        .into());
    }

    let (hint_about_config_toml, configs) = parse_configs(&work_dir, args.chip, &config_file_path)?;
    let initial_configs = configs.clone();
    let previous_config = initial_configs.clone();

    let repository = tui::Repository::new(configs.clone());

    // TUI stuff ahead
    let terminal = tui::init_terminal()?;

    // create app and run it
    let updated_cfg = tui::App::new(if hint_about_config_toml {
        Some("[env] section in base config.toml detected - avoid this and only add [env] sections to individual configs".to_string()) } else { None }, repository)
        .run(terminal)?;

    tui::restore_terminal()?;

    // done with the TUI
    if let Some(updated_cfg) = updated_cfg {
        apply_config(updated_cfg, previous_config, &config_file_path)?;
    }

    Ok(())
}

fn apply_config(
    updated_cfg: Vec<CrateConfig>,
    previous_cfg: Vec<CrateConfig>,
    config_toml_path: &PathBuf,
) -> Result<(), Box<dyn Error>> {
    let mut config = std::fs::read_to_string(config_toml_path)?
        .as_str()
        .parse::<DocumentMut>()?;

    if !config.contains_key("env") {
        config.insert("env", Item::Table(Table::new()));
    }

    let envs = config.get_mut("env").unwrap().as_table_mut().unwrap();

    for cfg in updated_cfg {
        let previous_crate_cfg = previous_cfg.iter().find(|c| c.name == cfg.name);

        for option in cfg.options {
            let previous_option = previous_crate_cfg.and_then(|c| {
                c.options
                    .iter()
                    .find(|o| o.option.name == option.option.name)
            });
            let key = option.option.full_env_var(&cfg.name);

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

    std::fs::write(config_toml_path, config.to_string().as_bytes())?;

    Ok(())
}

fn parse_configs(
    path: &Path,
    chip_from_args: Option<esp_metadata::Chip>,
    config_toml_path: &PathBuf,
) -> Result<(bool, Vec<CrateConfig>), Box<dyn Error>> {
    let mut hint_about_configs = false;

    // check if we find multiple potential config files - if yes and if the base `config.toml`
    // contains an [env] section let the user know this is not ideal
    if let Some(config_toml_dir) = config_toml_path.parent() {
        if config_toml_dir
            .read_dir()?
            .filter(|entry| {
                if let Ok(entry) = entry {
                    entry.path().is_file() && entry.path().extension().unwrap_or_default() == "toml"
                } else {
                    false
                }
            })
            .count()
            > 1
        {
            let base_toml = config_toml_dir.join("config.toml");
            if base_toml.exists() {
                let base_toml_content = std::fs::read_to_string(base_toml)?;
                let base_toml = base_toml_content.as_str().parse::<DocumentMut>()?;
                if base_toml.contains_key("env") {
                    hint_about_configs = true;
                }
            }
        }
    }

    let config_toml_content = std::fs::read_to_string(config_toml_path)?;
    let config_toml = config_toml_content.as_str().parse::<DocumentMut>()?;

    let envs: HashMap<String, String> = config_toml
        .get("env")
        .and_then(Item::as_table)
        .map(|table| {
            table
                .iter()
                .filter_map(|(k, v)| v.as_str().map(|s| (k.to_string(), s.to_string())))
                .collect()
        })
        .unwrap_or_default();

    // Get the metadata of the project to
    // - discover configurable crates
    // - get the active features on crates (e.g. to guess the chip the project is targeting)
    // this might fetch the dependencies from registries and/or git repositories
    // so this
    // - might take a few seconds (while it's usually very quick)
    // - might need an internet connection to succeed

    let meta = cargo_metadata::MetadataCommand::new()
        .current_dir(path)
        .verbose(true) // entertain the user by showing what exactly is going on
        .exec()
        // with verbose output the error we get doesn't contain anything useful or interesting
        .map_err(|_| "`cargo metadata` failed for your project. Make sure it's buildable.")?;

    // try to guess the chip from the metadata by looking at an active chip feature
    // for esp-hal
    let chip_from_meta = || {
        let mut chip = None;
        for pkg in &meta.root_package().unwrap().dependencies {
            if pkg.name == "esp-hal" {
                let possible_chip_feature_matches: Vec<esp_metadata::Chip> = pkg
                    .features
                    .iter()
                    .flat_map(|f| esp_metadata::Chip::from_str(f))
                    .collect::<Vec<esp_metadata::Chip>>();
                chip = possible_chip_feature_matches.first().cloned();
            }
        }
        chip
    };

    // the "ESP_CONFIG_CHIP" hint env-var if present
    let chip_from_config = || {
        envs.get("ESP_CONFIG_CHIP")
            .and_then(|chip_str| clap::ValueEnum::from_str(chip_str, true).ok())
    };

    // - if given as a parameter, use it
    // - if there is a hint in the config.toml, use it
    // - if we can infer it from metadata, use it
    // otherwise, fail
    let chip = chip_from_args
        .or_else(chip_from_config)
        .or_else(chip_from_meta);

    if chip.is_none() {
        return Err("No chip given or inferred. Try using the `--chip` argument.".into());
    }

    let mut configs = Vec::new();
    let chip = esp_metadata_generated::Chip::from_str(chip.unwrap().as_ref()).unwrap();
    let features = vec![];
    for krate in meta.packages {
        let maybe_cfg = krate.manifest_path.parent().unwrap().join("esp_config.yml");
        if maybe_cfg.exists() {
            let yaml = std::fs::read_to_string(&maybe_cfg)?;
            let (cfg, options) =
                esp_config::evaluate_yaml_config(&yaml, Some(chip), features.clone(), true)
                    .map_err(|e| {
                        format!(
                            "Error evaluating YAML config for crate {}: {}",
                            krate.name, e
                        )
                    })?;

            let crate_name = cfg.krate.clone();

            let options: Vec<ConfigItem> = options
                .iter()
                .map(|cfg| {
                    Ok::<ConfigItem, Box<dyn Error>>(ConfigItem {
                        option: cfg.clone(),
                        actual_value: {
                            let key = cfg.full_env_var(&crate_name);
                            let def_val = &cfg.default_value.to_string();
                            let val = envs.get(&key).unwrap_or(def_val);

                            let mut parsed_val = cfg.default_value.clone();
                            parsed_val.parse_in_place(val).map_err(|_| {
                                <std::string::String as Into<Box<dyn Error>>>::into(format!(
                                    "Unable to parse '{val}' for option '{}'",
                                    &cfg.name
                                ))
                            })?;
                            parsed_val
                        },
                    })
                })
                .collect::<Result<Vec<ConfigItem>, Box<dyn Error>>>()?;

            configs.push(CrateConfig {
                name: crate_name.clone(),
                checks: cfg.checks.clone(),
                options,
            });
        }
    }

    if configs.is_empty() {
        return Err("No config files found.".into());
    }

    Ok((hint_about_configs, configs))
}
