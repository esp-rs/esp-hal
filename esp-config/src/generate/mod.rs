use core::fmt::Display;
use std::{collections::HashMap, env, fmt, fs, io::Write, path::PathBuf};

use evalexpr::{ContextWithMutableFunctions, ContextWithMutableVariables};
use serde::{Deserialize, Serialize};

use crate::generate::{validator::Validator, value::Value};

mod markdown;
pub(crate) mod validator;
pub(crate) mod value;

/// Configuration errors.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Parse errors.
    Parse(String),
    /// Validation errors.
    Validation(String),
}

impl Error {
    /// Convenience function for creating parse errors.
    pub fn parse<S>(message: S) -> Self
    where
        S: Into<String>,
    {
        Self::Parse(message.into())
    }

    /// Convenience function for creating validation errors.
    pub fn validation<S>(message: S) -> Self
    where
        S: Into<String>,
    {
        Self::Validation(message.into())
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Parse(message) => write!(f, "{message}"),
            Error::Validation(message) => write!(f, "{message}"),
        }
    }
}

/// The root node of a configuration.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// The crate name.
    #[serde(rename = "crate")]
    pub krate: String,
    /// The config options for this crate.
    pub options: Vec<CfgOption>,
    /// Optionally additional checks.
    pub checks: Option<Vec<String>>,
}

fn true_default() -> String {
    "true".to_string()
}

fn unstable_default() -> Stability {
    Stability::Unstable
}

/// A default value for a configuration option.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CfgDefaultValue {
    /// Condition which makes this default value used.
    /// You can and have to have exactly one active default value.
    #[serde(rename = "if")]
    #[serde(default = "true_default")]
    pub if_: String,
    /// The default value.
    pub value: Value,
}

/// A configuration option.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CfgOption {
    /// Name of the configuration option
    pub name: String,
    /// Description of the configuration option.
    /// This will be visible in the documentation and in the tooling.
    pub description: String,
    /// A condition which specified when this option is active.
    #[serde(default = "true_default")]
    pub active: String,
    /// The default value.
    /// Exactly one of the items needs to be active at any time.
    pub default: Vec<CfgDefaultValue>,
    /// Constraints (Validators) to use.
    /// If given at most one item is allowed to be active at any time.
    pub constraints: Option<Vec<CfgConstraint>>,
    /// A display hint for the value.
    /// This is meant for tooling and/or documentation.
    pub display_hint: Option<DisplayHint>,
    /// The stability guarantees of this option.
    #[serde(default = "unstable_default")]
    pub stability: Stability,
}

/// A conditional constraint / validator for a config option.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CfgConstraint {
    /// Condition which makes this validator used.
    #[serde(rename = "if")]
    #[serde(default = "true_default")]
    if_: String,
    /// The validator to be used.
    #[serde(rename = "type")]
    type_: Validator,
}

/// Generate the config from a YAML definition.
///
/// The YAML follows the format outlined by [Config].
///
/// After deserializing the config and normalizing it, this will call
/// [generate_config] to finally get the currently active configuration.
pub fn generate_config_from_yaml_definition(
    yaml: &str,
    enable_unstable: bool,
    emit_md_tables: bool,
    chip: Option<esp_metadata::Config>,
) -> Result<HashMap<String, Value>, Error> {
    let features: Vec<String> = env::vars()
        .filter(|(k, _)| k.starts_with("CARGO_FEATURE_"))
        .map(|(k, _)| k)
        .map(|v| {
            v.strip_prefix("CARGO_FEATURE_")
                .unwrap_or_default()
                .to_string()
        })
        .collect();

    let (config, options) = evaluate_yaml_config(yaml, chip, features, false)?;

    let cfg = generate_config(&config.krate, &options, enable_unstable, emit_md_tables);

    if let Some(checks) = config.checks {
        let mut eval_ctx = evalexpr::HashMapContext::<evalexpr::DefaultNumericTypes>::new();
        for (k, v) in cfg.iter() {
            eval_ctx
                .set_value(
                    k.clone(),
                    match v {
                        Value::Bool(v) => evalexpr::Value::Boolean(*v),
                        Value::Integer(v) => evalexpr::Value::Int(*v),
                        Value::String(v) => evalexpr::Value::String(v.clone()),
                    },
                )
                .map_err(|err| Error::Parse(format!("Error setting value for {} ({})", k, err)))?;
        }
        for check in checks {
            if !evalexpr::eval_with_context(&check, &eval_ctx)
                .map_err(|err| {
                    Error::Validation(format!("Validation error: '{}' ({})", check, err))
                })?
                .as_boolean()
                .map_err(|err| {
                    Error::Validation(format!("Validation error: '{}' ({})", check, err))
                })?
            {
                return Err(Error::Validation(format!("Validation error: '{}'", check)));
            }
        }
    }

    Ok(cfg)
}

/// Evaluate the given YAML representation of a config definition.
pub fn evaluate_yaml_config(
    yaml: &str,
    chip: Option<esp_metadata::Config>,
    features: Vec<String>,
    is_tooling: bool,
) -> Result<(Config, Vec<ConfigOption>), Error> {
    let config: Config = serde_yml::from_str(yaml).map_err(|err| Error::Parse(err.to_string()))?;
    let mut options = Vec::new();
    let mut eval_ctx = evalexpr::HashMapContext::<evalexpr::DefaultNumericTypes>::new();
    if let Some(config) = chip {
        eval_ctx
            .set_value("chip".into(), evalexpr::Value::String(config.name()))
            .map_err(|err| Error::Parse(err.to_string()))?;

        eval_ctx
            .set_function(
                "feature".into(),
                evalexpr::Function::<evalexpr::DefaultNumericTypes>::new(move |arg| {
                    if let evalexpr::Value::String(which) = arg {
                        let res = config.contains(which);
                        Ok(evalexpr::Value::Boolean(res))
                    } else {
                        Err(evalexpr::EvalexprError::CustomMessage(format!(
                            "Bad argument: {:?}",
                            arg
                        )))
                    }
                }),
            )
            .map_err(|err| Error::Parse(err.to_string()))?;

        eval_ctx
            .set_function(
                "cargo_feature".into(),
                evalexpr::Function::<evalexpr::DefaultNumericTypes>::new(move |arg| {
                    if let evalexpr::Value::String(which) = arg {
                        let res = features.contains(&which.to_uppercase().replace("-", "_"));
                        Ok(evalexpr::Value::Boolean(res))
                    } else {
                        Err(evalexpr::EvalexprError::CustomMessage(format!(
                            "Bad argument: {:?}",
                            arg
                        )))
                    }
                }),
            )
            .map_err(|err| Error::Parse(err.to_string()))?;

        eval_ctx
            .set_function(
                "is_tooling".into(),
                evalexpr::Function::<evalexpr::DefaultNumericTypes>::new(move |arg| {
                    if let evalexpr::Value::Empty = arg {
                        Ok(evalexpr::Value::Boolean(is_tooling))
                    } else {
                        Err(evalexpr::EvalexprError::CustomMessage(format!(
                            "Bad argument: {:?}",
                            arg
                        )))
                    }
                }),
            )
            .map_err(|err| Error::Parse(err.to_string()))?;
    }
    for option in config.options.clone() {
        let active = evalexpr::eval_with_context(&option.active, &eval_ctx)
            .map_err(|err| {
                Error::Parse(format!(
                    "Error evaluating '{}', error = {:?}",
                    option.active, err
                ))
            })?
            .as_boolean()
            .map_err(|err| {
                Error::Parse(format!(
                    "Error evaluating '{}', error = {:?}",
                    option.active, err
                ))
            })?;

        let constraint = {
            let mut active_constraint = None;
            if let Some(constraints) = option.constraints {
                for constraint in constraints {
                    if evalexpr::eval_with_context(&constraint.if_, &eval_ctx)
                        .map_err(|err| {
                            Error::Parse(format!(
                                "Error evaluating '{}', error = {:?}",
                                constraint.if_, err
                            ))
                        })?
                        .as_boolean()
                        .map_err(|err| {
                            Error::Parse(format!(
                                "Error evaluating '{}', error = {:?}",
                                constraint.if_, err
                            ))
                        })?
                    {
                        if active_constraint.is_some() {
                            panic!(
                                "More than one constraints active for crate {}, option {}",
                                config.krate, option.name
                            );
                        }
                        active_constraint = Some(constraint.type_)
                    }
                }
            };
            active_constraint
        };

        let default_value = {
            let mut default_value = None;
            for value in option.default {
                if evalexpr::eval_with_context(&value.if_, &eval_ctx)
                    .map_err(|err| {
                        Error::Parse(format!(
                            "Error evaluating '{}', error = {:?}",
                            value.if_, err
                        ))
                    })?
                    .as_boolean()
                    .map_err(|err| {
                        Error::Parse(format!(
                            "Error evaluating '{}', error = {:?}",
                            value.if_, err
                        ))
                    })?
                {
                    if default_value.is_some() {
                        panic!(
                            "More than one default values active for crate {}, option {}",
                            config.krate, option.name
                        );
                    }
                    default_value = Some(value.value)
                }
            }
            default_value
        };

        let option = ConfigOption {
            name: option.name.clone(),
            description: option.description,
            default_value: default_value.ok_or(Error::Parse(format!(
                "No default value found for {}",
                &option.name
            )))?,
            constraint,
            stability: option.stability,
            active,
            display_hint: option.display_hint.unwrap_or(DisplayHint::None),
        };
        options.push(option);
    }
    Ok((config, options))
}

/// Generate and parse config from a prefix, and an array of [ConfigOption].
///
/// This function will parse any `SCREAMING_SNAKE_CASE` environment variables
/// that match the given prefix. It will then attempt to parse the [`Value`] and
/// run any validators which have been specified.
///
/// [`Stability::Unstable`] features will only be enabled if the `unstable`
/// feature is enabled in the dependant crate. If the `unstable` feature is not
/// enabled, setting these options will result in a build error.
///
/// Once the config has been parsed, this function will emit `snake_case` cfg's
/// _without_ the prefix which can be used in the dependant crate. After that,
/// it will create a markdown table in the `OUT_DIR` under the name
/// `{prefix}_config_table.md` where prefix has also been converted to
/// `snake_case`. This can be included in crate documentation to outline the
/// available configuration options for the crate.
///
/// Passing a value of true for the `emit_md_tables` argument will create and
/// write markdown files of the available configuration and selected
/// configuration which can be included in documentation.
///
/// Unknown keys with the supplied prefix will cause this function to panic.
pub fn generate_config(
    crate_name: &str,
    config: &[ConfigOption],
    enable_unstable: bool,
    emit_md_tables: bool,
) -> HashMap<String, Value> {
    let configs = generate_config_internal(std::io::stdout(), crate_name, config, enable_unstable);

    if emit_md_tables {
        let file_name = snake_case(crate_name);

        let mut doc_table = markdown::DOC_TABLE_HEADER.replace(
            "{prefix}",
            format!("{}_CONFIG_*", screaming_snake_case(crate_name)).as_str(),
        );
        let mut selected_config = String::from(markdown::SELECTED_TABLE_HEADER);

        for (name, option, value) in configs.iter() {
            if !option.active {
                continue;
            }
            markdown::write_doc_table_line(&mut doc_table, name, option);
            markdown::write_summary_table_line(&mut selected_config, name, value);
        }

        write_out_file(format!("{file_name}_config_table.md"), doc_table);
        write_out_file(format!("{file_name}_selected_config.md"), selected_config);
    }

    // Remove the ConfigOptions from the output
    configs.into_iter().map(|(k, _, v)| (k, v)).collect()
}

pub fn generate_config_internal<'a>(
    mut stdout: impl Write,
    crate_name: &str,
    config: &'a [ConfigOption],
    enable_unstable: bool,
) -> Vec<(String, &'a ConfigOption, Value)> {
    // Only rebuild if `build.rs` changed. Otherwise, Cargo will rebuild if any
    // other file changed.
    writeln!(stdout, "cargo:rerun-if-changed=build.rs").ok();

    // Ensure that the prefix is `SCREAMING_SNAKE_CASE`:
    let prefix = format!("{}_CONFIG_", screaming_snake_case(crate_name));

    let mut configs = create_config(&prefix, config);
    capture_from_env(crate_name, &prefix, &mut configs, enable_unstable);

    for (_, option, value) in configs.iter() {
        if let Some(ref validator) = option.constraint {
            validator.validate(value).unwrap();
        }
    }

    emit_configuration(&mut stdout, &configs);

    configs
}

/// The stability of the configuration option.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum Stability {
    /// Unstable options need to be activated with the `unstable` feature
    /// of the package that defines them.
    Unstable,
    /// Stable options contain the first version in which they were
    /// stabilized.
    Stable(String),
}

impl Display for Stability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stability::Unstable => write!(f, "⚠️ Unstable"),
            Stability::Stable(version) => write!(f, "Stable since {version}"),
        }
    }
}

/// A display hint (for tooling only)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum DisplayHint {
    /// No display hint
    None,

    /// Use a binary representation
    Binary,

    /// Use a hexadecimal representation
    Hex,
}

/// A configuration option.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConfigOption {
    /// The name of the configuration option.
    ///
    /// The associated environment variable has the format of
    /// `<PREFIX>_CONFIG_<NAME>`.
    pub name: String,

    /// The description of the configuration option.
    ///
    /// The description will be included in the generated markdown
    /// documentation.
    pub description: String,

    /// The default value of the configuration option.
    pub default_value: Value,

    /// An optional validator for the configuration option.
    pub constraint: Option<Validator>,

    /// The stability of the configuration option.
    pub stability: Stability,

    /// Whether the config option should be offered to the user.
    ///
    /// Inactive options are not included in the documentation, and accessing
    /// them provides the default value.
    pub active: bool,

    /// A display hint (for tooling)
    pub display_hint: DisplayHint,
}

impl ConfigOption {
    fn env_var(&self, prefix: &str) -> String {
        format!("{}{}", prefix, screaming_snake_case(&self.name))
    }

    fn cfg_name(&self) -> String {
        snake_case(&self.name)
    }

    fn is_stable(&self) -> bool {
        matches!(self.stability, Stability::Stable(_))
    }
}

fn create_config<'a>(
    prefix: &str,
    config: &'a [ConfigOption],
) -> Vec<(String, &'a ConfigOption, Value)> {
    let mut configs = Vec::with_capacity(config.len());

    for option in config {
        configs.push((option.env_var(prefix), option, option.default_value.clone()));
    }

    configs
}

fn capture_from_env(
    crate_name: &str,
    prefix: &str,
    configs: &mut Vec<(String, &ConfigOption, Value)>,
    enable_unstable: bool,
) {
    let mut unknown = Vec::new();
    let mut failed = Vec::new();
    let mut unstable = Vec::new();

    // Try and capture input from the environment:
    for (var, value) in env::vars() {
        if var.starts_with(prefix) {
            let Some((_, option, cfg)) = configs.iter_mut().find(|(k, _, _)| k == &var) else {
                unknown.push(var);
                continue;
            };

            if !option.active {
                unknown.push(var);
                continue;
            }

            if !enable_unstable && !option.is_stable() {
                unstable.push(var);
                continue;
            }

            if let Err(e) = cfg.parse_in_place(&value) {
                failed.push(format!("{var}: {e}"));
            }
        }
    }

    if !failed.is_empty() {
        panic!("Invalid configuration options detected: {failed:?}");
    }

    if !unstable.is_empty() {
        panic!(
            "The following configuration options are unstable: {unstable:?}. You can enable it by \
            activating the 'unstable' feature in {crate_name}."
        );
    }

    if !unknown.is_empty() {
        panic!("Unknown configuration options detected: {unknown:?}");
    }
}

fn emit_configuration(mut stdout: impl Write, configs: &[(String, &ConfigOption, Value)]) {
    for (env_var_name, option, value) in configs.iter() {
        let cfg_name = option.cfg_name();

        // Output the raw configuration as an env var. Values that haven't been seen
        // will be output here with the default value. Also trigger a rebuild if config
        // environment variable changed.
        writeln!(stdout, "cargo:rustc-env={env_var_name}={value}").ok();
        writeln!(stdout, "cargo:rerun-if-env-changed={env_var_name}").ok();

        // Emit known config symbol:
        writeln!(stdout, "cargo:rustc-check-cfg=cfg({cfg_name})").ok();

        // Emit specially-handled values:
        if let Value::Bool(true) = value {
            writeln!(stdout, "cargo:rustc-cfg={cfg_name}").ok();
        }

        // Emit extra symbols based on the validator (e.g. enumerated values):
        if let Some(validator) = option.constraint.as_ref() {
            validator.emit_cargo_extras(&mut stdout, &cfg_name, value);
        }
    }
}

fn write_out_file(file_name: String, json: String) {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir.join(file_name);
    fs::write(out_file, json).unwrap();
}

fn snake_case(name: &str) -> String {
    let mut name = name.replace("-", "_");
    name.make_ascii_lowercase();

    name
}

fn screaming_snake_case(name: &str) -> String {
    let mut name = name.replace("-", "_");
    name.make_ascii_uppercase();

    name
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::generate::{validator::Validator, value::Value};

    #[test]
    fn value_number_formats() {
        const INPUTS: &[&str] = &["0xAA", "0o252", "0b0000000010101010", "170"];
        let mut v = Value::Integer(0);

        for input in INPUTS {
            v.parse_in_place(input).unwrap();
            // no matter the input format, the output format should be decimal
            assert_eq!(v.to_string(), "170");
        }
    }

    #[test]
    fn value_bool_inputs() {
        let mut v = Value::Bool(false);

        v.parse_in_place("true").unwrap();
        assert_eq!(v.to_string(), "true");

        v.parse_in_place("false").unwrap();
        assert_eq!(v.to_string(), "false");

        v.parse_in_place("else")
            .expect_err("Only true or false are valid");
    }

    #[test]
    fn env_override() {
        temp_env::with_vars(
            [
                ("ESP_TEST_CONFIG_NUMBER", Some("0xaa")),
                ("ESP_TEST_CONFIG_NUMBER_SIGNED", Some("-999")),
                ("ESP_TEST_CONFIG_STRING", Some("Hello world!")),
                ("ESP_TEST_CONFIG_BOOL", Some("true")),
            ],
            || {
                let configs = generate_config(
                    "esp-test",
                    &[
                        ConfigOption {
                            name: String::from("number"),
                            description: String::from("NA"),
                            default_value: Value::Integer(999),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("number_signed"),
                            description: String::from("NA"),
                            default_value: Value::Integer(-777),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("string"),
                            description: String::from("NA"),
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("bool"),
                            description: String::from("NA"),
                            default_value: Value::Bool(false),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("number_default"),
                            description: String::from("NA"),
                            default_value: Value::Integer(999),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("string_default"),
                            description: String::from("NA"),
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("bool_default"),
                            description: String::from("NA"),
                            default_value: Value::Bool(false),
                            constraint: None,
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                    ],
                    false,
                    false,
                );

                // some values have changed
                assert_eq!(configs["ESP_TEST_CONFIG_NUMBER"], Value::Integer(0xaa));
                assert_eq!(
                    configs["ESP_TEST_CONFIG_NUMBER_SIGNED"],
                    Value::Integer(-999)
                );
                assert_eq!(
                    configs["ESP_TEST_CONFIG_STRING"],
                    Value::String("Hello world!".to_string())
                );
                assert_eq!(configs["ESP_TEST_CONFIG_BOOL"], Value::Bool(true));

                // the rest are the defaults
                assert_eq!(
                    configs["ESP_TEST_CONFIG_NUMBER_DEFAULT"],
                    Value::Integer(999)
                );
                assert_eq!(
                    configs["ESP_TEST_CONFIG_STRING_DEFAULT"],
                    Value::String("Demo".to_string())
                );
                assert_eq!(configs["ESP_TEST_CONFIG_BOOL_DEFAULT"], Value::Bool(false));
            },
        )
    }

    #[test]
    fn builtin_validation_passes() {
        temp_env::with_vars(
            [
                ("ESP_TEST_CONFIG_POSITIVE_NUMBER", Some("7")),
                ("ESP_TEST_CONFIG_NEGATIVE_NUMBER", Some("-1")),
                ("ESP_TEST_CONFIG_NON_NEGATIVE_NUMBER", Some("0")),
                ("ESP_TEST_CONFIG_RANGE", Some("9")),
            ],
            || {
                generate_config(
                    "esp-test",
                    &[
                        ConfigOption {
                            name: String::from("positive_number"),
                            description: String::from("NA"),
                            default_value: Value::Integer(-1),
                            constraint: Some(Validator::PositiveInteger),
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("negative_number"),
                            description: String::from("NA"),
                            default_value: Value::Integer(1),
                            constraint: Some(Validator::NegativeInteger),
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("non_negative_number"),
                            description: String::from("NA"),
                            default_value: Value::Integer(-1),
                            constraint: Some(Validator::NonNegativeInteger),
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                        ConfigOption {
                            name: String::from("range"),
                            description: String::from("NA"),
                            default_value: Value::Integer(0),
                            constraint: Some(Validator::IntegerInRange(5..10)),
                            stability: Stability::Stable(String::from("testing")),
                            active: true,
                            display_hint: DisplayHint::None,
                        },
                    ],
                    false,
                    false,
                )
            },
        );
    }

    #[test]
    #[should_panic]
    fn builtin_validation_bails() {
        temp_env::with_vars([("ESP_TEST_CONFIG_POSITIVE_NUMBER", Some("-99"))], || {
            generate_config(
                "esp-test",
                &[ConfigOption {
                    name: String::from("positive_number"),
                    description: String::from("NA"),
                    default_value: Value::Integer(-1),
                    constraint: Some(Validator::PositiveInteger),
                    stability: Stability::Stable(String::from("testing")),
                    active: true,
                    display_hint: DisplayHint::None,
                }],
                false,
                false,
            )
        });
    }

    #[test]
    #[should_panic]
    fn env_unknown_bails() {
        temp_env::with_vars(
            [
                ("ESP_TEST_CONFIG_NUMBER", Some("0xaa")),
                ("ESP_TEST_CONFIG_RANDOM_VARIABLE", Some("")),
            ],
            || {
                generate_config(
                    "esp-test",
                    &[ConfigOption {
                        name: String::from("number"),
                        description: String::from("NA"),
                        default_value: Value::Integer(999),
                        constraint: None,
                        stability: Stability::Stable(String::from("testing")),
                        active: true,
                        display_hint: DisplayHint::None,
                    }],
                    false,
                    false,
                );
            },
        );
    }

    #[test]
    #[should_panic]
    fn env_invalid_values_bails() {
        temp_env::with_vars([("ESP_TEST_CONFIG_NUMBER", Some("Hello world"))], || {
            generate_config(
                "esp-test",
                &[ConfigOption {
                    name: String::from("number"),
                    description: String::from("NA"),
                    default_value: Value::Integer(999),
                    constraint: None,
                    stability: Stability::Stable(String::from("testing")),
                    active: true,
                    display_hint: DisplayHint::None,
                }],
                false,
                false,
            );
        });
    }

    #[test]
    fn env_unknown_prefix_is_ignored() {
        temp_env::with_vars(
            [("ESP_TEST_OTHER_CONFIG_NUMBER", Some("Hello world"))],
            || {
                generate_config(
                    "esp-test",
                    &[ConfigOption {
                        name: String::from("number"),
                        description: String::from("NA"),
                        default_value: Value::Integer(999),
                        constraint: None,
                        stability: Stability::Stable(String::from("testing")),
                        active: true,
                        display_hint: DisplayHint::None,
                    }],
                    false,
                    false,
                );
            },
        );
    }

    #[test]
    fn enumeration_validator() {
        let mut stdout = Vec::new();
        temp_env::with_vars([("ESP_TEST_CONFIG_SOME_KEY", Some("variant-0"))], || {
            generate_config_internal(
                &mut stdout,
                "esp-test",
                &[ConfigOption {
                    name: String::from("some-key"),
                    description: String::from("NA"),
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Stable(String::from("testing")),
                    active: true,
                    display_hint: DisplayHint::None,
                }],
                false,
            );
        });

        let cargo_lines: Vec<&str> = std::str::from_utf8(&stdout).unwrap().lines().collect();
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key)"));
        assert!(cargo_lines.contains(&"cargo:rustc-env=ESP_TEST_CONFIG_SOME_KEY=variant-0"));
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key_variant_0)"));
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key_variant_1)"));
        assert!(cargo_lines.contains(&"cargo:rustc-cfg=some_key_variant_0"));
    }

    #[test]
    #[should_panic]
    fn unstable_option_panics_unless_enabled() {
        let mut stdout = Vec::new();
        temp_env::with_vars([("ESP_TEST_CONFIG_SOME_KEY", Some("variant-0"))], || {
            generate_config_internal(
                &mut stdout,
                "esp-test",
                &[ConfigOption {
                    name: String::from("some-key"),
                    description: String::from("NA"),
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Unstable,
                    active: true,
                    display_hint: DisplayHint::None,
                }],
                false,
            );
        });
    }

    #[test]
    #[should_panic]
    fn inactive_option_panics() {
        let mut stdout = Vec::new();
        temp_env::with_vars([("ESP_TEST_CONFIG_SOME_KEY", Some("variant-0"))], || {
            generate_config_internal(
                &mut stdout,
                "esp-test",
                &[ConfigOption {
                    name: String::from("some-key"),
                    description: String::from("NA"),
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Stable(String::from("testing")),
                    active: false,
                    display_hint: DisplayHint::None,
                }],
                false,
            );
        });
    }

    #[test]
    fn deserialization() {
        let yml = r#"
crate: esp-bootloader-esp-idf

options:
- name: mmu_page_size
  description: ESP32-C2, ESP32-C6 and ESP32-H2 support configurable page sizes. This is currently only used to populate the app descriptor.
  default:
    - value: "64k"
  stability: !Stable xxxx
  constraints:
  - if: true
    type:
      validator: enumeration
      value:
      - 8k
      - 16k
      - 32k
      - 64k

- name: esp_idf_version
  description: ESP-IDF version used in the application descriptor. Currently it's not checked by the bootloader.
  default:
    - if: 'chip == "esp32c6"'
      value: "esp32c6"
    - if: 'chip == "esp32"'
      value: "other"
  active: true

- name: partition-table-offset
  description: "The address of partition table (by default 0x8000). Allows you to \
    move the partition table, it gives more space for the bootloader. Note that the \
    bootloader and app will both need to be compiled with the same \
    PARTITION_TABLE_OFFSET value."
  default:
    - if: true
      value: 0x8000
  stability: Unstable
  active: 'chip == "esp32c6"'
"#;

        let (cfg, options) = evaluate_yaml_config(
            yml,
            Some(esp_metadata::Config::for_chip(&esp_metadata::Chip::Esp32c6).clone()),
            vec![],
            false,
        )
        .unwrap();

        assert_eq!("esp-bootloader-esp-idf", cfg.krate);

        assert_eq!(
            vec![
                    ConfigOption {
                        name: "mmu_page_size".to_string(),
                        description: "ESP32-C2, ESP32-C6 and ESP32-H2 support configurable page sizes. This is currently only used to populate the app descriptor.".to_string(),
                        default_value: Value::String("64k".to_string()),
                        constraint: Some(
                            Validator::Enumeration(
                                vec![
                                    "8k".to_string(),
                                    "16k".to_string(),
                                    "32k".to_string(),
                                    "64k".to_string(),
                                ],
                            ),
                        ),
                        stability: Stability::Stable("xxxx".to_string()),
                        active: true,
                        display_hint: DisplayHint::None,
                    },
                    ConfigOption {
                        name: "esp_idf_version".to_string(),
                        description: "ESP-IDF version used in the application descriptor. Currently it's not checked by the bootloader.".to_string(),
                        default_value: Value::String("esp32c6".to_string()),
                        constraint: None,
                        stability: Stability::Unstable,
                        active: true,
                        display_hint: DisplayHint::None,
                    },
                    ConfigOption {
                        name: "partition-table-offset".to_string(),
                        description: "The address of partition table (by default 0x8000). Allows you to move the partition table, it gives more space for the bootloader. Note that the bootloader and app will both need to be compiled with the same PARTITION_TABLE_OFFSET value.".to_string(),
                        default_value: Value::Integer(32768),
                        constraint: None,
                        stability: Stability::Unstable,
                        active: true,
                        display_hint: DisplayHint::None,
                    },
            ],
            options
        );
    }
}
