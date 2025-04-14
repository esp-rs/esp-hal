use core::fmt::Display;
use std::{collections::HashMap, env, fmt, fs, io::Write, path::PathBuf};

use serde::Serialize;

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

/// Generate and parse config from a prefix, and an array tuples containing the
/// name, description, default value, and an optional validator.
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

    #[cfg(not(test))]
    env_change_work_around(&mut stdout);

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

    #[cfg(not(test))]
    {
        let config_json = config_json(&configs, false);
        write_out_file(format!("{crate_name}_config_data.json"), config_json);
    }

    configs
}

fn config_json(config: &[(String, &ConfigOption, Value)], pretty: bool) -> String {
    #[derive(Serialize)]
    struct Item<'a> {
        #[serde(flatten)]
        option: &'a ConfigOption,
        actual_value: Value,
    }

    let mut to_write = Vec::new();
    for (_, option, value) in config.iter() {
        to_write.push(Item {
            actual_value: value.clone(),
            option,
        })
    }

    if pretty {
        serde_json::to_string_pretty(&to_write).unwrap()
    } else {
        serde_json::to_string(&to_write).unwrap()
    }
}

// A work-around for https://github.com/rust-lang/cargo/issues/10358
// This can be removed when https://github.com/rust-lang/cargo/pull/14058 is merged.
// Unlikely to work on projects in workspaces
#[cfg(not(test))]
fn env_change_work_around(mut stdout: impl Write) {
    let mut out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // We clean out_dir by removing all trailing directories, until it ends with
    // target
    while !out_dir.ends_with("target") {
        if !out_dir.pop() {
            return; // We ran out of directories...
        }
    }
    out_dir.pop();

    let dotcargo = out_dir.join(".cargo/");
    if dotcargo.exists() {
        if dotcargo.join("config.toml").exists() {
            writeln!(
                stdout,
                "cargo:rerun-if-changed={}",
                dotcargo.join("config.toml").display()
            )
            .ok();
        }
        if dotcargo.join("config").exists() {
            writeln!(
                stdout,
                "cargo:rerun-if-changed={}",
                dotcargo.join("config").display()
            )
            .ok();
        }
    }
}

/// The stability of the configuration option.
#[derive(Serialize, Clone, Copy)]
pub enum Stability {
    /// Unstable options need to be activated with the `unstable` feature
    /// of the package that defines them.
    Unstable,
    /// Stable options contain the first version in which they were
    /// stabilized.
    Stable(&'static str),
}

impl Display for Stability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stability::Unstable => write!(f, "⚠️ Unstable"),
            Stability::Stable(version) => write!(f, "Stable since {version}"),
        }
    }
}

/// A configuration option.
#[derive(Serialize)]
pub struct ConfigOption {
    /// The name of the configuration option.
    ///
    /// The associated environment variable has the format of
    /// `<PREFIX>_CONFIG_<NAME>`.
    pub name: &'static str,

    /// The description of the configuration option.
    ///
    /// The description will be included in the generated markdown
    /// documentation.
    pub description: &'static str,

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
}

impl ConfigOption {
    fn env_var(&self, prefix: &str) -> String {
        format!("{}{}", prefix, screaming_snake_case(self.name))
    }

    fn cfg_name(&self) -> String {
        snake_case(self.name)
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
        panic!("Invalid configuration options detected: {:?}", failed);
    }

    if !unstable.is_empty() {
        panic!(
            "The following configuration options are unstable: {unstable:?}. You can enable it by \
            activating the 'unstable' feature in {crate_name}."
        );
    }

    if !unknown.is_empty() {
        panic!("Unknown configuration options detected: {:?}", unknown);
    }
}

fn emit_configuration(mut stdout: impl Write, configs: &[(String, &ConfigOption, Value)]) {
    for (env_var_name, option, value) in configs.iter() {
        let cfg_name = option.cfg_name();

        // Output the raw configuration as an env var. Values that haven't been seen
        // will be output here with the default value. Also trigger a rebuild if config
        // environment variable changed.
        writeln!(stdout, "cargo:rustc-env={}={}", env_var_name, value).ok();
        writeln!(stdout, "cargo:rerun-if-env-changed={}", env_var_name).ok();

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
                            name: "number",
                            description: "NA",
                            default_value: Value::Integer(999),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "number_signed",
                            description: "NA",
                            default_value: Value::Integer(-777),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "string",
                            description: "NA",
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "bool",
                            description: "NA",
                            default_value: Value::Bool(false),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "number_default",
                            description: "NA",
                            default_value: Value::Integer(999),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "string_default",
                            description: "NA",
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "bool_default",
                            description: "NA",
                            default_value: Value::Bool(false),
                            constraint: None,
                            stability: Stability::Stable("testing"),
                            active: true,
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
                            name: "positive_number",
                            description: "NA",
                            default_value: Value::Integer(-1),
                            constraint: Some(Validator::PositiveInteger),
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "negative_number",
                            description: "NA",
                            default_value: Value::Integer(1),
                            constraint: Some(Validator::NegativeInteger),
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "non_negative_number",
                            description: "NA",
                            default_value: Value::Integer(-1),
                            constraint: Some(Validator::NonNegativeInteger),
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                        ConfigOption {
                            name: "range",
                            description: "NA",
                            default_value: Value::Integer(0),
                            constraint: Some(Validator::IntegerInRange(5..10)),
                            stability: Stability::Stable("testing"),
                            active: true,
                        },
                    ],
                    false,
                    false,
                )
            },
        );
    }

    #[test]
    fn custom_validation_passes() {
        temp_env::with_vars([("ESP_TEST_CONFIG_NUMBER", Some("13"))], || {
            generate_config(
                "esp-test",
                &[ConfigOption {
                    name: "number",
                    description: "NA",
                    default_value: Value::Integer(-1),
                    constraint: Some(Validator::Custom(Box::new(|value| {
                        let range = 10..20;
                        if !value.is_integer() || !range.contains(&value.as_integer()) {
                            Err(Error::validation("value does not fall within range"))
                        } else {
                            Ok(())
                        }
                    }))),
                    stability: Stability::Stable("testing"),
                    active: true,
                }],
                false,
                false,
            )
        });
    }

    #[test]
    #[should_panic]
    fn builtin_validation_bails() {
        temp_env::with_vars([("ESP_TEST_CONFIG_POSITIVE_NUMBER", Some("-99"))], || {
            generate_config(
                "esp-test",
                &[ConfigOption {
                    name: "positive_number",
                    description: "NA",
                    default_value: Value::Integer(-1),
                    constraint: Some(Validator::PositiveInteger),
                    stability: Stability::Stable("testing"),
                    active: true,
                }],
                false,
                false,
            )
        });
    }

    #[test]
    #[should_panic]
    fn custom_validation_bails() {
        temp_env::with_vars([("ESP_TEST_CONFIG_NUMBER", Some("37"))], || {
            generate_config(
                "esp-test",
                &[ConfigOption {
                    name: "number",
                    description: "NA",
                    default_value: Value::Integer(-1),
                    constraint: Some(Validator::Custom(Box::new(|value| {
                        let range = 10..20;
                        if !value.is_integer() || !range.contains(&value.as_integer()) {
                            Err(Error::validation("value does not fall within range"))
                        } else {
                            Ok(())
                        }
                    }))),
                    stability: Stability::Stable("testing"),
                    active: true,
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
                        name: "number",
                        description: "NA",
                        default_value: Value::Integer(999),
                        constraint: None,
                        stability: Stability::Stable("testing"),
                        active: true,
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
                    name: "number",
                    description: "NA",
                    default_value: Value::Integer(999),
                    constraint: None,
                    stability: Stability::Stable("testing"),
                    active: true,
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
                        name: "number",
                        description: "NA",
                        default_value: Value::Integer(999),
                        constraint: None,
                        stability: Stability::Stable("testing"),
                        active: true,
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
                    name: "some-key",
                    description: "NA",
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Stable("testing"),
                    active: true,
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
    fn json_output() {
        let mut stdout = Vec::new();
        let config = [
            ConfigOption {
                name: "some-key",
                description: "NA",
                default_value: Value::String("variant-0".to_string()),
                constraint: Some(Validator::Enumeration(vec![
                    "variant-0".to_string(),
                    "variant-1".to_string(),
                ])),
                stability: Stability::Stable("testing"),
                active: true,
            },
            ConfigOption {
                name: "some-key2",
                description: "NA",
                default_value: Value::Bool(true),
                constraint: None,
                stability: Stability::Unstable,
                active: true,
            },
        ];
        let configs =
            temp_env::with_vars([("ESP_TEST_CONFIG_SOME_KEY", Some("variant-0"))], || {
                generate_config_internal(&mut stdout, "esp-test", &config, false)
            });

        let json_output = config_json(&configs, true);
        pretty_assertions::assert_eq!(
            r#"[
  {
    "name": "some-key",
    "description": "NA",
    "default_value": {
      "String": "variant-0"
    },
    "constraint": {
      "Enumeration": [
        "variant-0",
        "variant-1"
      ]
    },
    "stability": {
      "Stable": "testing"
    },
    "active": true,
    "actual_value": {
      "String": "variant-0"
    }
  },
  {
    "name": "some-key2",
    "description": "NA",
    "default_value": {
      "Bool": true
    },
    "constraint": null,
    "stability": "Unstable",
    "active": true,
    "actual_value": {
      "Bool": true
    }
  }
]"#,
            json_output
        );
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
                    name: "some-key",
                    description: "NA",
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Unstable,
                    active: true,
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
                    name: "some-key",
                    description: "NA",
                    default_value: Value::String("variant-0".to_string()),
                    constraint: Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                    stability: Stability::Stable("testing"),
                    active: false,
                }],
                false,
            );
        });
    }
}
