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
    emit_md_tables: bool,
) -> HashMap<String, Value> {
    generate_config_internal(std::io::stdout(), crate_name, config, emit_md_tables)
        .into_iter()
        .map(|(k, (_, v))| (k, v))
        .collect()
}

pub fn generate_config_internal<'a>(
    mut stdout: impl Write,
    crate_name: &str,
    config: &'a [ConfigOption],
    emit_md_tables: bool,
) -> HashMap<String, (&'a ConfigOption, Value)> {
    // Only rebuild if `build.rs` changed. Otherwise, Cargo will rebuild if any
    // other file changed.
    writeln!(stdout, "cargo:rerun-if-changed=build.rs").ok();

    #[cfg(not(test))]
    env_change_work_around(&mut stdout);

    let mut doc_table = String::from(markdown::DOC_TABLE_HEADER);
    let mut selected_config = String::from(markdown::SELECTED_TABLE_HEADER);

    // Ensure that the prefix is `SCREAMING_SNAKE_CASE`:
    let prefix = format!("{}_CONFIG_", screaming_snake_case(crate_name));

    let mut configs = create_config(&mut stdout, &prefix, config, &mut doc_table);
    capture_from_env(&prefix, &mut configs);

    for (option, value) in configs.values() {
        if let Some(ref validator) = option.constraint {
            validator.validate(value).unwrap();
        }
    }

    emit_configuration(&mut stdout, &configs, &mut selected_config);

    #[cfg(not(test))]
    {
        let config_json = config_json(&configs);
        write_out_file(format!("{crate_name}_config_data.json"), config_json);
    }

    if emit_md_tables {
        let file_name = snake_case(crate_name);
        write_out_file(format!("{file_name}_config_table.md"), doc_table);
        write_out_file(format!("{file_name}_selected_config.md"), selected_config);
    }

    configs
}

fn config_json(config: &HashMap<String, (&ConfigOption, Value)>) -> String {
    #[derive(Serialize)]
    struct Item<'a> {
        #[serde(flatten)]
        option: &'a ConfigOption,
        actual_value: Value,
    }

    let mut to_write = Vec::new();
    for (option, value) in config.values() {
        to_write.push(Item {
            actual_value: value.clone(),
            option,
        })
    }

    serde_json::to_string(&to_write).unwrap()
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
}

impl ConfigOption {
    fn env_var(&self, prefix: &str) -> String {
        format!("{}{}", prefix, screaming_snake_case(self.name))
    }

    fn cfg_name(&self) -> String {
        snake_case(self.name)
    }
}

fn create_config<'a>(
    mut stdout: impl Write,
    prefix: &str,
    config: &'a [ConfigOption],
    doc_table: &mut String,
) -> HashMap<String, (&'a ConfigOption, Value)> {
    let mut configs = HashMap::new();

    for option in config {
        // Write documentation table line:
        markdown::write_doc_table_line(&mut *doc_table, prefix, &option);

        let name = option.env_var(prefix);
        // Rebuild if config environment variable changed:
        writeln!(stdout, "cargo:rerun-if-env-changed={}", name).ok();

        configs.insert(name, (option, option.default_value.clone()));
    }

    configs
}

fn capture_from_env(prefix: &str, configs: &mut HashMap<String, (&ConfigOption, Value)>) {
    let mut unknown = Vec::new();
    let mut failed = Vec::new();

    // Try and capture input from the environment:
    for (var, value) in env::vars() {
        if var.starts_with(prefix) {
            let Some((_, cfg)) = configs.get_mut(&var) else {
                unknown.push(var);
                continue;
            };

            if let Err(e) = cfg.parse_in_place(&value) {
                failed.push(format!("{var}: {e}"));
            }
        }
    }

    if !failed.is_empty() {
        panic!("Invalid configuration options detected: {:?}", failed);
    }

    if !unknown.is_empty() {
        panic!("Unknown configuration options detected: {:?}", unknown);
    }
}

fn emit_configuration(
    mut stdout: impl Write,
    configs: &HashMap<String, (&ConfigOption, Value)>,
    selected_config: &mut String,
) {
    for (name, (option, value)) in configs.iter() {
        let cfg_name = option.cfg_name();
        writeln!(stdout, "cargo:rustc-check-cfg=cfg({cfg_name})").ok();

        if let Value::Bool(true) = value {
            writeln!(stdout, "cargo:rustc-cfg={cfg_name}").ok();
        }

        if let Value::String(value) = value {
            if let Some(Validator::Enumeration(_)) = option.constraint.as_ref() {
                writeln!(stdout, "cargo:rustc-cfg={}_{}", cfg_name, snake_case(value)).ok();
            }
        }

        // Values that haven't been seen will be output here with the default value:
        writeln!(stdout, "cargo:rustc-env={}={}", name, value).ok();
        markdown::write_summary_table_line(&mut *selected_config, &name, value);
    }

    for (option, _) in configs.values() {
        if let Some(validator) = option.constraint.as_ref() {
            validator.emit_cargo_extras(&mut stdout, &option.cfg_name());
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
                        },
                        ConfigOption {
                            name: "number_signed",
                            description: "NA",
                            default_value: Value::Integer(-777),
                            constraint: None,
                        },
                        ConfigOption {
                            name: "string",
                            description: "NA",
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                        },
                        ConfigOption {
                            name: "bool",
                            description: "NA",
                            default_value: Value::Bool(false),
                            constraint: None,
                        },
                        ConfigOption {
                            name: "number_default",
                            description: "NA",
                            default_value: Value::Integer(999),
                            constraint: None,
                        },
                        ConfigOption {
                            name: "string_default",
                            description: "NA",
                            default_value: Value::String("Demo".to_string()),
                            constraint: None,
                        },
                        ConfigOption {
                            name: "bool_default",
                            description: "NA",
                            default_value: Value::Bool(false),
                            constraint: None,
                        },
                    ],
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
                        },
                        ConfigOption {
                            name: "negative_number",
                            description: "NA",
                            default_value: Value::Integer(1),
                            constraint: Some(Validator::NegativeInteger),
                        },
                        ConfigOption {
                            name: "non_negative_number",
                            description: "NA",
                            default_value: Value::Integer(-1),
                            constraint: Some(Validator::NonNegativeInteger),
                        },
                        ConfigOption {
                            name: "range",
                            description: "NA",
                            default_value: Value::Integer(0),
                            constraint: Some(Validator::IntegerInRange(5..10)),
                        },
                    ],
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
                }],
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
                }],
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
                }],
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
                    }],
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
                }],
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
                    }],
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
        let config = [ConfigOption {
            name: "some-key",
            description: "NA",
            default_value: Value::String("variant-0".to_string()),
            constraint: Some(Validator::Enumeration(vec![
                "variant-0".to_string(),
                "variant-1".to_string(),
            ])),
        }];
        let configs =
            temp_env::with_vars([("ESP_TEST_CONFIG_SOME_KEY", Some("variant-0"))], || {
                generate_config_internal(&mut stdout, "esp-test", &config, false)
            });

        let json_output = config_json(&configs);
        assert_eq!(
            r#"[{"name":"some-key","description":"NA","default_value":{"String":"variant-0"},"constraint":{"Enumeration":["variant-0","variant-1"]},"actual_value":{"String":"variant-0"}}]"#,
            json_output
        );
    }
}
