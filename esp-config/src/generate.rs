use core::fmt::Write;
use std::{collections::HashMap, env, fs, path::PathBuf};

const DOC_TABLE_HEADER: &str = r#"
| Name | Description | Default value |
|------|-------------|---------------|
"#;
const CHOSEN_TABLE_HEADER: &str = r#"
| Name | Selected value |
|------|----------------|
"#;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError(String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Value {
    Number(usize),
    Bool(bool),
    String(String),
}

impl Value {
    fn parse_in_place(&mut self, s: &str) -> Result<(), ParseError> {
        *self = match self {
            Value::Bool(_) => match s {
                "false" | "no" | "n" => Value::Bool(false),
                "true" | "yes" | "y" => Value::Bool(true),
                _ => return Err(ParseError(format!("Invalid boolean value: {}", s))),
            },
            Value::Number(_) => Value::Number(
                match s.as_bytes() {
                    [b'0', b'x', ..] => usize::from_str_radix(&s[2..], 16),
                    [b'0', b'o', ..] => usize::from_str_radix(&s[2..], 8),
                    [b'0', b'b', ..] => usize::from_str_radix(&s[2..], 2),
                    _ => usize::from_str_radix(&s, 10),
                }
                .map_err(|_| ParseError(format!("Invalid numerical value: {}", s)))?,
            ),
            Value::String(_) => Value::String(String::from(s)),
        };
        Ok(())
    }

    fn as_string(&self) -> String {
        match self {
            Value::Bool(value) => String::from(if *value { "true" } else { "false" }),
            Value::Number(value) => format!("{}", value),
            Value::String(value) => value.clone(),
        }
    }
}

/// Generate and parse config from a prefix, and array of key, default,
/// description tuples.
///
/// This function will parse any `SCREAMING_SNAKE_CASE` environment variables
/// that match the given prefix. It will then attempt to parse the [`Value`].
/// Once the config has been parsed, this function will emit `snake_case` cfg's
/// _without_ the prefix which can be used in the dependant crate. After that,
/// it will create a markdown table in the `OUT_DIR` under the name
/// `{prefix}_config_table.md` where prefix has also been converted
/// to `snake_case`. This can be included in crate documentation to outline the
/// available configuration options for the crate.
///
/// Passing a value of true for the `emit_md_tables` argument will create and
/// write markdown files of the available configuration and selected
/// configuration which can be included in documentation.
///
/// Unknown keys with the supplied prefix will cause this function to panic.
pub fn generate_config(
    prefix: &str,
    config: &[(&str, Value, &str)],
    emit_md_tables: bool,
) -> HashMap<String, Value> {
    // only rebuild if build.rs changed. Otherwise Cargo will rebuild if any
    // other file changed.
    println!("cargo:rerun-if-changed=build.rs");

    // ensure that the prefix is `SCREAMING_SNAKE_CASE`
    let prefix = format!("{}_", screaming_snake_case(prefix));
    let mut doc_table = String::from(DOC_TABLE_HEADER);
    let mut selected_config = String::from(CHOSEN_TABLE_HEADER);

    let mut configs = create_config(&prefix, config, &mut doc_table);
    capture_from_env(&prefix, &mut configs);
    emit_configuration(&prefix, &configs, &mut selected_config);

    if emit_md_tables {
        let file_name = snake_case(&prefix);
        write_config_tables(&file_name, doc_table, selected_config);
    }

    configs
}

fn emit_configuration(
    prefix: &str,
    configs: &HashMap<String, Value>,
    selected_config: &mut String,
) {
    // emit cfgs and set envs
    for (name, value) in configs.into_iter() {
        let cfg_name = snake_case(name.trim_start_matches(prefix));
        println!("cargo:rustc-check-cfg=cfg({cfg_name})");
        match value {
            Value::Bool(true) => {
                println!("cargo:rustc-cfg={cfg_name}")
            }
            _ => {}
        }

        let value = value.as_string();
        // values that haven't been seen will be output here with the default value
        println!("cargo:rustc-env={}={}", name, value);

        writeln!(selected_config, "|**{name}**|{value}|").unwrap();
    }
}

fn capture_from_env(prefix: &str, configs: &mut HashMap<String, Value>) {
    let mut unknown = Vec::new();
    let mut failed = Vec::new();

    // Try and capture input from the environment
    for (var, value) in env::vars() {
        if let Some(_) = var.strip_prefix(prefix) {
            let Some(cfg) = configs.get_mut(&var) else {
                unknown.push(var);
                continue;
            };

            if let Err(e) = cfg.parse_in_place(&value) {
                failed.push(format!("{}: {e:?}", var));
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

fn create_config(
    prefix: &str,
    config: &[(&str, Value, &str)],
    doc_table: &mut String,
) -> HashMap<String, Value> {
    // Generate the template for the config
    let mut configs = HashMap::new();
    for (name, default, desc) in config {
        let name = format!("{prefix}{}", screaming_snake_case(&name));
        configs.insert(name.clone(), default.clone());

        // write doc table line
        let default = default.as_string();
        writeln!(doc_table, "|**{name}**|{desc}|{default}|").unwrap();

        // Rebuild if config envvar changed.
        println!("cargo:rerun-if-env-changed={name}");
    }

    configs
}

fn write_config_tables(prefix: &str, doc_table: String, selected_config: String) {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir
        .join(format!("{prefix}config_table.md"))
        .to_string_lossy()
        .to_string();
    fs::write(out_file, doc_table).unwrap();

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir
        .join(format!("{prefix}selected_config.md"))
        .to_string_lossy()
        .to_string();
    fs::write(out_file, selected_config).unwrap();
}

// Converts a symbol name like
// "PLACE-spi_DRIVER-IN_ram"
// to
// "place_spi_driver_in_ram"
fn snake_case(name: &str) -> String {
    let mut name = name.replace("-", "_");
    name.make_ascii_lowercase();
    name
}

// Converts a symbol name like
// "PLACE-spi_DRIVER-IN_ram"
// to
// "PLACE_SPI_DRIVER_IN_RAM"
fn screaming_snake_case(name: &str) -> String {
    let mut name = name.replace("-", "_");
    name.make_ascii_uppercase();
    name
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn value_number_formats() {
        const INPUTS: &[&str] = &["0xAA", "0o252", "0b0000000010101010", "170"];
        let mut v = Value::Number(0);

        for input in INPUTS {
            v.parse_in_place(input).unwrap();
            // no matter the input format, the output format should be decimal
            assert_eq!(v.as_string(), "170");
        }
    }

    #[test]
    fn value_bool_inputs() {
        const TRUE_INPUTS: &[&str] = &["true", "y", "yes"];
        const FALSE_INPUTS: &[&str] = &["false", "n", "no"];
        let mut v = Value::Bool(false);

        for input in TRUE_INPUTS {
            v.parse_in_place(input).unwrap();
            // no matter the input variant, the output format should be "true"
            assert_eq!(v.as_string(), "true");
        }

        for input in FALSE_INPUTS {
            v.parse_in_place(input).unwrap();
            // no matter the input variant, the output format should be "false"
            assert_eq!(v.as_string(), "false");
        }
    }

    #[test]
    fn env_override() {
        temp_env::with_vars(
            [
                ("ESP_TEST_NUMBER", Some("0xaa")),
                ("ESP_TEST_STRING", Some("Hello world!")),
                ("ESP_TEST_BOOL", Some("true")),
            ],
            || {
                let configs = generate_config(
                    "esp-test",
                    &[
                        ("number", Value::Number(999), "NA"),
                        ("string", Value::String("Demo".to_owned()), "NA"),
                        ("bool", Value::Bool(false), "NA"),
                        ("number_default", Value::Number(999), "NA"),
                        ("string_default", Value::String("Demo".to_owned()), "NA"),
                        ("bool_default", Value::Bool(false), "NA"),
                    ],
                    false,
                );

                // some values have changed
                assert_eq!(
                    match configs.get("ESP_TEST_NUMBER").unwrap() {
                        Value::Number(num) => *num,
                        _ => unreachable!(),
                    },
                    0xaa
                );
                assert_eq!(
                    match configs.get("ESP_TEST_STRING").unwrap() {
                        Value::String(val) => val,
                        _ => unreachable!(),
                    },
                    "Hello world!"
                );
                assert_eq!(
                    match configs.get("ESP_TEST_BOOL").unwrap() {
                        Value::Bool(val) => *val,
                        _ => unreachable!(),
                    },
                    true
                );

                // the rest are the defaults
                assert_eq!(
                    match configs.get("ESP_TEST_NUMBER_DEFAULT").unwrap() {
                        Value::Number(num) => *num,
                        _ => unreachable!(),
                    },
                    999
                );
                assert_eq!(
                    match configs.get("ESP_TEST_STRING_DEFAULT").unwrap() {
                        Value::String(val) => val,
                        _ => unreachable!(),
                    },
                    "Demo"
                );
                assert_eq!(
                    match configs.get("ESP_TEST_BOOL_DEFAULT").unwrap() {
                        Value::Bool(val) => *val,
                        _ => unreachable!(),
                    },
                    false
                );
            },
        )
    }

    #[test]
    #[should_panic]
    fn env_unknown_bails() {
        temp_env::with_vars(
            [
                ("ESP_TEST_NUMBER", Some("0xaa")),
                ("ESP_TEST_RANDOM_VARIABLE", Some("")),
            ],
            || {
                generate_config("esp-test", &[("number", Value::Number(999), "NA")], false);
            },
        );
    }

    #[test]
    #[should_panic]
    fn env_invalid_values_bails() {
        temp_env::with_vars([("ESP_TEST_NUMBER", Some("Hello world"))], || {
            generate_config("esp-test", &[("number", Value::Number(999), "NA")], false);
        });
    }
}
