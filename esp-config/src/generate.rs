use std::{
    collections::HashMap,
    env,
    fmt::{self, Write as _},
    fs,
    io::Write,
    ops::Range,
    path::PathBuf,
};

use serde::Serialize;

const DOC_TABLE_HEADER: &str = r#"
| Name | Description | Default value | Allowed value |
|------|-------------|---------------|---------------|
"#;

const SELECTED_TABLE_HEADER: &str = r#"
| Name | Selected value |
|------|----------------|
"#;

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

/// Supported configuration value types.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum Value {
    /// Booleans.
    Bool(bool),
    /// Integers.
    Integer(i128),
    /// Strings.
    String(String),
}

// TODO: Do we want to handle negative values for non-decimal values?
impl Value {
    fn parse_in_place(&mut self, s: &str) -> Result<(), Error> {
        *self = match self {
            Value::Bool(_) => match s {
                "true" => Value::Bool(true),
                "false" => Value::Bool(false),
                _ => {
                    return Err(Error::parse(format!(
                        "Expected 'true' or 'false', found: '{s}'"
                    )))
                }
            },
            Value::Integer(_) => {
                let inner = match s.as_bytes() {
                    [b'0', b'x', ..] => i128::from_str_radix(&s[2..], 16),
                    [b'0', b'o', ..] => i128::from_str_radix(&s[2..], 8),
                    [b'0', b'b', ..] => i128::from_str_radix(&s[2..], 2),
                    _ => i128::from_str_radix(&s, 10),
                }
                .map_err(|_| Error::parse(format!("Expected valid intger value, found: '{s}'")))?;

                Value::Integer(inner)
            }
            Value::String(_) => Value::String(s.into()),
        };

        Ok(())
    }

    /// Convert the value to a [bool].
    pub fn as_bool(&self) -> bool {
        match self {
            Value::Bool(value) => *value,
            _ => panic!("attempted to convert non-bool value to a bool"),
        }
    }

    /// Convert the value to an [i128].
    pub fn as_integer(&self) -> i128 {
        match self {
            Value::Integer(value) => *value,
            _ => panic!("attempted to convert non-integer value to an integer"),
        }
    }

    /// Convert the value to a [String].
    pub fn as_string(&self) -> String {
        match self {
            Value::String(value) => value.to_owned(),
            _ => panic!("attempted to convert non-string value to a string"),
        }
    }

    /// Is the value a bool?
    pub fn is_bool(&self) -> bool {
        matches!(self, Value::Bool(_))
    }

    /// Is the value an integer?
    pub fn is_integer(&self) -> bool {
        matches!(self, Value::Integer(_))
    }

    /// Is the value a string?
    pub fn is_string(&self) -> bool {
        matches!(self, Value::String(_))
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(b) => write!(f, "{b}"),
            Value::Integer(i) => write!(f, "{i}"),
            Value::String(s) => write!(f, "{s}"),
        }
    }
}

/// Configuration value validation functions.
#[derive(Serialize)]
pub enum Validator {
    /// Only allow negative integers, i.e. any values less than 0.
    NegativeInteger,
    /// Only allow non-negative integers, i.e. any values greater than or equal
    /// to 0.
    NonNegativeInteger,
    /// Only allow positive integers, i.e. any values greater than to 0.
    PositiveInteger,
    /// Ensure that an integer value falls within the specified range.
    IntegerInRange(Range<i128>),
    /// String-Enumeration. Only allows one of the given Strings.
    Enumeration(Vec<String>),
    /// A custom validation function to run against any supported value type.
    #[serde(serialize_with = "serialize_custom")]
    #[serde(untagged)]
    Custom(Box<dyn Fn(&Value) -> Result<(), Error>>),
}

fn serialize_custom<S>(
    _: &Box<dyn Fn(&Value) -> Result<(), Error>>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    serializer.serialize_str("Unknown")
}

impl Validator {
    fn validate(&self, value: &Value) -> Result<(), Error> {
        match self {
            Validator::NegativeInteger => negative_integer(value)?,
            Validator::NonNegativeInteger => non_negative_integer(value)?,
            Validator::PositiveInteger => positive_integer(value)?,
            Validator::IntegerInRange(range) => integer_in_range(range, value)?,
            Validator::Enumeration(values) => enumeration(values, value)?,
            Validator::Custom(validator_fn) => validator_fn(value)?,
        }

        Ok(())
    }

    fn description(&self) -> Option<String> {
        match self {
            Validator::NegativeInteger => Some(String::from("Negative integer")),
            Validator::NonNegativeInteger => Some(String::from("Positive integer or 0")),
            Validator::PositiveInteger => Some(String::from("Positive integer")),
            Validator::IntegerInRange(range) => {
                Some(format!("Integer in range {}..{}", range.start, range.end))
            }
            Validator::Enumeration(values) => Some(format!("Any of {:?}", values)),
            Validator::Custom(_) => None,
        }
    }

    fn emit_cargo_extras<W: Write>(&self, stdout: &mut W, config_key: &str) {
        match self {
            Validator::Enumeration(values) => {
                let config_key = snake_case(config_key);
                for value in values {
                    writeln!(
                        stdout,
                        "cargo:rustc-check-cfg=cfg({config_key}_{})",
                        snake_case(value)
                    )
                    .ok();
                }
            }
            _ => (),
        }
    }
}

fn enumeration(values: &Vec<String>, value: &Value) -> Result<(), Error> {
    if let Value::String(value) = value {
        if !values.contains(value) {
            return Err(Error::validation(format!(
                "Expected one of {:?}, found '{}'",
                values, value
            )));
        }

        Ok(())
    } else {
        return Err(Error::parse(
            "Validator::Enumeration can only be used with string values",
        ));
    }
}

fn negative_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::NegativeInteger can only be used with integer values",
        ));
    } else if value.as_integer() >= 0 {
        return Err(Error::validation(format!(
            "Expected negative integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

fn non_negative_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::NonNegativeInteger can only be used with integer values",
        ));
    } else if value.as_integer() < 0 {
        return Err(Error::validation(format!(
            "Expected non-negative integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

fn positive_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::PositiveInteger can only be used with integer values",
        ));
    } else if value.as_integer() <= 0 {
        return Err(Error::validation(format!(
            "Expected positive integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

fn integer_in_range(range: &Range<i128>, value: &Value) -> Result<(), Error> {
    if !value.is_integer() || !range.contains(&value.as_integer()) {
        Err(Error::validation(format!(
            "Value '{}' does not fall within range '{:?}'",
            value, range
        )))
    } else {
        Ok(())
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
    config: &[(&str, &str, Value, Option<Validator>)],
    emit_md_tables: bool,
) -> HashMap<String, Value> {
    generate_config_internal(&mut std::io::stdout(), crate_name, config, emit_md_tables)
}

pub fn generate_config_internal<W: Write>(
    stdout: &mut W,
    crate_name: &str,
    config: &[(&str, &str, Value, Option<Validator>)],
    emit_md_tables: bool,
) -> HashMap<String, Value> {
    // Only rebuild if `build.rs` changed. Otherwise, Cargo will rebuild if any
    // other file changed.
    writeln!(stdout, "cargo:rerun-if-changed=build.rs").ok();

    #[cfg(not(test))]
    env_change_work_around(stdout);

    let mut doc_table = String::from(DOC_TABLE_HEADER);
    let mut selected_config = String::from(SELECTED_TABLE_HEADER);

    // Ensure that the prefix is `SCREAMING_SNAKE_CASE`:
    let prefix = format!("{}_CONFIG_", screaming_snake_case(crate_name));

    // Build a lookup table for any provided validators; we must prefix the
    // name of the config and transform it to SCREAMING_SNAKE_CASE so that
    // it matches the keys in the hash table produced by `create_config`.
    let config_validators = config
        .iter()
        .flat_map(|(name, _description, _default, validator)| {
            if let Some(validator) = validator {
                let name = format!("{prefix}{}", screaming_snake_case(name));
                Some((name, validator))
            } else {
                None
            }
        })
        .collect::<HashMap<_, _>>();

    let mut configs = create_config(stdout, &prefix, config, &mut doc_table);
    capture_from_env(&prefix, &mut configs);

    for (name, value) in configs.iter() {
        if let Some(validator) = config_validators.get(name) {
            validator.validate(value).unwrap();
        }
    }

    let validators: HashMap<String, &Validator> = config
        .iter()
        .filter_map(|(name, _, _, validator)| {
            if validator.is_some() {
                Some((snake_case(name), validator.as_ref().unwrap()))
            } else {
                None
            }
        })
        .collect();

    emit_configuration(stdout, &prefix, &configs, &validators, &mut selected_config);

    write_config_json(snake_case(crate_name), config, &configs);

    if emit_md_tables {
        let file_name = snake_case(crate_name);
        write_config_tables(&file_name, doc_table, selected_config);
    }

    configs
}

fn write_config_json(
    crate_name: String,
    config: &[(&str, &str, Value, Option<Validator>)],
    selected_config: &HashMap<String, Value>,
) {
    #[derive(Serialize)]
    struct Item<'a> {
        name: String,
        description: String,
        default_value: Value,
        actual_value: Value,
        constraint: &'a Option<Validator>,
    }

    let mut to_write: Vec<Item<'_>> = Vec::new();
    for item in config.iter() {
        let option_name = format!(
            "{}_CONFIG_{}",
            screaming_snake_case(&crate_name),
            screaming_snake_case(&item.0)
        );
        let val = match selected_config.get(&option_name) {
            Some(val) => val.clone(),
            None => item.2.clone(),
        };

        to_write.push(Item {
            name: item.0.to_string(),
            description: item.1.to_string(),
            default_value: item.2.clone(),
            actual_value: val,
            constraint: &item.3,
        })
    }

    #[cfg(not(test))]
    {
        let json = serde_json::to_string(&to_write).unwrap();
        let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
        let out_file = out_dir
            .join(format!("{crate_name}_config_data.json"))
            .display()
            .to_string();
        fs::write(out_file, json).unwrap();
    }
}

// A work-around for https://github.com/rust-lang/cargo/issues/10358
// This can be removed when https://github.com/rust-lang/cargo/pull/14058 is merged.
// Unlikely to work on projects in workspaces
#[cfg(not(test))]
fn env_change_work_around<W: Write>(stdout: &mut W) {
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

fn create_config<W: Write>(
    stdout: &mut W,
    prefix: &str,
    config: &[(&str, &str, Value, Option<Validator>)],
    doc_table: &mut String,
) -> HashMap<String, Value> {
    let mut configs = HashMap::new();

    for (name, description, default, validator) in config {
        let name = format!("{prefix}{}", screaming_snake_case(name));
        let allowed_values = if let Some(validator) = validator {
            validator.description()
        } else {
            None
        }
        .unwrap_or(String::from("-"));

        configs.insert(name.clone(), default.clone());

        // Write documentation table line:
        let default = default.to_string();
        writeln!(
            doc_table,
            "|**{name}**|{description}|{default}|{allowed_values}"
        )
        .unwrap();

        // Rebuild if config environment variable changed:
        writeln!(stdout, "cargo:rerun-if-env-changed={name}").ok();
    }

    configs
}

fn capture_from_env(prefix: &str, configs: &mut HashMap<String, Value>) {
    let mut unknown = Vec::new();
    let mut failed = Vec::new();

    // Try and capture input from the environment:
    for (var, value) in env::vars() {
        if var.starts_with(prefix) {
            let Some(cfg) = configs.get_mut(&var) else {
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

fn emit_configuration<W: Write>(
    stdout: &mut W,
    prefix: &str,
    configs: &HashMap<String, Value>,
    validators: &HashMap<String, &Validator>,
    selected_config: &mut String,
) {
    for (name, value) in configs.iter() {
        let cfg_name = snake_case(name.trim_start_matches(prefix));
        writeln!(stdout, "cargo:rustc-check-cfg=cfg({cfg_name})").ok();

        if let Value::Bool(true) = value {
            writeln!(stdout, "cargo:rustc-cfg={cfg_name}").ok();
        }

        if let Value::String(value) = value {
            if let Some(Validator::Enumeration(_)) = validators.get(&cfg_name) {
                let value = format!("{}_{}", cfg_name, snake_case(value));
                writeln!(stdout, "cargo:rustc-cfg={value}").ok();
            }
        }

        let value = value.to_string();

        // Values that haven't been seen will be output here with the default value:
        writeln!(stdout, "cargo:rustc-env={}={}", name, value).ok();
        writeln!(selected_config, "|**{name}**|{value}|").unwrap();
    }

    for (name, validator) in validators {
        validator.emit_cargo_extras(stdout, &name);
    }
}

fn write_config_tables(prefix: &str, doc_table: String, selected_config: String) {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let out_file = out_dir
        .join(format!("{prefix}_config_table.md"))
        .display()
        .to_string();
    fs::write(out_file, doc_table).unwrap();

    let out_file = out_dir
        .join(format!("{prefix}_selected_config.md"))
        .display()
        .to_string();
    fs::write(out_file, selected_config).unwrap();
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

    #[test]
    fn value_number_formats() {
        const INPUTS: &[&str] = &["0xAA", "0o252", "0b0000000010101010", "170"];
        let mut v = Value::Integer(0);

        for input in INPUTS {
            v.parse_in_place(input).unwrap();
            // no matter the input format, the output format should be decimal
            assert_eq!(format!("{v}"), "170");
        }
    }

    #[test]
    fn value_bool_inputs() {
        let mut v = Value::Bool(false);

        v.parse_in_place("true").unwrap();
        assert_eq!(format!("{v}"), "true");

        v.parse_in_place("false").unwrap();
        assert_eq!(format!("{v}"), "false");
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
                        ("number", "NA", Value::Integer(999), None),
                        ("number_signed", "NA", Value::Integer(-777), None),
                        ("string", "NA", Value::String("Demo".to_owned()), None),
                        ("bool", "NA", Value::Bool(false), None),
                        ("number_default", "NA", Value::Integer(999), None),
                        (
                            "string_default",
                            "NA",
                            Value::String("Demo".to_owned()),
                            None,
                        ),
                        ("bool_default", "NA", Value::Bool(false), None),
                    ],
                    false,
                );

                // some values have changed
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_NUMBER").unwrap() {
                        Value::Integer(num) => *num,
                        _ => unreachable!(),
                    },
                    0xaa
                );
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_NUMBER_SIGNED").unwrap() {
                        Value::Integer(num) => *num,
                        _ => unreachable!(),
                    },
                    -999
                );
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_STRING").unwrap() {
                        Value::String(val) => val,
                        _ => unreachable!(),
                    },
                    "Hello world!"
                );
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_BOOL").unwrap() {
                        Value::Bool(val) => *val,
                        _ => unreachable!(),
                    },
                    true
                );

                // the rest are the defaults
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_NUMBER_DEFAULT").unwrap() {
                        Value::Integer(num) => *num,
                        _ => unreachable!(),
                    },
                    999
                );
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_STRING_DEFAULT").unwrap() {
                        Value::String(val) => val,
                        _ => unreachable!(),
                    },
                    "Demo"
                );
                assert_eq!(
                    match configs.get("ESP_TEST_CONFIG_BOOL_DEFAULT").unwrap() {
                        Value::Bool(val) => *val,
                        _ => unreachable!(),
                    },
                    false
                );
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
                        (
                            "positive_number",
                            "NA",
                            Value::Integer(-1),
                            Some(Validator::PositiveInteger),
                        ),
                        (
                            "negative_number",
                            "NA",
                            Value::Integer(1),
                            Some(Validator::NegativeInteger),
                        ),
                        (
                            "non_negative_number",
                            "NA",
                            Value::Integer(-1),
                            Some(Validator::NonNegativeInteger),
                        ),
                        (
                            "range",
                            "NA",
                            Value::Integer(0),
                            Some(Validator::IntegerInRange(5..10)),
                        ),
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
                &[(
                    "number",
                    "NA",
                    Value::Integer(-1),
                    Some(Validator::Custom(Box::new(|value| {
                        let range = 10..20;
                        if !value.is_integer() || !range.contains(&value.as_integer()) {
                            Err(Error::validation("value does not fall within range"))
                        } else {
                            Ok(())
                        }
                    }))),
                )],
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
                &[(
                    "positive_number",
                    "NA",
                    Value::Integer(-1),
                    Some(Validator::PositiveInteger),
                )],
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
                &[(
                    "number",
                    "NA",
                    Value::Integer(-1),
                    Some(Validator::Custom(Box::new(|value| {
                        let range = 10..20;
                        if !value.is_integer() || !range.contains(&value.as_integer()) {
                            Err(Error::validation("value does not fall within range"))
                        } else {
                            Ok(())
                        }
                    }))),
                )],
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
                    &[("number", "NA", Value::Integer(999), None)],
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
                &[("number", "NA", Value::Integer(999), None)],
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
                    &[("number", "NA", Value::Integer(999), None)],
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
                &[(
                    "some-key",
                    "NA",
                    Value::String("variant-0".to_string()),
                    Some(Validator::Enumeration(vec![
                        "variant-0".to_string(),
                        "variant-1".to_string(),
                    ])),
                )],
                false,
            );
        });

        let cargo_lines: Vec<&str> = std::str::from_utf8(&stdout).unwrap().lines().collect();
        println!("{:#?}", cargo_lines);
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key)"));
        assert!(cargo_lines.contains(&"cargo:rustc-env=ESP_TEST_CONFIG_SOME_KEY=variant-0"));
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key_variant_0)"));
        assert!(cargo_lines.contains(&"cargo:rustc-check-cfg=cfg(some_key_variant_1)"));
        assert!(cargo_lines.contains(&"cargo:rustc-cfg=some_key_variant_0"));
    }
}
