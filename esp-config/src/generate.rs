use core::fmt::Write;
use std::{collections::HashMap, env, fs, path::PathBuf};

const TABLE_HEADER: &str = r#"
| Name | Description | Default value |
|------|-------------|---------------|
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
    fn parse(&mut self, s: &str) -> Result<(), ParseError> {
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

/// This function will parse any screaming snake case environment variables that
/// match the given prefix. It will then attempt to parse the [`Value`]. Once
/// the config has been parsed, this function will emit snake case cfg's
/// _without_ the prefix which can be used in the dependant crate. After that,
/// it will create a markdown table in the `OUT_DIR` under the name
/// `{prefix}_config_table.md` where prefix has also been converted
/// to snake case. This can be included in crate documentation to outline the
/// available configuration options for the crate.
///
/// The tuple ordering for the`config` array is key, default, description.
///
/// Unknown keys with the supplied prefix will cause this function to panic.
pub fn generate_config(prefix: &str, config: &[(&str, Value, &str)]) {
    let mut prefix = screaming_snake_case(prefix);

    let mut doc_table = String::from(TABLE_HEADER);
    for (key, default, desc) in config {
        let key = screaming_snake_case(key);
        let default = default.as_string();
        writeln!(doc_table, "|**{prefix}_{key}**|{desc}|{default}|").unwrap();
    }

    // only rebuild if build.rs changed. Otherwise Cargo will rebuild if any
    // other file changed.
    println!("cargo:rerun-if-changed=build.rs");

    // Generate the template for the config
    let mut configs = HashMap::new();
    for (name, default, _) in config {
        configs.insert(snake_case(name), default.clone());

        // Rebuild if config envvar changed.
        // Note this is currently buggy and requires a clean build - PR is pending https://github.com/rust-lang/cargo/pull/14058
        println!("cargo:rerun-if-env-changed={name}");
    }

    let mut unknown = Vec::new();
    let mut failed = Vec::new();

    // Try and capture input from the environment
    for (var, value) in env::vars() {
        if let Some(name) = var.strip_prefix(&format!("{prefix}_")) {
            let name = snake_case(name);
            let Some(cfg) = configs.get_mut(&name) else {
                unknown.push(format!("{prefix}_{}", screaming_snake_case(&name)));
                continue;
            };

            if let Err(e) = cfg.parse(&value) {
                failed.push(format!("{prefix}_{}: {e:?}", screaming_snake_case(&name)));
            }
        }
    }

    if !failed.is_empty() {
        panic!("Invalid configuration options detected: {:?}", failed);
    }

    if !unknown.is_empty() {
        panic!("Unknown configuration options detected: {:?}", unknown);
    }

    // emit cfgs and set envs
    for (name, value) in configs.into_iter() {
        println!("cargo:rustc-check-cfg=cfg({})", name);
        match value {
            Value::Bool(val) if val == true => println!("cargo:rustc-cfg={name}"),
            _ => {}
        }

        // values that haven't been seen will be output here with the default value
        println!(
            "cargo:rustc-env={}={}",
            format!("{prefix}_{}", screaming_snake_case(&name)),
            value.as_string()
        );
    }

    // convert to snake case
    prefix.make_ascii_lowercase();

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir
        .join(format!("{prefix}_config_table.md"))
        .to_string_lossy()
        .to_string();
    fs::write(out_file, doc_table).unwrap();
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
            v.parse(input).unwrap();
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
            v.parse(input).unwrap();
            // no matter the input variant, the output format should be "true"
            assert_eq!(v.as_string(), "true");
        }

        for input in FALSE_INPUTS {
            v.parse(input).unwrap();
            // no matter the input variant, the output format should be "false"
            assert_eq!(v.as_string(), "false");
        }
    }
}
