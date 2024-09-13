use core::fmt::Write;
use std::{collections::HashMap, env, fs, path::PathBuf};

const TABLE_HEADER: &str = r#"
| name | description | default value |
|------|-------------|---------------|
"#;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParseError;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Value {
    Number(usize),
    Bool(bool),
}

impl Value {
    fn parse(&mut self, s: &str) -> Result<(), ParseError> {
        *self = match self {
            Value::Bool(_) => match s {
                "false" => Value::Bool(false),
                _ => Value::Bool(true), // TODO should existance of the key mean true?
            },
            Value::Number(_) => Value::Number(s.parse().map_err(|_| ParseError)?),
        };
        Ok(())
    }

    fn as_string(&self) -> String {
        match self {
            Value::Bool(value) => String::from(if *value { "true" } else { "false" }),
            Value::Number(value) => format!("{}", value),
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
        configs.insert(snake_case(name), *default);

        // Rebuild if config envvar changed.
        // Note this is currently buggy and requires a clean build - PR is pending https://github.com/rust-lang/cargo/pull/14058
        println!("cargo:rerun-if-env-changed={name}");
    }

    // Try and capture input from the environment
    for (var, value) in env::vars() {
        if let Some(name) = var.strip_prefix(&format!("{prefix}_")) {
            let name = snake_case(name);
            // TODO should this be lowered to a warning for unknown configuration options?
            // we could instead mark this as unknown and _not_ emit
            // println!("cargo:rustc-check-cfg=cfg({})", name);
            let Some(cfg) = configs.get_mut(&name) else {
                panic!("Unknown env var {name}")
            };

            cfg.parse(&value)
                .expect(&format!("Invalid value for env var {name}: {value}"));
        }
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
