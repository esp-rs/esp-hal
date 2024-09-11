use std::{collections::HashMap, env};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParseError;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Value {
    // TODO
    // Number(usize),
    // String(String),
    Bool(bool),
}

impl Value {
    fn parse(&mut self, s: &str) -> Result<(), ParseError> {
        *self = match self {
            Value::Bool(_) => match s {
                "false" => Value::Bool(false),
                _ => Value::Bool(true), // TODO should existance of the key mean true?
            },
        };
        Ok(())
    }
}

/// Takes a array of configuration keys and default values
pub fn generate_config(prefix: &str, config: &[(&str, Value)]) {
    // only rebuild if build.rs changed. Otherwise Cargo will rebuild if any
    // other file changed.
    println!("cargo:rerun-if-changed=build.rs");

    // Generate the template for the config
    let mut configs = HashMap::new();
    for (name, default) in config {
        configs.insert(normalize_name(name), *default);

        // Rebuild if config envvar changed.
        // Note this is currently buggy and requires a clean build - PR is pending https://github.com/rust-lang/cargo/pull/14058
        println!("cargo:rerun-if-env-changed={name}");
    }

    let mut prefix = prefix.to_owned();
    prefix.make_ascii_uppercase();

    // Try and capture input from the environment
    for (var, value) in env::vars() {
        if let Some(name) = var.strip_prefix(&format!("{prefix}_")) {
            let name = normalize_name(name);
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

    // emit cfgs
    for (name, value) in configs.into_iter() {
        println!("cargo:rustc-check-cfg=cfg({})", name);
        match value {
            Value::Bool(val) if val == true => println!("cargo:rustc-cfg={name}"),
            _ => {}
        }
    }
}

// Converts a symbol name like
// "PLACE-spi_DRIVER-IN_ram"
// to
// "place_spi_driver_in_ram"
fn normalize_name(name: &str) -> String {
    let mut name = name.replace("-", "_");
    name.make_ascii_lowercase();
    name
}
