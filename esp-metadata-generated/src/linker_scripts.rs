// This file is not generated. esp-metadata's `generate_lib_rs` declares it.

//! Renders linker script templates for one chip.
//!
//! `#IF`, `#ELSE`, `#ELSE IF` and `#ENDIF` select lines. `${...}` inserts a value.
//! The variables are:
//!
//! - An esp-metadata symbol with no value is `true` when this chip defines it, and `false` when it
//!   does not.
//! - An esp-metadata symbol with a value is a string. The string is empty when this chip does not
//!   define the symbol.
//! - `CARGO_FEATURE("name")` is `true` when the package under build enables the Cargo feature
//!   `name`.
//! - [`LinkerScriptRenderer::bool`] and [`LinkerScriptRenderer::string`] add further variables.

use std::{error::Error, fs, path::Path};

use somni_template::{BlockStyle, Env, Syntax, Template};

use crate::Chip;

enum Value {
    Bool(bool),
    String(String),
}

/// Renders linker script templates for one chip.
#[cfg_attr(docsrs, doc(cfg(feature = "build-script")))]
pub struct LinkerScriptRenderer {
    chip: Chip,
    values: Vec<(String, Value)>,
}

impl Chip {
    /// Creates a renderer for the linker scripts of this chip.
    #[cfg_attr(docsrs, doc(cfg(feature = "build-script")))]
    pub fn linker_script_renderer(self) -> LinkerScriptRenderer {
        LinkerScriptRenderer {
            chip: self,
            values: Vec::new(),
        }
    }
}

impl LinkerScriptRenderer {
    /// Adds a boolean variable for an `#IF` condition.
    pub fn bool(&mut self, name: &str, value: bool) -> &mut Self {
        self.values.push((name.to_string(), Value::Bool(value)));
        self
    }

    /// Adds a string variable for a `${...}` expression.
    pub fn string(&mut self, name: &str, value: impl Into<String>) -> &mut Self {
        self.values
            .push((name.to_string(), Value::String(value.into())));
        self
    }

    /// Renders every file under `src` to the same relative path under `dst`.
    ///
    /// # Errors
    ///
    /// An I/O error when a directory or a file cannot be read or written.
    /// A message that includes the source path when a template is invalid.
    pub fn render_dir(
        &self,
        src: impl AsRef<Path>,
        dst: impl AsRef<Path>,
    ) -> Result<(), Box<dyn Error>> {
        let (src, dst) = (src.as_ref(), dst.as_ref());

        // The per-file directives miss files added to the directory.
        println!("cargo:rerun-if-changed={}", src.display());

        fs::create_dir_all(dst)?;
        for entry in fs::read_dir(src)? {
            let entry = entry?;
            let dst = dst.join(entry.file_name());
            if entry.file_type()?.is_dir() {
                self.render_dir(entry.path(), dst)?;
            } else {
                self.render_file(entry.path(), dst)?;
            }
        }

        Ok(())
    }

    /// Renders the template `src` to the file `dst`.
    ///
    /// # Errors
    ///
    /// An I/O error when `src` cannot be read or `dst` cannot be written.
    /// A message that includes the path of `src` when the template is invalid.
    pub fn render_file(
        &self,
        src: impl AsRef<Path>,
        dst: impl AsRef<Path>,
    ) -> Result<(), Box<dyn Error>> {
        let src = src.as_ref();
        println!("cargo:rerun-if-changed={}", src.display());

        let syntax = Syntax {
            expr: ("${".to_string(), "}".to_string()),
            block: BlockStyle::Line {
                prefix: "#".to_string(),
            },
        };

        let template_src = fs::read_to_string(src)?;
        let rendered = Template::compile(&template_src, &syntax)
            .and_then(|template| template.render(self.env()))
            .map_err(|e| format!("{}: {}", src.display(), e.display_with(&template_src)))?;

        fs::write(dst, rendered)?;

        Ok(())
    }

    fn env(&self) -> Env {
        let mut env = Env::new();

        env.function("CARGO_FEATURE", |feature: &str| {
            let vars = std::env::var("CARGO_CFG_FEATURE").unwrap_or_default();
            vars.split(',').any(|enabled| enabled == feature)
        });

        for symbol in Chip::all_possible_symbols() {
            if let Some((name, _)) = symbol.split_once(',') {
                // An empty string is never a valid value, but the variable needs to exist.
                env.value(name, "");
            } else {
                env.value(symbol, false);
            }
        }
        for symbol in self.chip.all_symbols() {
            if let Some((name, value)) = symbol.split_once('=') {
                env.value(name, value.trim().trim_matches('"'));
            } else {
                env.value(symbol, true);
            }
        }

        for (name, value) in &self.values {
            match value {
                Value::Bool(value) => env.value(name, *value),
                Value::String(value) => env.value(name, value.as_str()),
            };
        }

        env
    }
}
