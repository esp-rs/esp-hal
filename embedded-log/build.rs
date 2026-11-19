use std::{env, path::Path};

use log_04::LevelFilter;

fn main() {
    let log = cfg!(feature = "log-04");
    let defmt = cfg!(feature = "defmt");
    assert!(
        log != defmt,
        "Enable exactly one of the `log-04` and `defmt` features of `embedded-log`."
    );

    if env::var("ESP_LOGLEVEL").is_ok() || env::var("ESP_LOGFILTER").is_ok() {
        panic!("`ESP_LOGLEVEL` and `ESP_LOGFILTER` are not supported. Use `ESP_LOG` instead.");
    }

    let spec = env::var("ESP_LOG").ok();
    let table = generate_filter_table(spec.as_deref()).unwrap_or_else(|e| panic!("{e}"));
    let out_dir = env::var_os("OUT_DIR").unwrap();
    std::fs::write(Path::new(&out_dir).join("log_filter.rs"), table).unwrap();

    println!("cargo:rerun-if-env-changed=ESP_LOG");
}

#[derive(Default, Debug)]
struct ParseResult {
    directives: Vec<Directive>,
    errors: Vec<String>,
}

#[derive(Debug)]
struct Directive {
    name: Option<String>,
    level: LevelFilter,
}

/// Parses a logging specification string (e.g. `crate1,crate2::mod3,crate3::x=error/foo`).
///
/// Directives with a module name are sorted so that the longest, most specific prefix comes first.
fn parse_spec(spec: &str) -> ParseResult {
    let mut result = ParseResult::default();

    let mods = spec.split('/').next().unwrap_or_default();
    for s in mods.split(',').map(|ss| ss.trim()) {
        if s.is_empty() {
            continue;
        }
        let mut parts = s.split('=');
        let (level, name) = match (parts.next(), parts.next().map(|s| s.trim()), parts.next()) {
            // A single argument that parses as a level is the global fallback.
            (Some(part0), None, None) => match part0.parse() {
                Ok(num) => (num, None),
                Err(_) => (LevelFilter::max(), Some(part0)),
            },
            (Some(part0), Some(""), None) => (LevelFilter::max(), Some(part0)),
            (Some(part0), Some(part1), None) => match part1.parse() {
                Ok(num) => (num, Some(part0)),
                Err(_) => {
                    result
                        .errors
                        .push(format!("invalid logging spec '{part1}'"));
                    continue;
                }
            },
            _ => {
                result.errors.push(format!("invalid logging spec '{s}'"));
                continue;
            }
        };

        result.directives.push(Directive {
            name: name.map(|s| s.to_owned()),
            level,
        });
    }

    result
        .directives
        .sort_by(|a, b| match (a.name.as_ref(), b.name.as_ref()) {
            (Some(a), Some(b)) => b.len().cmp(&a.len()),
            _ => std::cmp::Ordering::Equal,
        });

    result
}

/// Generates the Rust source of the filter table that `src/logger.rs` includes.
fn generate_filter_table(spec: Option<&str>) -> Result<String, String> {
    let Some(spec) = spec else {
        // Without `ESP_LOG`, `init` installs a logger with every level disabled.
        return Ok(table_source(
            LevelFilter::Off,
            &[],
            Some(LevelFilter::Trace),
        ));
    };

    let res = parse_spec(spec);
    if !res.errors.is_empty() {
        return Err(format!("Error parsing `ESP_LOG`: {:?}", res.errors));
    }

    let mut global = None;
    let mut named = Vec::new();
    for directive in &res.directives {
        match &directive.name {
            Some(name) => named.push((name.as_str(), directive.level)),
            None if global.is_some() => {
                return Err("Multiple global log levels specified in `ESP_LOG`".to_string());
            }
            None => global = Some(directive.level),
        }
    }

    let max = res
        .directives
        .iter()
        .map(|d| d.level)
        .max()
        .unwrap_or(LevelFilter::Off);

    Ok(table_source(max, &named, global))
}

fn table_source(
    max: LevelFilter,
    named: &[(&str, LevelFilter)],
    global: Option<LevelFilter>,
) -> String {
    let mut src = format!(
        "pub(crate) const FILTER_MAX: log_04::LevelFilter = log_04::LevelFilter::{max:?};\n"
    );

    src.push_str("pub(crate) const FILTER: crate::logger::Filter = crate::logger::Filter {\n");
    src.push_str("    directives: &[\n");
    for (name, level) in named {
        src.push_str(&format!(
            "        ({name:?}, log_04::LevelFilter::{level:?}),\n"
        ));
    }
    src.push_str("    ],\n");
    match global {
        Some(level) => src.push_str(&format!(
            "    global: Some(log_04::LevelFilter::{level:?}),\n"
        )),
        None => src.push_str("    global: None,\n"),
    }
    src.push_str("};\n");

    src
}
