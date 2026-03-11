use std::io::Write as _;

use proc_macro::TokenStream;
use syn::{LitStr, parse_macro_input};
use termcolor::{Color, ColorChoice, ColorSpec, StandardStream, WriteColor};

// Adapted from:
// https://github.com/dtolnay/build-alert/blob/49d060e/src/lib.rs#L54-L93
pub fn do_alert(color: Color, input: TokenStream) -> TokenStream {
    let message = parse_macro_input!(input as LitStr).value();

    let stderr = &mut StandardStream::stderr(ColorChoice::Auto);
    let color_spec = ColorSpec::new().set_fg(Some(color)).clone();

    let mut has_nonspace = false;

    for mut line in message.lines() {
        if !has_nonspace {
            let (maybe_heading, rest) = split_heading(line);

            if let Some(heading) = maybe_heading {
                stderr.set_color(color_spec.clone().set_bold(true)).ok();
                write!(stderr, "\n{heading}").ok();
                has_nonspace = true;
            }

            line = rest;
        }

        if line.is_empty() {
            writeln!(stderr).ok();
        } else {
            stderr.set_color(&color_spec).ok();
            writeln!(stderr, "{line}").ok();

            has_nonspace = has_nonspace || line.contains(|ch: char| ch != ' ');
        }
    }

    stderr.reset().ok();
    writeln!(stderr).ok();

    TokenStream::new()
}

// Adapted from:
// https://github.com/dtolnay/build-alert/blob/49d060e/src/lib.rs#L95-L114
fn split_heading(s: &str) -> (Option<&str>, &str) {
    let mut end = 0;
    while end < s.len() && s[end..].starts_with(|ch: char| ch.is_ascii_uppercase()) {
        end += 1;
    }

    if end >= 3 && (end == s.len() || s[end..].starts_with(':')) {
        let (heading, rest) = s.split_at(end);
        (Some(heading), rest)
    } else {
        (None, s)
    }
}
