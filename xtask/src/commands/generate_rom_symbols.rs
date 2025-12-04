use std::{
    collections::HashSet,
    fs,
    fs::OpenOptions,
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{Context, Error};
use regex::Regex;

use crate::Chip;

/// Generates or updates the `generated_rom_symbols.rs` file with ROM symbol markers
/// for the specified target chip.
pub(crate) fn generate_rom_symbols(workspace: &Path, target_chip: &Chip) -> Result<(), Error> {
    // Convert chip to lowercase string for paths and modules
    let chip_name = target_chip.to_string();

    let rom_ld_dir = workspace.join(format!("ld/{}/rom", chip_name));

    let dest_path = workspace.join("src/generated_rom_symbols.rs");

    let current_chip_module_content =
        generate_module_content(&rom_ld_dir, &chip_name, &target_chip)?;

    let mut out_file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&dest_path)
        .context(format!(
            "Failed to open {} for writing",
            dest_path.display()
        ))?;

    writeln!(out_file, "{}", current_chip_module_content).context(format!(
        "Failed to write module content for chip {} to generated_rom_symbols.rs",
        chip_name
    ))?;

    Ok(())
}

// Helper function to handle the parsing and content generation.
fn generate_module_content(
    rom_ld_dir: &Path,
    chip_name: &str,
    _target_chip: &Chip,
) -> Result<String, Error> {
    let mut content = String::new();

    let linker_scripts: Vec<PathBuf> = fs::read_dir(rom_ld_dir)?
        .filter_map(|entry| {
            let path = entry.ok()?.path();
            if path.extension().and_then(|s| s.to_str()) == Some("ld") {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    content.push_str(&format!(
            "#[cfg(all(feature = \"{chip}\", feature = \"__internal_rom_symbols\"))]\npub mod rom_symbols {{\n",
            chip = chip_name
    ));

    let mut generated_symbols: HashSet<String> = HashSet::new();

    let assignment_re = Regex::new(r"^\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*[=:]\s*(.*?);")?;
    let provide_re = Regex::new(r"PROVIDE\s*\(\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*=\s*(.*?)\);")?;

    for script_path in linker_scripts {
        let script_content = fs::read_to_string(&script_path)?;

        for line in script_content.lines() {
            let line_no_comment = line
                .split("/*")
                .next()
                .unwrap()
                .split("//")
                .next()
                .unwrap()
                .trim();

            let mut symbol_name: Option<String> = None;

            if let Some(caps) = assignment_re.captures(line_no_comment) {
                symbol_name = Some(caps[1].to_owned());
            } else if let Some(caps) = provide_re.captures(line_no_comment) {
                symbol_name = Some(caps[1].to_owned());
            }

            if let Some(ref name) = symbol_name {
                let rust_ident = name.trim_start_matches('_').to_owned();

                if generated_symbols.contains(&rust_ident) {
                    continue;
                }

                generated_symbols.insert(rust_ident.clone());

                content.push_str(&format!("pub fn {}() {{}}\n\n", rust_ident));
            }
        }
    }

    // Close the chip module
    content.push_str("}\n");

    Ok(content)
}
