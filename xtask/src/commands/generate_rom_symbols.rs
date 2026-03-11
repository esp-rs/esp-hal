use std::{collections::HashSet, fs, fs::OpenOptions, io::Write, path::Path};

use anyhow::{Context, Error};
use regex::Regex;

use crate::Chip;

/// Generates or updates the `generated_rom_symbols.rs` file with ROM symbol markers
/// for the specified target chip.
pub(crate) fn generate_rom_symbols(workspace: &Path, target_chip: &Chip) -> Result<(), Error> {
    let chip_name = target_chip.to_string();
    let rom_functions = workspace.join(format!("ld/{}/rom-functions.x", chip_name));
    let dest_path = workspace.join("src/generated_rom_symbols.rs");

    let content = generate_module_content(&rom_functions)?;

    let mut out_file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&dest_path)
        .with_context(|| format!("Failed to open {} for writing", dest_path.display()))?;

    writeln!(out_file, "{}", content).with_context(|| {
        format!(
            "Failed to write module content for chip {} to generated_rom_symbols.rs",
            chip_name
        )
    })?;

    Ok(())
}

// Helper function to handle the parsing and content generation.
fn generate_module_content(rom_functions: &Path) -> Result<String, Error> {
    let mut content = String::new();
    content.push_str("pub mod rom_symbols {\n");

    let include_re = Regex::new(r#"^\s*INCLUDE\s+"([^"]+)""#)?;
    let assignment_re = Regex::new(r"^\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*=\s*(.*?);")?;
    let provide_re = Regex::new(r"PROVIDE\s*\(\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*=\s*(.*?)\);")?;

    let base_dir = rom_functions
        .parent()
        .context("rom-functions.x has no parent directory")?;

    let rom_functions_content = fs::read_to_string(rom_functions)
        .with_context(|| format!("Failed to read {}", rom_functions.display()))?;

    let mut linker_scripts = Vec::new();

    for line in rom_functions_content.lines() {
        if let Some(caps) = include_re.captures(line) {
            let rel_path = &caps[1];
            let full_path = base_dir.join(rel_path);

            if !full_path.exists() {
                return Err(anyhow::anyhow!(
                    "Included linker script not found: {}",
                    full_path.display()
                ));
            }

            linker_scripts.push(full_path);
        }
    }

    let mut generated_symbols = HashSet::new();

    for script_path in linker_scripts {
        let script_content = fs::read_to_string(&script_path)
            .with_context(|| format!("Failed to read {}", script_path.display()))?;

        let mut in_block_comment = false;

        for raw_line in script_content.lines() {
            let mut line = raw_line;

            // Handle /* ... */ block comments
            if in_block_comment {
                if let Some(end) = line.find("*/") {
                    line = &line[end + 2..];
                    in_block_comment = false;
                } else {
                    continue;
                }
            }

            if let Some(start) = line.find("/*") {
                if let Some(end) = line[start + 2..].find("*/") {
                    let end = start + 2 + end;
                    let cleaned = format!("{}{}", &line[..start], &line[end + 2..]);
                    line = Box::leak(cleaned.into_boxed_str());
                } else {
                    line = &line[..start];
                    in_block_comment = true;
                }
            }

            // Remove // comments
            let line = line.split("//").next().unwrap().trim();

            if line.is_empty() {
                continue;
            }

            let symbol = if let Some(caps) = assignment_re.captures(line) {
                Some(caps[1].to_owned())
            } else if let Some(caps) = provide_re.captures(line) {
                Some(caps[1].to_owned())
            } else {
                None
            };

            if let Some(name) = symbol {
                let rust_ident = name.trim_start_matches('_').to_string();

                if generated_symbols.insert(rust_ident.clone()) {
                    content.push_str(&format!("pub fn {}() {{}}\n", rust_ident));
                }
            }
        }
    }

    content.push_str("}\n");
    Ok(content)
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::Write, path::Path};

    use tempfile::TempDir;

    use super::*;

    const CONTENT: &str = r#"
memset = 0x40000488;
PROVIDE( abs = 0x40000558 );
EXTERN(__mktime);
PROVIDE ( mktime = __mktime );
syscall_table_ptr = 0x3fcdffd8;

PROVIDE( tlsf_create = 0x400002dc );
PROVIDE( tlsf_create_with_pool = 0x400002e0 );

PROVIDE( heap_tlsf_table_ptr = 0x3fcdffec );

mz_adler32 = 0x400000bc;
__absvsi2 = 0x400008ac;
_rom_chip_id = 0x40000010;

/** ROM APIs
 */
PROVIDE ( esp_rom_crc32_le = crc32_le );

/***************************************
 Group heap
 ***************************************/

/* Functions */
rtc_get_reset_reason = 0x40000018;

/* Data (.data, .bss, .rodata) */
PROVIDE( multi_heap_get_allocated_size = 0x3fcdffec );

/**
 * Multi heap function
 */

PROVIDE (multi_heap_malloc = multi_heap_malloc_impl);

/* 
_should_not_include = 0x00000000; 
*/

/* random comments
random comments
random = 0x12341234 
/*
/* 


_should_not_include1 = 0x40000010;


*/

/**
 * 
 * _should_not_include2 = 0x44444444;
 * 
 */
PROVIDE (multi_heap_register = multi_heap_register_impl);

"#;

    #[test]
    fn test_generate_module_content() -> Result<(), Error> {
        let temp_dir = TempDir::new().expect("failed to create temporary directory");
        let base = temp_dir.path();

        // Create directory structure
        let rom_dir = base.join("rom");
        std::fs::create_dir(&rom_dir)?;

        // Create test .ld file inside rom/
        let ld_file = rom_dir.join("tmp_rom.ld");
        {
            let mut file = File::create(&ld_file)?;
            file.write_all(CONTENT.as_bytes())?;
        }

        // Create rom-functions.x that includes the ld file
        let rom_functions = base.join("rom-functions.x");
        {
            let mut file = File::create(&rom_functions)?;
            writeln!(file, r#"INCLUDE "rom/tmp_rom.ld""#)?;
        }

        let generated = generate_module_content(&rom_functions)?;

        // Basic structure
        assert!(generated.contains("pub mod rom_symbols"));

        // Expected symbols
        assert!(generated.contains("pub fn memset()"));
        assert!(generated.contains("pub fn abs()"));
        assert!(generated.contains("pub fn mktime()"));
        assert!(generated.contains("pub fn syscall_table_ptr()"));
        assert!(generated.contains("pub fn esp_rom_crc32_le()"));
        assert!(generated.contains("pub fn tlsf_create()"));
        assert!(generated.contains("pub fn tlsf_create_with_pool()"));
        assert!(generated.contains("pub fn heap_tlsf_table_ptr()"));
        assert!(generated.contains("pub fn mz_adler32()"));
        assert!(generated.contains("pub fn absvsi2()"));
        assert!(generated.contains("pub fn rom_chip_id()"));
        assert!(generated.contains("pub fn multi_heap_malloc()"));
        assert!(generated.contains("pub fn multi_heap_get_allocated_size()"));
        assert!(generated.contains("pub fn rtc_get_reset_reason()"));
        assert!(generated.contains("pub fn multi_heap_register()"));

        // Should NOT be included
        assert!(!generated.contains("pub fn random()"));
        assert!(!generated.contains("_should_not_include"));

        Ok(())
    }
}
