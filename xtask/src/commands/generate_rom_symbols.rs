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

    let content = generate_module_content(&rom_ld_dir, &target_chip)?;

    let mut out_file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&dest_path)
        .context(format!(
            "Failed to open {} for writing",
            dest_path.display()
        ))?;

    writeln!(out_file, "{}", content).context(format!(
        "Failed to write module content for chip {} to generated_rom_symbols.rs",
        chip_name
    ))?;

    Ok(())
}

// Helper function to handle the parsing and content generation.
fn generate_module_content(rom_ld_dir: &Path, _target_chip: &Chip) -> Result<String, Error> {
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

    content.push_str(&format!("pub mod rom_symbols {{\n"));

    let mut generated_symbols: HashSet<String> = HashSet::new();

    let assignment_re = Regex::new(r"^\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*[=]\s*(.*?);")?;
    let provide_re = Regex::new(r"PROVIDE\s*\(\s*([_a-zA-Z][_a-zA-Z0-9]*)\s*=\s*(.*?)\);")?;

    for script_path in linker_scripts {
        let mut in_block_comment = false;
        let script_content = fs::read_to_string(&script_path)?;

        for line in script_content.lines() {
            let mut line = line;

            // Handle block comments
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
                    // /* ... */ on the same line
                    let end = start + 2 + end;
                    let mut cleaned = String::new();
                    cleaned.push_str(&line[..start]);
                    cleaned.push_str(&line[end + 2..]);
                    line = Box::leak(cleaned.into_boxed_str());
                } else {
                    // start of multi-line block comment
                    line = &line[..start];
                    in_block_comment = true;
                }
            }

            // Remove // comments
            let line = line.split("//").next().unwrap().trim();

            if line.is_empty() {
                continue;
            }

            let mut symbol_name: Option<String> = None;

            if let Some(caps) = assignment_re.captures(line) {
                symbol_name = Some(caps[1].to_owned());
            } else if let Some(caps) = provide_re.captures(line) {
                symbol_name = Some(caps[1].to_owned());
            }

            if let Some(name) = symbol_name {
                let rust_ident = name.trim_start_matches('_').to_owned();

                if generated_symbols.insert(rust_ident.clone()) {
                    content.push_str(&format!("pub fn {}() {{}}\n\n", rust_ident));
                }
            }
        }
    }

    // Close the chip module
    content.push_str("}\n");

    Ok(content)
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::Write, path::Path};

    use tempfile::TempDir;

    use super::*;

    // Helper to create a temporary .ld file
    fn create_test_file(dir: &Path) -> Result<(), Error> {
        let test_file_path = dir.join("tmp_rom.ld");
        let mut file = File::create(&test_file_path)?;

        let content = r#"
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
Bla bla 
bla = 0x12341234 
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

        file.write_all(content.as_bytes())?;
        Ok(())
    }

    #[test]
    fn test_generate_module_content() -> Result<(), Error> {
        let temp_dir = TempDir::new().expect("failed to create temporary directory");
        let dir_path = temp_dir.path();

        create_test_file(dir_path)?;

        let generated = generate_module_content(dir_path, &Chip::Esp32)?;

        // Check that generated Rust module starts correctly
        assert!(generated.contains("pub mod rom_symbols"));

        // Check expected parsed symbols
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
        assert!(generated.contains("pub fn heap_tlsf_table_ptr()"));
        assert!(generated.contains("pub fn multi_heap_malloc()"));
        assert!(generated.contains("pub fn multi_heap_get_allocated_size()"));
        assert!(generated.contains("pub fn rtc_get_reset_reason()"));
        assert!(generated.contains("pub fn multi_heap_register()"));

        // Should NOT be included
        assert!(!generated.contains("pub fn bla()"));
        assert!(!generated.contains("pub fn _should_not_include()"));
        assert!(!generated.contains("pub fn _should_not_include1()"));
        assert!(!generated.contains("pub fn _should_not_include2()"));

        Ok(())
    }
}
