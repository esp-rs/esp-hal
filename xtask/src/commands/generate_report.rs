use std::{
    fs,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use clap::Args;
use regex::Regex;

/// Generate a combined report from individual binary size reports.
#[derive(Debug, Args)]
pub struct ReportArgs {
    /// Input file path for parsing report data.
    #[arg(long)]
    input: PathBuf,
}

/// Generate a combined binary size report from individual reports in the specified directory.
pub fn generate_report(workspace: &Path, args: ReportArgs) -> Result<()> {
    let input_dir = &args.input;
    if !input_dir.is_dir() {
        anyhow::bail!("Expected a directory, got: {:?}", input_dir);
    }

    let combined_path = workspace.join("report.txt");
    let mut combined = File::create(&combined_path)
        .with_context(|| format!("Failed to create report file: {:?}", combined_path))?;

    writeln!(combined, "## Binary Size Report\n")?;

    // Precompile regex filters
    let exclude_re = Regex::new(r"\.(debug_.*|symtab|strtab|comment)").unwrap();
    let code_fence_re = Regex::new(r"^```").unwrap();
    let leading_ws_re = Regex::new(r"^[ \t]+").unwrap();

    // Iterate through files like ./results/result-pr-*.txt
    let mut entries: Vec<_> = fs::read_dir(input_dir)?
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.path()
                .file_name()
                .and_then(|n| n.to_str())
                .map(|n| n.starts_with("result-pr-") && n.ends_with(".txt"))
                .unwrap_or(false)
        })
        .collect();

    // Sort for consistent ordering
    entries.sort_by_key(|e| e.path());

    for entry in entries {
        let path = entry.path();
        let soc = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .replace("result-pr-", "");

        let content =
            fs::read_to_string(&path).with_context(|| format!("Failed to read {:?}", path))?;

        let qa_diff = extract_block(
            &content,
            "QA_DIFF<<EOF",
            "EOF",
            &exclude_re,
            &code_fence_re,
            &leading_ws_re,
        );
        let dhcp_diff = extract_block(
            &content,
            "DHCP_DIFF<<EOF",
            "EOF",
            &exclude_re,
            &code_fence_re,
            &leading_ws_re,
        );

        writeln!(combined, "### `{}`\n", soc)?;
        writeln!(
            combined,
            "##### sleep-timer Diff (PR vs. Base)\n```\n{}\n```\n",
            qa_diff
        )?;
        writeln!(
            combined,
            "##### embassy-dhcp Diff (PR vs. Base)\n```\n{}\n```\n",
            dhcp_diff
        )?;
    }

    Ok(())
}

fn extract_block(
    content: &str,
    start_marker: &str,
    end_marker: &str,
    exclude_re: &Regex,
    code_fence_re: &Regex,
    leading_ws_re: &Regex,
) -> String {
    let mut inside = false;
    let mut lines = Vec::new();

    for line in content.lines() {
        if line.contains(start_marker) {
            inside = true;
            continue;
        }
        if line.contains(end_marker) && inside {
            inside = false;
            continue;
        }
        if inside {
            if code_fence_re.is_match(line) || exclude_re.is_match(line) {
                continue;
            }
            lines.push(leading_ws_re.replace(line, "").to_string());
        }
    }

    lines.join("\n")
}
