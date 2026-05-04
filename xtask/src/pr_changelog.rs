//! Parser for changelog entries embedded in pull request descriptions.
//!
//! Pull requests may contain a `# Changelog` section and/or a `# Migration guide`
//! section. This module extracts structured data from those sections.
//!
//! # Expected format
//!
//! ```markdown
//! # Changelog
//!
//! ## esp-hal
//!
//! - Added: Support for the foo peripheral.
//! - Fixed: A bug in bar.
//!
//! ## esp-hal/RMT driver
//!
//! - Changed: The RMT API has been restructured.
//!
//! # Migration guide
//!
//! ## esp-hal/RMT driver
//!
//! ### `Foo` has been renamed to `Bar`
//!
//! Replace all uses of `Foo` with `Bar`.
//! ```
//!
//! Migration guide sections **must** include an area (`## crate/area`) and each
//! individual breaking change **must** be introduced by an H3 heading (`### Title`)
//! before the body text.

use std::fmt;

use anyhow::{Result, bail};

/// The kind of a changelog entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EntryKind {
    Added,
    Changed,
    Fixed,
    Removed,
}

impl EntryKind {
    fn from_str(s: &str) -> Option<Self> {
        match s {
            "Added" => Some(Self::Added),
            "Changed" => Some(Self::Changed),
            "Fixed" => Some(Self::Fixed),
            "Removed" => Some(Self::Removed),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Added => "Added",
            Self::Changed => "Changed",
            Self::Fixed => "Fixed",
            Self::Removed => "Removed",
        }
    }
}

impl fmt::Display for EntryKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A single changelog entry extracted from a PR description.
#[derive(Debug, Clone)]
pub struct ChangelogEntry {
    pub kind: EntryKind,
    pub text: String,
}

/// A crate/area section extracted from a PR description.
#[derive(Debug, Clone)]
pub struct PrSection {
    /// The crate name, e.g. `"esp-hal"`.
    pub crate_name: String,
    /// The optional area within the crate, e.g. `"RMT driver"`.
    pub area: Option<String>,
    /// Changelog entries (from `# Changelog`).
    pub changelog: Vec<ChangelogEntry>,
    /// Free-form migration guide text (from `# Migration guide`).
    pub migration_guide: Option<String>,
}

impl PrSection {
    fn new(crate_name: String, area: Option<String>) -> Self {
        Self {
            crate_name,
            area,
            changelog: Vec::new(),
            migration_guide: None,
        }
    }

    pub fn has_content(&self) -> bool {
        !self.changelog.is_empty() || self.migration_guide.is_some()
    }
}

/// All changelog/migration-guide information extracted from a single PR.
#[derive(Debug, Clone)]
pub struct PrChangelog {
    pub pr_number: u64,
    pub sections: Vec<PrSection>,
}

impl PrChangelog {
    /// Parse the changelog entries from a PR description body.
    ///
    /// Returns `Ok(None)` if the body contains neither a `# Changelog` nor a
    /// `# Migration guide` section.
    pub fn parse(pr_number: u64, body: &str) -> Result<Option<Self>> {
        // Normalize CRLF (GitHub) to LF so the parser doesn't need to handle both.
        let body = &body.replace("\r\n", "\n");

        let mut sections: Vec<PrSection> = Vec::new();

        parse_changelog_block(body, &mut sections)?;
        parse_migration_guide_block(body, &mut sections)?;

        let sections: Vec<_> = sections.into_iter().filter(|s| s.has_content()).collect();

        if sections.is_empty() {
            Ok(None)
        } else {
            Ok(Some(Self {
                pr_number,
                sections,
            }))
        }
    }
}

/// Parse a `## <crate>[/<area>]` heading into `(crate_name, area)`.
fn parse_section_heading(heading: &str) -> Option<(String, Option<String>)> {
    let heading = heading.trim();
    if heading.is_empty() {
        return None;
    }
    if let Some((crate_name, area)) = heading.split_once('/') {
        let crate_name = crate_name.trim();
        let area = area.trim();
        if crate_name.is_empty() || area.is_empty() {
            None
        } else {
            Some((crate_name.to_string(), Some(area.to_string())))
        }
    } else {
        Some((heading.to_string(), None))
    }
}

/// Find or create a `PrSection` for the given crate/area pair.
fn get_or_insert_section<'a>(
    sections: &'a mut Vec<PrSection>,
    crate_name: &str,
    area: Option<&str>,
) -> &'a mut PrSection {
    if let Some(pos) = sections
        .iter()
        .position(|s| s.crate_name == crate_name && s.area.as_deref() == area)
    {
        &mut sections[pos]
    } else {
        sections.push(PrSection::new(
            crate_name.to_string(),
            area.map(|s| s.to_string()),
        ));
        sections.last_mut().unwrap()
    }
}

/// Parse the `# Changelog` block and populate `sections`.
fn parse_changelog_block(body: &str, sections: &mut Vec<PrSection>) -> Result<()> {
    let Some(after) = find_h1_section(body, "Changelog") else {
        return Ok(());
    };

    let mut current: Option<(String, Option<String>)> = None;

    for line in non_comment_lines(after) {
        if line.starts_with("# ") {
            break;
        } else if let Some(h2) = line.strip_prefix("## ") {
            match parse_section_heading(h2) {
                Some((crate_name, area)) => current = Some((crate_name, area)),
                None => bail!("Invalid section heading in `# Changelog`: `## {h2}`"),
            }
        } else if let Some(item) = line.strip_prefix("- ") {
            let Some((crate_name, area)) = &current else {
                bail!("Changelog list item found before any section heading: `- {item}`");
            };
            let entry = parse_changelog_item(item)?;
            get_or_insert_section(sections, crate_name, area.as_deref())
                .changelog
                .push(entry);
        }
    }

    Ok(())
}

/// Parse a single list item `Kind: text` into a `ChangelogEntry`.
fn parse_changelog_item(item: &str) -> Result<ChangelogEntry> {
    let Some((kind_str, text)) = item.split_once(": ") else {
        bail!(
            "Changelog list item must start with 'Added: ', 'Changed: ', 'Fixed: ' or \
             'Removed: '; got: `- {item}`"
        );
    };

    let kind = EntryKind::from_str(kind_str.trim()).ok_or_else(|| {
        anyhow::anyhow!(
            "Unknown changelog entry kind `{kind_str}`. \
             Expected one of: Added, Changed, Fixed, Removed"
        )
    })?;

    Ok(ChangelogEntry {
        kind,
        text: text.trim().to_string(),
    })
}

/// Parse the `# Migration guide` block and populate `sections`.
fn parse_migration_guide_block(body: &str, sections: &mut Vec<PrSection>) -> Result<()> {
    let Some(after) = find_h1_section(body, "Migration guide") else {
        return Ok(());
    };

    let mut current: Option<(String, Option<String>)> = None;
    let mut current_lines: Vec<&str> = Vec::new();

    for line in non_comment_lines(after) {
        if line.starts_with("# ") {
            break;
        } else if let Some(h2) = line.strip_prefix("## ") {
            flush_migration(sections, &current, &current_lines);
            current_lines.clear();
            match parse_section_heading(h2) {
                Some((crate_name, Some(area))) => {
                    current = Some((crate_name, Some(area)));
                }
                Some((crate_name, None)) => bail!(
                    "Migration guide section `## {crate_name}` must include an area \
                     (e.g. `## {crate_name}/My area`)"
                ),
                None => bail!("Invalid section heading in `# Migration guide`: `## {h2}`"),
            }
        } else {
            current_lines.push(line);
        }
    }

    flush_migration(sections, &current, &current_lines);

    Ok(())
}

fn flush_migration(
    sections: &mut Vec<PrSection>,
    current: &Option<(String, Option<String>)>,
    lines: &[&str],
) {
    if let Some((crate_name, area)) = current {
        let text = lines.join("\n").trim().to_string();
        if !text.is_empty() {
            get_or_insert_section(sections, crate_name, area.as_deref()).migration_guide =
                Some(text);
        }
    }
}

/// Iterate over lines of `text`, skipping any that fall inside `<!-- ... -->` blocks.
fn non_comment_lines(text: &str) -> impl Iterator<Item = &str> {
    let mut in_comment = false;
    text.lines().filter(move |line| {
        if in_comment {
            if line.contains("-->") {
                in_comment = false;
            }
            return false;
        }
        if line.contains("<!--") {
            // Single-line comment: <!-- foo --> — skip but don't enter comment mode.
            if !line.contains("-->") {
                in_comment = true;
            }
            return false;
        }
        true
    })
}

/// Return the body text that follows a `# <title>` H1 heading (case-insensitive).
///
/// Assumes LF-only line endings (callers must normalize first).
/// Returns `None` if no matching heading is found.
fn find_h1_section<'a>(body: &'a str, title: &str) -> Option<&'a str> {
    let needle = format!("# {title}");
    let mut offset = 0usize;
    for line in body.lines() {
        let line_len = line.len();
        if line.trim().eq_ignore_ascii_case(&needle) {
            return Some(body[offset + line_len..].trim_start_matches('\n'));
        }
        offset += line_len + 1; // +1 for '\n'
    }
    None
}

/// Validate a PR description body, returning human-readable error messages.
///
/// Returns an empty `Vec` if the body is valid (or has no changelog sections at all).
pub fn validate(body: &str) -> Vec<String> {
    let body = &body.replace("\r\n", "\n");

    let mut errors = Vec::new();

    if let Some(after) = find_h1_section(body, "Changelog") {
        let mut in_section = false;
        for line in non_comment_lines(after) {
            if line.starts_with("# ") {
                break;
            } else if let Some(h2) = line.strip_prefix("## ") {
                if parse_section_heading(h2).is_some() {
                    in_section = true;
                } else {
                    in_section = false;
                    errors.push(format!(
                        "Invalid section heading in `# Changelog`: `## {h2}`"
                    ));
                }
            } else if let Some(item) = line.strip_prefix("- ") {
                if !in_section {
                    errors.push(format!(
                        "List item found before any section heading: `- {item}`"
                    ));
                } else if let Err(e) = parse_changelog_item(item) {
                    errors.push(e.to_string());
                }
            }
        }
    }

    if let Some(after) = find_h1_section(body, "Migration guide") {
        let mut in_section = false;
        // True while inside a valid section but no `### title` has been seen yet.
        let mut awaiting_h3 = false;

        for line in non_comment_lines(after) {
            if line.starts_with("# ") {
                break;
            } else if let Some(h2) = line.strip_prefix("## ") {
                in_section = false;
                awaiting_h3 = false;
                match parse_section_heading(h2) {
                    Some((_, Some(_))) => {
                        in_section = true;
                        awaiting_h3 = true;
                    }
                    Some((crate_name, None)) => {
                        errors.push(format!(
                            "Migration guide section `## {crate_name}` must include an area \
                             (e.g. `## {crate_name}/My area`)"
                        ));
                    }
                    None => {
                        errors.push(format!(
                            "Invalid section heading in `# Migration guide`: `## {h2}`"
                        ));
                    }
                }
            } else if let Some(h3) = line.strip_prefix("### ") {
                if !in_section {
                    errors.push(format!(
                        "`### {h3}` found before any `## crate/area` heading"
                    ));
                } else if h3.trim().is_empty() {
                    errors.push(
                        "Migration guide `### ` heading must have a non-empty title".to_string(),
                    );
                } else {
                    awaiting_h3 = false;
                }
            } else if awaiting_h3 && !line.trim().is_empty() {
                // Body text appearing before the first `###` in a section.
                errors.push(format!(
                    "Migration guide body text must be preceded by a `### Title` heading; \
                     found: `{line}`"
                ));
                awaiting_h3 = false; // report only the first offending line per section
            }
        }
    }

    errors
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE_BODY: &str = "\
This PR de-clutters the tantulum gluon autosequencers.

# Changelog

## esp-hal/Polymetric encabulator driver

- Added: The distributed heap-matrix
- Changed: The async flux-compiler overclocked its own dependency vortex again.
- Removed: A polyphase micro-kernel

## esp-radio

- Changed: A runaway garbage collector vaporized the entire semantic token field.

# Migration guide

## esp-hal/LOL-area

### The entropy pipeline must be reconfigured

The entropy-driven commit pipeline must be re-aligned with a synthetic opcode vortex.
All transient byte-lattices should undergo recursive de-serialization.
";

    #[test]
    fn parse_full_example() {
        let changelog = PrChangelog::parse(42, EXAMPLE_BODY).unwrap().unwrap();

        assert_eq!(changelog.pr_number, 42);
        assert_eq!(changelog.sections.len(), 3);

        let hal = changelog
            .sections
            .iter()
            .find(|s| {
                s.crate_name == "esp-hal"
                    && s.area.as_deref() == Some("Polymetric encabulator driver")
            })
            .unwrap();
        assert_eq!(hal.changelog.len(), 3);
        assert_eq!(hal.changelog[0].kind, EntryKind::Added);
        assert_eq!(hal.changelog[1].kind, EntryKind::Changed);
        assert_eq!(hal.changelog[2].kind, EntryKind::Removed);

        let radio = changelog
            .sections
            .iter()
            .find(|s| s.crate_name == "esp-radio" && s.area.is_none())
            .unwrap();
        assert_eq!(radio.changelog.len(), 1);
        assert_eq!(radio.changelog[0].kind, EntryKind::Changed);

        let migration = changelog
            .sections
            .iter()
            .find(|s| s.crate_name == "esp-hal" && s.area.as_deref() == Some("LOL-area"))
            .unwrap();
        assert!(migration.migration_guide.is_some());
        assert!(migration.changelog.is_empty());
    }

    #[test]
    fn parse_no_sections() {
        let body = "This PR fixes a typo in the README.\n\nNo changelog entries needed.";
        assert!(PrChangelog::parse(1, body).unwrap().is_none());
    }

    /// The PR template ships with placeholder entries inside HTML comments.
    /// Those must not be treated as real changelog entries.
    #[test]
    fn parse_template_placeholders_are_ignored() {
        // Matches the structure of .github/PULL_REQUEST_TEMPLATE.md
        let body = "\
## Thank you for your contribution!

# Changelog

<!-- Add one or more ## <crate> or ## <crate>/<area> sections below.
     Each entry must start with one of: Added, Changed, Fixed, Removed.

## esp-hal

- Added: Brief description of what was added.
- Changed: Brief description of what changed.
- Fixed: Brief description of what was fixed.
- Removed: Brief description of what was removed.

-->

# Migration guide

<!-- Add one or more ## <crate>/<area> sections below (area is required).
     Each breaking change needs a ### Title heading followed by the migration steps.
     Only needed when user code must be updated.

## esp-hal/SPI

### `OldType` has been renamed to `NewType`

Replace all uses of `OldType` with `NewType`.

-->
";
        assert!(
            PrChangelog::parse(1, body).unwrap().is_none(),
            "placeholder content inside HTML comments should not produce changelog entries"
        );
        let errors = validate(body);
        assert!(
            errors.is_empty(),
            "placeholder body should pass validation: {errors:?}"
        );
    }

    #[test]
    fn validate_bad_kind() {
        let body = "# Changelog\n\n## esp-hal\n\n- Tweaked: something\n";
        let errors = validate(body);
        assert!(!errors.is_empty());
        assert!(errors[0].contains("Unknown changelog entry kind"));
    }

    #[test]
    fn validate_missing_separator() {
        let body = "# Changelog\n\n## esp-hal\n\n- Just a description without kind\n";
        let errors = validate(body);
        assert!(!errors.is_empty());
    }

    #[test]
    fn validate_clean() {
        let errors = validate(EXAMPLE_BODY);
        assert!(errors.is_empty(), "Unexpected errors: {errors:?}");
    }

    #[test]
    fn section_heading_parsing() {
        assert_eq!(
            parse_section_heading("esp-hal"),
            Some(("esp-hal".into(), None))
        );
        assert_eq!(
            parse_section_heading("esp-hal/RMT driver"),
            Some(("esp-hal".into(), Some("RMT driver".into())))
        );
        assert_eq!(
            parse_section_heading(" esp-hal / RMT driver"),
            Some(("esp-hal".into(), Some("RMT driver".into())))
        );
        // Empty heading
        assert_eq!(parse_section_heading(""), None);
        // Missing area after slash
        assert_eq!(parse_section_heading("esp-hal/"), None);
        // Missing crate name before slash — must not produce an empty crate_name
        assert_eq!(parse_section_heading("/Some area"), None);
        assert_eq!(parse_section_heading("  /  Some area"), None);
    }

    /// A PR that only has a `# Migration guide` section for a crate must NOT be
    /// treated as having a changelog entry for that crate.  The `changelog` field
    /// of the resulting `PrSection` must be empty so that coverage checks
    /// correctly identify the package as uncovered.
    #[test]
    fn migration_guide_only_section_has_empty_changelog() {
        let body = "\
# Migration guide

## esp-hal/SPI

### The transfer API changed

Some breaking change description.
";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 1);
        let section = &cl.sections[0];
        assert_eq!(section.crate_name, "esp-hal");
        assert_eq!(section.area.as_deref(), Some("SPI"));
        assert!(
            section.changelog.is_empty(),
            "migration-guide-only section should have no changelog entries"
        );
        assert!(section.migration_guide.is_some());
    }

    #[test]
    fn validate_rejects_migration_section_without_area() {
        let body = "# Migration guide\n\n## esp-hal\n\n### Title\n\nContent.\n";
        let errors = validate(body);
        assert!(
            !errors.is_empty(),
            "expected error for migration section without area"
        );
        assert!(
            errors.iter().any(|e| e.contains("must include an area")),
            "unexpected errors: {errors:?}"
        );
    }

    #[test]
    fn validate_rejects_migration_body_before_h3() {
        let body =
            "# Migration guide\n\n## esp-hal/SPI\n\nBody text before any H3.\n\n### Title\n\nMore.\n";
        let errors = validate(body);
        assert!(
            !errors.is_empty(),
            "expected error for body text appearing before ### heading"
        );
        assert!(
            errors.iter().any(|e| e.contains("### Title")),
            "unexpected errors: {errors:?}"
        );
    }

    #[test]
    fn validate_accepts_valid_migration_guide() {
        let body = "\
# Migration guide

## esp-hal/SPI

### The foobulator has been encabulated

Update your call sites from `foo()` to `bar()`.

### Another breaking change

Do something else.
";
        let errors = validate(body);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    }

    /// A heading like `## /Some area` must be rejected by the validator, not silently
    /// accepted with an empty crate name that would match the workspace root.
    #[test]
    fn validate_rejects_empty_crate_name() {
        let body = "# Changelog\n\n## /Some area\n\n- Added: something\n";
        let errors = validate(body);
        assert!(
            !errors.is_empty(),
            "expected a validation error for `## /Some area` but got none"
        );
        assert!(
            errors.iter().any(|e| e.contains("Invalid section heading")),
            "unexpected errors: {errors:?}"
        );
    }
}
