//! Parser for changelog entries embedded in pull request descriptions.
//!
//! Pull requests may contain a `# Changelog` section and/or a `# Migration guide`
//! section. Changelog entries may also be placed inside
//! `<details><summary>Changelog</summary>…</details>` (GitHub's collapsible region).
//! This module extracts structured data from those sections.
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

/// Exact text of the per-crate changelog exemption marker.
///
/// A PR author can write `- No changelog necessary.` as the sole item under a
/// `## <crate>` section in the `# Changelog` block to explicitly declare that
/// the crate's changes do not need a public changelog entry.  The CI coverage
/// check will then treat that crate as "covered", so the `skip-changelog` label
/// is not required.
///
/// The marker must be the **only** item in its section; mixing it with real
/// changelog entries is an error.
pub const NO_CHANGELOG_MARKER: &str = "No changelog necessary.";

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
    /// Set when the author wrote `- No changelog necessary.` to explicitly
    /// declare that this crate's changes need no public changelog entry.
    pub exempted: bool,
}

impl PrSection {
    fn new(crate_name: String, area: Option<String>) -> Self {
        Self {
            crate_name,
            area,
            changelog: Vec::new(),
            migration_guide: None,
            exempted: false,
        }
    }

    pub fn has_content(&self) -> bool {
        self.exempted || !self.changelog.is_empty() || self.migration_guide.is_some()
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
///
/// Changelog entries may appear after a `# Changelog` heading and/or inside
/// `<details><summary>Changelog</summary> … </details>` (GitHub PR folding).
fn parse_changelog_block(body: &str, sections: &mut Vec<PrSection>) -> Result<()> {
    let detail_parts = collect_changelog_details_parts(body);
    let ranges: Vec<(usize, usize)> = detail_parts.iter().map(|p| p.span).collect();

    if let Some(after) = find_h1_section_outside_ranges(body, "Changelog", &ranges) {
        let cleaned = scrub_h1_changelog_tail(body, after, &ranges);
        parse_changelog_subsection(&cleaned, sections)?;
    }
    for part in &detail_parts {
        parse_changelog_subsection(part.markdown, sections)?;
    }
    Ok(())
}

/// Parse one contiguous changelog region (markdown after `# Changelog` and/or inside
/// a Changelog `<details>` block).
fn parse_changelog_subsection(text: &str, sections: &mut Vec<PrSection>) -> Result<()> {
    let after = strip_optional_h1(text, "Changelog");
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
            let section = get_or_insert_section(sections, crate_name, area.as_deref());
            if item.trim() == NO_CHANGELOG_MARKER {
                if !section.changelog.is_empty() {
                    bail!(
                        "`- {NO_CHANGELOG_MARKER}` in `## {crate_name}` cannot appear alongside \
                         real changelog entries; use one or the other"
                    );
                }
                section.exempted = true;
            } else {
                if section.exempted {
                    bail!(
                        "Changelog entry in `## {crate_name}` cannot appear alongside \
                         `- {NO_CHANGELOG_MARKER}`; use one or the other"
                    );
                }
                let entry = parse_changelog_item(item)?;
                section.changelog.push(entry);
            }
        }
    }

    Ok(())
}

/// Parse a single list item `Kind: text` into a `ChangelogEntry`.
fn parse_changelog_item(item: &str) -> Result<ChangelogEntry> {
    let Some((kind_str, text)) = item.split_once(": ") else {
        bail!(
            r#"Changelog list item must start with a category followed by a colon, or '- No changelog necessary.'; got: `- {item}`.

Accepted categories: Added, Changed, Fixed, Removed"#
        );
    };

    let kind = EntryKind::from_str(kind_str.trim()).ok_or_else(|| {
        anyhow::anyhow!(
            "Unknown changelog entry kind `{kind_str}`. \
             Expected one of: Added, Changed, Fixed, Removed"
        )
    })?;

    let text = text.trim();
    if text.is_empty() {
        bail!("Changelog entry description must not be empty: `- {item}`");
    }

    Ok(ChangelogEntry {
        kind,
        text: text.to_string(),
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

/// Like [`find_h1_section`], but ignores `# <title>` headings whose first character falls inside
/// one of the `[start, end)` byte ranges (e.g. `# Changelog` nested in a `<details>` block whose
/// body is parsed separately).
fn find_h1_section_outside_ranges<'a>(
    body: &'a str,
    title: &str,
    ranges: &[(usize, usize)],
) -> Option<&'a str> {
    let needle = format!("# {title}");
    let mut offset = 0usize;
    for line in body.lines() {
        let line_len = line.len();
        if line.trim().eq_ignore_ascii_case(&needle) {
            let excluded = ranges.iter().any(|&(s, e)| offset >= s && offset < e);
            if !excluded {
                return Some(body[offset + line_len..].trim_start_matches('\n'));
            }
        }
        offset += line_len + 1;
    }
    None
}

/// Remove `<details>…</details>` regions that hold a Changelog summary from `tail`, where `tail`
/// must be a substring of `body` (see [`parse_changelog_block`]). Prevents folded regions from
/// being parsed twice when a top-level `# Changelog` also wraps entries in `<details>`.
fn scrub_h1_changelog_tail(body: &str, tail: &str, detail_spans: &[(usize, usize)]) -> String {
    let base = (tail.as_ptr() as usize) - (body.as_ptr() as usize);
    let tail_end = base + tail.len();
    let mut rel_spans: Vec<(usize, usize)> = detail_spans
        .iter()
        .filter_map(|&(s, e)| {
            let rs = s.max(base);
            let re = e.min(tail_end);
            (rs < re).then_some((rs - base, re - base))
        })
        .collect();
    rel_spans.sort_by_key(|k| k.0);
    if rel_spans.is_empty() {
        return tail.to_string();
    }
    let mut out = String::with_capacity(tail.len());
    let mut pos = 0usize;
    for (s, e) in rel_spans {
        out.push_str(&tail[pos..s]);
        out.push('\n');
        pos = e;
    }
    out.push_str(&tail[pos..]);
    out
}

/// Returns the tail after `text`'s first line when that line is `# <title>` (case-insensitive),
/// otherwise returns `text` unchanged.
fn strip_optional_h1<'a>(text: &'a str, title: &str) -> &'a str {
    let t = text.trim_start();
    let needle = format!("# {title}");
    let Some(first) = t.lines().next() else {
        return t;
    };
    if first.trim().eq_ignore_ascii_case(&needle) {
        let rest = &t[first.len()..];
        return rest.trim_start_matches('\n').trim_start();
    }
    t
}

fn starts_with_ignore_case(haystack: &str, needle: &str) -> bool {
    let h = haystack.as_bytes();
    let n = needle.as_bytes();
    h.len() >= n.len()
        && h[..n.len()]
            .iter()
            .zip(n.iter())
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// Case-insensitive substring search (both strings must be ASCII-only for tags).
fn find_case_insensitive(haystack: &str, needle: &str) -> Option<usize> {
    if needle.is_empty() {
        return Some(0);
    }
    let ned = needle.as_bytes();
    haystack.as_bytes().windows(ned.len()).position(|w| {
        w.iter()
            .zip(ned.iter())
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
    })
}

/// The slice must begin with `<details`; the next character after the word is `>` or whitespace.
fn is_opening_details_tag(s: &str) -> bool {
    if !starts_with_ignore_case(s, "<details") {
        return false;
    }
    let rest = &s["<details".len()..];
    rest.chars()
        .next()
        .is_some_and(|c| c == '>' || c.is_whitespace())
}

/// One `<details><summary>Changelog</summary> … </details>` region in a PR body.
struct ChangelogDetailsPart<'a> {
    /// Byte range `[start, end)` covering the full `<details>…</details>` in the original body.
    span: (usize, usize),
    markdown: &'a str,
}

/// Changelog bodies from `<details><summary>Changelog</summary> … </details>` blocks.
fn collect_changelog_details_parts<'a>(body: &'a str) -> Vec<ChangelogDetailsPart<'a>> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos < body.len() {
        let rest = &body[pos..];
        let Some(rel) = find_case_insensitive(rest, "<details") else {
            break;
        };
        let open_at = pos + rel;
        let from_open = &body[open_at..];
        if !is_opening_details_tag(from_open) {
            pos = open_at + 1;
            continue;
        }
        let after_tag_word = &from_open["<details".len()..];
        let Some(gt) = after_tag_word.find('>') else {
            break;
        };
        let inner_start = open_at + "<details".len() + gt + 1;
        let inner_region = &body[inner_start..];
        let Some((inner_content, consumed)) = take_balanced_details_inner(inner_region) else {
            break;
        };
        let block_end = inner_start + consumed;
        if let Some(md) = body_after_changelog_summary(inner_content) {
            out.push(ChangelogDetailsPart {
                span: (open_at, block_end),
                markdown: md,
            });
        }
        pos = block_end;
    }
    out
}

/// `s` begins immediately after the opening `<details…>` `>`. Returns `(inner, total_bytes)`.
fn take_balanced_details_inner(s: &str) -> Option<(&str, usize)> {
    let mut i = 0usize;
    let mut depth = 1i32;
    while i < s.len() {
        if is_opening_details_tag(&s[i..]) {
            let after = &s[i + "<details".len()..];
            let gt = after.find('>')?;
            i += "<details".len() + gt + 1;
            depth += 1;
            continue;
        }
        if starts_with_ignore_case(&s[i..], "</details>") {
            depth -= 1;
            if depth == 0 {
                return Some((&s[..i], i + "</details>".len()));
            }
            i += "</details>".len();
            continue;
        }
        let adv = s[i..].chars().next()?.len_utf8();
        i += adv;
    }
    None
}

/// `details_inner` is the markup between `<details…>` and its matching `</details>`.
fn body_after_changelog_summary(details_inner: &str) -> Option<&str> {
    let s = details_inner.trim_start();
    let summary_rel = find_case_insensitive(s, "<summary")?;
    let after_summary_word = &s[summary_rel + "<summary".len()..];
    let gt = after_summary_word.find('>')?;
    let summary_value_and_rest = &after_summary_word[gt + 1..];
    let close_rel = find_case_insensitive(summary_value_and_rest, "</summary>")?;
    let title = summary_value_and_rest[..close_rel].trim();
    if !title.eq_ignore_ascii_case("Changelog") {
        return None;
    }
    let after = &summary_value_and_rest[close_rel + "</summary>".len()..];
    Some(after.trim_start())
}

fn validate_changelog_subsection(after: &str, errors: &mut Vec<String>) {
    let after = strip_optional_h1(after, "Changelog");
    let mut in_section = false;
    let mut section_exempted = false;
    let mut section_has_entries = false;

    for line in non_comment_lines(after) {
        if line.starts_with("# ") {
            break;
        } else if let Some(h2) = line.strip_prefix("## ") {
            section_exempted = false;
            section_has_entries = false;
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
            } else if item.trim() == NO_CHANGELOG_MARKER {
                if section_has_entries {
                    errors.push(format!(
                        "`- {NO_CHANGELOG_MARKER}` cannot appear alongside real changelog \
                         entries in the same section; use one or the other"
                    ));
                }
                section_exempted = true;
            } else {
                if section_exempted {
                    errors.push(format!(
                        "Changelog entry cannot appear alongside `- {NO_CHANGELOG_MARKER}` \
                         in the same section; use one or the other"
                    ));
                }
                if let Err(e) = parse_changelog_item(item) {
                    errors.push(e.to_string());
                } else {
                    section_has_entries = true;
                }
            }
        }
    }
}

/// Validate a PR description body, returning human-readable error messages.
///
/// Returns an empty `Vec` if the body is valid (or has no changelog sections at all).
pub fn validate(body: &str) -> Vec<String> {
    let body = &body.replace("\r\n", "\n");

    let mut errors = Vec::new();

    let detail_parts = collect_changelog_details_parts(body);
    let ranges: Vec<(usize, usize)> = detail_parts.iter().map(|p| p.span).collect();

    if let Some(after) = find_h1_section_outside_ranges(body, "Changelog", &ranges) {
        let cleaned = scrub_h1_changelog_tail(body, after, &ranges);
        validate_changelog_subsection(&cleaned, &mut errors);
    }
    for part in &detail_parts {
        validate_changelog_subsection(part.markdown, &mut errors);
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

    #[test]
    fn parse_changelog_inside_details_without_hash_heading() {
        let body = "\
<details>
<summary>Changelog</summary>

## esp-hal

- Added: A change from a folded section.
</details>";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 1);
        let hal = &cl.sections[0];
        assert_eq!(hal.crate_name, "esp-hal");
        assert_eq!(hal.changelog.len(), 1);
        assert_eq!(hal.changelog[0].kind, EntryKind::Added);
        let errors = validate(body);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    }

    #[test]
    fn parse_changelog_details_with_unicode_in_body_does_not_panic() {
        let body = "\
<details>
<summary>Changelog</summary>

## esp-hal

- Added: See “Peripheral support” table and link each ❌ in `esp-hal/README.md` to the tracking issue.
</details>";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 1);
        assert_eq!(cl.sections[0].changelog.len(), 1);
    }

    #[test]
    fn parse_changelog_details_with_optional_hash_heading_no_duplicate() {
        let body = "\
<details>
<summary>Changelog</summary>

# Changelog

## esp-hal

- Changed: Tweaked after an inner H1.
</details>";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 1);
        assert_eq!(cl.sections[0].changelog.len(), 1);
        assert_eq!(cl.sections[0].changelog[0].kind, EntryKind::Changed);
    }

    #[test]
    fn parse_changelog_top_level_plus_details_merges() {
        let body = "\
# Changelog

## esp-radio

- Fixed: Top-level section.

<details>
<summary>Changelog</summary>

## esp-hal

- Added: From details only.
</details>";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 2);
        assert!(
            cl.sections
                .iter()
                .any(|s| s.crate_name == "esp-radio" && s.changelog.len() == 1)
        );
        assert!(
            cl.sections
                .iter()
                .any(|s| s.crate_name == "esp-hal" && s.changelog.len() == 1)
        );
        let errors = validate(body);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
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
    fn validate_empty_description() {
        let body = "# Changelog\n\n## esp-hal\n\n- Added: \n";
        let errors = validate(body);
        assert!(!errors.is_empty(), "expected error for empty description");
        assert!(
            errors.iter().any(|e| e.contains("must not be empty")),
            "unexpected errors: {errors:?}"
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
        let body = "# Migration guide\n\n## esp-hal/SPI\n\nBody text before any H3.\n\n### Title\n\nMore.\n";
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

    // ---- NO_CHANGELOG_MARKER tests ----

    #[test]
    fn parse_exemption_marker() {
        let body = "# Changelog\n\n## esp-metadata\n\n- No changelog necessary.\n";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 1);
        let section = &cl.sections[0];
        assert_eq!(section.crate_name, "esp-metadata");
        assert!(section.exempted, "section should be marked as exempted");
        assert!(section.changelog.is_empty());
    }

    #[test]
    fn validate_accepts_exemption_marker() {
        let body = "# Changelog\n\n## esp-metadata\n\n- No changelog necessary.\n";
        let errors = validate(body);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    }

    #[test]
    fn parse_rejects_marker_mixed_with_entries_marker_first() {
        let body = "# Changelog\n\n## esp-hal\n\n- No changelog necessary.\n- Added: Something.\n";
        assert!(PrChangelog::parse(1, body).is_err());
    }

    #[test]
    fn parse_rejects_marker_mixed_with_entries_entry_first() {
        let body = "# Changelog\n\n## esp-hal\n\n- Added: Something.\n- No changelog necessary.\n";
        assert!(PrChangelog::parse(1, body).is_err());
    }

    #[test]
    fn validate_rejects_marker_mixed_with_entries() {
        let body = "# Changelog\n\n## esp-hal\n\n- Added: Something.\n- No changelog necessary.\n";
        let errors = validate(body);
        assert!(
            !errors.is_empty(),
            "expected error for mixed marker + entry"
        );
        assert!(
            errors.iter().any(|e| e.contains("No changelog necessary")),
            "unexpected errors: {errors:?}"
        );
    }

    #[test]
    fn exempted_section_has_content() {
        let mut section = PrSection::new("esp-metadata".into(), None);
        assert!(!section.has_content());
        section.exempted = true;
        assert!(section.has_content());
    }

    /// Two crates in one PR: one exempted, one with a real entry.
    #[test]
    fn parse_mixed_exempted_and_real() {
        let body = "\
# Changelog

## esp-hal

- Added: The new thing.

## esp-metadata

- No changelog necessary.
";
        let cl = PrChangelog::parse(1, body).unwrap().unwrap();
        assert_eq!(cl.sections.len(), 2);
        let hal = cl
            .sections
            .iter()
            .find(|s| s.crate_name == "esp-hal")
            .unwrap();
        assert!(!hal.exempted);
        assert_eq!(hal.changelog.len(), 1);
        let meta = cl
            .sections
            .iter()
            .find(|s| s.crate_name == "esp-metadata")
            .unwrap();
        assert!(meta.exempted);
        assert!(meta.changelog.is_empty());
    }
}
