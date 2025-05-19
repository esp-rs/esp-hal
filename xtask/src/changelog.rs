use std::{fmt::Display, iter::Peekable, str::Lines};

use anyhow::{Context, Result, bail};

use crate::Package;

#[derive(Debug)]
pub(crate) struct Changelog {
    entries: Vec<ChangelogForVersion>,
}

impl Changelog {
    pub fn parse(changelog: &str) -> Result<Self> {
        let mut lines = changelog.lines().peekable();

        let mut this = Self {
            entries: Vec::new(),
        };

        // Remove header
        let mut parsed_changelog = false;
        while let Some(line) = lines.peek() {
            if line.starts_with("## ") {
                let version = ChangelogForVersion::parse(&mut lines)?;
                // Add the version to the changelog
                this.entries.push(version);
                parsed_changelog = true;
            } else if parsed_changelog {
                if line.starts_with('[') {
                    let (version, tag) = parse_tag_link(line)?;
                    let entry = this
                        .entries
                        .iter_mut()
                        .find(|e| e.version == version)
                        .unwrap();
                    entry.tag = Some(tag.to_string());
                }
                lines.next();
            } else {
                lines.next();
            }
        }

        Ok(this)
    }

    pub fn finalize(
        &mut self,
        package: Package,
        version: &semver::Version,
        timestamp: jiff::Timestamp,
    ) {
        // Find the entry for the version
        let Some(unreleased_entry) = self.entries.iter_mut().find(|e| e.version == "Unreleased")
        else {
            // No unreleased changes.
            return;
        };

        unreleased_entry.version = format!("v{version}");
        unreleased_entry.tag = Some(package.tag(version));
        unreleased_entry.date = Some(timestamp.strftime("%Y-%m-%d").to_string());

        self.entries.insert(
            0,
            ChangelogForVersion {
                version: "Unreleased".to_string(),
                date: None,
                tag: None,
                groups: vec![
                    ChangelogGroup::new("Added"),
                    ChangelogGroup::new("Changed"),
                    ChangelogGroup::new("Fixed"),
                    ChangelogGroup::new("Removed"),
                ],
            },
        );
    }
}

impl Display for Changelog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            r#"# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

"#
        )?;
        for entry in &self.entries {
            write!(f, "{entry}")?;
        }

        let mut previous = None;
        for entry in self.entries.iter().rev() {
            let version_tag = if let Some(tag) = entry.tag.as_ref() {
                tag
            } else if entry.version == "Unreleased" {
                "HEAD"
            } else {
                continue;
            };

            if let Some(previous) = previous {
                writeln!(
                    f,
                    "[{}]: https://github.com/esp-rs/esp-hal/compare/{previous}...{version_tag}",
                    entry.version
                )?;
            } else if entry.version != "Unreleased" {
                writeln!(
                    f,
                    "[{}]: https://github.com/esp-rs/esp-hal/releases/tag/{version_tag}",
                    entry.version
                )?;
            }

            previous = Some(version_tag);
        }

        Ok(())
    }
}

#[derive(Debug)]
struct ChangelogForVersion {
    version: String,
    date: Option<String>,
    tag: Option<String>,
    /// The changelog groups
    groups: Vec<ChangelogGroup>,
}
impl ChangelogForVersion {
    fn parse(lines: &mut Peekable<Lines<'_>>) -> Result<Self> {
        let header = lines.next().unwrap();

        let mut parts = header.trim_start_matches("## ").splitn(2, " - ");

        let version = parts.next().unwrap().trim_matches(['[', ']', ' ']);
        log::debug!("Parsing changelog for version: {version}");
        let date = parts.next().map(|date| date.trim().to_string());

        let mut this = Self {
            version: version.to_string(),
            date,
            tag: None,
            groups: Vec::new(),
        };
        let mut current_group = None;

        while let Some(line) = lines.peek() {
            if line.starts_with("## ") {
                break;
            } else if let Some(group_header) = line.strip_prefix("### ") {
                let group_header = match group_header {
                    "Fixed" => "Fixed",
                    "Added" => "Added",
                    "Breaking" | "Changed" => "Changed",
                    "Removed" => "Removed",
                    _ => bail!("Unknown group found in changelog for {version}: {group_header}"),
                };

                if this.groups.iter().any(|g| g.name == group_header) {
                    current_group = this.groups.iter_mut().find(|g| g.name == group_header);
                } else {
                    // Create a new group
                    let group = ChangelogGroup::new(group_header);
                    this.groups.push(group);
                    current_group = this.groups.last_mut();
                }
            } else if line.starts_with("- ") {
                let line = line.trim_start_matches("- ");
                let Some(current_group) = current_group.as_deref_mut() else {
                    bail!("No group found for changelog entry in {version}: {line}");
                };

                // Add the entry to the current group
                let line = ChangelogLine::parse(line).with_context(|| {
                    format!("Could not parse changelog line for {version}: {line}")
                })?;

                current_group.lines.push(line);
            } else if line.trim().is_empty() {
                // Empty line, just skip
            } else {
                // Not ours any more.
                break;
            }
            lines.next();
        }

        Ok(this)
    }
}
impl Display for ChangelogForVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.tag.is_some() || self.version == "Unreleased" {
            write!(f, "## [{}]", self.version)?;
        } else {
            write!(f, "## {}", self.version)?;
        }
        if let Some(date) = &self.date {
            writeln!(f, " - {}", date)?;
        } else {
            writeln!(f)?;
        }
        writeln!(f)?;

        for group in self.groups.iter() {
            // Keep empty groups for Unreleased only
            if self.version == "Unreleased" || !group.lines.is_empty() {
                write!(f, "{group}")?;
            }
        }

        Ok(())
    }
}

#[derive(Debug)]
struct ChangelogGroup {
    name: &'static str,
    lines: Vec<ChangelogLine>,
}
impl ChangelogGroup {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            lines: Vec::new(),
        }
    }
}

impl Display for ChangelogGroup {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "### {}", self.name)?;
        writeln!(f)?;
        for line in &self.lines {
            writeln!(f, "{line}")?;
        }
        writeln!(f)?;
        Ok(())
    }
}

#[derive(Debug)]
struct ChangelogLine {
    indentation: usize,
    line: String,
    prs: Vec<usize>,
}
impl ChangelogLine {
    fn parse(line: &str) -> Result<Self> {
        // Format is: `- <description> (#<pr>[, #<pr>...])`
        let indentation = line.chars().take_while(|c| c.is_whitespace()).count();
        let line = line.trim();
        let (description, prs) = if let Some(pr_start) = line.rfind("(#") {
            let (description, prs) = line.split_at(pr_start);
            (description.trim(), Some(prs))
        } else {
            (line, None)
        };
        let description = description.trim_start_matches("- ").trim();

        let prs = if let Some(prs) = prs {
            prs.trim_start_matches('(')
                .trim_end_matches('.') // normalize "(#pr)."
                .trim_end_matches(')')
                .split(',')
                .map(|pr| pr.trim().trim_start_matches('#').parse::<usize>())
                .collect::<Result<Vec<_>, _>>()
                .with_context(|| format!("Could not parse PR number {prs}"))?
        } else if indentation == 0 {
            bail!("Missing PR number");
        } else {
            vec![]
        };

        Ok(Self {
            indentation,
            line: description.to_string(),
            prs,
        })
    }
}

impl Display for ChangelogLine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let indentation = " ".repeat(self.indentation);
        write!(f, "{indentation}- {}", self.line)?;
        if !self.prs.is_empty() {
            write!(f, " (")?;
            for (i, pr) in self.prs.iter().enumerate() {
                if i > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "#{pr}")?;
            }
            write!(f, ")")?;
        }

        Ok(())
    }
}

fn parse_tag_link(line: &str) -> Result<(&str, &str)> {
    let mut parts = line.split(']');
    let version = parts
        .next()
        .ok_or_else(|| anyhow::anyhow!("Could not parse tag link"))?
        .trim_start_matches('[');

    let link = parts
        .next()
        .ok_or_else(|| anyhow::anyhow!("Could not parse tag link"))?
        .trim_start_matches(':')
        .trim();

    let tag = if let Some(compare_link) =
        link.strip_prefix("https://github.com/esp-rs/esp-hal/compare/")
    {
        let mut parts = compare_link.split("..");
        parts.next(); // Skip the previous tag
        parts
            .next()
            .ok_or_else(|| anyhow::anyhow!("Could not parse tag link"))?
            .trim_start_matches('.') // normalize .. vs ...
            .trim()
    } else if let Some(release_link) =
        link.strip_prefix("https://github.com/esp-rs/esp-hal/releases/tag/")
    {
        release_link.trim()
    } else {
        bail!("Could not parse link {line}");
    };

    Ok((version, tag))
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_normalization() {
        struct TestCase {
            input: &'static str,
            expected: &'static str,
        }

        let changelog = [
            // Adds header and Unreleased link
            TestCase {
                input: "
## Unreleased

## [0.1.0] - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
                expected: "# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/v0.1.0...HEAD
",
            },
            // Turns header into link if it exists
            TestCase {
                input: "
## 0.1.0 - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
                expected: "# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
            },
            // Merges changelog groups
            TestCase {
                input: "
## [0.1.0] - 2023-10-01

### Added

- Added support for ESP32-S3 (#1)

### Added

- Added support for ESP32-C3 (#2)

### Changed

- Change (#1)

### Breaking

- Bad change (#2)

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
                expected: "# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2023-10-01

### Added

- Added support for ESP32-S3 (#1)
- Added support for ESP32-C3 (#2)

### Changed

- Change (#1)
- Bad change (#2)

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
            },
        ];

        for TestCase { input, expected } in changelog {
            let normalized = Changelog::parse(input).unwrap().to_string();

            pretty_assertions::assert_eq!(normalized, expected);
        }
    }

    #[test]
    fn test_finalization() {
        let mut changelog = Changelog::parse(
            "## Unreleased

### Added

- Added support for ESP32-S3 (#1)

### Added

- Added support for ESP32-C3 (#2)

### Changed

- Change (#1)

### Breaking

- Bad change (#2)


### Fixed


### Removed



## [0.1.0] - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
",
        )
        .unwrap();

        changelog.finalize(
            Package::EspHal,
            &semver::Version::new(0, 2, 0),
            "2025-05-08 08:36-04".parse().unwrap(),
        );

        let expected = "# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.2.0] - 2025-05-08

### Added

- Added support for ESP32-S3 (#1)
- Added support for ESP32-C3 (#2)

### Changed

- Change (#1)
- Bad change (#2)

## [0.1.0] - 2023-10-01

[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
[v0.2.0]: https://github.com/esp-rs/esp-hal/compare/v0.1.0...esp-hal-v0.2.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-hal-v0.2.0...HEAD
";

        pretty_assertions::assert_eq!(changelog.to_string(), expected);
    }
}
