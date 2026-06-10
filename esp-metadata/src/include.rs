use std::path::{Component, Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};

const INCLUDE_PREFIX: &str = "# {include ";

/// Expand `# {include <path>}` markers in `content`, resolving paths relative to `base_path`.
pub(crate) fn expand_includes(
    content: &str,
    base_path: &Path,
    devices_dir: &Path,
) -> Result<String> {
    let mut chain = vec![normalize_path(base_path)];
    expand_includes_recursive(content, base_path, devices_dir, &mut chain)
}

fn expand_includes_recursive(
    content: &str,
    base_path: &Path,
    devices_dir: &Path,
    chain: &mut Vec<PathBuf>,
) -> Result<String> {
    let base_dir = base_path
        .parent()
        .with_context(|| format!("include base path has no parent: {}", base_path.display()))?;

    let mut result = String::new();

    for line in content.lines() {
        if let Some(include_path) = parse_include_line(line) {
            validate_include_path(include_path, base_path, devices_dir)?;

            let resolved = base_dir.join(include_path);
            let resolved = normalize_path(&resolved);

            if chain.iter().any(|path| path == &resolved) {
                let mut display_chain: Vec<String> = chain
                    .iter()
                    .map(|path| display_path(path, devices_dir))
                    .collect();
                display_chain.push(display_path(&resolved, devices_dir));
                bail!("include cycle detected: {}", display_chain.join(" → "));
            }

            chain.push(resolved.clone());

            let included_content = std::fs::read_to_string(&resolved).with_context(|| {
                format!(
                    "failed to read include in {}: {} (path: {})",
                    display_path(base_path, devices_dir),
                    include_path,
                    resolved.display()
                )
            })?;

            let expanded =
                expand_includes_recursive(&included_content, &resolved, devices_dir, chain)?;

            chain.pop();

            if !result.is_empty() {
                result.push('\n');
            }
            result.push_str(&expanded);
        } else {
            if !result.is_empty() {
                result.push('\n');
            }
            result.push_str(line);
        }
    }

    Ok(result)
}

fn parse_include_line(line: &str) -> Option<&str> {
    let trimmed = line.trim_start();
    if !trimmed.starts_with(INCLUDE_PREFIX) {
        return None;
    }

    let rest = &trimmed[INCLUDE_PREFIX.len()..];
    if !rest.ends_with('}') {
        return None;
    }

    let path = &rest[..rest.len() - 1];
    if path.is_empty() || path.contains(' ') {
        return None;
    }

    Some(path)
}

fn validate_include_path(path: &str, base_path: &Path, devices_dir: &Path) -> Result<()> {
    let location = display_path(base_path, devices_dir);

    ensure!(
        !path.contains('\\'),
        "invalid include path in {location}: {path:?} (path must use forward slashes)"
    );

    ensure!(
        !Path::new(path).is_absolute(),
        "invalid include path in {location}: {path:?} (path must be relative)"
    );

    for component in Path::new(path).components() {
        ensure!(
            component != Component::ParentDir,
            "invalid include path in {location}: {path:?} (path must not contain '..')"
        );
    }

    Ok(())
}

fn normalize_path(path: &Path) -> PathBuf {
    let mut normalized = PathBuf::new();

    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::Normal(part) => normalized.push(part),
            Component::RootDir | Component::Prefix(_) => normalized.push(component.as_os_str()),
            Component::ParentDir => normalized.push(component.as_os_str()),
        }
    }

    normalized
}

fn display_path(path: &Path, devices_dir: &Path) -> String {
    path.strip_prefix(devices_dir)
        .map(|relative| relative.display().to_string().replace('\\', "/"))
        .unwrap_or_else(|_| path.display().to_string().replace('\\', "/"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(path: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/include")
            .join(path)
    }

    fn devices_dir() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/include")
    }

    #[test]
    fn expands_single_include() {
        let root = fixture("simple/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let expanded = expand_includes(&content, &root, &devices_dir()).unwrap();

        assert!(expanded.contains("name = \"test\""));
        assert!(expanded.contains("arch = \"riscv\""));
        assert!(!expanded.contains("# {include"));
    }

    #[test]
    fn expands_nested_includes() {
        let root = fixture("nested/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let expanded = expand_includes(&content, &root, &devices_dir()).unwrap();

        assert!(expanded.contains("name = \"nested\""));
        assert!(expanded.contains("arch = \"riscv\""));
        assert!(expanded.contains("cores = 1"));
    }

    #[test]
    fn leaves_non_matching_comments_untouched() {
        let root = fixture("non_match/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let expanded = expand_includes(&content, &root, &devices_dir()).unwrap();

        assert!(expanded.contains("# see {include fragment.toml}"));
        assert!(expanded.contains("# {include}"));
        assert!(expanded.contains("# {include fragment.toml} extra"));
        assert!(expanded.contains("# { include fragment.toml }"));
        assert!(expanded.contains("name = \"untouched\""));
    }

    #[test]
    fn rejects_parent_dir_in_path() {
        let root = fixture("parent_dir/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let error = expand_includes(&content, &root, &devices_dir()).unwrap_err();

        assert_eq!(
            error.to_string(),
            "invalid include path in parent_dir/root.toml: \"../fragment.toml\" (path must not contain '..')"
        );
    }

    #[test]
    fn reports_missing_include() {
        let root = fixture("missing/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let error = expand_includes(&content, &root, &devices_dir()).unwrap_err();

        let message = error.to_string();
        assert!(message.contains("failed to read include in missing/root.toml"));
        assert!(message.contains("fragment.toml"));
    }

    #[test]
    fn detects_include_cycles() {
        let root = fixture("cycle/root.toml");
        let content = std::fs::read_to_string(&root).unwrap();
        let error = expand_includes(&content, &root, &devices_dir()).unwrap_err();

        assert_eq!(
            error.to_string(),
            "include cycle detected: cycle/root.toml → cycle/a.toml → cycle/b.toml → cycle/a.toml"
        );
    }

    #[test]
    fn parse_include_line_accepts_indented_marker() {
        assert_eq!(
            parse_include_line("  # {include fragment.toml}"),
            Some("fragment.toml")
        );
    }

    #[test]
    fn existing_device_configs_still_load() {
        use strum::IntoEnumIterator;

        use crate::{Chip, Config};

        for chip in Chip::iter() {
            Config::for_chip(&chip);
        }
    }
}
