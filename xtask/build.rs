use std::{
    env,
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
};

struct DiscoveredPackage {
    name: String,
    variant: String,
    directory: String,
    aliases: Vec<String>,
    contains_standalone_projects: bool,
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace = manifest_dir
        .parent()
        .expect("xtask must live inside workspace root");
    let root_manifest = workspace.join("Cargo.toml");
    println!("cargo::rerun-if-changed={}", root_manifest.display());

    let packages = discover_packages(workspace, &root_manifest);
    let code = generate_packages_rs(&packages);

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out_dir.join("packages.rs"), code).expect("Failed to write generated packages.rs");
}

fn discover_packages(workspace: &Path, root_manifest: &Path) -> Vec<DiscoveredPackage> {
    let content = fs::read_to_string(root_manifest)
        .unwrap_or_else(|e| panic!("Failed to read {}: {e}", root_manifest.display()));
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .unwrap_or_else(|e| panic!("Failed to parse {}: {e}", root_manifest.display()));

    // Root `[workspace].exclude` lists all crates and project folders in the repo.
    let excludes = doc
        .get("workspace")
        .and_then(|w| w.get("exclude"))
        .and_then(|e| e.as_array())
        .expect("Root Cargo.toml must have [workspace].exclude");

    let mut discovered = Vec::new();
    for entry in excludes {
        let Some(rel_dir) = entry.as_str() else {
            continue;
        };
        // Skip helper tools that are not xtask packages.
        if rel_dir.starts_with("extras/") || rel_dir == "init-local-registry" {
            continue;
        }
        let path = workspace.join(rel_dir);
        if !path.is_dir() {
            continue;
        }
        // A Cargo.toml marks a single crate, otherwise the folder holds standalone projects.
        let manifest_path = path.join("Cargo.toml");
        if manifest_path.exists() {
            println!("cargo::rerun-if-changed={}", manifest_path.display());
            let (name, aliases) = read_package_metadata(&manifest_path)
                .unwrap_or_else(|| (rel_dir.to_string(), Vec::new()));
            discovered.push(make_discovered_package(
                name,
                rel_dir.to_string(),
                aliases,
                false,
            ));
        } else {
            println!("cargo::rerun-if-changed={}", path.display());
            let aliases = rel_dir
                .strip_suffix('s')
                .map(ToOwned::to_owned)
                .into_iter()
                .collect();
            discovered.push(make_discovered_package(
                rel_dir.to_string(),
                rel_dir.to_string(),
                aliases,
                true,
            ));
            if let Ok(entries) = fs::read_dir(&path) {
                let mut subdirs: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
                subdirs.sort();
                for sub_path in subdirs {
                    let sub_manifest = sub_path.join("Cargo.toml");
                    if sub_path.is_dir()
                        && sub_manifest.exists()
                        && let Some((pkg_name, aliases)) = read_package_metadata(&sub_manifest)
                    {
                        let sub_rel = sub_path
                            .strip_prefix(workspace)
                            .unwrap()
                            .to_string_lossy()
                            .replace('\\', "/");
                        // Also register multi-binary crates like `examples/qa` (`qa-test`).
                        if sub_path.join("src").join("bin").is_dir() {
                            println!("cargo::rerun-if-changed={}", sub_manifest.display());
                            discovered
                                .push(make_discovered_package(pkg_name, sub_rel, aliases, false));
                        }
                    }
                }
            }
        }
    }

    discovered
}

fn read_package_metadata(manifest_path: &Path) -> Option<(String, Vec<String>)> {
    let content = fs::read_to_string(manifest_path).ok()?;
    let doc = content.parse::<toml_edit::DocumentMut>().ok()?;
    let package = doc.get("package")?;
    let name = package.get("name")?.as_str()?.to_owned();
    let aliases = package
        .get("metadata")
        .and_then(|m| m.get("espressif"))
        .and_then(|e| e.get("aliases"))
        .and_then(|a| a.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|v| v.as_str().map(ToOwned::to_owned))
                .collect()
        })
        .unwrap_or_default();
    Some((name, aliases))
}

fn make_discovered_package(
    name: String,
    directory: String,
    aliases: Vec<String>,
    contains_standalone_projects: bool,
) -> DiscoveredPackage {
    let variant = kebab_to_pascal(&name);
    DiscoveredPackage {
        name,
        variant,
        directory,
        aliases,
        contains_standalone_projects,
    }
}

fn kebab_to_pascal(s: &str) -> String {
    s.split('-')
        .map(|part| {
            let mut chars = part.chars();
            match chars.next() {
                Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
                None => String::new(),
            }
        })
        .collect()
}

fn generate_packages_rs(packages: &[DiscoveredPackage]) -> String {
    let mut out = String::new();
    writeln!(
        out,
        "#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    clap::ValueEnum,
    strum::Display,
    strum::EnumIter,
    strum::AsRefStr,
    serde::Deserialize,
    serde::Serialize,
)]
#[serde(rename_all = \"kebab-case\")]
#[strum(serialize_all = \"kebab-case\")]
/// Represents the packages in the `esp-hal` workspace.
pub enum Package {{"
    )
    .unwrap();

    for pkg in packages {
        if !pkg.aliases.is_empty() {
            let aliases = pkg
                .aliases
                .iter()
                .map(|a| format!("alias = {a:?}"))
                .collect::<Vec<_>>()
                .join(", ");
            writeln!(out, "    #[value({aliases})]").unwrap();
        }
        writeln!(out, "    {},", pkg.variant).unwrap();
    }
    writeln!(out, "}}\n").unwrap();

    writeln!(out, "impl Package {{").unwrap();
    writeln!(
        out,
        "    /// Source directory relative to the workspace root."
    )
    .unwrap();
    writeln!(out, "    pub fn directory(&self) -> &str {{").unwrap();
    writeln!(out, "        match self {{").unwrap();
    for pkg in packages {
        if pkg.directory != pkg.name {
            writeln!(
                out,
                "            Self::{} => {:?},",
                pkg.variant, pkg.directory
            )
            .unwrap();
        }
    }
    writeln!(out, "            other => other.as_ref(),").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}\n").unwrap();

    writeln!(
        out,
        "    /// Is this \"package\" a directory of standalone projects rather than a single crate?"
    )
    .unwrap();
    writeln!(
        out,
        "    pub fn contains_standalone_projects(&self) -> bool {{"
    )
    .unwrap();
    let standalone_variants: Vec<String> = packages
        .iter()
        .filter(|p| p.contains_standalone_projects)
        .map(|p| format!("Self::{}", p.variant))
        .collect();
    if standalone_variants.is_empty() {
        writeln!(out, "        false").unwrap();
    } else {
        writeln!(
            out,
            "        matches!(self, {})",
            standalone_variants.join(" | ")
        )
        .unwrap();
    }
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();

    out
}
