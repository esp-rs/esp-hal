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
        if path.join("Cargo.toml").exists() {
            let name = read_package_name(&path.join("Cargo.toml")).unwrap_or(rel_dir.to_string());
            discovered.push(make_discovered_package(name, rel_dir.to_string(), false));
        } else {
            println!("cargo::rerun-if-changed={}", path.display());
            discovered.push(make_discovered_package(
                rel_dir.to_string(),
                rel_dir.to_string(),
                true,
            ));
            if let Ok(entries) = fs::read_dir(&path) {
                let mut subdirs: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
                subdirs.sort();
                for sub_path in subdirs {
                    if sub_path.is_dir()
                        && sub_path.join("Cargo.toml").exists()
                        && let Some(pkg_name) = read_package_name(&sub_path.join("Cargo.toml"))
                    {
                        let sub_rel = sub_path
                            .strip_prefix(workspace)
                            .unwrap()
                            .to_string_lossy()
                            .replace('\\', "/");
                        // Also register multi-binary crates like `examples/qa` (`qa-test`).
                        if sub_path.join("src").join("bin").is_dir() {
                            discovered.push(make_discovered_package(pkg_name, sub_rel, false));
                        }
                    }
                }
            }
        }
    }

    discovered
}

fn read_package_name(manifest_path: &Path) -> Option<String> {
    let content = fs::read_to_string(manifest_path).ok()?;
    let doc = content.parse::<toml_edit::DocumentMut>().ok()?;
    doc.get("package")?
        .get("name")?
        .as_str()
        .map(ToOwned::to_owned)
}

fn make_discovered_package(
    name: String,
    directory: String,
    contains_standalone_projects: bool,
) -> DiscoveredPackage {
    let variant = kebab_to_pascal(&name);
    DiscoveredPackage {
        name,
        variant,
        directory,
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
        match pkg.name.as_str() {
            "examples" => writeln!(out, "    #[value(alias = \"example\")]").unwrap(),
            "hil-test" => {
                writeln!(out, "    #[value(alias = \"test\", alias = \"tests\")]").unwrap()
            }
            "qa-test" => writeln!(out, "    #[value(alias = \"qa\")]").unwrap(),
            _ => {}
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
