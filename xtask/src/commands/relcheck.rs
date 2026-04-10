use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    str::from_utf8,
};

use anyhow::{Result, bail};
use clap::{Args, Subcommand};
use strum::IntoEnumIterator;

use crate::{Package, cargo::CargoToml, windows_safe_path};

#[derive(Debug, Subcommand)]
pub enum RelCheckCmds {
    /// Initialize the local registry
    Init,

    /// Deinitialize the local registry
    Deinit,

    /// Update the local registry from the crates in this repository
    Update(UpdateArgs),

    /// Remove the path-dependencies from examples and make sure the dependencies are available in
    /// the local registry.
    ReplacePathDeps,
}

#[derive(Args, Debug, Clone)]
pub struct UpdateArgs {
    #[arg(value_enum, default_values_t = Package::iter())]
    packages: Vec<Package>,
}

pub fn run_rel_check(args: RelCheckCmds) -> Result<()> {
    ensure_cargo_local_registry()?;

    match args {
        RelCheckCmds::Init => init_rel_check()?,
        RelCheckCmds::Deinit => deinit_rel_check()?,
        RelCheckCmds::Update(args) => update(args)?,
        RelCheckCmds::ReplacePathDeps => scrap_path_deps()?,
    }

    Ok(())
}

fn toolchain() -> String {
    "esp".to_string()
}

fn toolchain_folder(toolchain: &str) -> Result<PathBuf> {
    let toolchains = std::process::Command::new("rustup")
        .arg("toolchain")
        .arg("list")
        .arg("-v")
        .output()?;

    if !toolchains.status.success() {
        return Err(anyhow::anyhow!(
            "Unable to run rustup to learn about toolchains."
        ));
    }

    let toolchains = from_utf8(&toolchains.stdout)?.lines();
    let mut parsed: Vec<(String, String)> = Vec::new();
    let re = regex::Regex::new(r#"^(\S+)(?:\s+\([^)]*\))?\s+(\S.*)$"#)?;
    for line in toolchains {
        if let Some(parts) = re.captures(line) {
            let (name, folder) = if parts.len() == 3 {
                (parts[1].to_string(), parts[2].to_string())
            } else {
                (parts[1].to_string(), parts[3].to_string())
            };

            let name = if name.starts_with("stable") {
                "stable".to_string()
            } else if name.starts_with("nightly") {
                if regex::Regex::new(r#"nightly-\d\d\d\d-\d\d-\d\d.*"#)
                    .unwrap()
                    .is_match(&name)
                {
                    name[..18].to_string()
                } else {
                    "nightly".to_string()
                }
            } else if regex::Regex::new(r#"\d+\.\d+-.*"#).unwrap().is_match(&name) {
                let separator_idx = name.chars().position(|c| c == '-').unwrap();
                name[..(separator_idx)].to_string()
            } else {
                name
            };

            parsed.push((name, folder));
        };
    }

    match parsed.iter().find(|(name, _)| name == toolchain) {
        Some((_, folder)) => Ok(PathBuf::from(folder)),
        None => Err(anyhow::anyhow!(
            "Toolchain {toolchain} not found. Found {:?}",
            parsed
        )),
    }
}

fn deinit_rel_check() -> Result<()> {
    let _ = std::fs::remove_dir_all("compile-tests/.cargo");

    if std::fs::exists("target/local-registry")? {
        std::fs::remove_dir_all("target/local-registry")?;
    }
    revert_scrap_path_deps()?;

    Ok(())
}

fn init_rel_check() -> Result<()> {
    // cleanup
    if std::fs::exists("target/local-registry")? {
        std::fs::remove_dir_all("target/local-registry")?;
    }
    std::fs::create_dir("target/local-registry")?;

    // make sure we have the `Cargo.lock` file
    let project = Path::new("init-local-registry");
    let _ = std::fs::remove_file(project.join("Cargo.lock"));

    let status = std::process::Command::new("cargo")
        .arg(format!("+{}", toolchain()))
        .arg("metadata")
        .arg("--format-version=1")
        .current_dir(&project)
        .stdout(std::process::Stdio::null())
        .status()?;

    if !status.success() {
        log::warn!("Failed");
    }

    // add all dependencies referenced by the lock file
    let mut cmd = std::process::Command::new("cargo");
    cmd.arg("local-registry");
    cmd.arg("--no-delete");
    cmd.arg("--sync");
    cmd.arg("Cargo.lock");
    cmd.arg(windows_safe_path(
        &std::path::PathBuf::from("target/local-registry")
            .canonicalize()
            .unwrap(),
    ));
    cmd.stdout(std::process::Stdio::null());
    cmd.env_clear();
    cmd.envs(
        std::env::vars()
            .into_iter()
            .filter(|(k, _)| !k.starts_with("CARGO")),
    );
    cmd.current_dir(&project);

    log::info!("{:?}", cmd);
    cmd.status()?;

    // install dependencies needed for build-std
    let mut cmd = std::process::Command::new("cargo");
    cmd.arg("local-registry");
    cmd.arg("--no-delete");
    cmd.arg("--sync");
    cmd.arg(format!(
        "{}/lib/rustlib/src/rust/library/Cargo.lock",
        toolchain_folder(&toolchain())?.display()
    ));
    cmd.arg("target/local-registry");
    cmd.stdout(std::process::Stdio::null());
    cmd.env_clear();
    cmd.envs(
        std::env::vars()
            .into_iter()
            .filter(|(k, _)| !k.starts_with("CARGO")),
    );

    log::info!("{:?}", cmd);
    cmd.status()?;

    // "for reasons" (e.g. running examples later via the "normal" xtask) also do it for other
    // toolchains
    let mut cmd = std::process::Command::new("cargo");
    cmd.arg("local-registry");
    cmd.arg("--no-delete");
    cmd.arg("--sync");
    cmd.arg(format!(
        "{}/lib/rustlib/src/rust/library/Cargo.lock",
        toolchain_folder("nightly")?.display()
    ));
    cmd.arg("target/local-registry");
    cmd.stdout(std::process::Stdio::null());
    cmd.env_clear();
    cmd.envs(
        std::env::vars()
            .into_iter()
            .filter(|(k, _)| !k.starts_with("CARGO")),
    );

    log::info!("{:?}", cmd);
    cmd.status()?;

    let mut cmd = std::process::Command::new("cargo");
    cmd.arg("local-registry");
    cmd.arg("--no-delete");
    cmd.arg("--sync");
    cmd.arg(format!(
        "{}/lib/rustlib/src/rust/library/Cargo.lock",
        toolchain_folder("stable")?.display()
    ));
    cmd.arg("target/local-registry");
    cmd.stdout(std::process::Stdio::null());
    cmd.env_clear();
    cmd.envs(
        std::env::vars()
            .into_iter()
            .filter(|(k, _)| !k.starts_with("CARGO")),
    );

    log::info!("{:?}", cmd);
    cmd.status()?;

    Ok(())
}

fn update(args: UpdateArgs) -> Result<()> {
    if !std::fs::exists("target/local-registry")? {
        bail!("Cannot update - run `init` first.");
    }

    let workspace = Path::new(".");
    // Recursively collect dependencies. A bit inefficient, but we don't need to
    // sort a lot.

    let mut packages_to_release = args
        .packages
        .iter()
        .filter(|p| p.is_published())
        .flat_map(|p| related_crates(workspace, *p))
        .collect::<Vec<_>>();

    packages_to_release.sort();
    packages_to_release.dedup();

    let mut package_tomls = packages_to_release
        .iter()
        .map(|pkg| CargoToml::new(workspace, *pkg).map(|cargo_toml| (*pkg, cargo_toml)))
        .collect::<Result<HashMap<_, _>>>()?;

    // Determine package dependencies (package -> dependencies)
    let mut dep_graph = HashMap::new();
    for (package, toml) in package_tomls.iter_mut() {
        dep_graph.insert(*package, toml.repo_dependencies());
    }

    // Topological sort the packages into a release order. Note that this is not a stable order,
    // because the source data is a HashMap which does not have a stable insertion order. This is
    // okay, as long as the relationships of the dependencies are kept.
    let sorted = topological_sort(&dep_graph);
    log::info!("Sorted packages: {:?}", sorted);

    let original_config = std::fs::read_to_string(".cargo/config.toml")?;

    for package in sorted.iter() {
        log::info!("Package = {}", package);

        let toml = package.toml();
        let version = toml.version();

        if std::fs::exists(format!(
            "target/local-registry/{}-{}.crate",
            package, version
        ))? {
            log::warn!("Already exists as version {version}");
            continue;
        }
        core::mem::drop(toml);
        log::info!("Updating...");

        // make sure we have a lock file
        std::process::Command::new("cargo")
            .arg("update")
            .current_dir(package.toml().package_path())
            .env_clear()
            .envs(
                std::env::vars()
                    .into_iter()
                    .filter(|(k, _)| !k.starts_with("CARGO")),
            )
            .status()?;

        // prepare all the deps we need
        let mut cmd = std::process::Command::new("cargo");
        cmd.arg("local-registry");
        cmd.arg("--no-delete");
        cmd.arg("--sync");
        cmd.arg("Cargo.lock");
        cmd.arg("../target/local-registry");
        cmd.stdout(std::process::Stdio::null());
        cmd.env_clear();
        cmd.envs(
            std::env::vars()
                .into_iter()
                .filter(|(k, _)| !k.starts_with("CARGO")),
        );
        cmd.current_dir(&package.toml().package_path());

        log::info!("{:?}", cmd);
        cmd.status()?;

        std::fs::write(
            ".cargo/config.toml",
            format!(
                r#"{}
        # {}{}
        [source.crates-io]
        registry = 'sparse+https://index.crates.io/'
        replace-with = 'local-registry'

        [source.local-registry]
        local-registry = '{}'
    "#,
                &original_config,
                "STOP",
                "SHIP",
                windows_safe_path(
                    &std::path::PathBuf::from("target/local-registry")
                        .canonicalize()
                        .unwrap()
                )
                .display()
            ),
        )?;
        let res = std::process::Command::new("cargo")
            .arg("package")
            .arg("--no-verify")
            .arg("--verbose")
            .arg("--allow-dirty")
            .arg("--index=http://crates.io")
            .arg("--target-dir=../target")
            .current_dir(package.toml().package_path())
            .env_clear()
            .envs(
                std::env::vars()
                    .into_iter()
                    .filter(|(k, _)| !k.starts_with("CARGO")),
            )
            .status();

        log::info!("{:?}", res);
        std::fs::write(".cargo/config.toml", &original_config)?;

        // copy the crate to our registry
        let toml = package.toml();
        let krate = toml.manifest["package"]["name"].as_str().unwrap();
        let version = toml.manifest["package"]["version"].as_str().unwrap();
        std::fs::copy(
            PathBuf::from(".")
                .join("target")
                .join("package")
                .join(format!("{}-{}.crate", krate, version)),
            PathBuf::from(format!("target/local-registry/{}-{}.crate", krate, version)),
        )?;

        // copy metadata to the index
        let index_file = PathBuf::from("target/local-registry/index/")
            .join(&krate[..2])
            .join(&krate[2..][..2])
            .join(krate);

        if std::fs::exists(&index_file)? {
            let index_file_contents = std::fs::read_to_string(&index_file)?;
            let index_file_contents = index_file_contents.lines();
            let mut entries = Vec::new();
            for line in index_file_contents {
                let item: serde_json::Value = serde_json::de::from_str(line).unwrap();
                if item["vers"] != version {
                    entries.push(item);
                }
            }
            entries.push(
                serde_json::de::from_str(
                    &std::fs::read_to_string(
                        PathBuf::from(".")
                            .join("target")
                            .join("package")
                            .join("tmp-registry")
                            .join("index")
                            .join(&krate[..2])
                            .join(&krate[2..][..2])
                            .join(krate),
                    )
                    .unwrap(),
                )
                .unwrap(),
            );
            let mut contents = String::new();
            for entry in entries {
                contents.push_str(&serde_json::to_string(&entry).unwrap());
                contents.push('\n');
            }
            std::fs::write(&index_file, contents)?;
        } else {
            // just copy
            std::fs::copy(
                PathBuf::from(".")
                    .join("target")
                    .join("package")
                    .join("tmp-registry")
                    .join("index")
                    .join(&krate[..2])
                    .join(&krate[2..][..2])
                    .join(krate),
                &index_file,
            )?;
        }
    }

    Ok(())
}

fn latest_version_of_crate(krate: &str) -> Result<String> {
    let matching = std::fs::read_dir("target/local-registry/")?.filter(|f| {
        if let Ok(f) = f {
            f.file_name().to_str().unwrap().starts_with(krate)
        } else {
            false
        }
    });

    let prefix = format!("{krate}-");
    let mut version = None;
    for entry in matching {
        let entry = entry?;
        let name = entry.file_name();
        let vstr = name
            .to_str()
            .unwrap()
            .strip_prefix(&prefix)
            .unwrap()
            .strip_suffix(".crate")
            .unwrap();
        if let Ok(v) = vstr.parse::<semver::Version>() {
            if version.is_none() || &v > version.as_ref().unwrap() {
                version = Some(v);
            }
        }
    }

    if let Some(v) = version {
        Ok(v.to_string())
    } else {
        Err(anyhow::anyhow!("`{krate}` not found"))
    }
}

fn ensure_cargo_local_registry() -> Result<()> {
    let check = std::process::Command::new("cargo")
        .arg("local-registry")
        .arg("--help")
        .output();

    match check {
        Ok(out) => {
            if !out.status.success() {
                std::process::Command::new("cargo")
                    .arg("install")
                    .arg("cargo-local-registry")
                    .output()?;
            }
        }
        Err(_) => {
            std::process::Command::new("cargo")
                .arg("install")
                .arg("cargo-local-registry")
                .output()?;
        }
    }

    Ok(())
}

fn revert_scrap_path_deps() -> Result<()> {
    let pkgs = [
        crate::Package::Examples.to_string(),
        crate::Package::HilTest.to_string(),
        crate::Package::QaTest.to_string(),
    ];

    for pkg in pkgs {
        if let Ok(manifest_paths) = crate::find_packages(Path::new(&pkg)) {
            let manifest_paths = if !manifest_paths.is_empty() {
                manifest_paths
            } else {
                vec![PathBuf::from(&pkg)]
            };

            for manifest_path in manifest_paths {
                if std::fs::exists(manifest_path.join("Cargo.toml$"))? {
                    std::fs::remove_file(manifest_path.join("Cargo.toml"))?;
                    std::fs::rename(
                        manifest_path.join("Cargo.toml$"),
                        manifest_path.join("Cargo.toml"),
                    )?;
                }

                if std::fs::exists(manifest_path.join(".cargo/config.toml$"))? {
                    std::fs::remove_file(manifest_path.join(".cargo/config.toml"))?;
                    std::fs::rename(
                        manifest_path.join(".cargo/config.toml$"),
                        manifest_path.join(".cargo/config.toml"),
                    )?;
                }
            }
        }
    }

    Ok(())
}

fn scrap_path_deps() -> Result<()> {
    if !std::fs::exists("target/local-registry")? {
        bail!("Cannot scrap path dependencies - run `init` first.");
    }

    let pkgs = [
        crate::Package::Examples.to_string(),
        crate::Package::HilTest.to_string(),
        crate::Package::QaTest.to_string(),
    ];

    for pkg in pkgs {
        if let Ok(manifest_paths) = crate::find_packages(Path::new(&pkg)) {
            let manifest_paths = if !manifest_paths.is_empty() {
                manifest_paths
            } else {
                vec![PathBuf::from(&pkg)]
            };

            for manifest_path in manifest_paths {
                // make sure we have a lock file
                std::fs::remove_file(manifest_path.join("Cargo.lock")).ok();
                let status = std::process::Command::new("cargo")
                    .arg(format!("+{}", toolchain()))
                    .arg("metadata")
                    .arg("--format-version=1")
                    .current_dir(&manifest_path)
                    .stdout(std::process::Stdio::null())
                    .status()?;

                if !status.success() {
                    log::warn!("Failed");
                }

                // add dependencies to the local registry
                let mut cmd = std::process::Command::new("cargo");
                cmd.arg("local-registry");
                cmd.arg("--no-delete");
                cmd.arg("--sync");
                cmd.arg("Cargo.lock");
                cmd.arg(windows_safe_path(
                    &std::path::PathBuf::from("target/local-registry")
                        .canonicalize()
                        .unwrap(),
                ));
                cmd.stdout(std::process::Stdio::null());
                cmd.env_clear();
                cmd.envs(
                    std::env::vars()
                        .into_iter()
                        .filter(|(k, _)| !k.starts_with("CARGO")),
                );
                cmd.current_dir(&manifest_path);

                log::info!("{:?}", cmd);
                cmd.status()?;

                // rename files we are going to change
                std::fs::rename(
                    manifest_path.join("Cargo.toml"),
                    manifest_path.join("Cargo.toml$"),
                )?;
                if std::fs::exists(manifest_path.join(".cargo/config.toml"))? {
                    std::fs::rename(
                        manifest_path.join(".cargo/config.toml"),
                        manifest_path.join(".cargo/config.toml$"),
                    )?;
                }

                // scrap the path dependencies, use version from local registry
                let contents = std::fs::read_to_string(manifest_path.join("Cargo.toml$"))?;
                let mut toml = contents.parse::<toml_edit::DocumentMut>()?;
                for dep in toml["dependencies"].as_table_mut().unwrap().iter_mut() {
                    let krate = dep.0.get();

                    if krate.starts_with("esp-") {
                        let latest = latest_version_of_crate(krate)?;
                        dep.1.as_table_like_mut().and_then(|table| {
                            table.remove("path");
                            table.insert(
                                "version",
                                toml_edit::Item::Value(toml_edit::Value::String(
                                    toml_edit::Formatted::new(latest),
                                )),
                            );
                            Some(table)
                        });
                    }
                }

                let processed = format!("#{}{}\n{}", "STOP", "SHIP", toml.to_string());
                std::fs::write(manifest_path.join("Cargo.toml"), processed)?;

                // add the local registry to the config.toml
                let config = std::fs::read_to_string(manifest_path.join(".cargo/config.toml$"))?;
                if !config.contains("local-registry") {
                    std::fs::write(
                        manifest_path.join(".cargo/config.toml"),
                        format!(
                            r#"{}

# {}{}
[source.crates-io]
registry = 'sparse+https://index.crates.io/'
replace-with = 'local-registry'

[source.local-registry]
local-registry = '{}'
"#,
                            config,
                            "STOP",
                            "SHIP",
                            windows_safe_path(
                                &std::path::PathBuf::from("target/local-registry")
                                    .canonicalize()
                                    .unwrap()
                            )
                            .display()
                        ),
                    )?;
                }
            }
        }
    }

    Ok(())
}

// ---

// some code duplicated from `plan.rs` - de-dup once we want to do this for real!
fn related_crates(workspace: &Path, package: Package) -> Vec<Package> {
    related_crates_cb(package, &|p| {
        CargoToml::new(workspace, p).unwrap().repo_dependencies()
    })
}

fn topological_sort(dep_graph: &HashMap<Package, Vec<Package>>) -> Vec<Package> {
    let mut sorted = Vec::new();
    let mut dep_graph = dep_graph.clone();
    while !dep_graph.is_empty() {
        dep_graph.retain(|pkg, deps| {
            deps.retain(|dep| !sorted.contains(dep));

            if deps.is_empty() {
                sorted.push(*pkg);
                false
            } else {
                true
            }
        });
    }

    sorted
}

/// Collects dependencies recursively, based on a callback that provides direct dependencies.
///
/// This is an implementation detail of the `related_crates` function, separate for testing
/// purposes.
fn related_crates_cb(
    package: Package,
    direct_dependencies: &impl Fn(Package) -> Vec<Package>,
) -> Vec<Package> {
    let mut packages = vec![package];

    for dep in direct_dependencies(package) {
        for dep in related_crates_cb(dep, direct_dependencies) {
            if !packages.contains(&dep) {
                packages.push(dep);
            }
        }
    }

    packages
}
