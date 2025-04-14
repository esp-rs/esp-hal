use std::{
    collections::HashSet,
    fs::{self, create_dir_all},
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{ensure, Context as _, Result};
use clap::ValueEnum;
use esp_metadata::Config;
use kuchikiki::traits::*;
use minijinja::Value;
use serde::{Deserialize, Serialize};
use strum::IntoEnumIterator;

use crate::{cargo::CargoArgsBuilder, Chip, Package};

// ----------------------------------------------------------------------------
// Build Documentation

#[derive(Debug, Default, Clone, PartialEq, Eq, Deserialize, Serialize)]
struct Manifest {
    versions: HashSet<semver::Version>,
}

pub fn build_documentation(
    workspace: &Path,
    packages: &mut [Package],
    chips: &mut [Chip],
    base_url: Option<String>,
) -> Result<()> {
    let output_path = workspace.join("docs");

    fs::create_dir_all(&output_path)
        .with_context(|| format!("Failed to create {}", output_path.display()))?;

    packages.sort();

    for package in packages {
        // Not all packages need documentation built:
        if !package.is_published() {
            continue;
        }

        // Download the manifest from the documentation server if able,
        // otherwise just create a default (empty) manifest:
        let mut manifest = fetch_manifest(&base_url, package)?;

        // If the package does not have chip features, then just ignore
        // whichever chip(s) were specified as arguments:
        let chips = if package.has_chip_features() {
            // Some packages have chip features, but they have no effect on the public API;
            // in this case, there's no point building it multiple times, so just build one
            // copy of the docs. Otherwise, use the provided chip arguments:
            match package {
                _ if package.chip_features_matter() => chips.to_vec(),
                Package::XtensaLxRt => vec![Chip::Esp32s3],
                _ => vec![Chip::Esp32c6],
            }
        } else {
            log::debug!("Package '{package}' does not have chip features, ignoring argument");
            vec![]
        };

        // Update the package manifest to include the latest version:
        let version = crate::package_version(workspace, *package)?;
        manifest.versions.insert(version.clone());

        // Build the documentation for the package:
        if chips.is_empty() {
            build_documentation_for_package(workspace, package, None)?;
        } else {
            for chip in chips {
                build_documentation_for_package(workspace, package, Some(chip))?;
            }
        }

        // Write out the package manifest JSON file:
        fs::write(
            output_path.join(package.to_string()).join("manifest.json"),
            serde_json::to_string(&manifest)?,
        )?;

        // Patch the generated documentation to include a select box for the version:
        patch_documentation_index_for_package(workspace, package, &version, &base_url)?;
    }

    Ok(())
}

fn build_documentation_for_package(
    workspace: &Path,
    package: &Package,
    chip: Option<Chip>,
) -> Result<()> {
    let version = crate::package_version(workspace, *package)?;

    // Ensure that the package/chip combination provided are valid:
    if let Some(chip) = chip {
        if let Err(err) = package.validate_package_chip(&chip) {
            log::warn!("{err}");
            return Ok(());
        }
    }

    // Build the documentation for the specified package, targeting the
    // specified chip:
    let docs_path = cargo_doc(workspace, *package, chip)?;

    ensure!(
        docs_path.exists(),
        "Documentation not found at {}",
        docs_path.display()
    );

    let mut output_path = workspace
        .join("docs")
        .join(package.to_string())
        .join(version.to_string());

    if let Some(chip) = chip {
        // Sometimes we need to specify a chip feature, but it does not affect the
        // public API; so, only append the chip name to the path if it is significant:
        if package.chip_features_matter() {
            output_path = output_path.join(chip.to_string());
        }
    }

    let output_path = crate::windows_safe_path(&output_path);

    // Create the output directory, and copy the built documentation into it:
    fs::create_dir_all(&output_path)
        .with_context(|| format!("Failed to create {}", output_path.display()))?;

    crate::copy_dir_all(&docs_path, &output_path).with_context(|| {
        format!(
            "Failed to copy {} to {}",
            docs_path.display(),
            output_path.display()
        )
    })?;

    // create "/latest" redirect - assuming that the current version is the latest
    let latest_path = if package.chip_features_matter() {
        output_path
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .join("latest")
    } else {
        output_path.parent().unwrap().join("latest")
    };
    log::info!("Creating latest version redirect at {:?}", latest_path);
    create_dir_all(latest_path.clone())?;
    std::fs::File::create(latest_path.clone().join("index.html"))?.write_all(
        format!(
            "<meta http-equiv=\"refresh\" content=\"0; url=../{}/\" />",
            if package.chip_features_matter() {
                version.to_string()
            } else {
                format!(
                    "{}/{}",
                    version.to_string(),
                    package.to_string().replace('-', "_")
                )
            }
        )
        .as_bytes(),
    )?;

    Ok(())
}

fn cargo_doc(workspace: &Path, package: Package, chip: Option<Chip>) -> Result<PathBuf> {
    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    if let Some(chip) = chip {
        log::info!("Building '{package_name}' documentation targeting '{chip}'");
    } else {
        log::info!("Building '{package_name}' documentation");
    }

    // We require some nightly features to build the documentation:
    let toolchain = if chip.is_some_and(|chip| chip.is_xtensa()) {
        "esp"
    } else {
        "nightly"
    };

    // Determine the appropriate build target for the given package and chip,
    // if we're able to:
    let target = if let Some(ref chip) = chip {
        Some(package.target_triple(chip)?)
    } else {
        None
    };

    let mut features = vec![];
    if let Some(chip) = &chip {
        features.push(chip.to_string());
        features.extend(package.feature_rules(Config::for_chip(&chip)));
    } else {
        features.extend(package.feature_rules(&Config::empty()));
    }

    // Build up an array of command-line arguments to pass to `cargo`:
    let mut builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("doc")
        .features(&features)
        .arg("-Zrustdoc-map")
        .arg("--lib")
        .arg("--no-deps");

    if let Some(target) = target {
        builder = builder.target(target);
    }

    // Special case: `esp-metadata` requires `std`, and we get some really confusing
    // errors if we try to pass `-Zbuild-std=core`:
    if package != Package::EspMetadata {
        builder = builder.arg("-Zbuild-std=alloc,core");
    }

    let args = builder.build();
    log::debug!("{args:#?}");

    let mut envs = vec![("RUSTDOCFLAGS", "--cfg docsrs --cfg not_really_docsrs")];
    // Special case: `esp-storage` requires the optimization level to be 2 or 3:
    if package == Package::EspStorage {
        envs.push(("CARGO_PROFILE_DEBUG_OPT_LEVEL", "3"));
    }

    // Execute `cargo doc` from the package root:
    crate::cargo::run_with_env(&args, &package_path, envs, false)?;

    // Build up the path at which the built documentation can be found:
    let mut docs_path = if let Ok(target_path) = std::env::var("CARGO_TARGET_DIR") {
        PathBuf::from(target_path)
    } else {
        workspace.join(package.to_string()).join("target")
    };

    if let Some(target) = target {
        docs_path = docs_path.join(target);
    }
    docs_path = docs_path.join("doc");

    Ok(crate::windows_safe_path(&docs_path))
}

fn patch_documentation_index_for_package(
    workspace: &Path,
    package: &Package,
    version: &semver::Version,
    base_url: &Option<String>,
) -> Result<()> {
    let package_name = package.to_string().replace('-', "_");
    let package_path = workspace.join("docs").join(package.to_string());
    let version_path = package_path.join(version.to_string());

    let mut index_paths = Vec::new();

    if package.chip_features_matter() {
        for chip_path in fs::read_dir(version_path)? {
            let chip_path = chip_path?.path();
            if chip_path.is_dir() {
                let path = chip_path.join(&package_name).join("index.html");
                index_paths.push((version.clone(), path));
            }
        }
    } else {
        let path = version_path.join(&package_name).join("index.html");
        index_paths.push((version.clone(), path));
    }

    for (version, index_path) in index_paths {
        let html = fs::read_to_string(&index_path)?;
        let document = kuchikiki::parse_html().one(html);

        let elem = document
            .select_first(".sidebar-crate")
            .expect("Unable to select '.sidebar-crate' element in HTML");

        let base_url = base_url.clone().unwrap_or_default();
        let resources_path = workspace.join("resources");
        let html = render_template(
            &resources_path,
            "select.html.jinja",
            minijinja::context! { base_url => base_url, package => package, version => version },
        )?;

        let node = elem.as_node();
        node.append(kuchikiki::parse_html().one(html));

        fs::write(&index_path, document.to_string())?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Build Documentation Index

pub fn build_documentation_index(workspace: &Path, packages: &mut [Package]) -> Result<()> {
    let docs_path = workspace.join("docs");
    let resources_path = workspace.join("resources");

    packages.sort();

    for package in packages {
        // Not all packages have documentation built:
        if !package.is_published() {
            continue;
        }

        // If the chip features are not relevant, then there is no need to generate an
        // index for the given package's documentation:
        if !package.chip_features_matter() {
            log::warn!("Package '{package}' does not have device-specific documentation, no need to generate an index");
            continue;
        }

        let package_docs_path = docs_path.join(package.to_string());
        let mut device_doc_paths = Vec::new();

        // Each path we iterate over should be the directory for a given version of
        // the package's documentation: (except latest)
        for version_path in fs::read_dir(package_docs_path)? {
            let version_path = version_path?.path();
            if version_path.is_file() {
                log::debug!(
                    "Path is not a directory, skipping: '{}'",
                    version_path.display()
                );
                continue;
            }

            if version_path.file_name().unwrap() == "latest" {
                continue;
            }

            for path in fs::read_dir(&version_path)? {
                let path = path?.path();
                if path.is_dir() {
                    device_doc_paths.push(path);
                }
            }

            let mut chips = device_doc_paths
                .iter()
                .map(|path| {
                    let chip = path
                        .components()
                        .last()
                        .unwrap()
                        .as_os_str()
                        .to_string_lossy();

                    let chip = Chip::from_str(&chip, true).unwrap();

                    chip
                })
                .collect::<Vec<_>>();

            chips.sort();

            let meta = generate_documentation_meta_for_package(workspace, *package, &chips)?;

            // Render the template to HTML and write it out to the desired path:
            let html = render_template(
                &resources_path,
                "package_index.html.jinja",
                minijinja::context! { metadata => meta },
            )?;
            let path = version_path.join("index.html");
            fs::write(&path, html).context(format!("Failed to write index.html"))?;
            log::info!("Created {}", path.display());
        }
    }

    // Copy any additional assets to the documentation's output path:
    fs::copy(
        resources_path.join("esp-rs.svg"),
        docs_path.join("esp-rs.svg"),
    )
    .context("Failed to copy esp-rs.svg")?;

    let meta = generate_documentation_meta_for_index(&workspace)?;

    // Render the template to HTML and write it out to the desired path:
    let html = render_template(
        &resources_path,
        "index.html.jinja",
        minijinja::context! { metadata => meta },
    )?;
    let path = docs_path.join("index.html");
    fs::write(&path, html).context(format!("Failed to write index.html"))?;
    log::info!("Created {}", path.display());

    Ok(())
}

fn generate_documentation_meta_for_package(
    workspace: &Path,
    package: Package,
    chips: &[Chip],
) -> Result<Vec<Value>> {
    let version = crate::package_version(workspace, package)?;

    let mut metadata = Vec::new();

    for chip in chips {
        // Ensure that the package/chip combination provided are valid:
        package.validate_package_chip(chip)?;

        // Build the context object required for rendering this particular build's
        // information on the documentation index:
        metadata.push(minijinja::context! {
            name => package,
            version => version,
            chip => chip.to_string(),
            chip_pretty => chip.pretty_name(),
            package => package.to_string().replace('-', "_"),
        });
    }

    Ok(metadata)
}

fn generate_documentation_meta_for_index(workspace: &Path) -> Result<Vec<Value>> {
    let mut metadata = Vec::new();

    for package in Package::iter() {
        // Not all packages have documentation built:
        if !package.is_published() {
            continue;
        }

        let version = crate::package_version(workspace, package)?;

        let url = if package.chip_features_matter() {
            format!("{package}/{version}/index.html")
        } else {
            let crate_name = package.to_string().replace('-', "_");
            format!("{package}/{version}/{crate_name}/index.html")
        };

        metadata.push(minijinja::context! {
            name => package,
            version => version,
            url => url,
        });
    }

    Ok(metadata)
}

// ----------------------------------------------------------------------------
// Helper Functions

fn fetch_manifest(base_url: &Option<String>, package: &Package) -> Result<Manifest> {
    let mut manifest_url = base_url
        .clone()
        .unwrap_or_default()
        .trim_end_matches('/')
        .to_string();
    manifest_url.push_str(&format!("/{package}/manifest.json"));

    #[cfg(feature = "deploy-docs")]
    let manifest = match reqwest::blocking::get(manifest_url) {
        Ok(resp) => resp.json::<Manifest>()?,
        Err(err) => {
            log::warn!("Unable to fetch package manifest: {err}");
            Manifest::default()
        }
    };

    #[cfg(not(feature = "deploy-docs"))]
    let manifest = Manifest::default();

    Ok(manifest)
}

fn render_template<C>(resources: &Path, template: &str, ctx: C) -> Result<String>
where
    C: serde::Serialize,
{
    let source = fs::read_to_string(resources.join(template))
        .context(format!("Failed to read {template}"))?;

    let mut env = minijinja::Environment::new();
    env.add_template(template, &source)?;

    let tmpl = env.get_template(template)?;
    let html = tmpl.render(ctx)?;

    Ok(html)
}
