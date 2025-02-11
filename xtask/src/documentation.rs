use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{ensure, Context as _, Result};
use clap::ValueEnum;
use esp_metadata::Config;
use minijinja::Value;
use strum::IntoEnumIterator;

use crate::{cargo::CargoArgsBuilder, Chip, Package};

// ----------------------------------------------------------------------------
// Build Documentation

pub fn build_documentation(
    workspace: &Path,
    packages: &mut [Package],
    chips: &mut [Chip],
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
            log::warn!("Package '{package}' does not have chip features, ignoring argument");
            vec![]
        };

        if chips.is_empty() {
            build_documentation_for_package(workspace, package, None)?;
        } else {
            for chip in chips {
                build_documentation_for_package(workspace, package, Some(chip))?;
            }
        }
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
        if let Err(err) = crate::validate_package_chip(package, &chip) {
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
        Some(crate::target_triple(package, chip)?)
    } else {
        None
    };

    let mut features = vec![];
    if let Some(chip) = chip {
        features.push(chip.to_string());
        features.extend(apply_feature_rules(&package, Config::for_chip(&chip)));
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
    let mut docs_path = workspace.join(package.to_string()).join("target");
    if let Some(target) = target {
        docs_path = docs_path.join(target);
    }
    docs_path = docs_path.join("doc");

    Ok(crate::windows_safe_path(&docs_path))
}

fn apply_feature_rules(package: &Package, config: &Config) -> Vec<String> {
    let chip_name = &config.name();

    let mut features = vec![];
    match package {
        Package::EspBacktrace => features.push("defmt".to_owned()),
        Package::EspConfig => features.push("build".to_owned()),
        Package::EspHal => {
            features.push("unstable".to_owned());
            features.push("ci".to_owned());
            match chip_name.as_str() {
                "esp32" => features.push("psram".to_owned()),
                "esp32s2" => features.push("psram".to_owned()),
                "esp32s3" => features.push("psram".to_owned()),
                _ => {}
            };
        }
        Package::EspWifi => {
            features.push("esp-hal/unstable".to_owned());
            if config.contains("wifi") {
                features.push("wifi".to_owned());
                features.push("esp-now".to_owned());
                features.push("sniffer".to_owned());
                features.push("utils".to_owned());
                features.push("smoltcp/proto-ipv4".to_owned());
                features.push("smoltcp/proto-ipv6".to_owned());
            }
            if config.contains("ble") {
                features.push("ble".to_owned());
            }
            if config.contains("wifi") && config.contains("ble") {
                features.push("coex".to_owned());
            }
        }
        Package::EspHalEmbassy => {
            features.push("esp-hal/unstable".to_owned());
        }
        _ => {}
    }

    features
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
        // the package's documentation:
        for version_path in fs::read_dir(package_docs_path)? {
            let version_path = version_path?.path();
            if version_path.is_file() {
                log::debug!(
                    "Path is not a directory, skipping: '{}'",
                    version_path.display()
                );
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
            render_template(
                "package_index.html.jinja",
                "index.html",
                &version_path,
                &resources_path,
                minijinja::context! { metadata => meta },
            )?;
        }
    }

    // Copy any additional assets to the documentation's output path:
    fs::copy(
        resources_path.join("esp-rs.svg"),
        docs_path.join("esp-rs.svg"),
    )
    .context("Failed to copy esp-rs.svg")?;

    let meta = generate_documentation_meta_for_index(&workspace)?;

    render_template(
        "index.html.jinja",
        "index.html",
        &docs_path,
        &resources_path,
        minijinja::context! { metadata => meta },
    )?;

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
        crate::validate_package_chip(&package, chip)?;

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

fn render_template<C>(
    template: &str,
    name: &str,
    path: &Path,
    resources: &Path,
    ctx: C,
) -> Result<()>
where
    C: serde::Serialize,
{
    let source = fs::read_to_string(resources.join(template))
        .context(format!("Failed to read {template}"))?;

    let mut env = minijinja::Environment::new();
    env.add_template(template, &source)?;

    let tmpl = env.get_template(template)?;
    let html = tmpl.render(ctx)?;

    // Write out the rendered HTML to the desired path:
    let path = path.join(name);
    fs::write(&path, html).context(format!("Failed to write {name}"))?;
    log::info!("Created {}", path.display());

    Ok(())
}
