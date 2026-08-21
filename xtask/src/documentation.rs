use std::{
    fs::{self, create_dir_all},
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{Context as _, Result, ensure};
use clap::ValueEnum;
use serde::{Deserialize, Serialize};
use somni_expr::somni_struct;
use somni_template::{Env, Iter, SomniStruct, Syntax, Template, TemplateTypes};
use strum::IntoEnumIterator;

use crate::{
    Chip,
    Package,
    cargo::{CargoArgsBuilder, CargoCommandBatcher, CargoToml},
    metadata::Config,
    windows_safe_path,
};

/// A single row of template metadata (a struct value handed to the templates).
type Meta = SomniStruct<TemplateTypes>;

// ----------------------------------------------------------------------------
// Build Documentation

#[derive(Debug, Default, Clone, PartialEq, Eq, Deserialize, Serialize)]
struct Manifest {
    versions: Vec<String>,
}

impl Manifest {
    fn insert_version(&mut self, version: impl Into<String>) {
        let version = version.into();
        if !self.versions.contains(&version) {
            self.versions.push(version);
            self.normalize_versions();
        }
    }

    /// Sorts channel names first, then versions from newest to oldest.
    fn normalize_versions(&mut self) {
        self.versions.sort_by_key(|v| {
            let trimmed = v.trim();
            let semver = semver::Version::parse(trimmed).ok();
            let rank = match trimmed.to_lowercase().as_str() {
                "main" => 0,
                "git" => 1,
                _ if semver.is_some() => 2,
                _ => 3,
            };
            (rank, std::cmp::Reverse(semver), trimmed.to_lowercase())
        });
    }
}

/// Build the documentation for the specified packages and chips.
pub fn build_documentation(
    workspace: &Path,
    packages: &mut [Package],
    chips: &mut [Chip],
    base_url: Option<String>,
    channel: Option<String>,
) -> Result<()> {
    log::info!("Building documentation for packages: {packages:?} on chips: {chips:?}");
    let output_path = workspace.join("docs");

    fs::create_dir_all(&output_path)
        .with_context(|| format!("Failed to create {}", output_path.display()))?;

    packages.sort();

    for package in packages {
        // Not all packages need documentation built:
        if !package.is_published() {
            continue;
        }

        // Download the manifest from the documentation server. A package that was never
        // deployed has no manifest, but a failed download must not look like an empty
        // manifest: the manifest decides if this build may claim the `latest` redirect.
        let mut manifest = match fetch_manifest(&base_url, package)? {
            ManifestFetch::Found(manifest) => manifest,
            ManifestFetch::NotFound => Manifest::default(),
        };

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

        let version = crate::package_version(workspace, *package)?;
        let version_label = channel.clone().unwrap_or_else(|| version.to_string());

        // Update the package manifest to include the channel or crate version:
        manifest.insert_version(&version_label);

        // Build the documentation for the package:
        if chips.is_empty() {
            build_documentation_for_package(
                workspace,
                package,
                None,
                &version,
                channel.as_deref(),
                base_url.as_deref(),
                &manifest,
            )?;
        } else {
            for chip in chips {
                build_documentation_for_package(
                    workspace,
                    package,
                    Some(chip),
                    &version,
                    channel.as_deref(),
                    base_url.as_deref(),
                    &manifest,
                )?;
            }
        }

        let package_output_path = output_path.join(package.to_string());

        // Create the package subdirectory if it doesn't exist
        fs::create_dir_all(&package_output_path)
            .with_context(|| format!("Failed to create {}", package_output_path.display()))?;

        // Write out the package manifest JSON file:
        fs::write(
            package_output_path.join("manifest.json"),
            serde_json::to_string(&manifest)
                .with_context(|| format!("Failed to parse {manifest:?}"))?,
        )
        .with_context(|| format!("Failed to write out {}", output_path.display()))?;

        // Patch the generated documentation to include a select box for the version:
        #[cfg(feature = "deploy-docs")]
        patch_documentation_index_for_package(workspace, package, &version_label, &base_url)?;
    }

    Ok(())
}

fn build_documentation_for_package(
    workspace: &Path,
    package: &Package,
    chip: Option<Chip>,
    version: &semver::Version,
    channel: Option<&str>,
    base_url: Option<&str>,
    manifest: &Manifest,
) -> Result<()> {
    // Ensure that the package/chip combination provided are valid:
    if let Some(chip) = chip
        && let Err(err) = package.validate_package_chip(&chip)
    {
        log::warn!("{err}");
        return Ok(());
    }

    // Build the documentation for the specified package, targeting the
    // specified chip:
    let docs_path = cargo_doc(workspace, *package, chip, channel, base_url)?;

    ensure!(
        docs_path.exists(),
        "Documentation not found at {}",
        docs_path.display()
    );

    let version_component = docs_version_component(version, channel);
    let mut output_path = workspace
        .join("docs")
        .join(package.to_string())
        .join(&version_component);

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

    // Hosted docs rely on nginx to rewrite `/latest/...` deep links to `/latest/?path=...`.
    if should_write_latest_redirect(version, channel, manifest) {
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
        create_dir_all(latest_path.clone())
            .with_context(|| format!("Failed to create dir in {}", latest_path.display()))?;
        let latest_html = latest_redirect_html(workspace, package, version)
            .with_context(|| format!("Failed to render latest redirect for {}", package))?;
        std::fs::File::create(latest_path.clone().join("index.html"))?
            .write_all(latest_html.as_bytes())
            .with_context(|| format!("Failed to create or write to {}", latest_path.display()))?;
    }

    Ok(())
}

/// Directory name under `docs/<package>/` for this build.
fn docs_version_component(version: &semver::Version, channel: Option<&str>) -> String {
    match channel {
        Some(channel) => channel.to_string(),
        None => version.to_string(),
    }
}

/// Whether a version may claim the `latest` redirect or a root-index row.
///
/// Any stable release is eligible. A pre-release is eligible only when it is
/// `X.0.0-pre` for some major version `X >= 1` (for example `1.0.0-beta.0`).
fn is_eligible_docs_version(version: &semver::Version) -> bool {
    version.pre.is_empty() || (version.major >= 1 && version.minor == 0 && version.patch == 0)
}

/// Whether this build should write `latest/index.html`.
fn should_write_latest_redirect(
    version: &semver::Version,
    channel: Option<&str>,
    manifest: &Manifest,
) -> bool {
    if channel.is_some() {
        return false;
    }
    if !is_eligible_docs_version(version) {
        return false;
    }
    // Only a version that may claim the redirect itself can block this build from
    // claiming it. An ineligible pre-release must not hold `latest` hostage.
    match highest_eligible_version(manifest) {
        Some(highest) => *version >= highest,
        None => true,
    }
}

/// Highest eligible semver in `manifest`, if it holds one.
fn highest_eligible_version(manifest: &Manifest) -> Option<semver::Version> {
    let mut best: Option<semver::Version> = None;
    for entry in &manifest.versions {
        let Ok(ver) = semver::Version::parse(entry) else {
            continue;
        };
        if !is_eligible_docs_version(&ver) {
            continue;
        }
        best = Some(match best {
            None => ver,
            Some(b) => ver.max(b),
        });
    }
    best
}

/// Relative URL from `latest/index.html` to this package's versioned docs root.
fn latest_redirect_base(package: &Package, version: &semver::Version) -> String {
    if package.chip_features_matter() {
        format!("../{}/", version)
    } else {
        format!("../{}/{}/", version, package.to_string().replace('-', "_"))
    }
}

/// Renders `latest/index.html`, which redirects into the semver docs.
fn latest_redirect_html(
    workspace: &Path,
    package: &Package,
    version: &semver::Version,
) -> Result<String> {
    let base = latest_redirect_base(package, version);
    let resources_path = workspace.join("resources");

    render_template(&resources_path, "latest_redirect.html.somni", {
        let mut env = Env::new();
        env.value("base", base.as_str());
        env
    })
}

fn cargo_doc(
    workspace: &Path,
    package: Package,
    chip: Option<Chip>,
    channel: Option<&str>,
    base_url: Option<&str>,
) -> Result<PathBuf> {
    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    // Process Cargo.toml for documentation
    let pre_process_res = pre_process_cargo_toml(chip, &package_path);

    if pre_process_res.is_ok() {
        let cargo_doc_res =
            cargo_doc_without_pre_processing(workspace, package, chip, channel, base_url);

        // Restore the original Cargo.toml
        restore_cargo_toml(package_path)?;
        cargo_doc_res
    } else {
        // Restore the original Cargo.toml
        restore_cargo_toml(package_path)?;
        Err(pre_process_res.err().unwrap())
    }
}

fn cargo_doc_without_pre_processing(
    workspace: &Path,
    package: Package,
    chip: Option<Chip>,
    channel: Option<&str>,
    base_url: Option<&str>,
) -> Result<PathBuf> {
    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    // build documentation using the pre-processed Cargo.toml
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

    log::debug!("Using toolchain '{toolchain}'");

    // Determine the appropriate build target for the given package and chip,
    // if we're able to:
    let target = if let Some(ref chip) = chip {
        Some(package.target_triple(chip)?)
    } else {
        None
    };

    // cleaning the docs isn't strictly necessary but otherwise rustdoc will pick up
    // any docs which were built previously resulting in inconsistency regarding how esp-*
    // inter-crate links are rendered
    //
    // however a real problem is that this could pick up e.g. esp-hal docs for the wrong target chip
    // (rustdoc doesn't know about our chip features, it just knows about target triples!)
    //
    // this was never an issue when building via the GitHub workflow because of the way the job is
    // setup but better safe than sorry
    //
    // ideally we could just use `cargo clean --doc` but for `+esp` this turned out to be not
    // working as intended
    let mut docs_target = if let Ok(target_path) = std::env::var("CARGO_TARGET_DIR") {
        PathBuf::from(target_path)
    } else {
        workspace.join(package.to_string()).join("target")
    };
    if let Some(ref target) = target {
        docs_target = docs_target.join(&target);
    };
    docs_target = docs_target.join("doc");

    if docs_target.exists() {
        log::info!("Removing directory {:?}", docs_target);
        fs::remove_dir_all(&docs_target)
            .with_context(|| format!("Unable to remove {:?}", docs_target))?;
    }

    let mut features = vec![];
    let doc_config = if let Some(chip) = &chip {
        features.push(chip.to_string());
        package.doc_config_rules(Config::for_chip(chip))
    } else {
        package.doc_config_rules(&Config::default())
    };
    if let Some(doc_config) = &doc_config {
        features.extend(doc_config.features.clone());
    }

    // Build up an array of command-line arguments to pass to `cargo`:
    let mut builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
        .subcommand("doc")
        .features(&features)
        .arg("-Zrustdoc-map")
        .arg("--lib")
        .arg("--no-deps");

    if let Some(ref target) = target {
        builder = builder.target(target);
    }

    // Special case: `esp-metadata` requires `std`, and we get some really confusing
    // errors if we try to pass `-Zbuild-std=core`:
    if package.needs_build_std() {
        builder.add_arg("-Zbuild-std=alloc,core");
    }

    // Make `esp-*` eco-system inter-crate links work
    let docs_base = base_url
        .unwrap_or("https://docs.espressif.com/projects/rust")
        .trim_end_matches('/');
    let mut repo_html_root_urls = Vec::new();
    let mut manifest = CargoToml::new(workspace, package)
        .with_context(|| format!("Unable to get TOML for {}", package_name))?;
    let repo_deps = manifest.repo_dependencies();
    for dep in repo_deps {
        let chip_features_matter = dep.chip_features_matter();
        let toml = dep.toml();
        if let Some(ref toml) = *toml {
            let version = if let Some(channel) = channel {
                channel
            } else {
                toml.version()
            };

            let pkg = dep.to_string();

            let normalized_pkg = pkg.replace('-', "_");

            let chip = if let Some(chip) = chip
                && chip_features_matter
            {
                format!("{chip}/")
            } else {
                String::new()
            };

            repo_html_root_urls.push(format!(
                "--extern-html-root-url {normalized_pkg}={docs_base}/{pkg}/{version}/{chip}"
            ));
        }
    }

    let mut rustdocflags = Vec::from_iter(
        [
            "--cfg docsrs",
            "--cfg not_really_docsrs",
            "-Z unstable-options",
            // fix-up broken documentation links for `rand_core`
            "--extern-html-root-takes-precedence",
            "--extern-html-root-url rand_core_06=https://docs.rs/rand_core/0.6.4/",
            "--extern-html-root-url rand_core_09=https://docs.rs/rand_core/0.9.5/",
        ]
        .iter()
        .map(|v| v.to_string()),
    );

    rustdocflags.extend_from_slice(&repo_html_root_urls);

    let rustdocflags = rustdocflags.join(" ");

    if let Some(doc_config) = &doc_config {
        for (key, value) in &doc_config.env {
            builder.add_env_var(key, value);
        }
    }
    builder.add_env_var("RUSTDOCFLAGS", &rustdocflags);
    // Special case: `esp-storage` requires the optimization level to be 2 or 3:
    if package == Package::EspStorage {
        builder.add_env_var("CARGO_PROFILE_DEBUG_OPT_LEVEL", "3");
    }

    let command = CargoCommandBatcher::build_one_for_cargo(&builder);
    log::debug!("{command:#?}");
    crate::cargo::run_with_env(&command.command, &package_path, command.env_vars, false)
        .with_context(|| format!("Failed to run `cargo doc` for {}", package_name))?;

    // Build up the path at which the built documentation can be found:
    let mut docs_path = if let Ok(target_path) = std::env::var("CARGO_TARGET_DIR") {
        PathBuf::from(target_path)
    } else {
        workspace.join(package.to_string()).join("target")
    };

    if let Some(ref target) = target {
        docs_path = docs_path.join(target);
    }
    docs_path = docs_path.join("doc");

    Ok(crate::windows_safe_path(&docs_path))
}

/// Pre-process the Cargo.toml file
///
/// This will keep the original as "Cargo.toml_original"
///
/// This will check for `#DOC_IF <condition>` lines - evaluating the condition to false will turn
/// any documenting comments into non-documenting comments until a `#DOC_ENDIF` line is encountered.
///
/// The only currently supported function for expressions is `has(<SYMBOL>)`
/// e.g. `has("psram")`
fn pre_process_cargo_toml(chip: Option<Chip>, package_path: &Path) -> Result<(), anyhow::Error> {
    let cargo_toml = std::fs::read_to_string(windows_safe_path(&package_path.join("Cargo.toml")))
        .with_context(|| {
        format!(
            "Failed to read {}",
            package_path.join("Cargo.toml").display()
        )
    })?;

    std::fs::rename(
        windows_safe_path(&package_path.join("Cargo.toml")),
        windows_safe_path(&package_path.join("Cargo.toml_original")),
    )
    .with_context(|| {
        format!(
            "Failed to rename {}",
            package_path.join("Cargo.toml").display()
        )
    })?;

    let cargo_toml = cargo_toml.lines();

    let chip_cfg = chip.as_ref().map(|chip| Config::for_chip(chip));
    let mut processed_cargo_toml = Vec::new();
    let mut engine = somni_expr::Context::new();
    engine.add_function("has", move |cond: &str| -> bool {
        if let Some(chip_cfg) = chip_cfg {
            chip_cfg.symbols.iter().any(|symbol| cond == symbol)
        } else {
            false
        }
    });

    let mut hide = false;
    for line in cargo_toml {
        let mut line = line.to_string();

        if line.starts_with("#DOC_ENDIF") {
            hide = false;
            continue;
        }

        if line.starts_with("#DOC_IF ") {
            let expr = line.strip_prefix("#DOC_IF ").unwrap();
            hide = !(engine
                .evaluate::<bool>(expr)
                .map_err(|err| anyhow::anyhow!(format!("{:?}", err)))
                .with_context(|| format!("Error evaluating expression in Cargo.toml: {}", expr))?);
            continue;
        }

        if hide {
            if line.starts_with("#!") {
                line = format!("#{}", line.strip_prefix("#!").unwrap_or_default());
            } else if line.starts_with("##") {
                line = format!("#{}", line.strip_prefix("##").unwrap_or_default());
            }
        }

        processed_cargo_toml.push(line);
    }

    std::fs::write(
        windows_safe_path(&package_path.join("Cargo.toml")),
        processed_cargo_toml.join("\n"),
    )
    .with_context(|| {
        format!(
            "Failed to write pre-processed {}",
            package_path.join("Cargo.toml").display()
        )
    })?;

    Ok(())
}

/// Restore the original Cargo.toml file
fn restore_cargo_toml(package_path: PathBuf) -> Result<(), anyhow::Error> {
    if std::fs::exists(windows_safe_path(&package_path.join("Cargo.toml_original")))? {
        if std::fs::exists(windows_safe_path(&package_path.join("Cargo.toml")))? {
            std::fs::remove_file(windows_safe_path(&package_path.join("Cargo.toml")))?;
        }

        std::fs::rename(
            windows_safe_path(&package_path.join("Cargo.toml_original")),
            windows_safe_path(&package_path.join("Cargo.toml")),
        )?;
    }

    Ok(())
}

#[cfg(feature = "deploy-docs")]
fn patch_documentation_index_for_package(
    workspace: &Path,
    package: &Package,
    version: &str,
    base_url: &Option<String>,
) -> Result<()> {
    use kuchikiki::traits::*;

    let package_name = package.to_string().replace('-', "_");
    let package_path = workspace.join("docs").join(package.to_string());
    let version_path = package_path.join(version);

    let mut index_paths = Vec::new();

    if package.chip_features_matter() {
        for chip_path in fs::read_dir(&version_path)
            .with_context(|| format!("Failed to read {}", version_path.display()))?
        {
            let chip_path = chip_path?.path();
            if chip_path.is_dir() {
                let path = chip_path.join(&package_name).join("index.html");
                index_paths.push(path);
            }
        }
    } else {
        let path = version_path.join(&package_name).join("index.html");
        index_paths.push(path);
    }

    for index_path in index_paths {
        let html = fs::read_to_string(&index_path)
            .with_context(|| format!("Failed to read {}", index_path.display()))?;
        let document = kuchikiki::parse_html().one(html);

        let elem = document
            .select_first(".sidebar-crate")
            .expect("Unable to select '.sidebar-crate' element in HTML");

        let base_url = base_url.clone().unwrap_or_default();
        let resources_path = workspace.join("resources");

        let html = render_template(&resources_path, "select.html.somni", {
            let mut env = Env::new();
            env.value("base_url", base_url)
                .value("package", package.to_string())
                .value("version", version.to_string());
            env
        })?;

        let node = elem.as_node();
        node.append(kuchikiki::parse_html().one(html));

        fs::write(&index_path, document.to_string())
            .with_context(|| format!("Failed to write to {}", index_path.display()))?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Build Documentation Index

/// Build the documentation index for all packages.
pub fn build_documentation_index(
    workspace: &Path,
    packages: &mut [Package],
    base_url: Option<String>,
) -> Result<()> {
    let docs_path = workspace.join("docs");
    let resources_path = workspace.join("resources");

    packages.sort();

    for package in packages {
        log::debug!("Building documentation index for package '{package}'");
        // Not all packages have documentation built:
        if !package.is_published() {
            continue;
        }

        // If the chip features are not relevant, then there is no need to generate an
        // index for the given package's documentation:
        if !package.chip_features_matter() {
            log::warn!(
                "Package '{package}' does not have device-specific documentation, no need to generate an index"
            );
            continue;
        }

        let package_docs_path = docs_path.join(package.as_ref());

        // Each path we iterate over should be the directory for a given version of
        // the package's documentation (`latest` is skipped).
        if !package_docs_path.exists() {
            log::warn!(
                "Package documentation path does not exist: '{}', skipping",
                package_docs_path.display()
            );
            continue;
        }
        for version_path in fs::read_dir(&package_docs_path)
            .with_context(|| format!("Failed to read {}", package_docs_path.display()))?
        {
            let version_path = version_path?.path();
            if version_path.is_file() {
                log::debug!(
                    "Path is not a directory, skipping: '{}'",
                    version_path.display()
                );
                continue;
            }

            let version_label = version_path
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            if version_label == "latest" {
                continue;
            }

            // Collect all chip directories in this version's docs
            let mut chips = vec![];
            for path in fs::read_dir(&version_path)
                .with_context(|| format!("Failed to read {}", version_path.display()))?
            {
                let path = path?.path();
                if path.is_dir() {
                    let chip = path
                        .components()
                        .next_back()
                        .unwrap()
                        .as_os_str()
                        .to_string_lossy();

                    match Chip::from_str(&chip, true) {
                        Ok(chip) => chips.push(chip),
                        Err(e) => {
                            log::warn!("Folder name is not a valid chip name: {chip} - {e}");
                        }
                    }
                }
            }

            chips.sort();

            let meta = generate_documentation_meta_for_package(*package, &chips, &version_label)?;

            // Render the template to HTML and write it out to the desired path:
            let html = render_template(&resources_path, "package_index.html.somni", {
                let mut env = Env::new();
                env.value("package", package.to_string().replace('-', "_"))
                    .value("metadata", Iter(meta));
                env
            })?;
            let path = version_path.join("index.html");
            fs::write(&path, html).context("Failed to write index.html")?;
            log::info!("Created {}", path.display());
        }
    }

    // Copy any additional assets to the documentation's output path:
    fs::copy(
        resources_path.join("esp-rs.svg"),
        docs_path.join("esp-rs.svg"),
    )
    .context("Failed to copy esp-rs.svg")?;
    fs::copy(
        resources_path.join("esp-rs-grey-bg.svg"),
        docs_path.join("esp-rs-grey-bg.svg"),
    )
    .context("Failed to copy esp-rs-grey-bg.svg")?;

    let meta = generate_documentation_meta_for_index(workspace, &base_url)?;

    // Render the template to HTML and write it out to the desired path:
    let html = render_template(&resources_path, "index.html.somni", {
        let mut env = Env::new();
        env.value("metadata", Iter(meta));
        env
    })?;
    let path = docs_path.join("index.html");
    fs::write(&path, html).context("Failed to write index.html")?;
    log::info!("Created {}", path.display());

    // Render the 404 template (it has no placeholders to fill in):
    let html = render_template(&resources_path, "404.html.somni", Env::new())?;
    let path = docs_path.join("404.html");
    fs::write(&path, html).context("Failed to write 404.html")?;
    log::info!("Created {}", path.display());

    Ok(())
}

fn generate_documentation_meta_for_package(
    package: Package,
    chips: &[Chip],
    doc_tree_version: &str,
) -> Result<Vec<Meta>> {
    let mut metadata = Vec::new();
    let types = &mut TemplateTypes::default();

    let name = package.to_string();
    let crate_name = name.replace('-', "_");
    let version = doc_tree_version;

    for chip in chips {
        // Ensure that the package/chip combination provided are valid:
        if let Err(err) = package.validate_package_chip(chip) {
            log::warn!("{err}");
            continue;
        }

        // Build the context object required for rendering this particular build's
        // information on the documentation index:
        let chip_name = chip.to_string();
        metadata.push(somni_struct!(
            types,
            Meta {
                name: name.as_str(),
                version: version,
                chip: chip_name.as_str(),
                chip_pretty: chip.pretty_name(),
                package: crate_name.as_str(),
            }
        ));
    }

    log::debug!("Generated metadata for package '{package}': {metadata:#?}");

    Ok(metadata)
}

fn generate_documentation_meta_for_index(
    workspace: &Path,
    base_url: &Option<String>,
) -> Result<Vec<Meta>> {
    let mut metadata = Vec::new();
    let docs_path = workspace.join("docs");
    let types = &mut TemplateTypes::default();

    for package in Package::iter() {
        // Not all packages have documentation built:
        if !package.is_published() {
            continue;
        }

        let manifest = load_manifest_for_index(&docs_path, &package, base_url)?;
        // A manifest that holds no eligible version must not send the reader to a channel
        // that this package may never have deployed. The version of this checkout is the
        // best guess that a release run can make.
        let version = match highest_eligible_version(&manifest) {
            Some(version) => version.to_string(),
            None => crate::package_version(workspace, package)?.to_string(),
        };

        let url = if package.chip_features_matter() {
            format!("{package}/{version}/index.html")
        } else {
            let crate_name = package.to_string().replace('-', "_");
            format!("{package}/{version}/{crate_name}/index.html")
        };

        let name = package.to_string();
        metadata.push(somni_struct!(
            types,
            Meta {
                name: name.as_str(),
                version: version.as_str(),
                url: url.as_str(),
            }
        ));
    }

    log::debug!("Generated metadata for documentation index: {metadata:#?}");

    Ok(metadata)
}

fn load_manifest_for_index(
    docs_path: &Path,
    package: &Package,
    base_url: &Option<String>,
) -> Result<Manifest> {
    let local_path = docs_path.join(package.to_string()).join("manifest.json");
    if local_path.exists() {
        let contents = fs::read_to_string(&local_path)
            .with_context(|| format!("Failed to read {}", local_path.display()))?;
        return serde_json::from_str(&contents)
            .with_context(|| format!("Failed to parse {}", local_path.display()));
    }

    match fetch_manifest(base_url, package)? {
        ManifestFetch::Found(manifest) => Ok(manifest),
        ManifestFetch::NotFound => Ok(Manifest::default()),
    }
}

// ----------------------------------------------------------------------------
// Helper Functions

#[derive(Debug)]
enum ManifestFetch {
    /// Present when `deploy-docs` successfully downloads a manifest.
    #[allow(dead_code)]
    Found(Manifest),
    /// HTTP 404, missing base URL, or `deploy-docs` disabled.
    NotFound,
}

fn fetch_manifest(base_url: &Option<String>, package: &Package) -> Result<ManifestFetch> {
    let Some(base_url) = base_url
        .as_ref()
        .map(|url| url.trim_end_matches('/'))
        .filter(|url| !url.is_empty())
    else {
        return Ok(ManifestFetch::NotFound);
    };

    let manifest_url = format!("{base_url}/{package}/manifest.json");

    #[cfg(feature = "deploy-docs")]
    {
        /// A transient failure of the documentation server must not fail a build, but the
        /// manifest is also too important to give up on. Try this many times.
        const ATTEMPTS: u32 = 4;

        let mut last_error = None;
        for attempt in 1..=ATTEMPTS {
            if attempt > 1 {
                // Back off for 1s, 2s, then 4s.
                let delay = std::time::Duration::from_secs(1 << (attempt - 2));
                log::warn!(
                    "Retrying manifest download in {}s ({attempt}/{ATTEMPTS})",
                    delay.as_secs()
                );
                std::thread::sleep(delay);
            }

            last_error = match reqwest::blocking::get(&manifest_url) {
                Ok(resp) if resp.status().is_success() => {
                    return Ok(ManifestFetch::Found(resp.json()?));
                }
                // The package was never deployed. Retrying cannot change that.
                Ok(resp) if resp.status() == reqwest::StatusCode::NOT_FOUND => {
                    return Ok(ManifestFetch::NotFound);
                }
                Ok(resp) => Some(anyhow::anyhow!(
                    "Unable to fetch package manifest from {manifest_url}: {}",
                    resp.status()
                )),
                Err(err) => Some(anyhow::anyhow!(
                    "Unable to fetch package manifest from {manifest_url}: {err}"
                )),
            };
        }

        Err(last_error.unwrap().context(format!(
            "Failed to fetch the manifest of {package} in {ATTEMPTS} attempts"
        )))
    }

    #[cfg(not(feature = "deploy-docs"))]
    {
        let _ = manifest_url;
        let _ = package;
        Ok(ManifestFetch::NotFound)
    }
}

fn render_template(resources: &Path, template: &str, env: Env) -> Result<String> {
    let source = fs::read_to_string(resources.join(template))
        .context(format!("Failed to read {template}"))?;

    let tmpl = Template::compile(&source, &Syntax::brackets())
        .map_err(|err| anyhow::anyhow!("{}", err.display_with(&source)))
        .with_context(|| format!("Failed to compile {template}"))?;

    tmpl.render(env)
        .map_err(|err| anyhow::anyhow!("{}", err.display_with(&source)))
        .with_context(|| format!("Failed to render {template}"))
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use somni_expr::somni_struct;
    use somni_template::{Env, Iter, TemplateTypes};

    use super::{
        Manifest,
        docs_version_component,
        generate_documentation_meta_for_package,
        highest_eligible_version,
        is_eligible_docs_version,
        latest_redirect_base,
        latest_redirect_html,
        render_template,
        should_write_latest_redirect,
    };

    fn workspace() -> &'static Path {
        Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap()
    }

    fn version(s: &str) -> semver::Version {
        semver::Version::parse(s).unwrap()
    }

    fn manifest(versions: &[&str]) -> Manifest {
        let mut manifest = Manifest::default();
        for v in versions {
            manifest.insert_version(*v);
        }
        manifest
    }

    #[test]
    fn eligible_docs_version_rule() {
        assert!(is_eligible_docs_version(&version("0.5.0")));
        assert!(is_eligible_docs_version(&version("1.0.0")));
        assert!(is_eligible_docs_version(&version("1.0.0-beta.0")));
        assert!(is_eligible_docs_version(&version("2.0.0-rc.1")));
        assert!(!is_eligible_docs_version(&version("0.6.0-rc.0")));
        assert!(!is_eligible_docs_version(&version("1.1.0-rc.0")));
        assert!(!is_eligible_docs_version(&version("1.0.1-beta.0")));
    }

    #[test]
    fn latest_redirect_respects_channel_eligibility_and_manifest() {
        let empty = Manifest::default();
        assert!(should_write_latest_redirect(
            &version("1.0.0-beta.0"),
            None,
            &empty
        ));
        assert!(!should_write_latest_redirect(
            &version("1.0.0-beta.0"),
            Some("main"),
            &empty
        ));
        assert!(!should_write_latest_redirect(
            &version("1.1.0-rc.0"),
            None,
            &empty
        ));
        assert!(should_write_latest_redirect(
            &version("0.5.0"),
            None,
            &empty
        ));
        assert!(!should_write_latest_redirect(
            &version("0.6.0-rc.0"),
            None,
            &empty
        ));

        let with_beta = manifest(&["1.0.0-beta.0", "main"]);
        assert!(!should_write_latest_redirect(
            &version("0.24.0"),
            None,
            &with_beta
        ));
        assert!(should_write_latest_redirect(
            &version("1.0.0"),
            None,
            &with_beta
        ));
        assert!(!should_write_latest_redirect(
            &version("1.1.0-rc.0"),
            None,
            &with_beta
        ));

        // A pre-release that cannot claim the redirect must not block a release from
        // claiming it, even though it sorts higher.
        let with_rc = manifest(&["1.0.0", "1.1.0-rc.0", "main"]);
        assert!(should_write_latest_redirect(
            &version("1.0.1"),
            None,
            &with_rc
        ));
        // A rebuild of the release that holds the redirect keeps it.
        assert!(should_write_latest_redirect(
            &version("1.0.0"),
            None,
            &with_rc
        ));
        // An older release does not take the redirect from a newer one.
        assert!(!should_write_latest_redirect(
            &version("0.9.0"),
            None,
            &with_rc
        ));
    }

    #[test]
    fn channel_replaces_version_in_output_path_component() {
        assert_eq!(
            docs_version_component(&version("1.1.0"), Some("main")),
            "main"
        );
        assert_eq!(docs_version_component(&version("1.1.0"), None), "1.1.0");
    }

    #[test]
    fn index_version_selects_highest_eligible() {
        assert_eq!(
            highest_eligible_version(&manifest(&["1.0.0-beta.0", "1.1.0-rc.0", "main"])),
            Some(version("1.0.0-beta.0"))
        );
        assert_eq!(
            highest_eligible_version(&manifest(&["0.5.0", "0.6.0-rc.0"])),
            Some(version("0.5.0"))
        );
        assert_eq!(
            highest_eligible_version(&manifest(&["1.0.0", "1.1.0", "main"])),
            Some(version("1.1.0"))
        );
        assert_eq!(highest_eligible_version(&manifest(&["main"])), None);
        assert_eq!(highest_eligible_version(&Manifest::default()), None);
    }

    #[test]
    fn latest_redirect_base_paths() {
        let v = semver::Version::parse("2.1.0").unwrap();
        assert_eq!(
            latest_redirect_base(&crate::Package::EspHal, &v),
            "../2.1.0/"
        );
        assert_eq!(
            latest_redirect_base(&crate::Package::EspAlloc, &v),
            "../2.1.0/esp_alloc/"
        );
    }

    /// Small integration check for the redirect template.
    #[test]
    fn latest_redirect_html_matches_latest_redirect_base() {
        let ws = workspace();
        let ver = semver::Version::parse("1.0.0").unwrap();
        let pkg = crate::Package::EspHal;

        let html = latest_redirect_html(ws, &pkg, &ver).unwrap();
        let base = latest_redirect_base(&pkg, &ver);

        assert!(html.contains(&format!("var base = \"{base}\"")));
        assert!(html.contains(&format!("content=\"0; url={base}\"")));
    }

    /// The version select box is embedded in a page full of braces, so make sure the
    /// surrounding JavaScript survives templating untouched.
    #[test]
    fn version_select_renders() {
        let html = render_template(&workspace().join("resources"), "select.html.somni", {
            let mut env = Env::new();
            env.value("base_url", "https://docs.espressif.com/projects/rust")
                .value("package", "esp-hal")
                .value("version", "1.0.0");
            env
        })
        .unwrap();

        assert!(
            html.contains(r#"<option value="1.0.0" selected="selected">1.0.0</option>"#),
            "{html}"
        );
        assert!(
            html.contains(
                r#"const manifestUrl = "https://docs.espressif.com/projects/rust/esp-hal/manifest.json";"#
            ),
            "{html}"
        );
        assert!(html.contains(r#"const packageName = "esp-hal";"#), "{html}");
        assert!(html.contains("segments.indexOf(packageName)"), "{html}");
        assert!(html.contains(".then(({ versions }) => {"), "{html}");
        assert!(html.contains("versions.includes(selected)"), "{html}");
    }

    #[test]
    fn package_index_renders_a_row_per_chip() {
        let ws = workspace();
        let meta = generate_documentation_meta_for_package(
            crate::Package::EspHal,
            &[crate::Chip::Esp32c6, crate::Chip::Esp32s3],
            "1.0.0",
        )
        .unwrap();

        let html = render_template(&ws.join("resources"), "package_index.html.somni", {
            let mut env = Env::new();
            env.value("package", "esp_hal")
                .value("metadata", Iter(meta));
            env
        })
        .unwrap();

        assert!(
            html.contains("<title>esp_hal Documentation</title>"),
            "{html}"
        );
        assert!(
            html.contains(r#"<a href="esp32c6/esp_hal/index.html">"#),
            "{html}"
        );
        assert!(
            html.contains(r#"<a href="esp32s3/esp_hal/index.html">"#),
            "{html}"
        );
        assert!(
            html.contains("<span class=\"crate-version\">1.0.0</span>"),
            "{html}"
        );
    }

    #[test]
    fn package_index_renders_channel_label() {
        let meta = generate_documentation_meta_for_package(
            crate::Package::EspHal,
            &[crate::Chip::Esp32c6],
            "main",
        )
        .unwrap();

        let html = render_template(
            &workspace().join("resources"),
            "package_index.html.somni",
            {
                let mut env = Env::new();
                env.value("package", "esp_hal")
                    .value("metadata", Iter(meta));
                env
            },
        )
        .unwrap();

        assert!(
            html.contains("<span class=\"crate-version\">main</span>"),
            "{html}"
        );
    }

    #[test]
    fn index_renders_a_row_per_package() {
        // `generate_documentation_meta_for_index` resolves manifests relative to the current
        // directory, which is the crate root under `cargo test`, so build the rows by hand.
        let types = &mut TemplateTypes::default();
        let meta = vec![
            somni_struct!(
                types,
                Meta {
                    name: "esp-hal",
                    version: "1.0.0",
                    url: "esp-hal/1.0.0/index.html",
                }
            ),
            somni_struct!(
                types,
                Meta {
                    name: "esp-alloc",
                    version: "0.9.0",
                    url: "esp-alloc/0.9.0/esp_alloc/index.html",
                }
            ),
        ];

        let html = render_template(&workspace().join("resources"), "index.html.somni", {
            let mut env = Env::new();
            env.value("metadata", Iter(meta));
            env
        })
        .unwrap();

        assert!(
            html.contains(r#"<a href="esp-hal/1.0.0/index.html">esp-hal</a>"#),
            "{html}"
        );
        assert!(
            html.contains(r#"<a href="esp-alloc/0.9.0/esp_alloc/index.html">esp-alloc</a>"#),
            "{html}"
        );
        assert!(html.contains("<code>esp-hal</code>"), "{html}");
    }

    #[test]
    fn test_manifest_sorting_and_dedup() {
        let mut manifest = Manifest::default();
        let input = vec![
            "main",
            "1.0.0-beta.1",
            "0.9.0",
            "1.2.0",
            "1.1.1",
            "1.1.0",
            "1.1.0-rc.1",
            "1.1.0-rc.0",
            "1.1.0-beta.0",
            "git",
            "1.0.0",
            "nightly",
            "1.0.0-beta.0",
            "main", // Duplicated main
        ];

        for v in input {
            manifest.insert_version(v);
        }

        assert_eq!(
            manifest.versions,
            vec![
                "main", // Rank 0
                "git",  // Rank 1
                "1.2.0",
                "1.1.1",
                "1.1.0",
                "1.1.0-rc.1",
                "1.1.0-rc.0",
                "1.1.0-beta.0",
                "1.0.0",
                "1.0.0-beta.1",
                "1.0.0-beta.0",
                "0.9.0",   // Semver descending
                "nightly"  // Rank 3 + alphabetical
            ]
        );
    }
}
