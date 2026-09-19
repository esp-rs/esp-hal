use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{Context, Error};
use cargo_semver_checks::{Check, GlobalConfig, ReleaseType, Rustdoc};

use crate::{
    Package,
    cargo::{CargoArgsBuilder, CargoCommandBatcher},
    commands::checker::download_baselines,
    metadata::{Chip, Config},
};

/// Return the minimum required bump for the next release and the current
/// rustdoc JSON path.
/// Even if nothing changed this will be [ReleaseType::Patch]
pub fn minimum_update(
    workspace: &Path,
    package: Package,
    chip: Chip,
) -> Result<(ReleaseType, PathBuf), Error> {
    log::info!("Package = {}, Chip = {}", package, chip);

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    let current_path = build_prepared_doc_json(
        package,
        &chip,
        &package_path,
        None,
        &workspace_rom_symbols(workspace),
    )?;

    let Some(baseline_path_gz) = baseline_gz(workspace, package, chip)? else {
        log::warn!(
            "No baseline found for package '{}', chip '{}' — skipping semver check for this chip",
            package,
            chip
        );
        return Ok((ReleaseType::Patch, current_path));
    };
    let baseline_path =
        tempfile::NamedTempFile::new().context("Failed to create a temporary file")?;
    decompress_gz(&baseline_path_gz, baseline_path.path())?;

    let mut semver_check = Check::new(Rustdoc::from_path(current_path.clone()));
    semver_check.set_baseline(Rustdoc::from_path(baseline_path.path()));
    let mut cfg = GlobalConfig::new();
    cfg.set_log_level(Some(log::Level::Info));
    let result = semver_check.check_release(&mut cfg)?;
    log::trace!("Result {:?}", result);

    let mut min_required_update = ReleaseType::Patch;
    for (_, report) in result.crate_reports() {
        if let Some(required_bump) = report.required_bump() {
            let required_is_stricter = (min_required_update == ReleaseType::Patch)
                || (required_bump == ReleaseType::Major);
            if required_is_stricter {
                min_required_update = required_bump;
            }
        }
    }

    Ok((min_required_update, current_path))
}

/// `{chip}` when chip features change the API, otherwise `api`.
pub(crate) fn baseline_stem(package: Package, chip: Chip) -> String {
    if package.chip_features_matter() {
        chip.to_string()
    } else {
        "api".to_string()
    }
}

/// Gzipped semver baseline for `package`/`chip`, downloading the package's
/// baselines if they are absent. `None` when chip features matter and that
/// chip has no baseline.
pub(crate) fn baseline_gz(
    workspace: &Path,
    package: Package,
    chip: Chip,
) -> Result<Option<PathBuf>, Error> {
    let package_path = crate::windows_safe_path(&workspace.join(package.to_string()));
    let baseline_path_gz = package_path.join(format!(
        "api-baseline/{}.json.gz",
        baseline_stem(package, chip)
    ));
    if !baseline_path_gz.exists() {
        download_baselines(workspace, vec![package])?;
    }
    if package.chip_features_matter() && !baseline_path_gz.exists() {
        return Ok(None);
    }
    Ok(Some(baseline_path_gz))
}

pub(crate) fn workspace_rom_symbols(workspace: &Path) -> PathBuf {
    workspace.join("esp-rom-sys/src/generated_rom_symbols.rs")
}

pub(crate) fn decompress_gz(src: &Path, dest: &Path) -> Result<(), Error> {
    if let Some(parent) = dest.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut decoder = flate2::write::GzDecoder::new(Vec::new());
    decoder.write_all(&fs::read(src)?)?;
    fs::write(dest, decoder.finish()?)?;
    Ok(())
}

/// `prepare_semver_check`, build rustdoc JSON, then `clean_semver_check`, which
/// runs even when the build fails. `rom_symbols_path` is the workspace copy for
/// in-tree builds, or the extracted-tag copy for an old release.
pub(crate) fn build_prepared_doc_json(
    package: Package,
    chip: &Chip,
    package_path: &PathBuf,
    target_dir: Option<&Path>,
    rom_symbols_path: &Path,
) -> Result<PathBuf, Error> {
    package.prepare_semver_check(package_path, chip)?;
    let result = build_doc_json(package, chip, package_path, target_dir);
    if let Err(cleanup) = package.clean_semver_check(rom_symbols_path) {
        // A build failure is what the caller can act on, so it is not replaced.
        if result.is_err() {
            log::warn!("Failed to clean up after building the doc JSON: {cleanup:#}");
        } else {
            return Err(cleanup);
        }
    }
    result
}

/// Build the rustdoc JSON of `package` for `chip`, under `target_dir` when given
/// and the package's own `target` otherwise. The override is for documenting
/// sources checked out elsewhere, such as an old release tag.
pub(crate) fn build_doc_json(
    package: Package,
    chip: &Chip,
    package_path: &PathBuf,
    target_dir: Option<&Path>,
) -> Result<PathBuf, Error> {
    let target_path = if let Some(target) = target_dir {
        target.to_path_buf()
    } else if let Ok(target) = std::env::var("CARGO_TARGET_DIR") {
        PathBuf::from(target)
    } else {
        PathBuf::from(package_path).join("target")
    };
    let current_path = target_path
        .join(chip.target())
        .join("doc")
        .join(format!("{}.json", package.to_string().replace("-", "_")));

    std::fs::remove_file(&current_path).ok();

    let mut features = vec![];
    features.push(chip.to_string());

    let chip_config = Config::for_chip(chip);

    let semver_config = package.semver_config_rules(&chip_config);
    features.extend(semver_config.features.clone());

    log::info!(
        "Building doc json for {} with features: {:?}, env: {:?}",
        package,
        features,
        semver_config.env
    );

    // always use `esp` toolchain so we don't have to deal with potentially
    // different versions of the doc-json
    let mut cargo_builder = CargoArgsBuilder::default()
        .toolchain("esp")
        .subcommand("rustdoc")
        .manifest_path(package_path.join("Cargo.toml"))
        .features(&features)
        .target(chip.target())
        .arg("-Zunstable-options")
        .arg("-Zhost-config")
        .arg("-Ztarget-applies-to-host")
        .arg("--lib")
        .arg("--output-format=json")
        .arg("-Zbuild-std=alloc,core")
        .arg("--config=host.rustflags=[\"--cfg=instability_disable_unstable_docs\"]");

    for (key, value) in &semver_config.env {
        cargo_builder.add_env_var(key, value);
    }
    cargo_builder.add_env_var(
        "RUSTDOCFLAGS",
        "--cfg docsrs --cfg not_really_docsrs --cfg semver_checks",
    );
    // Pinned so that cargo writes where we are about to look for the JSON.
    cargo_builder.add_env_var("CARGO_TARGET_DIR", &target_path.display().to_string());

    let command = CargoCommandBatcher::build_one_for_cargo(&cargo_builder);
    log::debug!("{command:#?}");
    let cargo_command = command.command.clone();
    crate::cargo::run_with_env(&command.command, package_path, command.env_vars, false)
        .with_context(|| format!("Failed to run `cargo rustdoc` with {cargo_command:?}",))?;
    anyhow::ensure!(
        current_path.exists(),
        "cargo rustdoc succeeded but did not write {}",
        current_path.display()
    );
    Ok(current_path)
}
