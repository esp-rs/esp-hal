use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};

use cargo_semver_checks::{Check, GlobalConfig, ReleaseType, Rustdoc};
use esp_metadata::Chip;

use crate::{Package, cargo::CargoArgsBuilder};

/// Return the minimum required bump for the next release.
/// Even if nothing changed this will be [ReleaseType::Patch]
pub fn minimum_update(
    workspace: &Path,
    package: Package,
    chip: Chip,
) -> Result<ReleaseType, anyhow::Error> {
    log::info!("Package = {}, Chip = {}", package, chip);

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));
    let current_path = build_doc_json(package, &chip, &package_path)?;

    let file_name = if package.chip_features_matter() {
        chip.to_string()
    } else {
        "api".to_string()
    };

    let baseline_path_gz =
        PathBuf::from(&package_path).join(format!("api-baseline/{}.json.gz", file_name));
    let baseline_path = temp_file::TempFile::new()?;
    let buffer = Vec::new();
    let mut decoder = flate2::write::GzDecoder::new(buffer);
    decoder.write_all(&(fs::read(&baseline_path_gz)?))?;
    fs::write(baseline_path.path(), decoder.finish()?)?;

    let mut semver_check = Check::new(Rustdoc::from_path(current_path));
    semver_check.set_baseline(Rustdoc::from_path(baseline_path.path()));
    let mut cfg = GlobalConfig::new();
    cfg.set_log_level(Some(log::Level::Info));
    let result = semver_check.check_release(&mut cfg)?;
    log::info!("Result {:?}", result);

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

    Ok(min_required_update)
}

pub(crate) fn build_doc_json(
    package: Package,
    chip: &Chip,
    package_path: &PathBuf,
) -> Result<PathBuf, anyhow::Error> {
    let target_dir = std::env::var("CARGO_TARGET_DIR");

    let target_path = if let Ok(target) = target_dir {
        PathBuf::from(target)
    } else {
        PathBuf::from(package_path).join("target")
    };
    let current_path = target_path
        .join(chip.target())
        .join("doc")
        .join(format!("{}.json", package.to_string().replace("-", "_")));

    std::fs::remove_file(&current_path).ok();
    let features = if package.chip_features_matter() {
        vec![chip.to_string()]
    } else {
        vec![]
    };

    log::info!(
        "Building doc json for {} with features: {:?}",
        package,
        features
    );

    let envs = vec![(
        "RUSTDOCFLAGS",
        "--cfg docsrs --cfg not_really_docsrs --cfg semver_checks",
    )];

    // always use `esp` toolchain so we don't have to deal with potentially
    // different versions of the doc-json
    let cargo_builder = CargoArgsBuilder::default()
        .toolchain("esp")
        .subcommand("rustdoc")
        .features(&features)
        .target(chip.target())
        .arg("-Zunstable-options")
        .arg("-Zhost-config")
        .arg("-Ztarget-applies-to-host")
        .arg("--lib")
        .arg("--output-format=json")
        .arg("-Zbuild-std=alloc,core")
        .arg("--config=host.rustflags=[\"--cfg=instability_disable_unstable_docs\"]");
    let cargo_args = cargo_builder.build();
    log::debug!("{cargo_args:#?}");
    crate::cargo::run_with_env(&cargo_args, package_path, envs, false)?;
    Ok(current_path)
}
