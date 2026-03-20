use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};

use anyhow::{Context, Error};
use cargo_semver_checks::{Check, GlobalConfig, ReleaseType, Rustdoc};
use esp_metadata::{Chip, Config};

use crate::{Package, cargo::CargoArgsBuilder, commands::checker::download_baselines};

/// Return the minimum required bump for the next release.
/// Even if nothing changed this will be [ReleaseType::Patch]
pub fn minimum_update(
    workspace: &Path,
    package: Package,
    chip: Chip,
) -> Result<ReleaseType, Error> {
    log::info!("Package = {}, Chip = {}", package, chip);

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    package.prepare_semver_check(&package_path, &chip)?;

    let current_path = build_doc_json(package, &chip, &package_path)?;

    let dest_path = workspace.join("esp-rom-sys/src/generated_rom_symbols.rs");
    package.clean_semver_check(&dest_path)?;

    let file_name = if package.chip_features_matter() {
        chip.to_string()
    } else {
        "api".to_string()
    };

    let baseline_path_gz =
        PathBuf::from(&package_path).join(format!("api-baseline/{}.json.gz", file_name));
    if !baseline_path_gz.exists() {
        download_baselines(&workspace, vec![package])?;
    }
    let baseline_path =
        temp_file::TempFile::new().with_context(|| format!("Failed to create a TempFile!"))?;
    let buffer = Vec::new();
    let mut decoder = flate2::write::GzDecoder::new(buffer);
    decoder.write_all(&(fs::read(&baseline_path_gz)?))?;
    fs::write(baseline_path.path(), decoder.finish()?)?;

    let mut semver_check = Check::new(Rustdoc::from_path(current_path));
    semver_check.set_baseline(Rustdoc::from_path(baseline_path.path()));
    let mut cfg = GlobalConfig::new();
    cfg.set_log_level(Some(log::Level::Info));
    let result = semver_check.check_release(&mut cfg)?;
    log::trace!("Result {:?}", result);

    let required_bumps: Vec<ReleaseType> = result
        .crate_reports()
        .into_iter()
        .filter_map(|(_, report)| report.required_bump())
        .collect();

    let min_required_update = required_bump(&required_bumps, package.is_forever_unstable());
    Ok(min_required_update)
}

pub(crate) fn build_doc_json(
    package: Package,
    chip: &Chip,
    package_path: &PathBuf,
) -> Result<PathBuf, Error> {
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

    let mut features = vec![];
    features.push(chip.to_string());

    let chip_config = Config::for_chip(chip);

    let semver_feature = package.semver_feature_rules(&chip_config);
    features.extend(semver_feature);

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
    crate::cargo::run_with_env(&cargo_args, package_path, envs, false)
        .with_context(|| format!("Failed to run `cargo rustdoc` with {cargo_args:?}",))?;
    Ok(current_path)
}

fn required_bump(required_bumps: &[ReleaseType], forever_unstable: bool) -> ReleaseType {
    let mut min_required_update = ReleaseType::Patch;

    for &required_bump in required_bumps {
        let required_is_stricter =
            (min_required_update == ReleaseType::Patch) || (required_bump == ReleaseType::Major);

        if required_is_stricter {
            min_required_update = required_bump;
        }
    }

    if forever_unstable && min_required_update != ReleaseType::Patch {
        panic!(
            "Forever-unstable package can only receive a patch release but received {:?}!",
            min_required_update
        );
    }

    min_required_update
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    // Semver-check requiring a major bump for a 0.x crate
    fn major_bump_from_0x_to_0x_plus_1() {
        let bumps = [ReleaseType::Major];

        let result = required_bump(&bumps, false);

        assert_eq!(result, ReleaseType::Major);
    }

    #[test]
    // For crates >= 1.0.0, Major is still Major
    fn major_bump_from_1x_to_2x() {
        let bumps = [ReleaseType::Major];

        let result = required_bump(&bumps, false);

        assert_eq!(result, ReleaseType::Major);
    }

    #[test]
    #[should_panic(
        expected = "Forever-unstable package can only receive a patch release but received Major"
    )]
    fn forever_unstable_downgrades_major_to_patch() {
        let bumps = [ReleaseType::Major];

        let result = required_bump(&bumps, true);
    }

    #[test]
    #[should_panic(
        expected = "Forever-unstable package can only receive a patch release but received Minor"
    )]
    fn forever_unstable_downgrades_minor_to_patch() {
        let bumps = [ReleaseType::Minor];

        let result = required_bump(&bumps, true);
    }

    #[test]
    fn minor_stays_minor() {
        let bumps = [ReleaseType::Minor];

        let result = required_bump(&bumps, false);

        assert_eq!(result, ReleaseType::Minor);
    }

    #[test]
    fn multiple_bumps_select_major() {
        let bumps = [ReleaseType::Patch, ReleaseType::Minor, ReleaseType::Major];

        let result = required_bump(&bumps, false);

        assert_eq!(result, ReleaseType::Major);
    }
}
