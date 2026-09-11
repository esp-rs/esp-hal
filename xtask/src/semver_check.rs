use std::{
    collections::BTreeSet,
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

/// Return the minimum required bump for the next release, along with the
/// public API items that are stable now but were not at the last release.
/// Collected only when `collect_new_stable_api` is set.
///
/// Even if nothing changed the bump will be [ReleaseType::Patch].
pub fn minimum_update(
    workspace: &Path,
    package: Package,
    chip: Chip,
    collect_new_stable_api: bool,
) -> Result<(ReleaseType, BTreeSet<String>), Error> {
    log::info!("Package = {}, Chip = {}", package, chip);

    let package_name = package.to_string();
    let package_path = crate::windows_safe_path(&workspace.join(&package_name));

    package.prepare_semver_check(&package_path, &chip)?;

    let current_path = build_doc_json(package, &chip, &package_path, None)?;

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
    if package.chip_features_matter() && !baseline_path_gz.exists() {
        log::warn!(
            "No baseline found for package '{}', chip '{}' — skipping semver check for this chip",
            package,
            chip
        );
        return Ok((ReleaseType::Patch, BTreeSet::new()));
    }
    let baseline_path =
        temp_file::TempFile::new().with_context(|| format!("Failed to create a TempFile!"))?;
    let buffer = Vec::new();
    let mut decoder = flate2::write::GzDecoder::new(buffer);
    decoder.write_all(&(fs::read(&baseline_path_gz)?))?;
    fs::write(baseline_path.path(), decoder.finish()?)?;

    let newly_stable = if collect_new_stable_api {
        let release_path = release_tag_doc(workspace, package, chip);
        new_stable_items(
            release_path.as_deref().unwrap_or(baseline_path.path()),
            &current_path,
        )?
    } else {
        BTreeSet::new()
    };

    let mut semver_check = Check::new(Rustdoc::from_path(current_path));
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

    Ok((min_required_update, newly_stable))
}

/// Return the stable public API items present in `current` but not in `baseline`.
///
/// Both documents are built with `instability_disable_unstable_docs`, so an item
/// only present in `current` was either added or stabilized since `baseline` was
/// generated.
fn new_stable_items(baseline: &Path, current: &Path) -> Result<BTreeSet<String>, Error> {
    fn read(path: &Path) -> Result<serde_json::Value, Error> {
        let json = fs::read_to_string(path)
            .with_context(|| format!("Failed to read rustdoc JSON from {}", path.display()))?;
        serde_json::from_str(&json)
            .with_context(|| format!("Failed to parse rustdoc JSON from {}", path.display()))
    }

    let baseline_items = stable_api_items(&read(baseline)?);
    let current_items = stable_api_items(&read(current)?);

    if baseline_items.is_empty() && !current_items.is_empty() {
        log::warn!(
            "Could not extract any API item from the baseline at {} - skipping new stable API detection",
            baseline.display()
        );
        return Ok(BTreeSet::new());
    }

    Ok(roots_only(
        current_items.difference(&baseline_items).cloned().collect(),
    ))
}

/// Drop entries whose parent is also in `items`.
///
/// Relies on the two shapes [`stable_api_items`] emits: `impl Trait for Owner`
/// (parent is the owner) and `kind some::path` (parent is the path without its
/// last segment).
fn roots_only(items: BTreeSet<String>) -> BTreeSet<String> {
    fn parent_of(item: &str) -> Option<&str> {
        if let Some((_, owner)) = item.split_once(" for ") {
            return Some(owner);
        }
        let (_kind, path) = item.split_once(' ')?;
        let (parent, _name) = path.rsplit_once("::")?;
        Some(parent)
    }

    let parents: BTreeSet<&str> = items
        .iter()
        .filter(|item| !item.contains(" for "))
        .filter_map(|item| item.split_once(' ').map(|(_, path)| path))
        .collect();
    items
        .iter()
        .filter(|item| match parent_of(item) {
            Some(parent) => !parents.contains(parent),
            None => true,
        })
        .cloned()
        .collect()
}

/// Rustdoc JSON of `package` as of the last release tag, or `None` to fall back
/// to the semver baseline.
///
/// The baseline is regenerated from `main` on a breaking change, so it absorbs
/// mid-cycle accidents. The tag does not move.
fn release_tag_doc(workspace: &Path, package: Package, chip: Chip) -> Option<PathBuf> {
    let version = crate::package_version(workspace, package).ok()?;
    let tag = package.tag(&version);

    let doc_path = workspace
        .join("target/semver-release-doc")
        .join(&tag)
        .join(format!("{chip}.json"));
    if doc_path.exists() {
        log::info!("Reusing cached {tag} API document for {chip}");
        return Some(doc_path);
    }

    match build_release_tag_doc(workspace, package, chip, &tag, &doc_path) {
        Ok(()) => Some(doc_path),
        Err(error) => {
            log::warn!(
                "Could not build the {tag} API document for {chip} ({error:#}) - comparing new \
                 stable API against the semver baseline instead, which may hide items stabilized \
                 before the baseline was last regenerated"
            );
            None
        }
    }
}

/// Remove extracted tag sources and build artifacts; keep `target/semver-release-doc`.
pub(crate) fn discard_release_tag_scratch(workspace: &Path) {
    for scratch in ["target/semver-release-src", "target/semver-release-target"] {
        fs::remove_dir_all(workspace.join(scratch)).ok();
    }
}

fn build_release_tag_doc(
    workspace: &Path,
    package: Package,
    chip: Chip,
    tag: &str,
    doc_path: &Path,
) -> Result<(), Error> {
    fn run(command: &mut std::process::Command) -> Result<(), Error> {
        let status = command
            .status()
            .with_context(|| format!("Failed to run {command:?}"))?;
        anyhow::ensure!(status.success(), "{command:?} failed with {status}");
        Ok(())
    }

    // `git archive` rather than `git worktree add`: no metadata to unregister.
    let source_path = workspace.join("target/semver-release-src").join(tag);
    let package_path = crate::windows_safe_path(&source_path.join(package.to_string()));

    if !package_path.exists() {
        fs::remove_dir_all(&source_path).ok();
        fs::create_dir_all(&source_path)?;

        let archive = source_path.join("source.tar");
        let archive_tag = || {
            run(std::process::Command::new("git")
                .current_dir(workspace)
                .args(["archive", "--format=tar", "--output"])
                .arg(&archive)
                .arg(tag))
        };

        if archive_tag().is_err() {
            // `plan` runs locally; `origin` is usually a fork and has no tags.
            let upstream = crate::git::get_remote_name_for(crate::UPSTREAM_REPO)?;
            log::info!("Tag {tag} may not be present locally, fetching tags from {upstream}");
            run(std::process::Command::new("git")
                .current_dir(workspace)
                .arg("fetch")
                .arg(upstream)
                .args(["--tags", "--quiet"]))?;
            archive_tag()?;
        }

        run(std::process::Command::new("tar")
            .arg("-xf")
            .arg(&archive)
            .arg("-C")
            .arg(&source_path))?;

        fs::remove_file(&archive).ok();
    }

    anyhow::ensure!(
        package_path.exists(),
        "{package} does not exist at {tag}, so there is nothing to compare against"
    );

    // Extracted tree sits under `target`, inside the outer workspace. The tag's
    // root excludes this package, so cargo walks up and refuses to build it.
    // An empty `[workspace]` table makes the package its own root.
    let manifest_path = package_path.join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .with_context(|| format!("Failed to read {}", manifest_path.display()))?;
    if !manifest.contains("\n[workspace]") {
        fs::write(&manifest_path, format!("{manifest}\n[workspace]\n"))?;
    }

    let target_path = workspace.join("target/semver-release-target").join(tag);

    package.prepare_semver_check(&package_path, &chip)?;
    let built = build_doc_json(package, &chip, &package_path, Some(&target_path));
    package.clean_semver_check(&source_path.join("esp-rom-sys/src/generated_rom_symbols.rs"))?;

    fs::create_dir_all(doc_path.parent().expect("doc path has a parent"))?;
    fs::copy(built?, doc_path)?;

    Ok(())
}

/// Collect the stable public API from rustdoc JSON.
///
/// Untyped JSON so a baseline and a current document from different toolchains
/// can still be compared.
fn stable_api_items(doc: &serde_json::Value) -> BTreeSet<String> {
    /// Ids are numbers in current rustdoc JSON and strings in older versions,
    /// while the `index` and `paths` maps are always keyed by strings.
    fn id_key(id: &serde_json::Value) -> Option<String> {
        match id {
            serde_json::Value::String(s) => Some(s.clone()),
            serde_json::Value::Number(n) => Some(n.to_string()),
            _ => None,
        }
    }

    let (Some(paths), Some(index)) = (doc["paths"].as_object(), doc["index"].as_object()) else {
        return BTreeSet::new();
    };

    // A `paths` entry is not public: rustdoc keeps entries for items whose
    // module was stripped as unstable.
    let mut reachable = BTreeSet::new();
    let mut stack = id_key(&doc["root"]).into_iter().collect::<Vec<_>>();
    while let Some(key) = stack.pop() {
        if !reachable.insert(key.clone()) {
            continue;
        }
        let Some(inner) = index.get(&key).map(|item| &item["inner"]) else {
            continue;
        };
        for list in [
            &inner["module"]["items"],
            &inner["enum"]["variants"],
            &inner["trait"]["items"],
        ] {
            if let Some(items) = list.as_array() {
                stack.extend(items.iter().filter_map(id_key));
            }
        }
        stack.extend(id_key(&inner["use"]["id"]));
    }

    let is_local = |item: &serde_json::Value| item["crate_id"].as_u64() == Some(0);
    let path_of = |key: &str| {
        Some(
            paths.get(key)?["path"]
                .as_array()?
                .iter()
                .map(|s| s.as_str().unwrap_or_default())
                .collect::<Vec<_>>()
                .join("::"),
        )
    };
    let local_path = |key: &str| {
        if !is_local(paths.get(key)?) || !reachable.contains(key) {
            return None;
        }
        path_of(key)
    };

    let member_entry = |owner: &str, member: &serde_json::Value| {
        let member = index.get(&id_key(member)?)?;
        let name = member["name"].as_str()?;
        // `inner` is an externally tagged enum, so its single key is the name of
        // the variant and therefore of the item kind.
        let kind = member["inner"]
            .as_object()
            .and_then(|inner| inner.keys().next())
            .map(String::as_str)
            .unwrap_or("item");
        Some(format!("{kind} {owner}::{name}"))
    };

    let trait_impl = |owner: &str, trait_ref: &serde_json::Value| {
        // Prefer the trait's full path so two same-named traits from different
        // crates do not collapse into one entry.
        let path = id_key(&trait_ref["id"])
            .and_then(|key| path_of(&key))
            .or_else(|| trait_ref["path"].as_str().map(str::to_string))?;

        // The generic arguments are part of an impl's identity: without them
        // every `From<GpioN> for AnyPin` collapses into one entry and a newly
        // stabilized conversion goes unreported.
        let args = trait_ref["args"]["angle_bracketed"]["args"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|arg| {
                let ty = &arg["type"];
                ty["resolved_path"]["path"]
                    .as_str()
                    .or_else(|| ty["primitive"].as_str())
                    .or_else(|| arg["lifetime"].as_str())
                    .unwrap_or("_")
            })
            .collect::<Vec<_>>();

        Some(match args.is_empty() {
            true => format!("impl {path} for {owner}"),
            false => format!("impl {path}<{}> for {owner}", args.join(", ")),
        })
    };

    let mut items = BTreeSet::new();
    for (key, entry) in paths {
        if !is_local(entry) || !reachable.contains(key) {
            continue;
        }
        let Some(kind) = entry["kind"].as_str() else {
            continue;
        };
        let Some(path) = path_of(key) else {
            continue;
        };
        items.insert(format!("{kind} {path}"));
    }

    for (key, item) in index {
        if !is_local(item) {
            continue;
        }
        let inner = &item["inner"];

        let (owner, members) = if let Some(imp) = inner.get("impl") {
            // Blanket and auto-trait impls are the ~1300 entries per chip that say
            // nothing about what this crate itself exposes.
            if !imp["blanket_impl"].is_null() || imp["is_synthetic"].as_bool().unwrap_or(false) {
                continue;
            }
            let Some(owner) =
                id_key(&imp["for"]["resolved_path"]["id"]).and_then(|key| local_path(&key))
            else {
                continue;
            };
            if !imp["trait"].is_null() {
                items.extend(trait_impl(&owner, &imp["trait"]));
                continue;
            }
            (owner, &imp["items"])
        } else if let Some(members) = inner
            .get("trait")
            .map(|t| &t["items"])
            .or_else(|| inner.get("struct").map(|s| &s["kind"]["plain"]["fields"]))
        {
            let Some(owner) = local_path(key) else {
                continue;
            };
            (owner, members)
        } else {
            continue;
        };

        for member in members.as_array().into_iter().flatten() {
            items.extend(member_entry(&owner, member));
        }
    }

    items
}

/// Build the rustdoc JSON of `package` for `chip`.
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
    if target_dir.is_some() {
        // Otherwise an ambient `CARGO_TARGET_DIR` would win and cargo would write
        // somewhere other than where we are about to look for the JSON.
        cargo_builder.add_env_var("CARGO_TARGET_DIR", &target_path.display().to_string());
    }

    let command = CargoCommandBatcher::build_one_for_cargo(&cargo_builder);
    log::debug!("{command:#?}");
    let cargo_command = command.command.clone();
    crate::cargo::run_with_env(&command.command, package_path, command.env_vars, false)
        .with_context(|| format!("Failed to run `cargo rustdoc` with {cargo_command:?}",))?;
    Ok(current_path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roots(items: &[&str]) -> Vec<String> {
        roots_only(items.iter().map(|s| s.to_string()).collect())
            .into_iter()
            .collect()
    }

    #[test]
    fn roots_only_drops_entries_covered_by_a_new_owner() {
        const ENUM: &str = "enum esp_hal::interrupt::riscv::DirectBindableCpuInterrupt";
        const VARIANT: &str =
            "variant esp_hal::interrupt::riscv::DirectBindableCpuInterrupt::Interrupt1";
        const ANY_I2C: &str = "struct esp_hal::i2c::master::low_level::AnyI2c";
        const FROM_I2C1: &str =
            "impl core::convert::From<I2C1> for esp_hal::i2c::master::low_level::AnyI2c";

        // The #6267 accident: one stabilized enum, not one entry per variant.
        assert_eq!(roots(&[ENUM, VARIANT]), [ENUM]);
        assert_eq!(roots(&[ANY_I2C, FROM_I2C1]), [ANY_I2C]);
        assert_eq!(
            roots(&["module esp_hal::foo", "struct esp_hal::foo::Bar"]),
            ["module esp_hal::foo"]
        );
    }

    #[test]
    fn roots_only_keeps_members_of_an_already_stable_owner() {
        for item in [
            "variant esp_hal::soc::implementation::clocks::CpuClock::_96MHz",
            "function esp_hal::uart::Uart::read_ready",
            "impl core::convert::From<I2C1> for esp_hal::i2c::master::low_level::AnyI2c",
        ] {
            assert_eq!(roots(&[item]), [item]);
        }
    }
}
