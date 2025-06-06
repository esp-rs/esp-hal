use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};

use cargo_semver_checks::{Check, GlobalConfig, ReleaseType, Rustdoc};
use esp_metadata::Chip;
use rustdoc_types::ItemEnum;

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
    remove_unstable_items(&current_path)?;

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
        vec![chip.to_string(), "unstable".to_string()]
    } else {
        vec!["unstable".to_string()]
    };

    log::info!(
        "Building doc json for {} with features: {:?}",
        package,
        features
    );

    // always use `esp` toolchain so we don't have to deal with potentially
    // different versions of the doc-json
    let mut cargo_builder = CargoArgsBuilder::default()
        .toolchain("esp")
        .subcommand("rustdoc")
        .features(&features)
        .target(chip.target())
        .arg("-Zunstable-options")
        .arg("--lib")
        .arg("--output-format=json");
    cargo_builder = cargo_builder.arg("-Zbuild-std=alloc,core");
    let cargo_args = cargo_builder.build();
    log::debug!("{cargo_args:#?}");
    crate::cargo::run(&cargo_args, package_path)?;
    Ok(current_path)
}

pub(crate) fn remove_unstable_items(path: &Path) -> Result<(), anyhow::Error> {
    // this leaves orphaned items! cargo-semver-checks seems to be fine with that
    // however the json fmt is unstable - we might fail when using the "wrong"
    // version of `rustdoc_types` here
    //
    // Hopefully this whole pre-processing is just a stop-gap solution until it's
    // possible to generate docs for the stable-API only.

    log::info!("{:?}", path);
    let json_string = std::fs::read_to_string(path)?;
    let mut krate: rustdoc_types::Crate = serde_json::from_str(&json_string)?;

    let mut to_remove = vec![];

    // first pass - just look for cfg-gated items
    //
    // the string to match depends on the rustfmt-json version!
    // later version emit `#[<cfg>(...` instead of `#[cfg(..`
    let cfg_gates = vec![
        "#[<cfg>(any(doc, feature = \"unstable\"))]",
        "#[cfg(any(doc, feature = \"unstable\"))]",
        "#[<cfg>(feature = \"unstable\")]",
        "#[cfg(feature = \"unstable\")]",
    ];

    for (id, item) in &mut krate.index {
        if item
            .attrs
            .iter()
            .any(|attr| cfg_gates.contains(&attr.as_str()))
        {
            // remove the item itself
            to_remove.push(id.clone());

            // remove sub-items - shouldn't be needed but shouldn't hurt
            match &item.inner {
                ItemEnum::Module(module) => {
                    to_remove.extend(&module.items);
                }
                ItemEnum::Struct(s) => {
                    to_remove.extend(&s.impls);
                }
                ItemEnum::Enum(e) => {
                    to_remove.extend(&e.impls);
                    to_remove.extend(&e.variants);
                }
                ItemEnum::Impl(i) => {
                    to_remove.extend(&i.items);
                }
                _ => (),
            }
        }
    }

    log::debug!("Items to remove {:?}", to_remove);

    for id in &to_remove {
        krate.index.remove(&id);
        krate.paths.remove(&id);
    }

    for (_id, item) in &mut krate.index {
        match &mut item.inner {
            ItemEnum::Module(module) => {
                module.items.retain(|id| !to_remove.contains(id));
            }
            ItemEnum::Struct(struct_) => {
                struct_.impls.retain(|id| !to_remove.contains(id));

                match &mut struct_.kind {
                    rustdoc_types::StructKind::Unit => (),
                    rustdoc_types::StructKind::Tuple(_ids) => (),
                    rustdoc_types::StructKind::Plain {
                        fields,
                        has_stripped_fields: _,
                    } => {
                        for id in &to_remove {
                            if let Some(found) = fields.iter().enumerate().find(|i| i.1 == id) {
                                fields.remove(found.0);
                            }
                        }
                    }
                }

                if struct_.impls.is_empty() {
                    to_remove.push(_id.clone());
                }
            }
            ItemEnum::Enum(enum_) => {
                enum_.impls.retain(|id| !to_remove.contains(id));

                enum_.variants.retain(|id| !to_remove.contains(id));

                if enum_.impls.is_empty() {
                    to_remove.push(_id.clone());
                }
            }
            ItemEnum::Impl(impl_) => {
                impl_.items.retain(|id| !to_remove.contains(id));

                if impl_.items.is_empty() {
                    to_remove.push(_id.clone());
                }
            }

            // don't honor (because they either don't contain sub-items (= already handled in the
            // first pass) or we currently don't use them)
            //
            // ItemEnum::Use(_)
            // ItemEnum::Union(union)
            // ItemEnum::StructField(_)
            // ItemEnum::Variant(variant)
            // ItemEnum::Function(function)
            // ItemEnum::Trait(_)
            // ItemEnum::TraitAlias(trait_alias)
            // ItemEnum::TypeAlias(type_alias)
            // ItemEnum::Constant {
            // ItemEnum::Static(_)
            // ItemEnum::ExternType =>
            // ItemEnum::Macro(_)
            // ItemEnum::ProcMacro(proc_macro)
            // ItemEnum::Primitive(primitive)
            // ItemEnum::AssocConst {
            // ItemEnum::AssocType {
            _ => (),
        }
    }

    // if we added something more to remove (because the items are "empty" now -
    // remove them, too)
    for id in &to_remove {
        krate.index.remove(&id);
        krate.paths.remove(&id);
    }

    std::fs::write(path, serde_json::to_string(&krate)?)?;

    Ok(())
}
