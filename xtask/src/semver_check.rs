use std::{
    fs,
    path::{Path, PathBuf},
};

use esp_metadata::Chip;
use rustdoc_types::ItemEnum;

use crate::{Package, cargo::CargoArgsBuilder};

pub fn generate_baseline(
    workspace: &PathBuf,
    packages: Vec<Package>,
    chips: Vec<Chip>,
) -> Result<(), anyhow::Error> {
    for package in packages {
        log::info!("Generating API baseline for {package}");

        for chip in &chips {
            log::info!("Chip = {}", chip.to_string());
            let package_name = package.to_string();
            let package_path = crate::windows_safe_path(&workspace.join(&package_name));

            let current_path = build_doc_json(package, chip, &package_path)?;

            let suffix = if package.chip_features_matter() {
                format!("_{}", chip.to_string())
            } else {
                "".to_string()
            };

            let to_path = PathBuf::from(&package_path).join(format!("api_baseline{}.json", suffix));

            remove_unstable_items(&current_path)?;

            log::debug!("Copy {current_path:?}");
            fs::copy(current_path, to_path)?;

            if !package.chip_features_matter() {
                break;
            }
        }
    }

    Ok(())
}

pub fn check(
    workspace: &PathBuf,
    packages: Vec<Package>,
    chips: Vec<Chip>,
) -> Result<(), anyhow::Error> {
    for package in packages {
        log::info!("Semver-check API for {package}");

        for chip in &chips {
            log::info!("Chip = {}", chip.to_string());
            let package_name = package.to_string();
            let package_path = crate::windows_safe_path(&workspace.join(&package_name));

            let current_path = build_doc_json(package, chip, &package_path)?;

            let suffix = if package.chip_features_matter() {
                format!("_{}", chip.to_string())
            } else {
                "".to_string()
            };

            let baseline_path = package_path.join(format!("api_baseline{}.json", suffix));

            remove_unstable_items(&current_path)?;

            let mut check = cargo_semver_checks::Check::new(
                cargo_semver_checks::Rustdoc::from_path(current_path),
            );
            check.set_baseline(cargo_semver_checks::Rustdoc::from_path(baseline_path));
            let mut cfg = cargo_semver_checks::GlobalConfig::new();
            // seems there is no way of getting the details via the API - so lets log them
            cfg.set_log_level(Some(log::Level::Info));
            let check_result = check.check_release(&mut cfg)?;

            log::info!("Result {:?}", check_result);

            for (_, report) in check_result.crate_reports() {
                if let Some(required_bump) = report.required_bump() {
                    if required_bump == cargo_semver_checks::ReleaseType::Major {
                        return Err(anyhow::anyhow!(
                            "Semver check failed: {:?}",
                            report.required_bump()
                        ));
                    }
                }
            }

            if !package.chip_features_matter() {
                break;
            }
        }
    }

    Ok(())
}

fn build_doc_json(
    package: Package,
    chip: &Chip,
    package_path: &PathBuf,
) -> Result<PathBuf, anyhow::Error> {
    let current_path = PathBuf::from(package_path)
        .join("target")
        .join(chip.target())
        .join("doc")
        .join(format!("{}.json", package.to_string().replace("-", "_")));
    std::fs::remove_file(&current_path).ok();
    let toolchain = "esp";
    let features = if package.chip_features_matter() {
        vec![chip.to_string(), "unstable".to_string()]
    } else {
        vec!["unstable".to_string()]
    };
    let mut cargo_builder = CargoArgsBuilder::default()
        .toolchain(toolchain)
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

fn remove_unstable_items(path: &Path) -> Result<(), anyhow::Error> {
    // this leaves orphaned items! cargo-semver-checks seems to be fine with that however
    // the json fmt is unstable - we might fail when using the "wrong" version of `rustdoc_types` here

    log::info!("{:?}", path);
    let json_string = std::fs::read_to_string(path)?;
    let mut krate: rustdoc_types::Crate = serde_json::from_str(&json_string)?;

    let mut to_remove = vec![];

    // first pass - just look for cfg-gated items
    for (id, item) in &mut krate.index {
        // the string to match depends on the rustfmt-json version!
        // later version emit `#[<cfg>(...` instead
        if item
            .attrs
            .contains(&"#[cfg(any(doc, feature = \"unstable\"))]".to_string())
            || item
                .attrs
                .contains(&"#[cfg(feature = \"unstable\")]".to_string())
        {
            to_remove.push(id.clone());
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
                module.items = module
                    .items
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();
            }
            ItemEnum::Struct(struct_) => {
                struct_.impls = struct_
                    .impls
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();

                struct_.impls = struct_
                    .impls
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();

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
                enum_.impls = enum_
                    .impls
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();

                enum_.variants = enum_
                    .variants
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();

                if enum_.impls.is_empty() {
                    to_remove.push(_id.clone());
                }
            }
            ItemEnum::Impl(impl_) => {
                impl_.items = impl_
                    .items
                    .iter()
                    .filter(|id| !to_remove.contains(id))
                    .map(|id| id.clone())
                    .collect();

                if impl_.items.is_empty() {
                    to_remove.push(_id.clone());
                }
            }

            // don't honor:
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

    // if we added something more to remove (because the items are "empty" now - remove them, too)
    for id in &to_remove {
        krate.index.remove(&id);
        krate.paths.remove(&id);
    }

    std::fs::write(path, serde_json::to_string(&krate)?)?;

    Ok(())
}
