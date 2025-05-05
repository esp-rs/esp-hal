use std::{
    fs, io::Write, path::{Path, PathBuf}
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

            remove_unstable_items(&current_path)?;

            let file_name = if package.chip_features_matter() {
                format!("{}", chip.to_string())
            } else {
                "api".to_string()
            };

            let to_path = PathBuf::from(&package_path).join(format!("api-baseline/{}.json.gz", file_name));
            fs::create_dir_all(to_path.parent().unwrap())?;

            log::debug!("Compress into {current_path:?}");
            let mut encoder = flate2::write::GzEncoder::new(fs::File::create(to_path)?, flate2::Compression::default());
            encoder.write_all(&std::fs::read(current_path)?)?;

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
            remove_unstable_items(&current_path)?;

            let file_name = if package.chip_features_matter() {
                format!("{}", chip.to_string())
            } else {
                "api".to_string()
            };

            let baseline_path_gz = PathBuf::from(&package_path).join(format!("api-baseline/{}.json.gz", file_name));

            let baseline_path = temp_file::TempFile::new()?;
            let buffer = Vec::new();
            let mut decoder = flate2::write::GzDecoder::new(buffer);
            decoder.write_all(&(fs::read(&baseline_path_gz)?))?;
            fs::write( baseline_path.path(), decoder.finish()?)?;

            let mut check = cargo_semver_checks::Check::new(
                cargo_semver_checks::Rustdoc::from_path(current_path),
            );
            check.set_baseline(cargo_semver_checks::Rustdoc::from_path(baseline_path.path()));
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
    //
    // Hopefully this whole pre-processing is just a stop-gap solution until it's possible to generate docs for the stable-API only.

    log::info!("{:?}", path);
    let json_string = std::fs::read_to_string(path)?;
    let mut krate: rustdoc_types::Crate = serde_json::from_str(&json_string)?;

    let mut to_remove = vec![];

    // first pass - just look for cfg-gated items
    //
    // the string to match depends on the rustfmt-json version!
    // later version emit `#[<cfg>(...` instead
    let cfg_gates = vec![
        "#[cfg(any(doc, feature = \"unstable\"))]",
        "#[cfg(feature = \"unstable\")]",
    ];

    for (id, item) in &mut krate.index {
        if item
            .attrs
            .iter()
            .any(|attr| cfg_gates.contains(&attr.as_str()))
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

            // don't honor (because they either don't contain sub-items (= already handled in the first pass) or we currently don't use them)
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

    // if we added something more to remove (because the items are "empty" now - remove them, too)
    for id in &to_remove {
        krate.index.remove(&id);
        krate.paths.remove(&id);
    }

    std::fs::write(path, serde_json::to_string(&krate)?)?;

    Ok(())
}
