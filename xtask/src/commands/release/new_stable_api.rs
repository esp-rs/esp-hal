use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Error};
use public_api::{PublicApi, PublicItem, diff::PublicApiDiff};
use rustdoc_types::{Crate, GenericArgs, Id, Impl, ItemEnum, Type};

use crate::{
    Package,
    metadata::Chip,
    semver_check::{baseline_gz, build_prepared_doc_json, decompress_gz},
};

/// Visible in `new_stable_api` when a chip's comparison did not happen.
const COMPARISON_FAILED: &str = "<comparison failed — see log>";

struct Reference {
    /// Which side the comparison ran against, for the log line.
    kind: &'static str,
    path: PathBuf,
}

/// Public items stable in `current_docs` but not at the last release.
///
/// `base_tag` overrides the release compared against, which is otherwise the
/// tag matching the package's in-tree version.
pub(crate) fn newly_stable(
    workspace: &Path,
    package: Package,
    current_docs: &[(Chip, PathBuf)],
    base_tag: Option<&str>,
) -> anyhow::Result<Vec<String>> {
    // Tag sources are extracted outside the repo so cargo does not treat the tree
    // as part of the outer workspace. One directory per run, so that concurrent
    // `plan` invocations do not delete each other's checkouts.
    let scratch = tempfile::Builder::new()
        .prefix("esp-hal-semver-release-")
        .tempdir()
        .context("Failed to create a scratch directory for the release tag sources")?;

    let mut newly_stable: BTreeMap<String, BTreeSet<Chip>> = BTreeMap::new();
    let mut with_reference = 0usize;

    for (chip, current_path) in current_docs {
        let Some(reference) = reference_doc(workspace, scratch.path(), package, *chip, base_tag)?
        else {
            log::info!("No tag or baseline rustdoc for {package} {chip} - skipping new stable API");
            continue;
        };
        with_reference += 1;

        match new_stable_items(&reference, current_path, *chip) {
            Ok(items) => {
                for item in items {
                    newly_stable.entry(item).or_default().insert(*chip);
                }
            }
            // An empty list and a comparison that never happened look the same in
            // the plan file, so the failure has to show up in the list itself.
            Err(error) => {
                log::error!(
                    "Could not compare {package} {chip} against the last release: {error:#}"
                );
                newly_stable
                    .entry(COMPARISON_FAILED.to_string())
                    .or_default()
                    .insert(*chip);
            }
        }
    }

    if with_reference == 0 && !current_docs.is_empty() {
        log::error!(
            "Could not compare {package} against the last release on any chip - the empty \
             new_stable_api list would be a false all-clear"
        );
        for (chip, _) in current_docs {
            newly_stable
                .entry(COMPARISON_FAILED.to_string())
                .or_default()
                .insert(*chip);
        }
    }

    Ok(format_entries(newly_stable))
}

/// `item [chip, chip]` per entry, chips in `Chip` declaration order.
fn format_entries(newly_stable: BTreeMap<String, BTreeSet<Chip>>) -> Vec<String> {
    newly_stable
        .into_iter()
        .map(|(item, chips)| {
            let chips = chips
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ");
            format!("{item} [{chips}]")
        })
        .collect()
}

/// Return the stable public API items present in `current` but not in `reference`.
fn new_stable_items(
    reference: &Reference,
    current: &Path,
    chip: Chip,
) -> Result<BTreeSet<String>, Error> {
    let current_crate = rustdoc_crate(current)?;
    let reference_api = public_api_from_rustdoc(&reference.path)?;
    let current_api = public_api_from_rustdoc(current)?;

    let reference_count = reference_api.items().count();
    let current_count = current_api.items().count();
    anyhow::ensure!(
        reference_count > 0,
        "Could not extract any API item from the {} at {}",
        reference.kind,
        reference.path.display()
    );
    anyhow::ensure!(
        current_count > 0,
        "Could not extract any API item from the current rustdoc at {}",
        current.display()
    );

    let diff = PublicApiDiff::between(reference_api, current_api);
    let added: Vec<AddedItem> = diff.added.into_iter().map(AddedItem::from).collect();
    let added_count = added.len();
    let reported = roots_only(&current_crate, added);
    log::info!(
        "{chip}: compared against the {}, {added_count} added, {} reported",
        reference.kind,
        reported.len()
    );
    Ok(reported)
}

fn rustdoc_crate(path: &Path) -> Result<Crate, Error> {
    let json = fs::read_to_string(path)
        .with_context(|| format!("Failed to read rustdoc JSON from {}", path.display()))?;
    let mut deserializer = serde_json::Deserializer::from_str(&json);
    deserializer.disable_recursion_limit();
    let krate: Crate = serde::Deserialize::deserialize(&mut deserializer)
        .with_context(|| format!("Failed to parse rustdoc JSON from {}", path.display()))?;
    log::debug!(
        "{} is format_version {}",
        path.display(),
        krate.format_version
    );
    Ok(krate)
}

fn public_api_from_rustdoc(path: &Path) -> Result<PublicApi, Error> {
    public_api::Builder::from_rustdoc_json(path)
        .omit_blanket_impls(true)
        .omit_auto_trait_impls(true)
        .omit_auto_derived_impls(true)
        .build()
        .with_context(|| format!("Failed to parse public API from {}", path.display()))
}

/// One added item, reduced to what the collapse needs. `public-api` keeps the
/// fields behind `PublicItem::id`/`parent_id` crate-private.
struct AddedItem {
    id: Id,
    parent_id: Option<Id>,
    rendered: String,
}

impl From<PublicItem> for AddedItem {
    fn from(item: PublicItem) -> Self {
        Self {
            id: item.id(),
            parent_id: item.parent_id(),
            rendered: item.to_string(),
        }
    }
}

/// Drop added items whose owner (or whose type) was added in the same diff.
fn roots_only(krate: &Crate, added: Vec<AddedItem>) -> BTreeSet<String> {
    let added_ids: HashSet<Id> = added.iter().map(|item| item.id).collect();
    added
        .into_iter()
        .filter(|item| !is_covered(krate, item.id, item.parent_id, &added_ids))
        .map(|item| item.rendered)
        .collect()
}

fn is_covered(krate: &Crate, id: Id, parent_id: Option<Id>, added_ids: &HashSet<Id>) -> bool {
    if parent_id.is_some_and(|parent| added_ids.contains(&parent)) {
        return true;
    }
    match krate.index.get(&id).map(|rustdoc_item| &rustdoc_item.inner) {
        Some(ItemEnum::Impl(impl_)) => impl_uses_added(impl_, added_ids),
        Some(ItemEnum::StructField(ty)) => {
            resolved_id(ty).is_some_and(|id| added_ids.contains(&id))
        }
        _ => false,
    }
}

fn impl_uses_added(impl_: &Impl, added_ids: &HashSet<Id>) -> bool {
    if resolved_id(&impl_.for_).is_some_and(|id| added_ids.contains(&id)) {
        return true;
    }
    let Some(trait_) = &impl_.trait_ else {
        return false;
    };
    if added_ids.contains(&trait_.id) {
        return true;
    }
    match trait_.args.as_deref() {
        Some(GenericArgs::AngleBracketed { args, .. }) => args.iter().any(|arg| {
            matches!(
                arg,
                rustdoc_types::GenericArg::Type(ty)
                    if resolved_id(ty).is_some_and(|id| added_ids.contains(&id))
            )
        }),
        _ => false,
    }
}

fn resolved_id(ty: &Type) -> Option<Id> {
    match ty {
        Type::ResolvedPath(path) => Some(path.id),
        _ => None,
    }
}

/// Last-release rustdoc JSON for `package`/`chip`.
///
/// Prefers a tag rustdoc (cached under `target/semver-release-doc`), then the
/// semver baseline.
fn reference_doc(
    workspace: &Path,
    scratch: &Path,
    package: Package,
    chip: Chip,
    base_tag: Option<&str>,
) -> Result<Option<Reference>, Error> {
    if let Some(path) = release_tag_doc(workspace, scratch, package, chip, base_tag) {
        return Ok(Some(Reference {
            kind: "release tag",
            path,
        }));
    }

    // Falling back is right for the derived tag and wrong for a requested one,
    // which would silently change what the list is measured against.
    if let Some(tag) = base_tag {
        anyhow::bail!("Could not build the {tag} API document for {chip}; see the log above");
    }

    let Some(baseline_path_gz) = baseline_gz(workspace, package, chip)? else {
        return Ok(None);
    };

    let dest = workspace
        .join("target/semver-release-doc/baseline")
        .join(package.to_string())
        .join(format!("{chip}.json"));
    decompress_gz(&baseline_path_gz, &dest)?;
    Ok(Some(Reference {
        kind: "semver baseline",
        path: dest,
    }))
}

/// Rustdoc JSON of `package` as of the last release tag, or `None` to fall back
/// to the semver baseline. The baseline is regenerated from `main` on a breaking
/// change, so it absorbs mid-cycle accidents; the tag does not move.
fn release_tag_doc(
    workspace: &Path,
    scratch: &Path,
    package: Package,
    chip: Chip,
    base_tag: Option<&str>,
) -> Option<PathBuf> {
    let tag = match base_tag {
        Some(tag) => tag.to_string(),
        None => package.tag(&crate::package_version(workspace, package).ok()?),
    };
    let doc_path = workspace
        .join("target/semver-release-doc")
        .join(&tag)
        .join(format!("{chip}.json"));

    if doc_path.exists() {
        log::info!("Reusing cached {tag} API document for {chip}");
        return Some(doc_path);
    }

    match build_release_tag_doc(workspace, scratch, package, chip, &tag, &doc_path) {
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

fn run(command: &mut std::process::Command) -> Result<(), Error> {
    let status = command
        .status()
        .with_context(|| format!("Failed to run {command:?}"))?;
    anyhow::ensure!(status.success(), "{command:?} failed with {status}");
    Ok(())
}

fn build_release_tag_doc(
    workspace: &Path,
    scratch: &Path,
    package: Package,
    chip: Chip,
    tag: &str,
    doc_path: &Path,
) -> Result<(), Error> {
    // The whole tree is extracted, not just the package, so that the tag's root
    // `Cargo.toml` and `.cargo/config.toml` come along.
    let source_path = scratch.join("src").join(tag);
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
            let upstream = crate::git::get_remote_name_for(crate::UPSTREAM_REPO)?;
            log::info!("Tag {tag} may not be present locally, fetching it from {upstream}");
            run(std::process::Command::new("git")
                .current_dir(workspace)
                .arg("fetch")
                .arg(upstream)
                .arg(format!("refs/tags/{tag}:refs/tags/{tag}"))
                .args(["--no-tags", "--quiet"]))?;
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

    let target_path = scratch.join("target").join(tag);
    let rom_symbols = source_path.join("esp-rom-sys/src/generated_rom_symbols.rs");
    let built = build_prepared_doc_json(
        package,
        &chip,
        &package_path,
        Some(&target_path),
        &rom_symbols,
    )?;

    fs::create_dir_all(doc_path.parent().expect("doc path has a parent"))?;
    fs::copy(built, doc_path)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use rustdoc_types::{GenericArg, Generics, Item, Path as RustdocPath, Target, Visibility};

    use super::*;

    const GPIO21: u32 = 1;
    const ANY_PIN: u32 = 2;
    const PIN_TRAIT: u32 = 3;
    const INPUT_PIN_TRAIT: u32 = 4;
    const FROM_TRAIT: u32 = 5;
    const PERIPHERALS: u32 = 6;
    const FIELD: u32 = 7;
    const UNRELATED: u32 = 99;

    const GPIO21_STRUCT: &str = "#[non_exhaustive] pub struct esp_hal::peripherals::GPIO21<'a>";
    const PIN_IMPL_RENDER: &str = "impl esp_hal::gpio::Pin for esp_hal::peripherals::GPIO21<'_>";
    const INPUT_PIN_IMPL_RENDER: &str =
        "impl esp_hal::gpio::InputPin for esp_hal::peripherals::GPIO21<'_>";
    const PERIPHERALS_FIELD: &str = "pub esp_hal::peripherals::Peripherals::GPIO21: \
                                     esp_hal::peripherals::GPIO21<'static>";
    const MAC_ADDRESS_ENUM: &str = "#[non_exhaustive] pub enum esp_hal::efuse::InterfaceMacAddress";
    const STATION_VARIANT: &str = "pub esp_hal::efuse::InterfaceMacAddress::Station";
    const ACCESS_POINT_VARIANT: &str = "pub esp_hal::efuse::InterfaceMacAddress::AccessPoint";
    const CTS_CONFIG_ENUM: &str = "pub enum esp_hal::uart::CtsConfig";

    fn id(n: u32) -> Id {
        Id(n)
    }

    fn added(ids: &[u32]) -> HashSet<Id> {
        ids.iter().copied().map(id).collect()
    }

    fn added_item(item_id: u32, parent: Option<u32>, rendered: &str) -> AddedItem {
        AddedItem {
            id: id(item_id),
            parent_id: parent.map(id),
            rendered: rendered.to_string(),
        }
    }

    fn resolved(item_id: u32) -> Type {
        Type::ResolvedPath(RustdocPath {
            path: "T".into(),
            id: id(item_id),
            args: None,
        })
    }

    /// `impl <trait_id>[<arg_id>] for <for_id>`.
    fn trait_impl(for_id: u32, trait_id: u32, arg_id: Option<u32>) -> Impl {
        Impl {
            is_unsafe: false,
            generics: Generics {
                params: vec![],
                where_predicates: vec![],
            },
            provided_trait_methods: vec![],
            trait_: Some(RustdocPath {
                path: "T".into(),
                id: id(trait_id),
                args: arg_id.map(|arg| {
                    Box::new(GenericArgs::AngleBracketed {
                        args: vec![GenericArg::Type(resolved(arg))],
                        constraints: vec![],
                    })
                }),
            }),
            for_: resolved(for_id),
            items: vec![],
            is_negative: false,
            is_synthetic: false,
            blanket_impl: None,
        }
    }

    fn item(item_id: u32, inner: ItemEnum) -> Item {
        Item {
            id: id(item_id),
            crate_id: 0,
            name: None,
            span: None,
            visibility: Visibility::Public,
            docs: None,
            links: HashMap::new(),
            attrs: vec![],
            deprecation: None,
            stability: None,
            const_stability: None,
            inner,
        }
    }

    fn impl_item(item_id: u32, for_id: u32, trait_id: u32) -> Item {
        item(item_id, ItemEnum::Impl(trait_impl(for_id, trait_id, None)))
    }

    fn field_item(item_id: u32, ty_id: u32) -> Item {
        item(item_id, ItemEnum::StructField(resolved(ty_id)))
    }

    /// A crate whose index holds only what [`is_covered`] reads.
    fn krate(items: Vec<Item>) -> Crate {
        Crate {
            root: id(0),
            crate_version: None,
            includes_private: false,
            index: items.into_iter().map(|item| (item.id, item)).collect(),
            paths: HashMap::new(),
            external_crates: HashMap::new(),
            target: Target {
                triple: "riscv32imac-unknown-none-elf".into(),
                target_features: vec![],
            },
            format_version: rustdoc_types::FORMAT_VERSION,
        }
    }

    #[test]
    fn impl_uses_added_matches_self_type_trait_and_trait_argument() {
        // A new trait would otherwise be listed once per implementor.
        let new_self = trait_impl(GPIO21, PIN_TRAIT, None);
        let new_trait = trait_impl(ANY_PIN, PIN_TRAIT, None);
        let new_arg = trait_impl(ANY_PIN, FROM_TRAIT, Some(GPIO21));

        assert!(impl_uses_added(&new_self, &added(&[GPIO21])));
        assert!(impl_uses_added(&new_trait, &added(&[PIN_TRAIT])));
        assert!(impl_uses_added(&new_arg, &added(&[GPIO21])));
        assert!(!impl_uses_added(&new_arg, &added(&[UNRELATED])));
    }

    #[test]
    fn peripherals_field_of_a_stable_type_is_kept() {
        let krate = krate(vec![field_item(FIELD, ANY_PIN)]);

        assert!(!is_covered(
            &krate,
            id(FIELD),
            Some(id(PERIPHERALS)),
            &added(&[GPIO21, FIELD])
        ));
    }

    #[test]
    fn new_peripheral_surface_collapses_to_the_struct() {
        const PIN_IMPL: u32 = 52;
        const INPUT_IMPL: u32 = 53;

        // `PIN_IMPL` hangs off the stable `Pin` trait, so it can only be matched
        // through `krate.index`; `INPUT_IMPL` short-circuits on `parent_id`.
        let krate = krate(vec![
            impl_item(PIN_IMPL, GPIO21, PIN_TRAIT),
            impl_item(INPUT_IMPL, GPIO21, INPUT_PIN_TRAIT),
            field_item(FIELD, GPIO21),
        ]);
        let added = vec![
            added_item(GPIO21, None, GPIO21_STRUCT),
            added_item(PIN_IMPL, Some(PIN_TRAIT), PIN_IMPL_RENDER),
            added_item(INPUT_IMPL, Some(GPIO21), INPUT_PIN_IMPL_RENDER),
            added_item(FIELD, Some(PERIPHERALS), PERIPHERALS_FIELD),
        ];

        assert_eq!(
            roots_only(&krate, added),
            BTreeSet::from([GPIO21_STRUCT.to_string()])
        );
    }

    #[test]
    fn enum_collapses_only_when_the_enum_itself_is_new() {
        const ENUM: u32 = 10;
        const STATION: u32 = 11;
        const ACCESS_POINT: u32 = 12;
        const STABLE_ENUM: u32 = 30;

        // The #6267 accident: one stabilized enum, not one entry per variant.
        let new_enum = vec![
            added_item(ENUM, None, MAC_ADDRESS_ENUM),
            added_item(STATION, Some(ENUM), STATION_VARIANT),
            added_item(ACCESS_POINT, Some(ENUM), ACCESS_POINT_VARIANT),
        ];
        assert_eq!(
            roots_only(&krate(vec![]), new_enum),
            BTreeSet::from([MAC_ADDRESS_ENUM.to_string()])
        );

        let new_variant = vec![added_item(STATION, Some(STABLE_ENUM), STATION_VARIANT)];
        assert_eq!(
            roots_only(&krate(vec![]), new_variant),
            BTreeSet::from([STATION_VARIANT.to_string()])
        );
    }

    #[test]
    fn format_entries_orders_chips_and_keeps_the_failure_marker() {
        let newly_stable = BTreeMap::from([
            (
                CTS_CONFIG_ENUM.to_string(),
                BTreeSet::from([Chip::Esp32s31, Chip::Esp32, Chip::Esp32c6]),
            ),
            (
                COMPARISON_FAILED.to_string(),
                BTreeSet::from([Chip::Esp32c2]),
            ),
        ]);

        assert_eq!(
            format_entries(newly_stable),
            vec![
                format!("{COMPARISON_FAILED} [esp32c2]"),
                format!("{CTS_CONFIG_ENUM} [esp32, esp32c6, esp32s31]"),
            ]
        );
    }
}
