use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    path::Path,
};

use anyhow::Context;
use public_api::{PublicApi, PublicItem, diff::PublicApiDiff};
use serde::{Deserialize, Serialize};

use crate::{
    Package,
    metadata::Chip,
    semver_check::{baseline_gz, build_prepared_doc_json, decompress_gz},
};

/// One `new_stable_api` entry.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct NewStableItem {
    pub item: String,
    pub chips: BTreeSet<Chip>,
}

/// Public items stable in the working tree but not in the API baseline.
///
/// Returns the items and the chips that have no baseline to compare against.
pub(crate) fn newly_stable(
    workspace: &Path,
    package: Package,
    chips: &[Chip],
) -> anyhow::Result<(Vec<NewStableItem>, BTreeSet<Chip>)> {
    let package_path = crate::windows_safe_path(&workspace.join(package.to_string()));
    let mut newly_stable: BTreeMap<String, BTreeSet<Chip>> = BTreeMap::new();
    let mut unchecked_chips = BTreeSet::new();

    for chip in chips
        .iter()
        .copied()
        .filter(|chip| package.supports_chip(*chip))
    {
        let Some(baseline_gz_path) = baseline_gz(workspace, package, chip)? else {
            log::warn!("Not checking new stable API for {package} {chip}: no API baseline");
            unchecked_chips.insert(chip);
            continue;
        };
        let baseline = workspace
            .join("target/semver-baseline-doc")
            .join(package.to_string())
            .join(format!("{chip}.json"));
        decompress_gz(&baseline_gz_path, &baseline)?;
        let current = build_prepared_doc_json(workspace, package, &chip, &package_path)?;

        let diff = PublicApiDiff::between(
            public_api_from_rustdoc(&baseline)?,
            public_api_from_rustdoc(&current)?,
        );
        // A new enum would otherwise also list every variant.
        let added_ids: HashSet<_> = diff.added.iter().map(PublicItem::id).collect();
        for item in &diff.added {
            if !item
                .parent_id()
                .is_some_and(|parent| added_ids.contains(&parent))
            {
                newly_stable
                    .entry(item.to_string())
                    .or_default()
                    .insert(chip);
            }
        }

        if !package.chip_features_matter() {
            break;
        }
    }

    let items = newly_stable
        .into_iter()
        .map(|(item, chips)| NewStableItem { item, chips })
        .collect();

    Ok((items, unchecked_chips))
}

fn public_api_from_rustdoc(path: &Path) -> anyhow::Result<PublicApi> {
    public_api::Builder::from_rustdoc_json(path)
        .omit_blanket_impls(true)
        .omit_auto_trait_impls(true)
        .omit_auto_derived_impls(true)
        .build()
        .with_context(|| format!("Failed to parse public API from {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const CTS_CONFIG_ENUM: &str = "pub enum esp_hal::uart::CtsConfig";

    #[test]
    fn new_stable_item_serializes_item_and_chips_as_separate_keys() {
        let entry = NewStableItem {
            item: CTS_CONFIG_ENUM.to_string(),
            chips: BTreeSet::from([Chip::Esp32s31, Chip::Esp32, Chip::Esp32c6]),
        };

        assert_eq!(
            serde_json::to_value(&entry).unwrap(),
            serde_json::json!({
                "item": CTS_CONFIG_ENUM,
                "chips": ["esp32", "esp32c6", "esp32s31"],
            })
        );
    }
}
