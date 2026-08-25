//! Queries crates.io for version numbers that are already spoken for.

use std::collections::HashMap;

use anyhow::{Context, Result, bail};
use tame_index::{
    IndexKrate,
    IndexLocation,
    IndexUrl,
    SparseIndex,
    external::reqwest::blocking::ClientBuilder,
    index::{FileLock, RemoteSparseIndex},
};

use crate::{
    Package,
    commands::{VersionBump, do_version_bump},
};

/// Upper bound on how many times [`RegistrySnapshot::next_free_version`] will
/// step forward before giving up.
const MAX_STEPS: usize = 64;

/// Whether crates.io will accept a given version number for a crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    /// Never used. The version is available.
    Free,
    /// Published and then withdrawn. crates.io keeps the number reserved
    /// forever, and hides it from every cargo read path.
    Yanked,
    /// A resolvable release occupies the number.
    Live,
}

/// Pull the version/yanked pairs out of one index entry.
fn versions_of(krate: &IndexKrate) -> Vec<(semver::Version, bool)> {
    krate
        .versions
        .iter()
        .filter_map(|v| match v.version.parse::<semver::Version>() {
            Ok(parsed) => Some((parsed, v.is_yanked())),
            Err(e) => {
                // Pre-semver crates exist in the index. They cannot collide
                // with anything we generate, so drop them rather than fail.
                log::debug!("Ignoring unparseable index version {:?}: {e}", v.version);
                None
            }
        })
        .collect()
}

/// What crates.io already holds for a set of packages, fetched in one go.
#[derive(Debug, Default)]
pub struct RegistrySnapshot {
    /// Per package: every version the index knows, and whether it is yanked.
    taken: HashMap<Package, Vec<(semver::Version, bool)>>,
}

impl RegistrySnapshot {
    /// A snapshot that knows nothing, so every version looks free. Used when
    /// the caller opts out of the registry lookup.
    pub fn skipped() -> Self {
        Self::default()
    }

    /// Look up every package in one batch.
    pub fn fetch(packages: impl IntoIterator<Item = Package>) -> Result<Self> {
        let by_name = packages
            .into_iter()
            .map(|p| (p.to_string(), p))
            .collect::<HashMap<_, _>>();

        if by_name.is_empty() {
            return Ok(Self::default());
        }

        let index = SparseIndex::new(IndexLocation::new(IndexUrl::CratesIoSparse))
            .context("Failed to open the crates.io sparse index")?;

        let client = ClientBuilder::new()
            .build()
            .context("Failed to build an HTTP client for the crates.io index")?;

        let results = RemoteSparseIndex::new(index, client).krates(
            by_name.keys().cloned().collect(),
            false,
            &FileLock::unlocked(),
        );

        let mut taken = HashMap::with_capacity(by_name.len());
        for (name, result) in results {
            let krate = result
                .with_context(|| format!("Failed to query the crates.io index for {name}"))?;

            match krate {
                Some(krate) => {
                    taken.insert(by_name[&name], versions_of(&krate));
                }
                None => log::debug!("{name} has never been published to crates.io"),
            }
        }

        Ok(Self { taken })
    }

    /// What crates.io holds for this exact version number.
    pub fn slot(&self, package: Package, version: &semver::Version) -> Slot {
        let Some((_, yanked)) = self
            .taken
            .get(&package)
            .and_then(|versions| versions.iter().find(|(v, _)| v == version))
        else {
            return Slot::Free;
        };

        if *yanked { Slot::Yanked } else { Slot::Live }
    }

    /// Advance `planned` until it lands on a version crates.io has not seen.
    pub fn next_free_version(
        &self,
        package: Package,
        planned: &semver::Version,
        bump: &VersionBump,
    ) -> Result<semver::Version> {
        let step = match bump.pre {
            Some(ref pre) => VersionBump::pre(pre.clone()),
            None => VersionBump::patch(),
        };

        let mut version = planned.clone();

        for _ in 0..MAX_STEPS {
            match self.slot(package, &version) {
                Slot::Free => return Ok(version),
                Slot::Live => bail!(
                    "{package} {version} is already published on crates.io, but the workspace \
                     expected it to be free. The version in Cargo.toml is behind the registry; \
                     reconcile the two before releasing."
                ),
                Slot::Yanked => {
                    let next = do_version_bump(&version, &step)?;
                    log::warn!(
                        "{package} {version} was published and yanked. crates.io does not free \
                         yanked version numbers, so the release moves to {next}."
                    );
                    version = next;
                }
            }
        }

        bail!(
            "Could not find a free version for {package} within {MAX_STEPS} steps of {planned}. \
             Something is wrong with either the release plan or the crates.io index."
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Version;

    /// A snapshot holding the given versions for [`Package::EspSync`].
    fn snapshot(versions: &[(&str, bool)]) -> RegistrySnapshot {
        let taken = versions
            .iter()
            .map(|(v, yanked)| (v.parse().unwrap(), *yanked))
            .collect();

        RegistrySnapshot {
            taken: HashMap::from([(Package::EspSync, taken)]),
        }
    }

    /// One line of a crates.io index file: newline-delimited JSON, one object
    /// per published version. Only the fields the parser requires are filled in.
    fn index_line(version: &str, yanked: bool) -> String {
        let cksum = "0".repeat(64);
        format!(
            r#"{{"name":"esp-sync","vers":"{version}","deps":[],"cksum":"{cksum}","features":{{}},"yanked":{yanked}}}"#
        )
    }

    /// The esp-sync index as it stood when esp-rs/esp-hal#5385 was filed.
    fn esp_sync_index() -> RegistrySnapshot {
        let raw = [
            index_line("0.1.0", false),
            index_line("0.2.0", true),
            index_line("0.1.1", false),
        ]
        .join("\n");

        let krate = IndexKrate::from_slice(raw.as_bytes()).unwrap();

        RegistrySnapshot {
            taken: HashMap::from([(Package::EspSync, versions_of(&krate))]),
        }
    }

    #[track_caller]
    fn assert_free(planned: &str, bump: VersionBump, snapshot: &RegistrySnapshot, expected: &str) {
        let planned = planned.parse().unwrap();
        let free = snapshot
            .next_free_version(Package::EspSync, &planned, &bump)
            .expect("expected a free version");
        assert_eq!(free.to_string(), expected);
    }

    #[test]
    fn free_slot_is_left_alone() {
        assert_free(
            "0.2.1",
            VersionBump::minor(),
            &snapshot(&[("0.2.0", true)]),
            "0.2.1",
        );
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &RegistrySnapshot::default(),
            "0.2.0",
        );
    }

    #[test]
    fn a_skipped_snapshot_never_moves_the_version() {
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &RegistrySnapshot::skipped(),
            "0.2.0",
        );
    }

    #[test]
    fn yanked_slot_is_skipped() {
        // esp-rs/esp-hal#5385: esp-sync 0.1.1 + Minor lands on the yanked 0.2.0.
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &snapshot(&[("0.1.0", false), ("0.1.1", false), ("0.2.0", true)]),
            "0.2.1",
        );
    }

    #[test]
    fn consecutive_yanked_slots_are_skipped() {
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &snapshot(&[("0.2.0", true), ("0.2.1", true), ("0.2.2", true)]),
            "0.2.3",
        );
    }

    #[test]
    fn pre_release_steps_the_counter() {
        assert_free(
            "1.1.0-beta.3",
            VersionBump::pre("beta"),
            &snapshot(&[("1.1.0-beta.3", true)]),
            "1.1.0-beta.4",
        );
        // Starting a fresh cycle on a bumped base steps the counter too.
        assert_free(
            "1.1.0-alpha.0",
            VersionBump::base_and_pre(Version::Minor, "alpha"),
            &snapshot(&[("1.1.0-alpha.0", true)]),
            "1.1.0-alpha.1",
        );
    }

    #[test]
    fn other_packages_are_unaffected_by_a_reservation() {
        // Reservations are per crate; esp-hal must not inherit esp-sync's.
        let snapshot = snapshot(&[("0.2.0", true)]);
        let planned = "0.2.0".parse().unwrap();

        let free = snapshot
            .next_free_version(Package::EspHal, &planned, &VersionBump::minor())
            .unwrap();

        assert_eq!(free.to_string(), "0.2.0");
    }

    #[test]
    fn yanked_flag_is_read_from_index_data() {
        let snapshot = esp_sync_index();
        let slot = |v: &str| snapshot.slot(Package::EspSync, &v.parse().unwrap());

        assert_eq!(slot("0.1.0"), Slot::Live);
        assert_eq!(slot("0.1.1"), Slot::Live);
        assert_eq!(slot("0.2.0"), Slot::Yanked);
        assert_eq!(slot("0.2.1"), Slot::Free);
    }

    #[test]
    fn issue_5385_is_fixed_end_to_end() {
        // The whole failure in one test: a Minor bump computed from the
        // workspace alone lands on the reserved 0.2.0, so the release has to
        // move to 0.2.1.
        let snapshot = esp_sync_index();
        let bump = VersionBump::minor();

        let planned = do_version_bump(&"0.1.1".parse().unwrap(), &bump).unwrap();
        assert_eq!(planned.to_string(), "0.2.0", "workspace-only bump");

        let resolved = snapshot
            .next_free_version(Package::EspSync, &planned, &bump)
            .unwrap();
        assert_eq!(resolved.to_string(), "0.2.1", "after consulting crates.io");
    }

    /// Smoke test for the network path, which no other test covers. Assumes
    /// esp-sync 0.2.0 stays yanked, which is not something anyone will undo.
    ///
    /// `cargo test -p esp-devtool --features release -- --ignored`
    #[test]
    #[ignore = "requires network access to crates.io"]
    fn live_index_reports_the_yanked_esp_sync_release() {
        let snapshot = RegistrySnapshot::fetch([Package::EspSync]).unwrap();
        let slot = |v: &str| snapshot.slot(Package::EspSync, &v.parse().unwrap());

        assert_eq!(slot("0.2.0"), Slot::Yanked);
        assert_eq!(slot("0.1.1"), Slot::Live);
    }

    #[test]
    fn live_release_in_the_way_is_an_error() {
        let planned: semver::Version = "0.2.0".parse().unwrap();

        for occupied in [
            // Planned slot itself is live.
            snapshot(&[("0.2.0", false)]),
            // Reached by stepping over a yanked slot.
            snapshot(&[("0.2.0", true), ("0.2.1", false)]),
        ] {
            let result =
                occupied.next_free_version(Package::EspSync, &planned, &VersionBump::minor());
            assert!(result.is_err(), "expected an error for {occupied:?}");
        }
    }
}
