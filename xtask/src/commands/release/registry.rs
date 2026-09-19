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

/// Pull the version numbers out of one index entry.
fn versions_of(krate: &IndexKrate) -> Vec<semver::Version> {
    krate
        .versions
        .iter()
        .filter_map(|v| match v.version.parse::<semver::Version>() {
            Ok(parsed) => Some(parsed),
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
    /// Per package: every version the index knows, yanked ones included.
    taken: HashMap<Package, Vec<semver::Version>>,
}

impl RegistrySnapshot {
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

    /// Whether crates.io has seen this exact number. Yanked releases stay in
    /// the index and keep their number reserved forever.
    fn is_taken(&self, package: Package, version: &semver::Version) -> bool {
        self.taken
            .get(&package)
            .is_some_and(|versions| versions.contains(version))
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
            if !self.is_taken(package, &version) {
                return Ok(version);
            }

            let next = do_version_bump(&version, &step)?;
            log::warn!(
                "crates.io already holds {package} {version}, moving the release to {next}."
            );
            version = next;
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
    fn snapshot(versions: &[&str]) -> RegistrySnapshot {
        let taken = versions.iter().map(|v| v.parse().unwrap()).collect();

        RegistrySnapshot {
            taken: HashMap::from([(Package::EspSync, taken)]),
        }
    }

    /// A snapshot parsed out of raw crates.io index data: newline-delimited
    /// JSON, one object per published version. Only the fields the parser
    /// requires are filled in. This is the only way to express the yanked flag,
    /// which the index carries but [`RegistrySnapshot`] deliberately drops.
    fn index_snapshot(versions: &[(&str, bool)]) -> RegistrySnapshot {
        let cksum = "0".repeat(64);
        let raw = versions
            .iter()
            .map(|(version, yanked)| {
                format!(
                    r#"{{"name":"esp-sync","vers":"{version}","deps":[],"cksum":"{cksum}","features":{{}},"yanked":{yanked}}}"#
                )
            })
            .collect::<Vec<_>>()
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
    fn free_version_is_left_alone() {
        assert_free(
            "0.2.1",
            VersionBump::minor(),
            &snapshot(&["0.2.0"]),
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
    fn taken_version_is_skipped() {
        // esp-rs/esp-hal#5385: esp-sync 0.1.1 + Minor lands on the yanked 0.2.0.
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &snapshot(&["0.1.0", "0.1.1", "0.2.0"]),
            "0.2.1",
        );
    }

    #[test]
    fn consecutive_taken_versions_are_skipped() {
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &snapshot(&["0.2.0", "0.2.1", "0.2.2"]),
            "0.2.3",
        );
    }

    #[test]
    fn pre_release_steps_the_counter() {
        assert_free(
            "1.1.0-beta.3",
            VersionBump::pre("beta"),
            &snapshot(&["1.1.0-beta.3"]),
            "1.1.0-beta.4",
        );
        // Starting a fresh cycle on a bumped base steps the counter too.
        assert_free(
            "1.1.0-alpha.0",
            VersionBump::base_and_pre(Version::Minor, "alpha"),
            &snapshot(&["1.1.0-alpha.0"]),
            "1.1.0-alpha.1",
        );
    }

    #[test]
    fn other_packages_are_unaffected_by_a_reservation() {
        // Reservations are per crate; esp-hal must not inherit esp-sync's.
        let snapshot = snapshot(&["0.2.0"]);
        let planned = "0.2.0".parse().unwrap();

        let free = snapshot
            .next_free_version(Package::EspHal, &planned, &VersionBump::minor())
            .unwrap();

        assert_eq!(free.to_string(), "0.2.0");
    }

    #[test]
    fn yanked_version_from_index_data_is_taken() {
        // The esp-sync index as it stood when esp-rs/esp-hal#5385 was filed.
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &index_snapshot(&[("0.1.0", false), ("0.2.0", true), ("0.1.1", false)]),
            "0.2.1",
        );
    }

    #[test]
    fn live_release_is_stepped_over() {
        // A number held by a resolvable release is as unavailable as a yanked
        // one. Stepping over it lets `plan` recover from a partially completed
        // release instead of forcing the plan to be unpicked by hand.
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &index_snapshot(&[("0.2.0", false)]),
            "0.2.1",
        );
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &index_snapshot(&[("0.2.0", true), ("0.2.1", false)]),
            "0.2.2",
        );
    }

    #[test]
    fn live_index_reports_the_yanked_esp_sync_release() {
        let snapshot = RegistrySnapshot::fetch([Package::EspSync]).unwrap();

        // cargo cannot resolve 0.2.0 at all, but the number stays reserved.
        assert!(snapshot.is_taken(Package::EspSync, &"0.2.0".parse().unwrap()));
        // A number nobody will ever publish, so the fetch is not reporting
        // everything as taken.
        assert!(!snapshot.is_taken(Package::EspSync, &"99.0.0".parse().unwrap()));
    }
}
