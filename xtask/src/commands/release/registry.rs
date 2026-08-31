//! Queries crates.io for version numbers that are already spoken for.

use std::{collections::HashMap, time::Duration};

#[cfg(feature = "release")]
use anyhow::Context;
use anyhow::{Result, bail};
#[cfg(feature = "release")]
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

/// Pull the version numbers out of one index entry.
#[cfg(feature = "release")]
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

#[cfg(feature = "release")]
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
            // Fail a stalled request instead of hanging the release.
            .timeout(Duration::from_secs(15))
            .build()
            .context("Failed to build an HTTP client for the crates.io index")?;

        let remote = RemoteSparseIndex::new(index, client);

        // A crate that was never published comes back as `Ok(None)`, so only
        // transport and parse failures are retried.
        crate::retry_on_failure("The crates.io index query", || {
            let results = remote.krates(
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
        })
    }
}

impl RegistrySnapshot {
    /// Whether crates.io has seen this exact number. Yanked releases stay in
    /// the index and keep their number reserved forever.
    fn is_taken(&self, package: Package, version: &semver::Version) -> bool {
        self.taken
            .get(&package)
            .is_some_and(|versions| versions.contains(version))
    }

    /// Move `planned` on by one step if crates.io already holds it. Two
    /// collisions in a row are left to a reviewer.
    pub fn next_free_version(
        &self,
        package: Package,
        planned: &semver::Version,
        bump: &VersionBump,
    ) -> Result<semver::Version> {
        if !self.is_taken(package, planned) {
            return Ok(planned.clone());
        }

        let step = match bump.pre {
            Some(ref pre) => VersionBump::pre(pre.clone()),
            None => VersionBump::patch(),
        };
        let next = do_version_bump(planned, &step)?;

        if self.is_taken(package, &next) {
            bail!(
                "crates.io already holds both {package} {planned} and {next}. Change the bump \
                 of {package} in the release plan to pick a version."
            );
        }

        log::warn!("crates.io already holds {package} {planned}, moving the release to {next}.");

        Ok(next)
    }
}

#[cfg(test)]
impl RegistrySnapshot {
    /// Build a snapshot without touching the network.
    pub(crate) fn from_taken(
        taken: impl IntoIterator<Item = (Package, Vec<semver::Version>)>,
    ) -> Self {
        Self {
            taken: taken.into_iter().collect(),
        }
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
    #[cfg(feature = "release")]
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

    #[track_caller]
    fn assert_no_free_version(planned: &str, bump: VersionBump, snapshot: &RegistrySnapshot) {
        let planned = planned.parse().unwrap();
        let error = snapshot
            .next_free_version(Package::EspSync, &planned, &bump)
            .expect_err("expected no free version");
        assert!(
            error.to_string().contains("already holds both"),
            "unexpected error: {error}"
        );
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
    fn consecutive_taken_versions_are_left_to_a_reviewer() {
        assert_no_free_version(
            "0.2.0",
            VersionBump::minor(),
            &snapshot(&["0.2.0", "0.2.1"]),
        );
        assert_no_free_version(
            "1.1.0-beta.3",
            VersionBump::pre("beta"),
            &snapshot(&["1.1.0-beta.3", "1.1.0-beta.4"]),
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

    #[cfg(feature = "release")]
    #[test]
    fn yanked_version_from_index_data_is_taken() {
        // The esp-sync index as it stood when esp-rs/esp-hal#5385 was filed,
        // plus a pre-semver entry of the kind the real index still carries.
        assert_free(
            "0.2.0",
            VersionBump::minor(),
            &index_snapshot(&[
                ("0.1", false),
                ("0.1.0", false),
                ("0.2.0", true),
                ("0.1.1", false),
            ]),
            "0.2.1",
        );
    }

    #[cfg(feature = "release")]
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
    }

    /// The only coverage of [`RegistrySnapshot::fetch`], so it queries the real
    /// index. Both assertions are stable: crates.io never frees a published
    /// number, yanked or not.
    #[cfg(feature = "release")]
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
