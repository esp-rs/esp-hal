use clap::Subcommand;

pub mod bump_version;
#[cfg(feature = "release")]
pub mod plan;
pub mod publish;
pub mod semver_check;
pub mod tag_releases;

pub use bump_version::*;
#[cfg(feature = "release")]
pub use plan::*;
pub use publish::*;
pub use semver_check::*;
pub use tag_releases::*;

// ----------------------------------------------------------------------------
// Subcommands

#[derive(Debug, Subcommand)]
pub enum Release {
    /// Create a release plan. This is the first step in the release process.
    /// Accepts zero or more package names. If no package names are
    /// specified, all packages are included.
    ///
    /// The result of this command is a json file that can be customized to
    /// control what and how gets released.
    #[cfg(feature = "release")]
    Plan(PlanArgs),
    /// Bump the version of the specified package(s).
    ///
    /// This command will, for each specified package:
    /// - Verify that the crate can be released (e.g. it doesn't refer to git
    ///   dependencies)
    /// - Update the version in `Cargo.toml` files
    /// - Update the version in dependencies' `Cargo.toml` files
    /// - Check if the changelog can be finalized
    /// - Update the version in the changelog
    /// - Replaces `{{currentVersion}}` markers in source files and the
    ///   migration guide.
    BumpVersion(BumpVersionArgs),
    /// Attempt to publish the specified package.
    Publish(PublishArgs),
    /// Generate git tags for all new package releases.
    TagReleases(TagReleasesArgs),
}
