use clap::Subcommand;

pub mod bump_version;
#[cfg(feature = "release")]
pub mod execute_plan;
#[cfg(feature = "release")]
pub mod plan;
#[cfg(feature = "release")]
pub mod post_release;
pub mod publish;
#[cfg(feature = "release")]
pub mod publish_plan;
pub mod semver_check;
pub mod tag_releases;

pub use bump_version::*;
#[cfg(feature = "release")]
pub use execute_plan::*;
#[cfg(feature = "release")]
pub use plan::*;
pub use publish::*;
#[cfg(feature = "release")]
pub use publish_plan::*;
pub use semver_check::*;
pub use tag_releases::*;
#[cfg(feature = "release")]
pub use post_release::*;

pub const PLACEHOLDER: &str = "{{currentVersion}}";

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
    /// Execute a release plan. This command will update versions, changelogs,
    /// and opens a pull request with the changes.
    #[cfg(feature = "release")]
    ExecutePlan(ApplyPlanArgs),
    /// Attempt to publish the specified package.
    ///
    /// This command will double check based on the release plan that the
    /// command is called on the right branch, and that the crate(s) to be
    /// released have the right version. After publishing, the command tags
    /// the release and pushes the tags.
    #[cfg(feature = "release")]
    PublishPlan(PublishPlanArgs),
    /// Rollover migrations steps post release.
    /// - Create new migration guides for packages that have a migration guide
    #[cfg(feature = "release")]
    PostRelease,
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
