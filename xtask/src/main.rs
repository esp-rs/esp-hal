use anyhow::{Context, Result};
use clap::{Args, Parser};
use esp_devtool as xtask;
use strum::IntoEnumIterator;
use xtask::{
    Package,
    commands::{dispatch::Verb, *},
    metadata::Chip,
};

// ----------------------------------------------------------------------------
// Command-line Interface

#[derive(Debug, Parser)]
enum Cli {
    /// Build examples or HIL tests.
    Build(DispatchArgs),
    /// Run an example or qa binary.
    Run(DispatchArgs),
    /// Check crates, or try-build examples and tests.
    #[clap(alias = "check-packages")]
    Check(DispatchArgs),
    /// Run HIL tests.
    Test(DispatchArgs),
    /// Build rustdoc for the specified packages and chips.
    #[clap(alias = "docs", alias = "doc")]
    Documentation(BuildDocumentationArgs),
    /// Build the documentation index.
    #[cfg(feature = "deploy-docs")]
    DocumentationIndex(BuildDocumentationIndexArgs),
    /// Run rustdoc tests for the specified chip.
    DocTests(DocTestArgs),
    /// Run prebuilt ELFs with probe-rs.
    Elfs(RunElfsArgs),
    /// Release-related subcommands
    #[clap(subcommand)]
    Release(Release),

    /// Perform (parts of) the checks done in CI
    Ci(CiArgs),
    /// Format all packages in the workspace with rustfmt
    #[clap(name = "fmt", alias = "fmt-packages", alias = "format-packages")]
    FmtPackages(FmtPackagesArgs),
    /// Run cargo clean
    Clean(CleanArgs),
    /// Lint all packages in the workspace with clippy
    #[clap(name = "lint", alias = "lint-packages")]
    LintPackages(LintPackagesArgs),
    /// Semver Checks
    SemverCheck(SemverCheckArgs),
    /// Check the changelog for packages.
    CheckChangelog(CheckChangelogArgs),
    /// Validate the changelog format of a PR description.
    ///
    /// Reads the body from stdin by default. Pass `--pr` to fetch from GitHub.
    CheckPrChangelog(CheckPrChangelogArgs),
    /// Run host-tests in the workspace with `cargo test`
    HostTests(HostTestsArgs),
    /// Check global symbols in the compiled `.rlib` of esp-hal for the given chips.
    CheckGlobalSymbols {
        /// Chip(s) to check. Omitted means every chip.
        #[arg(value_enum, value_delimiter = ',', default_values_t = Chip::iter())]
        chips: Vec<Chip>,
    },
    #[cfg(feature = "report")]
    /// Generate reports from CI data.
    GenerateReport(generate_report::ReportArgs),
    /// Tasks for checking compile tests with a local registry.
    #[cfg(feature = "rel-check")]
    #[clap(subcommand)]
    RelCheck(relcheck::RelCheckCmds),

    /// Start the MCP server (stdio transport, for use with Claude Code).
    #[cfg(feature = "mcp")]
    Mcp,
}

#[derive(Debug, Args)]
struct CheckPrChangelogArgs {
    /// GitHub pull-request number to fetch and validate.
    ///
    /// When omitted, the PR body is read from stdin.
    #[arg(long)]
    pr: Option<u64>,
}

#[derive(Debug, Args)]
struct CheckChangelogArgs {
    /// Package(s) to tag.
    #[arg(long, alias = "package", value_enum, value_delimiter = ',', default_values_t = Package::iter())]
    packages: Vec<Package>,

    /// Re-generate the changelog with consistent formatting.
    #[arg(long)]
    normalize: bool,
}

// ----------------------------------------------------------------------------
// Application

fn main() -> Result<()> {
    // In MCP mode stdout is the JSON-RPC channel — log to stderr instead so
    // we don't corrupt the protocol.  We detect MCP early (before clap parse)
    // so the logger is set up correctly before anything else runs.
    #[cfg(feature = "mcp")]
    let is_mcp = std::env::args().any(|a| a == "mcp");
    #[cfg(not(feature = "mcp"))]
    let is_mcp = false;

    let mut builder =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"));
    if is_mcp {
        builder.target(env_logger::Target::Stderr);
    } else {
        builder.target(env_logger::Target::Stdout);
    }
    builder.init();

    let workspace =
        std::env::current_dir().with_context(|| "Failed to get the current dir!".to_string())?;
    let target_path = workspace.join("target");

    if std::env::var("CARGO_TARGET_DIR").is_err() {
        unsafe { std::env::set_var("CARGO_TARGET_DIR", target_path.to_str().unwrap()) };
    }

    match Cli::parse() {
        Cli::Build(args) => dispatch::dispatch(&workspace, Verb::Build, args, |args| {
            check_packages(&workspace, args)
        }),
        Cli::Run(args) => dispatch::dispatch(&workspace, Verb::Run, args, |args| {
            check_packages(&workspace, args)
        }),
        Cli::Check(args) => dispatch::dispatch(&workspace, Verb::Check, args, |args| {
            check_packages(&workspace, args)
        }),
        Cli::Test(args) => dispatch::dispatch(&workspace, Verb::Test, args, |args| {
            check_packages(&workspace, args)
        }),
        Cli::Documentation(args) => build_documentation(&workspace, args),
        #[cfg(feature = "deploy-docs")]
        Cli::DocumentationIndex(args) => build_documentation_index(&workspace, args),
        Cli::DocTests(args) => run_doc_tests(&workspace, args),
        Cli::Elfs(args) => run_elfs(args),

        // Release-related subcommands:
        Cli::Release(release) => match release {
            Release::ChangelogPreview(args) => changelog_preview(&workspace, args),
            Release::BumpVersion(args) => bump_version(&workspace, args),
            Release::TagReleases(args) => tag_releases(&workspace, args),
            Release::Publish(args) => publish(&workspace, args),
            #[cfg(feature = "release")]
            Release::Plan(args) => plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::ExecutePlan(args) => execute_plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::PublishPlan(args) => publish_plan(&workspace, args),
            #[cfg(feature = "release")]
            Release::PostRelease(args) => post_release(&workspace, args),
            #[cfg(feature = "release")]
            Release::BumpMsrv(args) => bump_msrv::bump_msrv(&workspace, args),
        },

        Cli::Ci(args) => run_ci_checks(&workspace, args),
        Cli::FmtPackages(args) => fmt_packages(&workspace, args),
        Cli::Clean(args) => clean(&workspace, args),
        Cli::LintPackages(args) => lint_packages(&workspace, args),
        Cli::SemverCheck(args) => semver_checks(&workspace, args),
        Cli::CheckChangelog(args) => check_changelog(&workspace, &args.packages, args.normalize),
        Cli::CheckPrChangelog(args) => check_pr_changelog(&workspace, args.pr),
        Cli::HostTests(args) => host_tests(&workspace, args),
        Cli::CheckGlobalSymbols { chips } => check_global_symbols(&chips),
        #[cfg(feature = "report")]
        Cli::GenerateReport(args) => generate_report::generate_report(&workspace, args),
        #[cfg(feature = "rel-check")]
        Cli::RelCheck(relcheck) => relcheck::run_rel_check(relcheck),

        #[cfg(feature = "mcp")]
        Cli::Mcp => tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?
            .block_on(xtask::commands::mcp::run_mcp_server()),
    }
}
