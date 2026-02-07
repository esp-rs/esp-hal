//! MCP tool definitions for xtask commands.
//!
//! This module defines MCP tools that wrap xtask CLI commands, making them
//! accessible to AI agents via the Model Context Protocol.

use std::path::PathBuf;

use esp_metadata::Chip;
use rmcp::{
    ServerHandler,
    handler::server::{router::tool::ToolRouter, wrapper::Parameters},
    model::{Implementation, ServerCapabilities, ServerInfo},
    schemars,
    tool,
    tool_handler,
    tool_router,
};
use xtask::{Package, Version, commands::SemverCheckCmd};

/// MCP server that exposes xtask operations as tools.
#[derive(Clone)]
pub struct XtaskMcpServer {
    workspace: PathBuf,
    tool_router: ToolRouter<Self>,
}

impl XtaskMcpServer {
    /// Execute a cargo xtask command and capture output.
    fn run_xtask_command(&self, args: &[&str]) -> String {
        use std::process::Command;

        let output = match Command::new("cargo")
            .arg("xtask")
            .args(args)
            .current_dir(&self.workspace)
            .output()
        {
            Ok(output) => output,
            Err(e) => return format!("Failed to execute command: {}", e),
        };

        let stdout = String::from_utf8_lossy(&output.stdout).to_string();
        let stderr = String::from_utf8_lossy(&output.stderr).to_string();

        if output.status.success() {
            format!("{}\n{}", stdout, stderr)
        } else {
            format!(
                "Command failed with status {}:\nstdout: {}\nstderr: {}",
                output.status, stdout, stderr
            )
        }
    }
}

// ============================================================================
// Parameter structures for tools
// ============================================================================

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct BuildDocumentationParams {
    /// Packages to document.
    pub packages: Option<Vec<Package>>,
    /// Chips to build docs for.
    pub chips: Option<Vec<Chip>>,
    /// Base URL for deployed documentation links.
    pub base_url: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct BuildExamplesParams {
    /// Name of the example to build, or omit to build all examples.
    pub example: Option<String>,
    /// Target chip.
    pub chip: Option<Chip>,
    /// Package containing the examples (defaults to "examples").
    pub package: Option<Package>,
    /// Build in debug mode only.
    pub debug: Option<bool>,
    /// Toolchain to use for building.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct BuildPackageParams {
    /// Package to build.
    pub package: Package,
    /// Target triple to build for.
    pub target: Option<String>,
    /// Comma-separated list of features to enable.
    pub features: Option<String>,
    /// Toolchain to use for building.
    pub toolchain: Option<String>,
    /// Disable default features.
    pub no_default_features: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct BuildTestsParams {
    /// Target chip.
    pub chip: Chip,
    /// Specific test(s) to build (comma-separated).
    pub test: Option<String>,
    /// Toolchain to use for building.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct RunDocTestsParams {
    /// Target chip.
    pub chip: Chip,
    /// Packages to test.
    pub packages: Option<Vec<Package>>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct RunExampleParams {
    /// Name of the example to run.
    pub example: String,
    /// Target chip.
    pub chip: Option<Chip>,
    /// Package containing the example.
    pub package: Option<Package>,
    /// Toolchain to use.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct RunTestsParams {
    /// Target chip.
    pub chip: Chip,
    /// Specific test(s) to run (comma-separated).
    pub test: Option<String>,
    /// Number of times to repeat the tests.
    pub repeat: Option<u32>,
    /// Toolchain to use.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct FmtPackagesParams {
    /// Run in check mode (exit with error if not formatted).
    pub check: Option<bool>,
    /// Packages to format.
    pub packages: Option<Vec<Package>>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct LintPackagesParams {
    /// Packages to lint.
    pub packages: Option<Vec<Package>>,
    /// Chips to lint for.
    pub chips: Option<Vec<Chip>>,
    /// Automatically apply fixes.
    pub fix: Option<bool>,
    /// Toolchain to use.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct CheckPackagesParams {
    /// Packages to check.
    pub packages: Option<Vec<Package>>,
    /// Chips to check for.
    pub chips: Option<Vec<Chip>>,
    /// Toolchain to use.
    pub toolchain: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct CheckChangelogParams {
    /// Packages to check.
    pub packages: Option<Vec<Package>>,
    /// Re-generate changelogs with consistent formatting.
    pub normalize: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct HostTestsParams {
    /// Packages to test.
    pub packages: Option<Vec<Package>>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct UpdateMetadataParams {
    /// Run in check mode (exit with error if changes needed).
    pub check: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct CiParams {
    /// Target chip.
    pub chip: Chip,
    /// Toolchain to use.
    pub toolchain: Option<String>,
    /// Skip running lints.
    pub no_lint: Option<bool>,
    /// Skip building documentation.
    pub no_docs: Option<bool>,
    /// Skip checking crates.
    pub no_check_crates: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct CleanParams {
    /// Packages to clean.
    pub packages: Option<Vec<Package>>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct SemverCheckParams {
    /// Subcommand to run.
    pub action: SemverCheckCmd,
    /// Packages to check.
    pub packages: Option<Vec<Package>>,
    /// Chips to check.
    pub chips: Option<Vec<Chip>>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct BumpVersionParams {
    /// Version bump type.
    pub bump: Version,
    /// Packages to bump.
    pub packages: Option<Vec<Package>>,
    /// Dry run (show what would change without making changes).
    pub dry_run: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct PublishParams {
    /// Package to publish.
    pub package: Package,
    /// Dry run (validate without publishing).
    pub dry_run: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct TagReleasesParams {
    /// Packages to tag.
    pub packages: Option<Vec<Package>>,
    /// Dry run (show what would be tagged).
    pub dry_run: Option<bool>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct HelpParams {
    /// Subcommand to get help for (e.g., "build", "run", "release").
    pub command: Option<String>,
}

// ============================================================================
// Tool implementations
// ============================================================================

/// Helper to convert a Vec of displayable items into a comma-separated string.
fn to_csv<T: std::fmt::Display>(items: &[T]) -> String {
    items
        .iter()
        .map(|i| i.to_string())
        .collect::<Vec<_>>()
        .join(",")
}

#[tool_router]
impl XtaskMcpServer {
    pub fn new(workspace: PathBuf) -> Self {
        Self {
            workspace,
            tool_router: Self::tool_router(),
        }
    }

    #[tool(description = "Build documentation for esp-hal packages")]
    fn build_documentation(
        &self,
        Parameters(params): Parameters<BuildDocumentationParams>,
    ) -> String {
        let mut args = vec!["build".to_string(), "documentation".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        let chips_str;
        if let Some(ref c) = params.chips {
            chips_str = to_csv(c);
            args.push("--chips".to_string());
            args.push(chips_str.clone());
        }
        if let Some(ref u) = params.base_url {
            args.push("--base-url".to_string());
            args.push(u.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Build examples for a specific chip")]
    fn build_examples(&self, Parameters(params): Parameters<BuildExamplesParams>) -> String {
        let mut args = vec!["build".to_string(), "examples".to_string()];
        if let Some(ref e) = params.example {
            args.push(e.clone());
        }
        let chip_str;
        if let Some(ref c) = params.chip {
            chip_str = c.to_string();
            args.push("--chip".to_string());
            args.push(chip_str.clone());
        }
        let package_str;
        if let Some(ref p) = params.package {
            package_str = p.to_string();
            args.push("--package".to_string());
            args.push(package_str.clone());
        }
        if params.debug == Some(true) {
            args.push("--debug".to_string());
        }
        if let Some(ref t) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(t.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Build a specific package with custom options")]
    fn build_package(&self, Parameters(params): Parameters<BuildPackageParams>) -> String {
        let package_str = params.package.to_string();
        let mut args = vec!["build".to_string(), "package".to_string(), package_str];
        if let Some(ref t) = params.target {
            args.push("--target".to_string());
            args.push(t.clone());
        }
        if let Some(ref f) = params.features {
            args.push("--features".to_string());
            args.push(f.clone());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        if params.no_default_features == Some(true) {
            args.push("--no-default-features".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Build tests for a specific chip")]
    fn build_tests(&self, Parameters(params): Parameters<BuildTestsParams>) -> String {
        let chip_str = params.chip.to_string();
        let mut args = vec!["build".to_string(), "tests".to_string(), chip_str];
        if let Some(ref t) = params.test {
            args.push("--test".to_string());
            args.push(t.clone());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run documentation tests for a chip")]
    fn run_doc_tests(&self, Parameters(params): Parameters<RunDocTestsParams>) -> String {
        let chip_str = params.chip.to_string();
        let mut args = vec!["run".to_string(), "doc-tests".to_string(), chip_str];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run an example for a specific chip")]
    fn run_example(&self, Parameters(params): Parameters<RunExampleParams>) -> String {
        let mut args = vec![
            "run".to_string(),
            "example".to_string(),
            params.example.clone(),
        ];
        let chip_str;
        if let Some(ref c) = params.chip {
            chip_str = c.to_string();
            args.push("--chip".to_string());
            args.push(chip_str.clone());
        }
        let package_str;
        if let Some(ref p) = params.package {
            package_str = p.to_string();
            args.push("--package".to_string());
            args.push(package_str.clone());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run tests for a specific chip")]
    fn run_tests(&self, Parameters(params): Parameters<RunTestsParams>) -> String {
        let chip_str = params.chip.to_string();
        let mut args = vec!["run".to_string(), "tests".to_string(), chip_str];
        if let Some(ref t) = params.test {
            args.push("--test".to_string());
            args.push(t.clone());
        }
        if let Some(r) = params.repeat {
            args.push("--repeat".to_string());
            args.push(r.to_string());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Format all packages in the workspace with rustfmt")]
    fn fmt_packages(&self, Parameters(params): Parameters<FmtPackagesParams>) -> String {
        let mut args = vec!["fmt-packages".to_string()];
        if params.check == Some(true) {
            args.push("--check".to_string());
        }
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push(packages_str.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Lint all packages in the workspace with clippy")]
    fn lint_packages(&self, Parameters(params): Parameters<LintPackagesParams>) -> String {
        let mut args = vec!["lint-packages".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push(packages_str.clone());
        }
        let chips_str;
        if let Some(ref c) = params.chips {
            chips_str = to_csv(c);
            args.push("--chips".to_string());
            args.push(chips_str.clone());
        }
        if params.fix == Some(true) {
            args.push("--fix".to_string());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Check all packages with cargo check")]
    fn check_packages(&self, Parameters(params): Parameters<CheckPackagesParams>) -> String {
        let mut args = vec!["check-packages".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push(packages_str.clone());
        }
        let chips_str;
        if let Some(ref c) = params.chips {
            chips_str = to_csv(c);
            args.push("--chips".to_string());
            args.push(chips_str.clone());
        }
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Check the changelog for packages")]
    fn check_changelog(&self, Parameters(params): Parameters<CheckChangelogParams>) -> String {
        let mut args = vec!["check-changelog".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        if params.normalize == Some(true) {
            args.push("--normalize".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run host tests in the workspace")]
    fn host_tests(&self, Parameters(params): Parameters<HostTestsParams>) -> String {
        let mut args = vec!["host-tests".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push(packages_str.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Re-generate metadata and the chip support table")]
    fn update_metadata(&self, Parameters(params): Parameters<UpdateMetadataParams>) -> String {
        let mut args = vec!["update-metadata".to_string()];
        if params.check == Some(true) {
            args.push("--check".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run CI checks for a specific chip")]
    fn ci(&self, Parameters(params): Parameters<CiParams>) -> String {
        let chip_str = params.chip.to_string();
        let mut args = vec!["ci".to_string(), chip_str];
        if let Some(ref tc) = params.toolchain {
            args.push("--toolchain".to_string());
            args.push(tc.clone());
        }
        if params.no_lint == Some(true) {
            args.push("--no-lint".to_string());
        }
        if params.no_docs == Some(true) {
            args.push("--no-docs".to_string());
        }
        if params.no_check_crates == Some(true) {
            args.push("--no-check-crates".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Clean build artifacts for packages")]
    fn clean(&self, Parameters(params): Parameters<CleanParams>) -> String {
        let mut args = vec!["clean".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push(packages_str.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Run semver checks on packages")]
    fn semver_check(&self, Parameters(params): Parameters<SemverCheckParams>) -> String {
        let action_str = match params.action {
            SemverCheckCmd::GenerateBaseline => "generate-baseline",
            SemverCheckCmd::Check => "check",
            SemverCheckCmd::DownloadBaselines => "download-baselines",
        };
        let mut args = vec!["semver-check".to_string(), action_str.to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        let chips_str;
        if let Some(ref c) = params.chips {
            chips_str = to_csv(c);
            args.push("--chips".to_string());
            args.push(chips_str.clone());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Bump the version of specified packages")]
    fn bump_version(&self, Parameters(params): Parameters<BumpVersionParams>) -> String {
        let bump_str = params.bump.to_string();
        let mut args = vec!["release".to_string(), "bump-version".to_string(), bump_str];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        if params.dry_run == Some(true) {
            args.push("--dry-run".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Publish a package to crates.io")]
    fn publish(&self, Parameters(params): Parameters<PublishParams>) -> String {
        let package_str = params.package.to_string();
        let mut args = vec!["release".to_string(), "publish".to_string(), package_str];
        if params.dry_run == Some(true) {
            args.push("--dry-run".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "Generate git tags for package releases")]
    fn tag_releases(&self, Parameters(params): Parameters<TagReleasesParams>) -> String {
        let mut args = vec!["release".to_string(), "tag-releases".to_string()];
        let packages_str;
        if let Some(ref p) = params.packages {
            packages_str = to_csv(p);
            args.push("--packages".to_string());
            args.push(packages_str.clone());
        }
        if params.dry_run == Some(true) {
            args.push("--dry-run".to_string());
        }
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }

    #[tool(description = "List all available packages in the workspace")]
    fn list_packages(&self) -> String {
        use strum::IntoEnumIterator;
        Package::iter()
            .map(|p| p.to_string())
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[tool(description = "List all supported ESP32 chips")]
    fn list_chips(&self) -> String {
        use strum::IntoEnumIterator;
        Chip::iter()
            .map(|c| {
                let desc = match c {
                    Chip::Esp32 => "Xtensa LX6 dual-core, WiFi, Bluetooth Classic, BLE",
                    Chip::Esp32c2 => "RISC-V single-core, WiFi, BLE",
                    Chip::Esp32c3 => "RISC-V single-core, WiFi, BLE",
                    Chip::Esp32c5 => "RISC-V single-core, WiFi 6, BLE 5, 802.15.4",
                    Chip::Esp32c6 => "RISC-V single-core, WiFi 6, BLE 5, 802.15.4",
                    Chip::Esp32h2 => "RISC-V single-core, BLE 5, 802.15.4",
                    Chip::Esp32s2 => "Xtensa LX7 single-core, WiFi",
                    Chip::Esp32s3 => "Xtensa LX7 dual-core, WiFi, BLE",
                };
                format!("{} - {}", c, desc)
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[tool(description = "Get help text for xtask CLI commands")]
    fn help(&self, Parameters(params): Parameters<HelpParams>) -> String {
        let mut args = vec![];
        if let Some(ref cmd) = params.command {
            args.push(cmd.clone());
        }
        args.push("--help".to_string());
        let args_refs: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        self.run_xtask_command(&args_refs)
    }
}

#[tool_handler]
impl ServerHandler for XtaskMcpServer {
    fn get_info(&self) -> ServerInfo {
        // Read instructions from the copilot-instructions.md file
        let instructions_path = self.workspace.join(".github/copilot-instructions.md");
        let instructions = std::fs::read_to_string(&instructions_path)
            .unwrap_or_else(|_| "esp-hal xtask MCP server. See .github/copilot-instructions.md for usage.".into());

        ServerInfo {
            capabilities: ServerCapabilities::builder().enable_tools().build(),
            server_info: Implementation::from_build_env(),
            instructions: Some(instructions),
            ..Default::default()
        }
    }
}
