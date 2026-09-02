use std::{path::Path, time::Instant};

use anyhow::{Context, Result, bail};
use clap::Args;
use strum::IntoEnumIterator;

use super::{
    build::{BuildDocumentationArgs, build_documentation},
    check::{CheckPackagesArgs, check_packages},
    examples::examples,
    lint::{LintPackagesArgs, lint_packages},
    run::{DocTestArgs, run_doc_tests},
    tests::tests,
};
use crate::{Package, cargo::CargoAction, metadata::Chip};

/// Arguments for the `ci` subcommand.
#[cfg_attr(
    feature = "mcp",
    xtask_mcp_macros::mcp_tool(
        description = "Perform (parts of) the checks done in CI for a given chip",
        command = "ci"
    )
)]
#[derive(Debug, Args)]
pub struct CiArgs {
    /// Chip to target.
    #[arg(value_enum)]
    pub chip: Chip,

    /// The toolchain used to run the lints
    #[arg(long)]
    pub toolchain: Option<String>,

    /// Steps to run in the CI pipeline.
    #[arg(long, value_delimiter = ',')]
    pub steps: Vec<String>,

    /// Whether to skip running lints
    #[arg(long)]
    pub no_lint: bool,

    /// Whether to skip building documentation
    #[arg(long)]
    pub no_docs: bool,

    /// Whether to skip checking the crates itself
    #[arg(long)]
    pub no_check_crates: bool,
}

struct Runner {
    failed: Vec<&'static str>,
    started_at: Instant,
    skip_steps: Vec<String>,
    run_steps: Vec<String>,
    steps_executed: usize,
}

impl Runner {
    fn new(options: &CiArgs) -> Self {
        Self {
            failed: Vec::new(),
            started_at: Instant::now(),
            skip_steps: {
                let mut skip = vec![];
                if options.no_lint {
                    skip.push(String::from("lint"));
                }
                if options.no_docs {
                    skip.push(String::from("docs"));
                    skip.push(String::from("doc-tests"));
                }
                if options.no_check_crates {
                    skip.push(String::from("check"));
                }
                skip
            },
            run_steps: options.steps.clone(),
            steps_executed: 0,
        }
    }

    fn run(&mut self, id: &str, group: &'static str, op: impl FnOnce() -> Result<()>) {
        if self.skip_steps.iter().any(|s| s == id) {
            log::debug!("{group} skipped by user request");
            return;
        }
        if !self.run_steps.is_empty() && !self.run_steps.iter().any(|s| s == id) {
            log::debug!("{group} skipped by user request");
            return;
        }

        self.steps_executed += 1;

        // Output grouped logs
        // https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#grouping-log-lines
        println!("::group::{group}");
        if let Err(e) = op() {
            log::error!("{group} failed: {e:?}");
            self.failed.push(group);
        } else {
            log::debug!("{group} succeeded");
        }
        println!("::endgroup::");
    }

    fn finish(self) -> Result<()> {
        fn write_summary(message: &str) {
            if let Some(summary_file) = std::env::var_os("GITHUB_STEP_SUMMARY") {
                std::fs::write(summary_file, message).unwrap();
            }
        }

        let expected_to_run = self
            .run_steps
            .iter()
            .filter(|s| !self.skip_steps.contains(s))
            .cloned()
            .collect::<Vec<_>>();
        if self.steps_executed == 0 && !expected_to_run.is_empty() {
            bail!(
                "The following steps were requested but not executed: {}. Perhaps they contain typos?",
                expected_to_run.join(", ")
            );
        }

        log::info!("CI checks completed in {:?}", self.started_at.elapsed());

        if !self.failed.is_empty() {
            let mut summary = String::new();
            summary.push_str("# Summary of failed CI checks\n");
            for failed_check in self.failed {
                summary.push_str(&format!("* {failed_check}\n"));
            }
            println!("{summary}");
            write_summary(&summary);
            bail!("CI checks failed");
        }

        Ok(())
    }
}

/// Perform (parts of) the checks done in CI for a given chip.
pub fn run_ci_checks(workspace: &Path, args: CiArgs) -> Result<()> {
    log::info!("Running CI checks for chip: {}", args.chip);
    println!("::add-matcher::.github/rust-matchers.json");

    let run_locally = !std::env::var("CI").is_ok();

    let mut runner = Runner::new(&args);

    if !run_locally && args.steps.is_empty() {
        runner.skip_steps.push(String::from("tests"));
    }

    unsafe {
        std::env::set_var("CI", "true");
    }

    runner.run("check", "Check crates", || {
        check_packages(
            workspace,
            CheckPackagesArgs {
                packages: Package::iter().collect(),
                chips: vec![args.chip],
                toolchain: args.toolchain.clone(),
            },
        )
    });

    runner.run("lint", "Lint", || {
        lint_packages(
            workspace,
            LintPackagesArgs {
                packages: Package::iter().collect(),
                chips: vec![args.chip],
                fix: false,
                toolchain: args.toolchain.clone(),
            },
        )
    });

    runner.run("doc-tests", "Run Doc Test", || {
        run_doc_tests(
            workspace,
            DocTestArgs {
                packages: Package::iter().collect(),
                chip: args.chip,
            },
        )
    });

    runner.run("docs", "Build Docs", || {
        build_documentation(
            workspace,
            BuildDocumentationArgs {
                packages: vec![Package::EspHal, Package::EspRadio, Package::EspRtos],
                chips: vec![args.chip],
                ..Default::default()
            },
        )
    });

    // for chips with esp-lp-hal: Build all supported examples for the low-power
    // core first
    if args.chip.has_lp_core() {
        // Build prerequisite examples (esp-lp-hal)
        // `examples` copies the examples to a folder with the chip name as the last
        // path element then we copy it to the place where the HP core example
        // expects it
        runner.run("lp-examples", "Build LP-HAL Examples", || {
            // The LP examples aren't really that demanding, but they need to be at a certain place.
            // Instead of trying to figure out where the results are, let's just make sure the
            // target folder is set up as expected.
            let original_target_dir = std::env::var("CARGO_TARGET_DIR");

            unsafe {
                std::env::set_var(
                    "CARGO_TARGET_DIR",
                    workspace.join("esp-lp-hal").join("target"),
                );
            }
            let result = examples(
                workspace,
                Package::EspLpHal,
                args.chip,
                Some("all"),
                CargoAction::Build(None),
                false,
                args.toolchain.as_deref(),
                false,
            );

            // Still need to rename examples to remove the fingerprint off of their names:
            let dir = workspace
                .join("esp-lp-hal")
                .join("target")
                .join(Package::EspLpHal.target_triple(&args.chip)?)
                .join("release")
                .join("examples");
            let examples = dir
                .read_dir()
                .with_context(|| format!("Failed to read examples directory: {}", dir.display()))?;
            for example in examples {
                let example = example.context("Failed to read example")?;
                let example_path = example.path();
                if example
                    .file_type()
                    .with_context(|| {
                        format!(
                            "Failed to get file type for example: {}",
                            example_path.display()
                        )
                    })?
                    .is_file()
                    && example_path.extension().is_none()
                {
                    let example_name = example.file_name().to_string_lossy().to_string();
                    let Some((without_fingerprint, fingerprint)) = example_name.rsplit_once('-')
                    else {
                        // Skip non-fingerprinted files to avoid self-copying
                        // artifacts (for example "blinky" -> "blinky").
                        continue;
                    };
                    // Cargo fingerprints are hexadecimal, so only rewrite files
                    // that actually look like fingerprinted artifacts.
                    if !fingerprint.chars().all(|c| c.is_ascii_hexdigit()) {
                        continue;
                    }

                    let dst = dir.join(without_fingerprint);

                    if dst.exists() {
                        std::fs::remove_file(&dst).with_context(|| {
                            format!(
                                "Failed to remove destination file {} before copying",
                                dst.display()
                            )
                        })?;
                    }

                    log::debug!("Copying {} to {}", example_path.display(), dst.display());

                    // Copy so we don't trigger a rebuild unnecessarily by deleting the original
                    std::fs::copy(&example_path, &dst).with_context(|| {
                        format!(
                            "Failed to copy example: {} to {}",
                            example_path.display(),
                            dst.display()
                        )
                    })?;
                    // Check that the destination file matches the source file
                    let dst_contents = std::fs::read(&dst).with_context(|| {
                        format!("Failed to read destination file: {}", dst.display())
                    })?;
                    let src_contents = std::fs::read(&example_path).with_context(|| {
                        format!("Failed to read source file: {}", example_path.display())
                    })?;
                    if dst_contents != src_contents {
                        return Err(anyhow::anyhow!(
                            "Destination file {} does not match source file {}",
                            dst.display(),
                            example_path.display()
                        ));
                    }
                }
            }

            // Restore the original target directory
            unsafe {
                if let Ok(target) = original_target_dir {
                    std::env::set_var("CARGO_TARGET_DIR", target);
                } else {
                    std::env::remove_var("CARGO_TARGET_DIR");
                }
            }

            result
        });

        // Check documentation. Reuse the "docs" ID, as the docs build should include all applicable
        // packages.
        runner.run("docs", "Build LP-HAL docs", || {
            build_documentation(
                workspace,
                BuildDocumentationArgs {
                    packages: vec![Package::EspLpHal],
                    chips: vec![args.chip],
                    ..Default::default()
                },
            )
        });
    }

    runner.run("examples", "Build examples", || {
        // The `ota_example` expects a file named `target/ota_image` - it doesn't
        // care about the contents however
        std::fs::create_dir_all("./target")
            .with_context(|| "Failed to create `./target`".to_string())?;
        std::fs::write("./target/ota_image", "DUMMY").with_context(|| {
            "Failed to create a dummy file required by ota example!".to_string()
        })?;

        examples(
            workspace,
            Package::Examples,
            args.chip,
            Some("all"),
            CargoAction::Build(None),
            true,
            args.toolchain.as_deref(),
            false,
        )
    });

    runner.run("qa-test", "Build qa-test", || {
        examples(
            workspace,
            Package::QaTest,
            args.chip,
            Some("all"),
            CargoAction::Build(None),
            true,
            args.toolchain.as_deref(),
            false,
        )
    });

    // Try-build tests
    runner.run("tests", "Build tests", || {
        let target_path = workspace.join("target");

        tests(
            workspace,
            Package::HilTest,
            args.chip,
            None,
            CargoAction::Build(Some(target_path.join("tests"))),
            1,
            None,
            false,
        )?;

        tests(
            workspace,
            Package::HilTestRadio,
            args.chip,
            None,
            CargoAction::Build(Some(target_path.join("tests"))),
            1,
            None,
            false,
        )
    });

    runner.finish()
}
