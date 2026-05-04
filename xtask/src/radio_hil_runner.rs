use std::{
    path::{Path, PathBuf},
    process::{Child, Command},
    thread,
    time::Duration,
};

use anyhow::{Result, anyhow, bail};

use crate::cargo::BuiltCommand;

/// Specification for the optional harness pre-step.
pub struct HarnessSpec<'a> {
    /// Path to the already-built harness firmware ELF.
    pub binary_path: &'a Path,
    /// Cargo target triple of the firmware.
    pub target: &'a str,
}

/// Run a radio HIL test.
///
/// This is the entry point for radio HIL test orchestration. It takes the
/// regular `cargo run` command for the DUT and, when a harness specification
/// is supplied, performs the pre-step of flashing the support firmware on a
/// secondary probe before handing back to the normal `cargo run` flow.
pub fn run_radio_test(
    dut_command: &BuiltCommand,
    harness: Option<HarnessSpec<'_>>,
    probes: Option<String>,
) -> Result<()> {
    let probes = detect_probes(probes)?;
    let (harness_probe, dut_probe) = match harness.as_ref() {
        Some(_) => {
            if probes.len() < 2 {
                bail!(
                    "Expected at least 2 probes for test + harness, found {}",
                    probes.len()
                );
            }
            (Some(probes[0].as_str()), probes[1].as_str())
        }
        None => {
            if probes.is_empty() {
                bail!("No probes detected");
            }
            (None, probes[0].as_str())
        }
    };

    log::info!("Resetting DUT probe ({dut_probe})");
    reset_probe(dut_probe)?;

    let mut harness_guard = match (harness_probe, harness.as_ref()) {
        (Some(harness_probe), Some(spec)) => {
            if !spec.binary_path.exists() {
                bail!("Harness binary not found: {}", spec.binary_path.display());
            }
            log::info!(
                "Resetting harness probe ({harness_probe}) and starting support firmware ({})",
                spec.binary_path.display()
            );
            reset_probe(harness_probe)?;
            let child = spawn_probe_run("HARNESS", spec.binary_path, harness_probe)?;
            // Give support firmware time to initialize support part.
            thread::sleep(Duration::from_millis(2000));
            Some(HarnessGuard { child })
        }
        _ => None,
    };

    let mut dut_command = dut_command.clone();
    if let Some(spec) = harness.as_ref() {
        let runner_key = runner_env_key(spec.target);
        let runner_value = format!("probe-rs run --preverify --probe {}", dut_probe);
        log::debug!("Pinning DUT probe via {runner_key}={runner_value}");

        dut_command.env_vars.push((runner_key, runner_value));
    }

    let dut_result = dut_command.run(false);

    if let Some(guard) = harness_guard.as_mut() {
        guard.terminate();
    }

    dut_result.map(|_| ())
}

/// Run a radio HIL test directly from prebuilt ELFs.
///
/// Used by `run elfs`, where there is no `cargo run` command to delegate to.
pub fn run_radio_test_elf(
    dut_binary_path: &Path,
    harness_binary_path: Option<&Path>,
    timeout_secs: u64,
    probes: Option<String>,
) -> Result<()> {
    use std::time::Instant;

    if !dut_binary_path.exists() {
        bail!("DUT binary not found: {}", dut_binary_path.display());
    }

    if let Some(path) = harness_binary_path
        && !path.exists()
    {
        bail!("Harness binary not found: {}", path.display());
    }

    let probes = detect_probes(probes)?;
    let (harness_probe, dut_probe) = match harness_binary_path {
        Some(_) => {
            if probes.len() < 2 {
                bail!(
                    "Expected at least 2 probes for test + harness, found {}",
                    probes.len()
                );
            }
            (Some(probes[0].as_str()), probes[1].as_str())
        }
        None => {
            if probes.is_empty() {
                bail!("No probes detected");
            }
            (None, probes[0].as_str())
        }
    };

    log::info!("Resetting DUT probe ({dut_probe})");
    reset_probe(dut_probe)?;

    let mut harness_guard = if let (Some(harness_probe), Some(harness_binary_path)) =
        (harness_probe, harness_binary_path)
    {
        log::info!(
            "Resetting harness probe ({harness_probe}) and starting support firmware ({})",
            harness_binary_path.display()
        );
        reset_probe(harness_probe)?;
        let child = spawn_probe_run("HARNESS", harness_binary_path, harness_probe)?;
        thread::sleep(Duration::from_millis(2000));
        Some(HarnessGuard { child })
    } else {
        None
    };

    let mut dut_child = spawn_probe_run("DUT", dut_binary_path, dut_probe)?;
    let timeout = Duration::from_secs(timeout_secs);
    let start = Instant::now();
    let dut_passed = loop {
        if let Some(guard) = harness_guard.as_mut()
            && let Some(status) = guard.child.try_wait()?
        {
            guard.terminate();
            terminate_child(&mut dut_child, "DUT");
            bail!("Harness firmware exited unexpectedly with status {status} before DUT finished");
        }

        if let Some(status) = dut_child.try_wait()? {
            break status.success();
        }

        if start.elapsed() > timeout {
            terminate_child(&mut dut_child, "DUT");
            if let Some(guard) = harness_guard.as_mut() {
                guard.terminate();
            }
            bail!("DUT timed out after {}s", timeout_secs);
        }

        thread::sleep(Duration::from_millis(100));
    };

    if let Some(guard) = harness_guard.as_mut() {
        guard.terminate();
    }

    if !dut_passed {
        bail!("DUT test failed");
    }

    Ok(())
}

/// Locate a built artifact in `target/<triple>/release/`.
pub fn resolve_release_binary(workspace: &Path, target: &str, artifact_name: &str) -> PathBuf {
    let release_dir = workspace.join("target").join(target).join("release");
    let exact = release_dir.join(artifact_name);
    if exact.exists() {
        return exact;
    }

    let base_name = artifact_name
        .split('_')
        .take_while(|part| *part != "has" && *part != "no")
        .collect::<Vec<_>>()
        .join("_");
    release_dir.join(base_name)
}

struct HarnessGuard {
    child: Child,
}

impl HarnessGuard {
    fn terminate(&mut self) {
        terminate_child(&mut self.child, "HARNESS");
    }
}

impl Drop for HarnessGuard {
    fn drop(&mut self) {
        self.terminate();
    }
}

fn runner_env_key(target: &str) -> String {
    format!(
        "CARGO_TARGET_{}_RUNNER",
        target.replace(['-', '.'], "_").to_uppercase()
    )
}

fn spawn_probe_run(name: &str, binary_path: &Path, probe: &str) -> Result<Child> {
    log::info!(
        "[{name}] probe-rs run --probe {probe} {}",
        binary_path.display()
    );
    Command::new("probe-rs")
        .args(["run", "--probe", probe])
        .arg(binary_path)
        .env("DEFMT_LOG", "info")
        .spawn()
        .map_err(|e| anyhow!("[{name}] failed to spawn probe-rs run: {e}"))
}

fn terminate_child(child: &mut Child, name: &str) {
    if child.try_wait().ok().flatten().is_none() {
        let _ = child.kill();
    }
    let _ = child.wait();
    log::debug!("[{name}] process terminated");
}

fn detect_probes(probes: Option<String>) -> Result<Vec<String>> {
    if let Some(p) = probes {
        let parsed = p
            .split([',', ' ', '\n', '\t'])
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(ToString::to_string)
            .collect::<Vec<_>>();
        if parsed.is_empty() {
            bail!("Provided probe list is empty");
        }
        return Ok(parsed);
    }

    let output = Command::new("probe-rs").args(["list"]).output()?;
    if !output.status.success() {
        bail!("Failed to run `probe-rs list`");
    }

    let list_output = String::from_utf8_lossy(&output.stdout);
    let mut discovered = Vec::<String>::new();

    for line in list_output.lines() {
        if let Some((_, suffix)) = line.split_once("--")
            && let Some(serial) = suffix.split_whitespace().next()
        {
            let serial = serial.trim().to_string();
            if !serial.is_empty() && !discovered.iter().any(|s| s == &serial) {
                discovered.push(serial);
            }
        }
    }

    if discovered.is_empty() {
        bail!("No probes were discovered from `probe-rs list` output");
    }

    Ok(discovered)
}

fn reset_probe(probe: &str) -> Result<()> {
    let output = Command::new("probe-rs")
        .args(["reset", "--probe", probe])
        .output()?;
    if !output.status.success() {
        bail!("Failed to reset probe {probe}");
    }
    thread::sleep(Duration::from_millis(500));
    Ok(())
}
