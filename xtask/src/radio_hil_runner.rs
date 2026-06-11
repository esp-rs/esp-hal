use std::{
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
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

    let mut harness_guard = match (harness_probe, harness.as_ref()) {
        (Some(harness_probe), Some(spec)) => {
            if !spec.binary_path.exists() {
                bail!("Harness binary not found: {}", spec.binary_path.display());
            }
            log::info!(
                "Starting support firmware on harness probe ({harness_probe}): {}",
                spec.binary_path.display()
            );
            let child = spawn_harness_and_wait_until_ready(
                spec.binary_path,
                harness_probe,
                HARNESS_BOOT_TIMEOUT,
            )?;
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

    let mut harness_guard = if let (Some(harness_probe), Some(harness_binary_path)) =
        (harness_probe, harness_binary_path)
    {
        log::info!(
            "Starting support firmware on harness probe ({harness_probe}): {}",
            harness_binary_path.display()
        );

        let child = spawn_harness_and_wait_until_ready(
            harness_binary_path,
            harness_probe,
            HARNESS_BOOT_TIMEOUT,
        )?;
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

// Maximum time we wait for the harness firmware to flash and emit its
// first defmt log line before giving up.
const HARNESS_BOOT_TIMEOUT: Duration = Duration::from_secs(90);
const PROBE_LIST_TIMEOUT: Duration = Duration::from_secs(15);

/// Spawn `probe-rs run` for the harness, capture its stdout, and block
/// until the firmware emits its first defmt log line (i.e. flashing is
/// complete and the support firmware is actually running).  
fn spawn_harness_and_wait_until_ready(
    binary_path: &Path,
    probe: &str,
    boot_timeout: Duration,
) -> Result<Child> {
    log::info!(
        "[HARNESS] probe-rs run --preverify --probe {probe} {} (waiting for first defmt line, timeout {}s)",
        binary_path.display(),
        boot_timeout.as_secs(),
    );

    let mut child = Command::new("probe-rs")
        .args(["run", "--preverify", "--probe", probe])
        .arg(binary_path)
        .env("DEFMT_LOG", "info")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| anyhow!("[HARNESS] failed to spawn probe-rs run: {e}"))?;

    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| anyhow!("[HARNESS] could not capture probe-rs stdout"))?;

    let (ready_tx, ready_rx) = mpsc::channel::<()>();
    thread::spawn(move || {
        let mut reader = BufReader::new(stdout);
        let mut line = String::new();
        let mut signaled = false;
        loop {
            line.clear();
            match reader.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) => {
                    print!("[HARNESS] {line}");
                    if !signaled && looks_like_defmt_line(&line) {
                        let _ = ready_tx.send(());
                        signaled = true;
                    }
                }
                Err(_) => break,
            }
        }
    });

    match ready_rx.recv_timeout(boot_timeout) {
        Ok(()) => {
            log::info!("[HARNESS] firmware booted; starting DUT");
            Ok(child)
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
            let _ = child.kill();
            let _ = child.wait();
            bail!(
                "[HARNESS] firmware did not produce any defmt output within {}s",
                boot_timeout.as_secs()
            );
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
            let _ = child.kill();
            let _ = child.wait();
            bail!("[HARNESS] probe-rs exited before producing any defmt output");
        }
    }
}

fn looks_like_defmt_line(line: &str) -> bool {
    let trimmed = line.trim_start();
    trimmed.starts_with("[INFO ")
        || trimmed.starts_with("[WARN ")
        || trimmed.starts_with("[ERROR]")
        || trimmed.starts_with("[DEBUG]")
        || trimmed.starts_with("[TRACE]")
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
        target.replace('-', "_").replace('.', "_").to_uppercase()
    )
}

fn spawn_probe_run(name: &str, binary_path: &Path, probe: &str) -> Result<Child> {
    log::info!(
        "[{name}] probe-rs run --preverify --probe {probe} {}",
        binary_path.display()
    );
    Command::new("probe-rs")
        .args(["run", "--preverify", "--probe", probe])
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

    let mut command = Command::new("probe-rs");
    command.args(["list"]);
    let output = run_command_with_output_timeout(command, "probe-rs list", PROBE_LIST_TIMEOUT)?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        bail!("Failed to run `probe-rs list`: {}", stderr.trim());
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

fn run_command_with_output_timeout(
    mut command: Command,
    name: &str,
    timeout: Duration,
) -> Result<std::process::Output> {
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| anyhow!("Failed to spawn {name}: {e}"))?;
    let start = Instant::now();

    loop {
        if child.try_wait()?.is_some() {
            return child
                .wait_with_output()
                .map_err(|e| anyhow!("Failed to collect {name} output: {e}"));
        }

        if start.elapsed() > timeout {
            let _ = child.kill();
            let _ = child.wait();
            bail!("{name} timed out after {}s", timeout.as_secs());
        }

        thread::sleep(Duration::from_millis(100));
    }
}
