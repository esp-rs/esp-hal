use std::{
    path::Path,
    process::{Child, Command},
    thread,
    time::{Duration, Instant},
};

use anyhow::{Result, anyhow, bail};

/// Run a radio HIL test.
///
/// If `harness_binary_path` is provided, the harness firmware is started on the
/// first probe and the DUT test is started on the second probe.
pub fn run_radio_test(
    dut_binary_path: &Path,
    harness_binary_path: Option<&Path>,
    timeout_secs: u64,
    probes: Option<String>,
) -> Result<()> {
    if !dut_binary_path.exists() {
        bail!("DUT binary not found: {}", dut_binary_path.display());
    }

    if let Some(harness_binary_path) = harness_binary_path
        && !harness_binary_path.exists()
    {
        bail!(
            "Harness binary not found: {}",
            harness_binary_path.display()
        );
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

    let mut harness_child = if let (Some(harness_probe), Some(harness_binary_path)) =
        (harness_probe, harness_binary_path)
    {
        log::info!(
            "Resetting harness probe ({harness_probe}) and starting support firmware ({})",
            harness_binary_path.display()
        );
        reset_probe(harness_probe)?;

        let child = spawn_probe_run("HARNESS", harness_binary_path, harness_probe)?;
        // Give support firmware time to initialize AP/server stack.
        thread::sleep(Duration::from_millis(2000));
        Some(child)
    } else {
        None
    };

    let timeout = Duration::from_secs(timeout_secs);
    let mut dut_child = spawn_probe_run("DUT", dut_binary_path, dut_probe)?;
    let start = Instant::now();
    let dut_passed = loop {
        if let Some(child) = harness_child.as_mut()
            && let Some(status) = child.try_wait()?
        {
            terminate_child(child, "HARNESS");
            terminate_child(&mut dut_child, "DUT");
            bail!(
                "Harness firmware exited unexpectedly with status {status} before DUT finished"
            );
        }

        if let Some(status) = dut_child.try_wait()? {
            break status.success();
        }

        if start.elapsed() > timeout {
            terminate_child(&mut dut_child, "DUT");
            if let Some(child) = harness_child.as_mut() {
                terminate_child(child, "HARNESS");
            }
            bail!("DUT timed out after {}s", timeout_secs);
        }

        thread::sleep(Duration::from_millis(100));
    };

    if let Some(child) = harness_child.as_mut() {
        terminate_child(child, "HARNESS");
    }

    if !dut_passed {
        bail!("DUT test failed");
    }

    Ok(())
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
