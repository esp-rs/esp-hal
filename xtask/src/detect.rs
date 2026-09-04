//! Chip inference from a connected device.
//!
//! When the chip is omitted, `run` uses [`with_espflash`] and `test` uses [`with_probe_rs`], so
//! that detection picks the port or probe the command is about to drive.
//!
//! Detection shells out to tools already on `PATH`, this crate does not link them. With several
//! devices attached, a terminal gets a prompt and everything else gets an error.

use std::{fmt, io::IsTerminal, process::Command, str::FromStr, time::Duration};

use inquire::Select;
use serde::Deserialize;

use crate::metadata::Chip;

const SHORT_TIMEOUT: Duration = Duration::from_secs(10);
const ESPFLASH_TIMEOUT: Duration = Duration::from_secs(20);

/// Infer the chip via `espflash`, used by `run` because it resets the board anyway.
pub fn with_espflash() -> Option<Chip> {
    pick_connected(detect_via_espflash())
}

/// Infer the chip via `probe-rs`, used by `test` because it runs through the probe anyway.
pub fn with_probe_rs() -> Option<Chip> {
    pick_connected(detect_via_probe_rs())
}

fn pick_connected(devices: Vec<ConnectedDevice>) -> Option<Chip> {
    let device = match devices.len() {
        0 => return None,
        1 => devices.into_iter().next().unwrap(),
        _ => pick_device(devices)?,
    };
    export_connection(&device);
    log::info!("Using connected chip {device}");
    Some(device.chip)
}

fn pick_device(devices: Vec<ConnectedDevice>) -> Option<ConnectedDevice> {
    let summary = devices
        .iter()
        .map(|device| device.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    if !std::io::stdin().is_terminal() {
        log::info!(
            "Multiple ESP devices connected ({summary}). Pass the chip name, or run from a terminal to pick one."
        );
        return None;
    }
    Select::new("Select the connected chip:", devices)
        .prompt()
        .ok()
}

fn export_connection(device: &ConnectedDevice) {
    match &device.via {
        Connection::Serial(port) => set_env_if_unset("ESPFLASH_PORT", port),
        Connection::Probe { selector, .. } => set_env_if_unset("PROBE_RS_PROBE", selector),
    }
}

fn set_env_if_unset(key: &str, value: &str) {
    match std::env::var(key) {
        Ok(existing) if !existing.is_empty() => {}
        _ => unsafe { std::env::set_var(key, value) },
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ConnectedDevice {
    chip: Chip,
    via: Connection,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Connection {
    Serial(String),
    Probe { selector: String, name: String },
}

impl fmt::Display for ConnectedDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.via {
            Connection::Serial(port) => write!(f, "{} on {port}", self.chip),
            Connection::Probe { name, selector } if selector.is_empty() => {
                write!(f, "{} via {name}", self.chip)
            }
            Connection::Probe { name, selector } => {
                write!(f, "{} via {name} ({selector})", self.chip)
            }
        }
    }
}

fn detect_via_probe_rs() -> Vec<ConnectedDevice> {
    if let Ok(selector) = std::env::var("PROBE_RS_PROBE")
        && !selector.is_empty()
    {
        return probe_rs_mi_info(Some(&selector), "probe")
            .into_iter()
            .collect();
    }

    let probes = probe_rs_list();
    if probes.is_empty() {
        return probe_rs_mi_info(None, "probe").into_iter().collect();
    }

    probes
        .into_iter()
        .filter_map(|probe| probe_rs_mi_info(Some(&probe.selector), &probe.name))
        .collect()
}

struct ProbeRef {
    name: String,
    selector: String,
}

fn probe_rs_list() -> Vec<ProbeRef> {
    let Some(output) = run_cli("probe-rs", &["list"], SHORT_TIMEOUT) else {
        return Vec::new();
    };
    if !output.status.success() {
        log::debug!(
            "probe-rs list failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        return Vec::new();
    }
    parse_probe_list(&String::from_utf8_lossy(&output.stdout))
}

fn probe_rs_mi_info(probe: Option<&str>, name: &str) -> Option<ConnectedDevice> {
    let mut args = vec!["mi", "info", "--non-interactive"];
    if let Some(probe) = probe {
        args.extend_from_slice(&["--probe", probe]);
    }
    let output = run_cli("probe-rs", &args, SHORT_TIMEOUT)?;
    if !output.status.success() {
        log::debug!(
            "probe-rs mi info failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        return None;
    }
    let chip = chip_from_mi_info(&String::from_utf8_lossy(&output.stdout))?;
    Some(ConnectedDevice {
        chip,
        via: Connection::Probe {
            selector: probe.unwrap_or_default().to_string(),
            name: name.to_string(),
        },
    })
}

fn detect_via_espflash() -> Vec<ConnectedDevice> {
    let ports = match std::env::var("ESPFLASH_PORT") {
        Ok(port) if !port.is_empty() => vec![port],
        _ => espflash_ports(),
    };
    if ports.len() > 1 {
        log::info!("Identifying {} connected serial devices…", ports.len());
    }
    ports
        .into_iter()
        .filter_map(|port| {
            let chip = espflash_board_info(&port)?;
            Some(ConnectedDevice {
                chip,
                via: Connection::Serial(port),
            })
        })
        .collect()
}

fn espflash_board_info(port: &str) -> Option<Chip> {
    let output = run_cli(
        "espflash",
        &[
            "board-info",
            "--non-interactive",
            "--skip-update-check",
            "-p",
            port,
        ],
        ESPFLASH_TIMEOUT,
    )?;
    if !output.status.success() {
        log::debug!(
            "espflash board-info {port} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        return None;
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    chip_from_board_info(&stdout).or_else(|| chip_from_board_info(&stderr))
}

fn espflash_ports() -> Vec<String> {
    let Some(output) = run_cli(
        "espflash",
        &["list-ports", "--name-only", "--skip-update-check"],
        SHORT_TIMEOUT,
    ) else {
        return Vec::new();
    };
    if !output.status.success() {
        log::debug!(
            "espflash list-ports failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        return Vec::new();
    }
    let mut ports = Vec::new();
    for line in String::from_utf8_lossy(&output.stdout).lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        // `/dev/cu.foo` and `/dev/tty.foo` are the same UART on macOS, prefer `cu`.
        let port = match line.strip_prefix("/dev/tty.") {
            Some(rest) => format!("/dev/cu.{rest}"),
            None => line.to_string(),
        };
        if !ports.contains(&port) {
            ports.push(port);
        }
    }
    ports
}

fn run_cli(program: &str, args: &[&str], timeout: Duration) -> Option<std::process::Output> {
    let mut command = Command::new(program);
    command.args(args);
    match crate::run_command_with_output_timeout(command, program, timeout) {
        Ok(output) => Some(output),
        Err(err) => {
            log::debug!("{err:#}");
            None
        }
    }
}

#[derive(Debug, Deserialize)]
struct MiInfo {
    chip: String,
}

fn chip_from_mi_info(stdout: &str) -> Option<Chip> {
    let info: MiInfo = serde_json::from_str(stdout.trim()).ok()?;
    chip_from_tool_name(&info.chip)
}

fn chip_from_board_info(text: &str) -> Option<Chip> {
    for line in text.lines() {
        let Some(rest) = line.trim().strip_prefix("Chip type:") else {
            continue;
        };
        let name = rest.trim().split_whitespace().next()?;
        return chip_from_tool_name(name);
    }
    None
}

fn chip_from_tool_name(name: &str) -> Option<Chip> {
    let mut normalized = name.trim().to_ascii_lowercase();
    if let Some(stripped) = normalized.strip_suffix("_lp") {
        normalized = stripped.to_string();
    }
    normalized.retain(|c| c != '-');
    Chip::from_str(&normalized).ok()
}

fn parse_probe_list(stdout: &str) -> Vec<ProbeRef> {
    stdout.lines().filter_map(parse_probe_list_line).collect()
}

fn parse_probe_list_line(line: &str) -> Option<ProbeRef> {
    let rest = line.trim().strip_prefix('[')?;
    let (_index, rest) = rest.split_once(']')?;
    let rest = rest.strip_prefix(':')?.trim();

    // `ESP JTAG -- 303a:1001:30:ED:A0:E4:19:20 (EspJtag)`
    if let Some((name, rest)) = rest.split_once(" -- ") {
        let selector = rest
            .rsplit_once(" (")
            .map(|(selector, _)| selector)
            .unwrap_or(rest)
            .trim()
            .to_string();
        if selector.is_empty() {
            return None;
        }
        return Some(ProbeRef {
            name: name.trim().to_string(),
            selector,
        });
    }

    // Older: `ESP JTAG (VID: 303a, PID: 1001, Serial: …, EspJtag)`
    let (name, paren) = rest.rsplit_once(" (")?;
    let paren = paren.strip_suffix(')')?;

    let mut vid = None;
    let mut pid = None;
    let mut serial = None;
    for part in paren.split(", ") {
        if let Some(value) = part.strip_prefix("VID: ") {
            vid = Some(value);
        } else if let Some(value) = part.strip_prefix("PID: ") {
            pid = Some(value);
        } else if let Some(value) = part.strip_prefix("Serial: ") {
            serial = Some(value);
        }
    }
    let vid = vid?;
    let pid = pid?;
    let selector = match serial {
        Some(serial) if !serial.is_empty() => format!("{vid}:{pid}:{serial}"),
        _ => format!("{vid}:{pid}"),
    };
    Some(ProbeRef {
        name: name.to_string(),
        selector,
    })
}
