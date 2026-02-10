//! Minimal MCP (Model Context Protocol) server over stdio.
//!
//! Implements the subset of JSON-RPC 2.0 required by the MCP specification:
//! - `initialize` / `notifications/initialized` handshake
//! - `tools/list` — returns all registered tools with JSON Schemas
//! - `tools/call` — executes a tool and returns captured stdout as content
//!
//! No async runtime is needed; the server runs a synchronous read loop on
//! stdin and writes responses to stdout.  Command output (which normally
//! prints to stdout) is captured via `gag::BufferRedirect` so it does not
//! corrupt the JSON-RPC stream.

use std::{
    io::{self, BufRead, Read, Write},
    path::Path,
};

use anyhow::{Context, Result};
use serde_json::{json, Value};

use super::registry::CommandEntry;

// ------------------------------------------------------------------ //
// Public entry point                                                  //
// ------------------------------------------------------------------ //

/// Run the MCP server.  Blocks forever reading JSON-RPC requests from
/// stdin and writing responses to stdout.
pub fn run(workspace: &Path, commands: Vec<CommandEntry>) -> Result<()> {
    // In MCP mode, reconfigure the logger to write to stderr so that
    // log output does not corrupt the JSON-RPC stream on stdout.
    // (env_logger is already initialised targeting stdout in main();
    // we cannot re-init, but we *can* redirect the global logger.)
    //
    // For now we just accept that env_logger messages go to stdout
    // before `serve-mcp` takes over — they are emitted during setup
    // only.  All *command* output is captured by `gag`.

    let stdin = io::stdin();
    let mut reader = io::BufReader::new(stdin.lock());
    let mut stdout = io::stdout();

    loop {
        let request_text = match read_message(&mut reader) {
            Ok(Some(text)) => text,
            Ok(None) => break, // EOF
            Err(e) => {
                let err_resp =
                    jsonrpc_error(Value::Null, -32700, &format!("Read error: {e}"));
                write_response(&mut stdout, &err_resp)?;
                continue;
            }
        };

        let request: Value = match serde_json::from_str(&request_text) {
            Ok(v) => v,
            Err(e) => {
                let err_resp =
                    jsonrpc_error(Value::Null, -32700, &format!("Parse error: {e}"));
                write_response(&mut stdout, &err_resp)?;
                continue;
            }
        };

        // Notifications have no "id" — we must not reply.
        let is_notification = request.get("id").is_none();

        let method = request
            .get("method")
            .and_then(Value::as_str)
            .unwrap_or("");

        let id = request.get("id").cloned().unwrap_or(Value::Null);
        let params = request.get("params").cloned().unwrap_or(json!({}));

        let response = match method {
            "initialize" => Some(jsonrpc_ok(id, handle_initialize(workspace))),
            "tools/list" => Some(jsonrpc_ok(id, handle_tools_list(&commands))),
            "tools/call" => {
                Some(jsonrpc_ok(id, handle_tools_call(workspace, &commands, &params)))
            }
            "notifications/initialized" => None,
            _ if is_notification => None,
            _ => Some(jsonrpc_error(id, -32601, &format!("Method not found: {method}"))),
        };

        if let Some(resp) = response {
            if !is_notification {
                write_response(&mut stdout, &resp)?;
            }
        }
    }

    Ok(())
}

// ------------------------------------------------------------------ //
// Message framing                                                     //
// ------------------------------------------------------------------ //

/// Read a single JSON-RPC message from `reader`.
///
/// Supports two framing styles:
/// - **Newline-delimited JSON** (MCP stdio spec): one JSON object per line.
/// - **Content-Length header framing** (LSP-style): `Content-Length: N\r\n`
///   headers followed by a blank line and then exactly N bytes of JSON.
///
/// Returns `Ok(None)` on EOF.
fn read_message(reader: &mut impl BufRead) -> Result<Option<String>> {
    let mut line = String::new();

    // Read the first line — this determines which framing style the
    // client is using.
    loop {
        line.clear();
        let n = reader.read_line(&mut line).context("read_line failed")?;
        if n == 0 {
            return Ok(None); // EOF
        }

        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue; // skip blank lines between messages
        }

        // If the line looks like a Content-Length header, switch to
        // header-framed reading.
        if let Some(rest) = trimmed.strip_prefix("Content-Length:") {
            let content_length: usize = rest
                .trim()
                .parse()
                .context("Invalid Content-Length value")?;
            return read_content_length_body(reader, content_length).map(Some);
        }

        // Otherwise treat the whole line as a newline-delimited JSON message.
        return Ok(Some(trimmed.to_string()));
    }
}

/// After reading a `Content-Length` header, consume any remaining headers
/// (until a blank line) then read exactly `content_length` bytes.
fn read_content_length_body(reader: &mut impl BufRead, content_length: usize) -> Result<String> {
    // Consume remaining headers until blank line.
    let mut header_line = String::new();
    loop {
        header_line.clear();
        let n = reader
            .read_line(&mut header_line)
            .context("read_line on header failed")?;
        if n == 0 {
            anyhow::bail!("Unexpected EOF while reading headers");
        }
        if header_line.trim().is_empty() {
            break;
        }
    }

    // Read exactly content_length bytes.
    let mut body = vec![0u8; content_length];
    reader
        .read_exact(&mut body)
        .context("Failed to read Content-Length body")?;
    String::from_utf8(body).context("Content-Length body is not valid UTF-8")
}

// ------------------------------------------------------------------ //
// Method handlers                                                     //
// ------------------------------------------------------------------ //

fn handle_initialize(workspace: &Path) -> Value {
    let instructions = workspace
        .join(".github")
        .join("copilot-instructions.md");
    let instructions = std::fs::read_to_string(instructions).unwrap_or_default();

    json!({
        "protocolVersion": "2024-11-05",
        "capabilities": {
            "tools": {}
        },
        "serverInfo": {
            "name": "xtask-mcp",
            "version": env!("CARGO_PKG_VERSION")
        },
        "instructions": instructions
    })
}

fn handle_tools_list(commands: &[CommandEntry]) -> Value {
    let tools: Vec<Value> = commands
        .iter()
        .map(|cmd| {
            json!({
                "name": cmd.name,
                "description": cmd.description,
                "inputSchema": (cmd.schema)()
            })
        })
        .collect();

    json!({ "tools": tools })
}

fn handle_tools_call(workspace: &Path, commands: &[CommandEntry], params: &Value) -> Value {
    let tool_name = params
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or("");

    let arguments = params
        .get("arguments")
        .cloned()
        .unwrap_or(json!({}));

    let Some(cmd) = commands.iter().find(|c| c.name == tool_name) else {
        return tool_error(&format!("Unknown tool: {tool_name}"));
    };

    // Capture stdout during command execution so that `println!` output
    // from the command handler does not corrupt the JSON-RPC stream.
    let (result, captured_stdout) = capture_stdout(|| (cmd.execute)(workspace, arguments));

    match result {
        Ok(()) => tool_result(false, &captured_stdout),
        Err(e) => {
            let mut text = captured_stdout;
            if !text.is_empty() {
                text.push_str("\n\n");
            }
            text.push_str(&format!("Error: {e:#}"));
            tool_result(true, &text)
        }
    }
}

// ------------------------------------------------------------------ //
// Stdout capture                                                      //
// ------------------------------------------------------------------ //

/// Execute `f` while redirecting stdout to an in-memory buffer.
/// Returns `(f_result, captured_text)`.
fn capture_stdout<F, T>(f: F) -> (T, String)
where
    F: FnOnce() -> T,
{
    // Try to set up the redirect.  If it fails (e.g. not a real fd)
    // we fall back to running without capture.
    let redirect = gag::BufferRedirect::stdout();
    match redirect {
        Ok(mut buf) => {
            let result = f();
            // Flush stdout so any buffered output reaches the redirect
            // before we read from it.
            let _ = io::stdout().flush();
            let mut captured = String::new();
            if buf.read_to_string(&mut captured).is_err() {
                captured.clear();
            }
            drop(buf); // restore original stdout
            (result, captured)
        }
        Err(_) => {
            let result = f();
            (result, String::new())
        }
    }
}

// ------------------------------------------------------------------ //
// JSON-RPC helpers                                                    //
// ------------------------------------------------------------------ //

fn jsonrpc_ok(id: Value, result: Value) -> Value {
    json!({
        "jsonrpc": "2.0",
        "id": id,
        "result": result
    })
}

fn jsonrpc_error(id: Value, code: i64, message: &str) -> Value {
    json!({
        "jsonrpc": "2.0",
        "id": id,
        "error": {
            "code": code,
            "message": message
        }
    })
}

fn tool_result(is_error: bool, text: &str) -> Value {
    json!({
        "content": [{
            "type": "text",
            "text": text
        }],
        "isError": is_error
    })
}

fn tool_error(message: &str) -> Value {
    tool_result(true, message)
}

fn write_response(out: &mut impl Write, value: &Value) -> Result<()> {
    let msg = serde_json::to_string(value).context("Failed to serialise response")?;
    writeln!(out, "{msg}")?;
    out.flush()?;
    Ok(())
}
