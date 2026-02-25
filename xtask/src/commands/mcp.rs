use std::io::Write as _;

use anyhow::Result;
use rmcp::{
    RoleServer,
    ServerHandler,
    ServiceExt,
    model::{
        CallToolRequestParams,
        CallToolResult,
        Content,
        ErrorData,
        ListToolsResult,
        PaginatedRequestParams,
        ServerCapabilities,
        ServerInfo,
        Tool,
    },
    service::RequestContext,
    transport::stdio,
};
use serde_json::Value;

// ---------------------------------------------------------------------------
// Subprocess helper

/// Spawn the current xtask executable with `args`, feed newlines into its
/// stdin (to silently dismiss any interactive `inquire` prompts), and capture
/// stdout + stderr.
pub fn run_xtask_subprocess(args: &[String]) -> Result<String> {
    let exe = std::env::current_exe()?;
    let mut child = std::process::Command::new(&exe)
        .args(args)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()?;

    // Send newlines then drop to close stdin — this dismisses interactive
    // `inquire` prompts so MCP calls don't hang. Once stdin is dropped the
    // subprocess sees EOF and any remaining prompts fail gracefully.
    if let Some(mut stdin) = child.stdin.take() {
        let _ = stdin.write_all(&b"\n".repeat(20));
        // stdin dropped here, subprocess sees EOF
    }

    let output = child.wait_with_output()?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("STDOUT:\n{stdout}\nSTDERR:\n{stderr}");

    if output.status.success() {
        Ok(combined)
    } else {
        anyhow::bail!("Command failed ({})\n\n{combined}", output.status)
    }
}

// ---------------------------------------------------------------------------
// Schema conversion

/// Convert a `serde_json::Value` (expected to be an Object produced by
/// `schemars::schema_for!`) into the `Arc<JsonObject>` that `Tool::new`
/// expects.
fn value_to_json_object(val: Value) -> serde_json::Map<String, Value> {
    match val {
        Value::Object(m) => m,
        other => {
            log::warn!("Expected JSON object for schema, got: {other}");
            serde_json::Map::new()
        }
    }
}

// ---------------------------------------------------------------------------
// Dynamic MCP server

struct EspHalServer;

impl ServerHandler for EspHalServer {
    fn get_info(&self) -> ServerInfo {
        use strum::IntoEnumIterator;

        let chips: Vec<String> = esp_metadata::Chip::iter().map(|c| c.to_string()).collect();
        let packages: Vec<String> = crate::Package::iter().map(|p| p.to_string()).collect();

        // Read the copilot-instructions file for agent onboarding context.
        let copilot_instructions = std::env::current_dir()
            .ok()
            .map(|ws| ws.join(".github/copilot-instructions.md"))
            .and_then(|path| std::fs::read_to_string(path).ok())
            .unwrap_or_default();

        let instructions = format!(
            "esp-hal xtask automation tools. Use these to build, lint, format, test, \
             and check the esp-hal workspace.\n\n\
             Valid chip values: {}\n\n\
             Valid package values: {}\n\n\
             {copilot_instructions}",
            chips.join(", "),
            packages.join(", "),
        );

        ServerInfo {
            instructions: Some(instructions.into()),
            capabilities: ServerCapabilities::builder()
                .enable_tools()
                .build(),
            ..Default::default()
        }
    }

    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        let tools: Vec<Tool> = inventory::iter::<crate::McpToolRegistration>()
            .map(|r| {
                let schema_val = (r.input_schema_fn)();
                let schema_obj = std::sync::Arc::new(value_to_json_object(schema_val));
                Tool::new(r.name, r.description, schema_obj)
            })
            .collect();
        Ok(ListToolsResult::with_all_items(tools))
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResult, ErrorData> {
        let tool_name = request.name.as_ref();
        let reg = inventory::iter::<crate::McpToolRegistration>()
            .find(|r| r.name == tool_name)
            .ok_or_else(|| {
                ErrorData::invalid_params(
                    format!("Unknown tool: {tool_name}"),
                    None,
                )
            })?;

        let json = Value::Object(request.arguments.unwrap_or_default());
        match (reg.execute_fn)(json) {
            Ok(output) => Ok(CallToolResult::success(vec![Content::text(output)])),
            Err(e) => Ok(CallToolResult::error(vec![Content::text(e.to_string())])),
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point

pub async fn run_mcp_server() -> Result<()> {
    let service = EspHalServer.serve(stdio()).await?;
    service.waiting().await?;
    Ok(())
}
