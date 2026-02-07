//! MCP server entry point.

use std::path::PathBuf;

use anyhow::Result;
use rmcp::{ServiceExt, transport::stdio};

use super::tools::XtaskMcpServer;

/// Run the MCP server over stdio.
pub async fn run_mcp_server(workspace: PathBuf) -> Result<()> {
    // Create the MCP server with the workspace path
    let server = XtaskMcpServer::new(workspace);

    // Create stdio transport
    let transport = stdio();

    // Start serving and wait for completion
    let service = server.serve(transport).await?;

    // Wait for the service to complete
    service.waiting().await?;

    Ok(())
}
