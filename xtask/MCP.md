# MCP Server for esp-hal xtask

The MCP (Model Context Protocol) server exposes xtask CLI operations as tools for AI agents.

## Quick Start

```bash
cargo xmcp  # Start MCP server
```

The server reads instructions from `.github/copilot-instructions.md` and exposes all xtask commands as MCP tools.

## Client Configuration

### VS Code / GitHub Copilot

Create `.vscode/mcp.json`:
```json
{
  "servers": {
    "esp-hal-xtask": {
      "type": "stdio",
      "command": "cargo",
      "args": ["xmcp"],
      "cwd": "${workspaceFolder}"
    }
  }
}
```

### Claude Desktop

Add to `~/.config/claude/config.json` (Linux) or `~/Library/Application Support/Claude/config.json` (macOS):
```json
{
  "mcpServers": {
    "esp-hal-xtask": {
      "command": "cargo",
      "args": ["xtask", "--features", "mcp", "mcp"],
      "cwd": "/path/to/esp-hal"
    }
  }
}
```

## Building from Source

```bash
cargo build -p xtask --features mcp
```

## Troubleshooting

- **Server doesn't start**: Ensure `mcp` feature is enabled
- **Commands fail**: Check you're in the workspace root with required toolchains installed
- **Timeouts**: Use `--chip` and `--packages` flags to scope operations

For full agent instructions and workflow documentation, see `.github/copilot-instructions.md`.
