//! MCP (Model Context Protocol) server for xtask.
//!
//! This module provides an MCP server that exposes all xtask CLI operations
//! as MCP tools, enabling AI agents to interact with the esp-hal build system.
//!
//! # Usage
//!
//! Start the MCP server:
//! ```bash
//! cargo xtask --features mcp mcp
//! ```
//!
//! The server communicates over stdio using JSON-RPC 2.0.

mod server;
mod tools;

pub use server::run_mcp_server;
