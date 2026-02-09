use std::path::Path;

/// A single command entry in the MCP tool registry.
///
/// Each entry maps a tool name to its schema generator and executor.
/// Only commands with entries here are exposed via MCP — this serves
/// as an implicit allow-list.
pub struct CommandEntry {
    /// The MCP tool name (e.g. `"build-examples"`).
    pub name: &'static str,
    /// Human-readable description shown to MCP clients.
    pub description: &'static str,
    /// Execute the command: deserializes `serde_json::Value` into the
    /// command's Args type internally, then calls the handler.
    pub execute: fn(&Path, serde_json::Value) -> anyhow::Result<()>,
    /// Return the JSON Schema for this command's arguments.
    pub schema: fn() -> serde_json::Value,
}

/// Helper macro to construct a [`CommandEntry`] with type-safe
/// deserialization and schema generation.
///
/// # Usage
///
/// ```ignore
/// command!(
///     "build-documentation",
///     "Build documentation for specified chips",
///     BuildDocumentationArgs,
///     |workspace, args| build_documentation(workspace, args)
/// )
/// ```
///
/// The macro generates inner helper functions that:
/// 1. Deserialize a `serde_json::Value` into `$args_ty`
/// 2. Call the provided handler expression
/// 3. Generate a JSON Schema from `$args_ty` via `schemars`
///
/// The resulting expression evaluates to a [`CommandEntry`].
#[macro_export]
macro_rules! command {
    ($name:literal, $desc:literal, $args_ty:ty, |$ws:ident, $args:ident| $body:expr) => {{
        fn __execute(
            $ws: &::std::path::Path,
            __value: ::serde_json::Value,
        ) -> ::anyhow::Result<()> {
            let $args: $args_ty = ::serde_json::from_value(__value)
                .map_err(|e| ::anyhow::anyhow!("Failed to parse arguments: {e}"))?;
            $body
        }

        fn __schema() -> ::serde_json::Value {
            let schema = ::schemars::schema_for!($args_ty);
            ::serde_json::to_value(schema).expect("schema serialization cannot fail")
        }

        $crate::mcp::registry::CommandEntry {
            name: $name,
            description: $desc,
            execute: __execute,
            schema: __schema,
        }
    }};
}
