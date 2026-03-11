use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{
    Attribute, Expr, Ident, ItemStruct, Lit, Meta, Token, Type,
    parse::Parser,
    parse_macro_input,
    punctuated::Punctuated,
};

// ---------------------------------------------------------------------------
// Attribute parsing helpers

/// Parsed information from a `#[arg(...)]` attribute.
struct ArgInfo {
    /// True if `long` (or `long = "name"`) was present.
    has_long: bool,
    /// Explicit long name. Empty string means "derive from field name".
    long_name: String,
    /// `value_delimiter = ','` character, if specified.
    value_delimiter: Option<char>,
}

impl Default for ArgInfo {
    fn default() -> Self {
        Self {
            has_long: false,
            long_name: String::new(),
            value_delimiter: None,
        }
    }
}

/// Parse a string literal from an expression.
fn expr_lit_str(expr: &Expr) -> Option<String> {
    if let Expr::Lit(el) = expr {
        if let Lit::Str(s) = &el.lit {
            return Some(s.value());
        }
    }
    None
}

/// Parse a char literal from an expression.
fn expr_lit_char(expr: &Expr) -> Option<char> {
    if let Expr::Lit(el) = expr {
        if let Lit::Char(c) = &el.lit {
            return Some(c.value());
        }
    }
    None
}

/// Extract and concatenate doc-comment strings from a list of attributes.
fn extract_doc(attrs: &[Attribute]) -> String {
    attrs
        .iter()
        .filter_map(|attr| {
            if !attr.path().is_ident("doc") {
                return None;
            }
            if let Meta::NameValue(nv) = &attr.meta {
                if let Expr::Lit(el) = &nv.value {
                    if let Lit::Str(s) = &el.lit {
                        return Some(s.value().trim().to_string());
                    }
                }
            }
            None
        })
        .filter(|s| !s.is_empty())
        .collect::<Vec<_>>()
        .join(" ")
}

/// Parse all `#[arg(...)]` attributes on a field into a single `ArgInfo`.
fn parse_arg_attrs(attrs: &[Attribute]) -> ArgInfo {
    let mut info = ArgInfo::default();

    for attr in attrs {
        if !attr.path().is_ident("arg") {
            continue;
        }
        let Meta::List(list) = &attr.meta else {
            continue;
        };
        let parser = Punctuated::<Meta, Token![,]>::parse_terminated;
        let Ok(items) = parser.parse2(list.tokens.clone()) else {
            continue;
        };
        for item in &items {
            match item {
                Meta::Path(p) if p.is_ident("long") => {
                    info.has_long = true;
                }
                Meta::NameValue(nv) if nv.path.is_ident("long") => {
                    info.has_long = true;
                    if let Some(s) = expr_lit_str(&nv.value) {
                        info.long_name = s;
                    }
                }
                Meta::NameValue(nv) if nv.path.is_ident("value_delimiter") => {
                    if let Some(c) = expr_lit_char(&nv.value) {
                        info.value_delimiter = Some(c);
                    }
                }
                _ => {}
            }
        }
    }

    info
}

// ---------------------------------------------------------------------------
// Type classification

#[derive(PartialEq)]
enum TypeClass {
    Bool,
    Integer, // usize / u32 / u64 / i64
    Option,
    Vec,
    Other, // enums, String, PathBuf, etc. — treated as required string
}

fn classify_type(ty: &Type) -> TypeClass {
    let Type::Path(tp) = ty else {
        return TypeClass::Other;
    };
    if tp.qself.is_some() {
        return TypeClass::Other;
    }
    let segs = &tp.path.segments;
    if segs.len() != 1 {
        return TypeClass::Other;
    }
    match segs[0].ident.to_string().as_str() {
        "bool" => TypeClass::Bool,
        "usize" | "u32" | "u64" | "i64" | "i32" => TypeClass::Integer,
        "Option" => TypeClass::Option,
        "Vec" => TypeClass::Vec,
        _ => TypeClass::Other,
    }
}

// ---------------------------------------------------------------------------
// Field descriptor

struct FieldDesc {
    /// Rust identifier of the field (for the MCP input struct).
    ident: Ident,
    /// MCP-input field type tokens.
    mcp_ty: TokenStream2,
    /// Doc comment string (may be empty).
    doc: String,
    /// The CLI flag string, e.g. `"--chips"`.
    flag: String,
    /// How to emit CLI args for this field.
    cli_kind: CliKind,
    /// Any `#[cfg(...)]` attributes on the original field, propagated onto
    /// the generated MCP field and CLI push code so feature-gated fields
    /// are only present when the corresponding feature is active.
    cfg_attrs: Vec<TokenStream2>,
}

enum CliKind {
    /// bool flag: emit `--flag` if `Some(true)`.
    BoolFlag,
    /// Optional string value with named flag: `--flag value`.
    NamedOpt,
    /// Positional optional string: emit value if `Some(v)`.
    PositionalOpt,
    /// Vec with value_delimiter: `--flag a,b,c` if `Some(values)`.
    VecDelimited { delimiter: char },
    /// Vec positional with defaults: emit each value if `Some(values)`.
    VecPositional,
    /// Vec named without delimiter: `--flag a --flag b`.
    VecNamedMulti,
    /// Optional integer value with named flag: `--flag N`.
    NamedInt,
    /// Required positional string (no long, no Option).
    RequiredPositional,
}

fn build_field_desc(
    field: &syn::Field,
) -> Option<FieldDesc> {
    let ident = field.ident.as_ref()?.clone();
    let arg = parse_arg_attrs(&field.attrs);
    let doc = extract_doc(&field.attrs);
    let tc = classify_type(&field.ty);

    // Collect any #[cfg(...)] attributes to propagate onto generated code.
    let cfg_attrs: Vec<TokenStream2> = field
        .attrs
        .iter()
        .filter(|a| a.path().is_ident("cfg"))
        .map(|a| quote! { #a })
        .collect();

    // Derive the CLI flag name.
    let flag_name = if arg.long_name.is_empty() {
        ident.to_string().replace('_', "-")
    } else {
        arg.long_name.clone()
    };
    let flag = format!("--{flag_name}");

    let (mcp_ty, cli_kind) = match tc {
        TypeClass::Bool => (
            quote! { Option<bool> },
            CliKind::BoolFlag,
        ),
        TypeClass::Option => {
            if arg.has_long {
                (quote! { Option<String> }, CliKind::NamedOpt)
            } else {
                (quote! { Option<String> }, CliKind::PositionalOpt)
            }
        }
        TypeClass::Vec => {
            if arg.has_long {
                if let Some(delim) = arg.value_delimiter {
                    (
                        quote! { Option<Vec<String>> },
                        CliKind::VecDelimited { delimiter: delim },
                    )
                } else {
                    (quote! { Option<Vec<String>> }, CliKind::VecNamedMulti)
                }
            } else {
                // Positional vec (e.g. `packages` with default_values_t)
                (quote! { Option<Vec<String>> }, CliKind::VecPositional)
            }
        }
        TypeClass::Integer => {
            // Always treated as an optional named arg in MCP.
            (quote! { Option<u64> }, CliKind::NamedInt)
        }
        TypeClass::Other => {
            if arg.has_long {
                // Named required-ish — still optional in MCP for flexibility.
                (quote! { Option<String> }, CliKind::NamedOpt)
            } else {
                // Positional required arg (e.g. `chip: Chip`).
                (quote! { String }, CliKind::RequiredPositional)
            }
        }
    };

    Some(FieldDesc {
        ident,
        mcp_ty,
        doc,
        flag,
        cli_kind,
        cfg_attrs,
    })
}

// ---------------------------------------------------------------------------
// Code generation helpers

fn gen_mcp_field(fd: &FieldDesc) -> TokenStream2 {
    let ident = &fd.ident;
    let ty = &fd.mcp_ty;
    let cfgs = &fd.cfg_attrs;
    if fd.doc.is_empty() {
        quote! {
            #(#cfgs)*
            pub #ident: #ty,
        }
    } else {
        let doc = &fd.doc;
        quote! {
            #(#cfgs)*
            #[doc = #doc]
            pub #ident: #ty,
        }
    }
}

fn gen_cli_push(fd: &FieldDesc) -> TokenStream2 {
    let ident = &fd.ident;
    let flag = &fd.flag;
    let cfgs = &fd.cfg_attrs;

    let body = match &fd.cli_kind {
        CliKind::BoolFlag => quote! {
            if input.#ident.unwrap_or(false) {
                args.push(#flag.to_string());
            }
        },
        CliKind::NamedOpt => quote! {
            if let Some(ref v) = input.#ident {
                args.push(#flag.to_string());
                args.push(v.clone());
            }
        },
        CliKind::PositionalOpt => quote! {
            if let Some(ref v) = input.#ident {
                args.push(v.clone());
            }
        },
        CliKind::VecDelimited { delimiter } => {
            let delim_str = delimiter.to_string();
            quote! {
                if let Some(ref vals) = input.#ident {
                    if !vals.is_empty() {
                        args.push(#flag.to_string());
                        args.push(vals.join(#delim_str));
                    }
                }
            }
        }
        CliKind::VecPositional => quote! {
            if let Some(ref vals) = input.#ident {
                for v in vals {
                    args.push(v.clone());
                }
            }
        },
        CliKind::VecNamedMulti => quote! {
            if let Some(ref vals) = input.#ident {
                for v in vals {
                    args.push(#flag.to_string());
                    args.push(v.clone());
                }
            }
        },
        CliKind::NamedInt => quote! {
            if let Some(n) = input.#ident {
                args.push(#flag.to_string());
                args.push(n.to_string());
            }
        },
        CliKind::RequiredPositional => quote! {
            args.push(input.#ident.clone());
        },
    };

    if cfgs.is_empty() {
        body
    } else {
        quote! {
            #(#cfgs)*
            { #body }
        }
    }
}

// ---------------------------------------------------------------------------
// The `mcp_tool` attribute macro

/// Annotate a Clap `Args` struct to automatically register it as an MCP tool.
///
/// # Usage
///
/// ```ignore
/// #[cfg_attr(feature = "mcp", xtask_mcp_macros::mcp_tool(
///     description = "Short description visible in tool listing",
///     command = "subcommand-name"
/// ))]
/// #[derive(Debug, Args)]
/// struct MyArgs { ... }
/// ```
///
/// The macro generates:
/// 1. A `MyArgsMcpInput` struct (`Deserialize` + `JsonSchema`)
/// 2. A helper function `my_args_mcp_to_cli_args` that converts the input to
///    a `Vec<String>` of CLI arguments.
/// 3. An `inventory::submit!` block that registers the tool.
#[proc_macro_attribute]
pub fn mcp_tool(attrs: TokenStream, input: TokenStream) -> TokenStream {
    let item = parse_macro_input!(input as ItemStruct);
    let description = match parse_mcp_tool_attrs(attrs.into()) {
        Ok(d) => d,
        Err(e) => return e.into_compile_error().into(),
    };
    match expand_mcp_tool(description, &item) {
        Ok(ts) => ts.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

struct McpToolAttrs {
    description: String,
    command: String,
}

fn parse_mcp_tool_attrs(attrs: TokenStream2) -> syn::Result<McpToolAttrs> {
    let parser = Punctuated::<Meta, Token![,]>::parse_terminated;
    let items = parser.parse2(attrs)?;

    let mut description = None;
    let mut command = None;

    for item in &items {
        let Meta::NameValue(nv) = item else {
            continue;
        };
        if nv.path.is_ident("description") {
            description = expr_lit_str(&nv.value);
        } else if nv.path.is_ident("command") {
            command = expr_lit_str(&nv.value);
        }
    }

    let description = description.ok_or_else(|| {
        syn::Error::new(
            proc_macro2::Span::call_site(),
            "mcp_tool requires `description = \"...\"`",
        )
    })?;
    let command = command.ok_or_else(|| {
        syn::Error::new(
            proc_macro2::Span::call_site(),
            "mcp_tool requires `command = \"...\"`",
        )
    })?;

    Ok(McpToolAttrs { description, command })
}

fn expand_mcp_tool(
    attrs: McpToolAttrs,
    item: &ItemStruct,
) -> syn::Result<TokenStream2> {
    let struct_name = &item.ident;
    let input_type_name = format_ident!("{}McpInput", struct_name);

    // Derive the tool name from the command string: spaces/hyphens → underscores.
    let tool_name = attrs.command.replace(' ', "_").replace('-', "_");

    // Split the command string into individual CLI tokens.
    let command_parts: Vec<String> = attrs.command.split_whitespace().map(str::to_string).collect();
    let command_parts_lit = command_parts.iter().map(|p| quote! { #p.to_string(), });

    let syn::Fields::Named(fields) = &item.fields else {
        return Err(syn::Error::new_spanned(struct_name, "mcp_tool only supports structs with named fields"));
    };

    let mut mcp_fields = Vec::new();
    let mut cli_pushes = Vec::new();

    for field in &fields.named {
        let Some(fd) = build_field_desc(field) else {
            continue;
        };
        cli_pushes.push(gen_cli_push(&fd));
        mcp_fields.push(gen_mcp_field(&fd));
    }

    let to_cli_fn = format_ident!("{}_mcp_to_cli_args", to_snake(struct_name.to_string()));
    let schema_fn = format_ident!("{}_mcp_schema", to_snake(struct_name.to_string()));

    let description = &attrs.description;

    let generated = quote! {
        // 1. MCP input type
        #[derive(::serde::Deserialize, ::schemars::JsonSchema)]
        pub struct #input_type_name {
            #(#mcp_fields)*
        }

        // 2. CLI conversion function
        fn #to_cli_fn(input: &#input_type_name) -> Vec<String> {
            let mut args: Vec<String> = vec![#(#command_parts_lit)*];
            #(#cli_pushes)*
            args
        }

        // 3. Schema function (plain fn pointer, not closure)
        fn #schema_fn() -> ::serde_json::Value {
            let schema = ::schemars::schema_for!(#input_type_name);
            ::serde_json::to_value(&schema)
                .expect("schemars Schema serialization cannot fail")
        }

        // 4. Inventory registration
        ::inventory::submit!(crate::McpToolRegistration {
            name: #tool_name,
            description: #description,
            input_schema_fn: #schema_fn,
            execute_fn: |json| {
                let input: #input_type_name = ::serde_json::from_value(json)?;
                let cli_args = #to_cli_fn(&input);
                crate::commands::mcp::run_xtask_subprocess(&cli_args)
            },
        });
    };

    // Emit: original struct unchanged, then the generated code.
    let original = quote! { #item };
    Ok(quote! {
        #original
        #generated
    })
}

/// Convert `MyStructName` → `my_struct_name` (PascalCase → snake_case).
fn to_snake(s: String) -> String {
    let mut out = String::new();
    for (i, ch) in s.chars().enumerate() {
        if ch.is_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(ch.to_lowercase().next().unwrap());
    }
    out
}
