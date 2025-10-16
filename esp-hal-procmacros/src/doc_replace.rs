use std::collections::HashMap;

use proc_macro2::{TokenStream, TokenStream as TokenStream2};
use syn::{
    AttrStyle,
    Attribute,
    Expr,
    Item,
    Lit,
    LitStr,
    Meta,
    MetaNameValue,
    Token,
    braced,
    parse::Parse,
    punctuated::Punctuated,
    spanned::Spanned,
    token,
};

struct Replacements {
    replacements: HashMap<String, Vec<TokenStream2>>,
}

impl Replacements {
    fn get(&self, line: &str, outer: bool) -> Option<Vec<Attribute>> {
        let attributes = match line {
            "# {before_snippet}" if outer => vec![syn::parse_quote! {
                #[doc = crate::before_snippet!()]
            }],
            "# {before_snippet}" => vec![syn::parse_quote! {
                #![doc = crate::before_snippet!()]
            }],
            "# {after_snippet}" if outer => vec![syn::parse_quote! {
                #[doc = crate::after_snippet!()]
            }],
            "# {after_snippet}" => vec![syn::parse_quote! {
                #![doc = crate::after_snippet!()]
            }],
            _ => {
                let lines = self.replacements.get(line).cloned()?;

                let mut attrs = vec![];

                for line in lines {
                    if outer {
                        attrs.push(syn::parse_quote! { #[ #line ] });
                    } else {
                        attrs.push(syn::parse_quote! { #![ #line ] });
                    }
                }

                attrs
            }
        };

        Some(attributes)
    }
}

impl Parse for Replacements {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut replacements = HashMap::new();
        if !input.is_empty() {
            let args = Punctuated::<Replacement, Token![,]>::parse_terminated(input)?;
            for arg in args {
                let replacement = match arg.replacement {
                    ReplacementKind::Literal(expr) => vec![quote::quote! {
                        doc = #expr
                    }],
                    ReplacementKind::Choice(items) => {
                        let mut branches = vec![];
                        let mut cfgs = vec![];

                        for branch in items {
                            let body = branch.body;
                            match branch.condition {
                                Some(Meta::List(cfg)) if cfg.path.is_ident("cfg") => {
                                    let condition = cfg.tokens;

                                    cfgs.push(condition.clone());
                                    branches.push(quote::quote! {
                                        cfg_attr(#condition, doc = #body)
                                    });
                                }
                                None => {
                                    branches.push(quote::quote! {
                                        cfg_attr(not(any( #(#cfgs),*) ), doc = #body)
                                    });
                                }
                                _ => {
                                    return Err(syn::Error::new(
                                        branch.condition.span(),
                                        "Expected a cfg condition or catch-all condition using `_`",
                                    ));
                                }
                            };
                        }

                        branches
                    }
                };

                replacements.insert(arg.placeholder, replacement);
            }
        }

        Ok(Self { replacements })
    }
}

struct Replacement {
    placeholder: String,
    replacement: ReplacementKind,
}

impl Parse for Replacement {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let placeholder: LitStr = input.parse()?;
        let _arrow: Token![=>] = input.parse()?;
        let replacement: ReplacementKind = input.parse()?;

        Ok(Self {
            placeholder: format!("# {{{}}}", placeholder.value()),
            replacement,
        })
    }
}

enum ReplacementKind {
    Literal(Expr),
    Choice(Punctuated<Branch, Token![,]>),
}

impl Parse for ReplacementKind {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        if !input.peek(token::Brace) {
            return input.parse().map(Self::Literal);
        }

        let choices;
        braced!(choices in input);

        let branches = Punctuated::<Branch, Token![,]>::parse_terminated(&choices)?;

        Ok(Self::Choice(branches))
    }
}

struct Branch {
    condition: Option<Meta>,
    body: Expr,
}

impl Parse for Branch {
    fn parse(input: syn::parse::ParseStream<'_>) -> syn::Result<Self> {
        let condition: Option<Meta> = if input.parse::<Token![_]>().is_ok() {
            None
        } else {
            Some(input.parse()?)
        };

        input.parse::<syn::Token![=>]>()?;

        let body: Expr = input.parse()?;

        Ok(Branch { condition, body })
    }
}

pub(crate) fn replace(attr: TokenStream, input: TokenStream) -> TokenStream {
    let replacements: Replacements = match syn::parse2(attr) {
        Ok(replacements) => replacements,
        Err(e) => return e.into_compile_error(),
    };

    let mut item: Item = syn::parse2(input).expect("failed to parse input");

    let mut replacement_attrs = Vec::new();

    let attrs = item.attrs_mut();
    for attr in attrs {
        if let Meta::NameValue(MetaNameValue { path, value, .. }) = &attr.meta
            && let Some(ident) = path.get_ident()
            && ident == "doc"
            && let Expr::Lit(lit) = value
            && let Lit::Str(doc) = &lit.lit
            && let Some(replacement) =
                replacements.get(doc.value().trim(), attr.style == AttrStyle::Outer)
        {
            replacement_attrs.extend(replacement);
        } else {
            replacement_attrs.push(attr.clone());
        }
    }

    *item.attrs_mut() = replacement_attrs;

    quote::quote! { #item }
}

trait ItemLike {
    fn attrs_mut(&mut self) -> &mut Vec<syn::Attribute>;
}

impl ItemLike for Item {
    fn attrs_mut(&mut self) -> &mut Vec<syn::Attribute> {
        match self {
            Item::Fn(item_fn) => &mut item_fn.attrs,
            Item::Struct(item_struct) => &mut item_struct.attrs,
            Item::Enum(item_enum) => &mut item_enum.attrs,
            Item::Trait(item_trait) => &mut item_trait.attrs,
            Item::Mod(module) => &mut module.attrs,
            Item::Macro(item_macro) => &mut item_macro.attrs,
            _ => panic!("Unsupported item type for switch macro"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let result = replace(
            quote::quote! {}.into(),
            quote::quote! {
                #[doc = "# Configuration"]
                #[doc = "## Overview"]
                #[doc = "This module contains the initial configuration for the system."]
                #[doc = "## Configuration"]
                #[doc = "In the [`esp_hal::init()`][crate::init] method, we can configure different"]
                #[doc = "parameters for the system:"]
                #[doc = "- CPU clock configuration."]
                #[doc = "- Watchdog configuration."]
                #[doc = "## Examples"]
                #[doc = "### Default initialization"]
                #[doc = "```rust, no_run"]
                #[doc = "# {before_snippet}"]
                #[doc = "let peripherals = esp_hal::init(esp_hal::Config::default());"]
                #[doc = "# {after_snippet}"]
                #[doc = "```"]
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(result.to_string(), quote::quote! {
            #[doc = "# Configuration"]
            #[doc = "## Overview"]
            #[doc = "This module contains the initial configuration for the system."]
            #[doc = "## Configuration"]
            #[doc = "In the [`esp_hal::init()`][crate::init] method, we can configure different"]
            #[doc = "parameters for the system:"]
            #[doc = "- CPU clock configuration."]
            #[doc = "- Watchdog configuration."]
            #[doc = "## Examples"]
            #[doc = "### Default initialization"]
            #[doc = "```rust, no_run"]
            #[doc = crate::before_snippet!()]
            #[doc = "let peripherals = esp_hal::init(esp_hal::Config::default());"]
            #[doc = crate::after_snippet!()]
            #[doc = "```"]
            struct Foo {}
        }.to_string());
    }

    #[test]
    fn test_custom_replacements() {
        let result = replace(
            quote::quote! {
                "freq" => {
                    cfg(esp32h2) => "let freq = Rate::from_mhz(32);",
                    _ => "let freq = Rate::from_mhz(80);"
                },
            }.into(),
            quote::quote! {
                #[doc = "# Configuration"]
                #[doc = "## Overview"]
                #[doc = "This module contains the initial configuration for the system."]
                #[doc = "## Configuration"]
                #[doc = "In the [`esp_hal::init()`][crate::init] method, we can configure different"]
                #[doc = "parameters for the system:"]
                #[doc = "- CPU clock configuration."]
                #[doc = "- Watchdog configuration."]
                #[doc = "## Examples"]
                #[doc = "### Default initialization"]
                #[doc = "```rust, no_run"]
                #[doc = "# {freq}"]
                #[doc = "# {before_snippet}"]
                #[doc = "let peripherals = esp_hal::init(esp_hal::Config::default());"]
                #[doc = "# {after_snippet}"]
                #[doc = "```"]
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(result.to_string(), quote::quote! {
            #[doc = "# Configuration"]
            #[doc = "## Overview"]
            #[doc = "This module contains the initial configuration for the system."]
            #[doc = "## Configuration"]
            #[doc = "In the [`esp_hal::init()`][crate::init] method, we can configure different"]
            #[doc = "parameters for the system:"]
            #[doc = "- CPU clock configuration."]
            #[doc = "- Watchdog configuration."]
            #[doc = "## Examples"]
            #[doc = "### Default initialization"]
            #[doc = "```rust, no_run"]
            #[cfg_attr (esp32h2 , doc = "let freq = Rate::from_mhz(32);")] 
            #[cfg_attr (not (any (esp32h2)) , doc = "let freq = Rate::from_mhz(80);")]
            #[doc = crate::before_snippet!()]
            #[doc = "let peripherals = esp_hal::init(esp_hal::Config::default());"]
            #[doc = crate::after_snippet!()]
            #[doc = "```"]
            struct Foo {}
        }.to_string());
    }
}
