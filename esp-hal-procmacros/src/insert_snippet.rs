use std::collections::HashMap;

use proc_macro::TokenStream;
use syn::{
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
    parse_quote,
    punctuated::Punctuated,
    spanned::Spanned,
    token,
};

struct Replacements {
    replacements: HashMap<String, Vec<Attribute>>,
}

impl Replacements {
    fn get(&self, line: &str) -> Option<Vec<Attribute>> {
        let tokens = match line {
            "#{before_snippet}" => vec![syn::parse_quote! {
                #[doc = crate::before_snippet!()]
            }],
            "#{after_snippet}" => vec![syn::parse_quote! {
                #[doc = crate::after_snippet!()]
            }],
            _ => return self.replacements.get(line).cloned(),
        };

        Some(tokens)
    }
}

impl Parse for Replacements {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut replacements = HashMap::new();
        if !input.is_empty() {
            let args = Punctuated::<Replacement, Token![,]>::parse_terminated(input)?;
            for arg in args {
                let replacement = match arg.replacement {
                    ReplacementKind::Literal(expr) => vec![parse_quote! {
                        #[doc = #expr]
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
                                    branches.push(syn::parse_quote! {
                                        #[cfg_attr(#condition, doc = #body)]
                                    });
                                }
                                None => {
                                    branches.push(syn::parse_quote! {
                                        #[cfg_attr(not(any( #(#cfgs),*) ), doc = #body)]
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
            placeholder: format!("#{{{}}}", placeholder.value()),
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

pub(crate) fn insert(attr: TokenStream, input: TokenStream) -> TokenStream {
    let replacements: Replacements = match syn::parse(attr) {
        Ok(replacements) => replacements,
        Err(e) => return e.into_compile_error().into(),
    };

    let mut item: Item = syn::parse(input).expect("failed to parse input");

    let mut replacement_attrs = Vec::new();

    let attrs = item.attrs_mut();
    let mut in_code_block = false;
    for attr in attrs {
        if let Meta::NameValue(MetaNameValue { path, value, .. }) = &attr.meta
            && let Some(ident) = path.get_ident()
            && ident == "doc"
            && let Expr::Lit(lit) = value
            && let Lit::Str(doc) = &lit.lit
        {
            let doc_line = doc.value();
            let doc_line = doc_line.trim();
            if doc_line.starts_with("```") {
                if in_code_block {
                    // Closing code block
                    replacement_attrs.push(attr.clone());
                    in_code_block = false;
                } else {
                    // Opening code block
                    in_code_block = true;
                    replacement_attrs.push(attr.clone());
                }
            } else if let Some(replacement) = replacements.get(&doc_line) {
                replacement_attrs.extend(replacement);
            } else {
                replacement_attrs.push(attr.clone());
            }
        } else {
            replacement_attrs.push(attr.clone());
        }
    }

    *item.attrs_mut() = replacement_attrs;

    quote::quote! { #item }.into()
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
            _ => panic!("Unsupported item type for switch macro"),
        }
    }
}
