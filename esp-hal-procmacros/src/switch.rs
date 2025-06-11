use proc_macro::TokenStream;
use syn::{
    Expr,
    Item,
    Meta,
    Token,
    parse::{Parse, ParseStream},
    punctuated::Punctuated,
    spanned::Spanned,
};

struct Branch {
    condition: Option<Meta>,
    body: Expr,
}

impl Parse for Branch {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
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

pub fn enable_doc_switch(_: TokenStream, input: TokenStream) -> TokenStream {
    let mut item: Item = syn::parse(input).expect("failed to parse input");

    let mut replacement_attrs = Vec::new();

    let attrs = item.attrs_mut();
    for attr in attrs {
        match attr.path().get_ident() {
            Some(ident) if ident == "doc_switch" => {
                let branches = attr
                    .parse_args_with(Punctuated::<Branch, Token![,]>::parse_terminated)
                    .expect("failed to parse branches");

                let mut cfgs = Vec::new();

                for branch in branches.iter() {
                    let body = &branch.body;
                    match &branch.condition {
                        Some(Meta::List(cfg)) if cfg.path.is_ident("cfg") => {
                            let condition = &cfg.tokens;

                            cfgs.push(&cfg.tokens);
                            replacement_attrs.push(syn::parse_quote! {
                                #[cfg_attr(#condition, doc = #body)]
                            });
                        }
                        None => {
                            replacement_attrs.push(syn::parse_quote! {
                                #[cfg_attr(not(any( #(#cfgs),*) ), doc = #body)]
                            });
                        }
                        _ => {
                            return syn::Error::new(
                                branch.condition.span(),
                                "Expected a cfg condition or catch-all condition using `_`",
                            )
                            .into_compile_error()
                            .into();
                        }
                    };
                }
            }
            _ => replacement_attrs.push(attr.clone()),
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
