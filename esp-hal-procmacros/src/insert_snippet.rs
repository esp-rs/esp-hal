use proc_macro::TokenStream;
use syn::{Expr, Item, Lit, Meta, MetaNameValue};

pub(crate) fn insert(_: TokenStream, input: TokenStream) -> TokenStream {
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
            && doc.value().trim().starts_with("```")
        {
            if in_code_block {
                // Closing code block
                replacement_attrs.push(attr.clone());
                in_code_block = false;
            } else {
                // Opening code block
                in_code_block = true;
                replacement_attrs.push(attr.clone());
                replacement_attrs.push(syn::parse_quote! {
                    #[doc = crate::before_snippet!()]
                });
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
