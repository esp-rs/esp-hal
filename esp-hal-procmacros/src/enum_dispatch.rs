use proc_macro2::{Group, TokenStream, TokenTree};
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream, Result},
    Ident,
};

#[derive(Debug)]
pub(crate) struct MakeGpioEnumDispatchMacro {
    pub name: String,
    pub filter: Vec<String>,
    pub elements: Vec<(String, usize)>,
}

impl Parse for MakeGpioEnumDispatchMacro {
    fn parse(input: ParseStream) -> Result<Self> {
        let name = input.parse::<Ident>()?.to_string();
        let filter = input
            .parse::<Group>()?
            .stream()
            .into_iter()
            .map(|v| match v {
                TokenTree::Group(_) => String::new(),
                TokenTree::Ident(ident) => ident.to_string(),
                TokenTree::Punct(_) => String::new(),
                TokenTree::Literal(_) => String::new(),
            })
            .filter(|p| !p.is_empty())
            .collect();

        let mut elements = vec![];

        let stream = input.parse::<Group>()?.stream().into_iter();
        let mut element_name = String::new();
        for v in stream {
            match v {
                TokenTree::Ident(ident) => {
                    element_name = ident.to_string();
                }
                TokenTree::Literal(lit) => {
                    let index = lit.to_string().parse().unwrap();
                    elements.push((element_name.clone(), index));
                }
                _ => (),
            }
        }

        Ok(MakeGpioEnumDispatchMacro {
            name,
            filter,
            elements,
        })
    }
}

pub(crate) fn build_match_arms(input: MakeGpioEnumDispatchMacro) -> Vec<TokenStream> {
    let mut arms = Vec::new();
    for (gpio_type, num) in input.elements {
        let enum_name = format_ident!("ErasedPin");
        let variant_name = format_ident!("Gpio{}", num);

        if input.filter.contains(&gpio_type) {
            arms.push({
                quote! { #enum_name::#variant_name($target) => $body }
            });
        } else {
            arms.push({
                quote! {
                    #[allow(unused)]
                    #enum_name::#variant_name($target) => { panic!("Unsupported") }
                }
            });
        }
    }

    arms
}
