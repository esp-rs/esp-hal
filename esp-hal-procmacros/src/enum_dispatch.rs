use proc_macro2::{Group, TokenTree};
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

        let mut stream = input.parse::<Group>()?.stream().into_iter();
        let mut element_name = String::new();
        loop {
            match stream.next() {
                Some(v) => match v {
                    TokenTree::Ident(ident) => {
                        element_name = ident.to_string();
                    }
                    TokenTree::Literal(lit) => {
                        let index = lit.to_string().parse().unwrap();
                        elements.push((element_name.clone(), index));
                    }
                    _ => (),
                },
                None => break,
            }
        }

        Ok(MakeGpioEnumDispatchMacro {
            name,
            filter,
            elements,
        })
    }
}
