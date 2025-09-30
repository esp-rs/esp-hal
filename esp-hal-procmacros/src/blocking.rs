#![deny(warnings)]

use proc_macro::TokenStream;
use proc_macro2::Span;
use syn::{
    ItemFn,
    parse::{self},
    parse_macro_input,
};

pub fn main(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);

    if f.sig.asyncness.is_some() {
        return parse::Error::new(
            Span::call_site(),
            "If you want to use `async` please use `esp-rtos`'s `#[esp_rtos::main]` macro instead.",
        )
        .to_compile_error()
        .into();
    }

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    let root = match proc_macro_crate::crate_name("esp-hal") {
        Ok(proc_macro_crate::FoundCrate::Name(ref name)) => quote::format_ident!("{name}"),
        _ => quote::format_ident!("esp_hal"),
    };

    quote::quote!(
        #[#root::__macro_implementation::__entry]
        #f
    )
    .into()
}
