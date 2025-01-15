#![deny(warnings)]

use proc_macro::TokenStream;
use proc_macro2::Span;
use syn::{
    parse::{self},
    parse_macro_input,
    ItemFn,
};

pub fn main(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);

    if f.sig.asyncness.is_some() {
        return parse::Error::new(Span::call_site(), "If you want to use `async` please use `esp-hal-embassy`'s `#[esp_hal_embassy::main]` macro instead. (See https://docs.rs/esp-hal-embassy/latest/esp_hal_embassy/)")
            .to_compile_error()
            .into();
    }

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    quote::quote!(
        #[esp_hal::__macro_implementation::__entry]
        #f
    )
    .into()
}
