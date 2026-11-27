use proc_macro2::{Span, TokenStream};
use syn::{Item, Token, parse::Parser, parse2, punctuated::Punctuated, spanned::Spanned};

use crate::ram::hal_crate;

pub fn psram(args: TokenStream, input: TokenStream) -> TokenStream {
    let attr_args = match Punctuated::<syn::Meta, Token![,]>::parse_terminated.parse2(args) {
        Ok(v) => v,
        Err(e) => return e.to_compile_error(),
    };

    let mut zeroed = false;
    for attr_arg in &attr_args {
        match attr_arg {
            syn::Meta::Path(path) if path.is_ident("zeroed") => zeroed = true,
            _ => {
                return syn::Error::new(attr_arg.span(), "Unknown `psram` argument")
                    .into_compile_error();
            }
        }
    }
    if !zeroed {
        return syn::Error::new(
            Span::call_site(),
            "`psram` needs `zeroed`: only zeroed statics can be placed in PSRAM",
        )
        .into_compile_error();
    }

    let item: Item = match parse2(input) {
        Ok(item) => item,
        Err(e) => return e.to_compile_error(),
    };
    let Item::Static(ref static_item) = item else {
        return syn::Error::new(Span::call_site(), "Expected a `static`").into_compile_error();
    };
    let name = static_item.ident.to_string();

    // The compiler emits a static as NOBITS, and rejects an initializer that is not all zero
    // bytes, only when the section name starts with `.bss`.
    let hal = hal_crate();
    quote::quote! {
        #[unsafe(link_section = ::core::concat!(".bss.psram.", ::core::module_path!(), ".", #name))]
        #item
        const _: () = #hal::__macro_implementation::assert_psram_available();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zeroed() {
        let result = psram(
            quote::quote! {
                zeroed
            },
            quote::quote! {
                static mut FOO: [u8; 10] = [0; 10];
            },
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe(link_section = ::core::concat!(".bss.psram.", ::core::module_path!(), ".", "FOO"))]
                static mut FOO:[u8;10] = [0;10];
                const _: () = crate::__macro_implementation::assert_psram_available();
            }
            .to_string()
        );
    }

    #[test]
    fn test_needs_zeroed() {
        let result = psram(
            quote::quote! {},
            quote::quote! {
                static mut FOO: [u8; 10] = [0; 10];
            },
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "`psram` needs `zeroed`: only zeroed statics can be placed in PSRAM" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_unknown_argument() {
        let result = psram(
            quote::quote! {
                persistent
            },
            quote::quote! {
                static mut FOO: [u8; 10] = [0; 10];
            },
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "Unknown `psram` argument" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_fn() {
        let result = psram(
            quote::quote! {
                zeroed
            },
            quote::quote! {
                fn foo() {}
            },
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "Expected a `static`" }
            }
            .to_string()
        );
    }
}
