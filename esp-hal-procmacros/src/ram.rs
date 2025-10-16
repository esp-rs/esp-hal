use proc_macro2::{Ident, Span, TokenStream};
use syn::{Item, Token, parse::Parser, parse2, punctuated::Punctuated, spanned::Spanned};

pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    let attr_args = match Punctuated::<syn::Meta, Token![,]>::parse_terminated.parse2(args) {
        Ok(v) => v,
        Err(e) => return e.to_compile_error(),
    };

    let mut rtc_fast = false;
    let mut rtc_slow = false;
    let mut dram2_uninit = false;
    let mut persistent = false;
    let mut zeroed = false;

    for attr_arg in &attr_args {
        match attr_arg {
            syn::Meta::List(list) if list.path.is_ident("unstable") => {
                let nested = &list.tokens;
                let nested_args = match Punctuated::<syn::Meta, Token![,]>::parse_terminated
                    .parse2(nested.clone())
                {
                    Ok(v) => v,
                    Err(e) => return e.to_compile_error(),
                };

                for meta in nested_args {
                    match meta {
                        syn::Meta::Path(path) => {
                            let Some(ident) = path.get_ident() else {
                                return syn::Error::new(
                                    path.span(),
                                    "Expected identifier inside `unstable(...)`",
                                )
                                .into_compile_error();
                            };
                            let arg = match ident {
                                i if i == "rtc_fast" => &mut rtc_fast,
                                i if i == "rtc_slow" => &mut rtc_slow,
                                i if i == "persistent" => &mut persistent,
                                i if i == "zeroed" => &mut zeroed,
                                i => {
                                    return syn::Error::new(
                                        i.span(),
                                        format!("Unknown unstable argument `{i}`"),
                                    )
                                    .into_compile_error();
                                }
                            };

                            if *arg {
                                return syn::Error::new(
                                    ident.span(),
                                    format!("Argument `{ident}` is already set"),
                                )
                                .into_compile_error();
                            }

                            *arg = true;
                        }
                        _ => {
                            return syn::Error::new(
                                list.span(),
                                "Expected identifiers inside `unstable(...)`",
                            )
                            .into_compile_error();
                        }
                    }
                }
            }

            syn::Meta::Path(path) => {
                let Some(ident) = path.get_ident() else {
                    return syn::Error::new(path.span(), "Expected identifier")
                        .into_compile_error();
                };
                let arg = match ident {
                    i if i == "reclaimed" => &mut dram2_uninit,
                    _ => {
                        return syn::Error::new(
                            ident.span(),
                            format!("`{ident}` must be wrapped in `unstable(...)`"),
                        )
                        .into_compile_error();
                    }
                };

                if *arg {
                    return syn::Error::new(
                        ident.span(),
                        format!("Argument `{ident}` is already set"),
                    )
                    .into_compile_error();
                }
                *arg = true;
            }

            _ => {
                return syn::Error::new(attr_arg.span(), "Unsupported attribute syntax for `ram`")
                    .into_compile_error();
            }
        }
    }

    let item: Item = parse2(input).expect("failed to parse input");

    #[cfg(not(feature = "rtc-slow"))]
    if rtc_slow {
        return syn::Error::new(
            Span::call_site(),
            "rtc_slow is not available for this target",
        )
        .into_compile_error();
    }

    let is_fn = matches!(item, Item::Fn(_));
    let section_name = match (is_fn, rtc_fast, rtc_slow, dram2_uninit, persistent, zeroed) {
        (true, false, false, false, false, false) => Ok(".rwtext"),
        (true, true, false, false, false, false) => Ok(".rtc_fast.text"),
        (true, false, true, false, false, false) => Ok(".rtc_slow.text"),

        (false, false, false, false, false, false) => Ok(".data"),
        (false, false, false, true, false, false) => Ok(".dram2_uninit"),

        (false, true, false, false, false, false) => Ok(".rtc_fast.data"),
        (false, true, false, false, true, false) => Ok(".rtc_fast.persistent"),
        (false, true, false, false, false, true) => Ok(".rtc_fast.bss"),

        (false, false, true, false, false, false) => Ok(".rtc_slow.data"),
        (false, false, true, false, true, false) => Ok(".rtc_slow.persistent"),
        (false, false, true, false, false, true) => Ok(".rtc_slow.bss"),

        _ => Err(()),
    };

    let section = match (is_fn, section_name) {
        (true, Ok(section_name)) => quote::quote! {
            #[unsafe(link_section = #section_name)]
            #[inline(never)] // make certain function is not inlined
        },
        (false, Ok(section_name)) => quote::quote! {
            #[unsafe(link_section = #section_name)]
        },
        (_, Err(_)) => {
            return syn::Error::new(Span::call_site(), "Invalid combination of ram arguments")
                .into_compile_error();
        }
    };

    let trait_check = if zeroed {
        Some("zeroable")
    } else if persistent {
        Some("persistable")
    } else if dram2_uninit {
        Some("uninit")
    } else {
        None
    };
    let trait_check = trait_check.map(|name| {
        use proc_macro_crate::{FoundCrate, crate_name};

        let hal = Ident::new(
            match crate_name("esp-hal") {
                Ok(FoundCrate::Name(ref name)) => name,
                _ => "crate",
            },
            Span::call_site(),
        );

        let assertion = quote::format_ident!("assert_is_{name}");
        let Item::Static(ref item) = item else {
            return syn::Error::new(Span::call_site(), "Expected a `static`").into_compile_error();
        };
        let ty = &item.ty;
        quote::quote! {
            const _: () = #hal::__macro_implementation::#assertion::<#ty>();
        }
    });

    quote::quote! {
        #section
        #item
        #trait_check
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rtc_fast() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast)
            }
            .into(),
            quote::quote! {
                fn foo() {}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe (link_section = ".rtc_fast.text")]
                #[inline (never)]
                fn foo () { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_reclaimed() {
        let result = ram(
            quote::quote! {
                reclaimed
            }
            .into(),
            quote::quote! {
                static mut FOO: [u8; 10] = [0; 10];
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe(link_section = ".dram2_uninit")]
                static mut FOO: [u8;10] = [0;10];
                const _ : () = crate::__macro_implementation::assert_is_uninit::<[u8;10]>();
            }
            .to_string()
        );
    }
}
