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
                    i if i == "reclaimed" => {
                        if !cfg!(test) && !cfg!(feature = "__esp_idf_bootloader") {
                            return syn::Error::new(
                                ident.span(),
                                "`ram(reclaimed)` requires the esp-idf bootloader",
                            )
                            .into_compile_error();
                        }

                        &mut dram2_uninit
                    }
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

    let item: Item = crate::unwrap_or_compile_error!(parse2(input));

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
    fn test_rwtext() {
        let result = ram(
            quote::quote! {}.into(),
            quote::quote! {
                fn foo() {}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe (link_section = ".rwtext")]
                #[inline (never)]
                fn foo () { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_rtc_fast_text() {
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

    #[cfg(feature = "rtc-slow")]
    #[test]
    fn test_rtc_slow_text() {
        let result = ram(
            quote::quote! {
                unstable(rtc_slow)
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
                #[unsafe (link_section = ".rtc_slow.text")]
                #[inline (never)]
                fn foo () { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_data() {
        let result = ram(
            quote::quote! {}.into(),
            quote::quote! {
                static mut FOO: [u8; 10] = [0; 10];
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe (link_section = ".data")]
                static mut FOO:[u8;10] = [0;10];
            }
            .to_string()
        );
    }

    #[test]
    fn test_rtc_fast_data() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast)
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
                #[unsafe (link_section = ".rtc_fast.data")]
                static mut FOO:[u8;10] = [0;10];
            }
            .to_string()
        );
    }

    #[cfg(feature = "rtc-slow")]
    #[test]
    fn test_rtc_slow_data() {
        let result = ram(
            quote::quote! {
                unstable(rtc_slow)
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
                #[unsafe (link_section = ".rtc_slow.data")]
                static mut FOO:[u8;10] = [0;10];
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

    #[test]
    fn test_rtc_fast_data_zeroed() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast,zeroed)
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
                #[unsafe (link_section = ".rtc_fast.bss")]
                static mut FOO:[u8;10] = [0;10];
                const _: () = crate::__macro_implementation::assert_is_zeroable::<[u8; 10]>();
            }
            .to_string()
        );
    }

    #[test]
    fn test_rtc_fast_data_persistent() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast,persistent)
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
                #[unsafe (link_section = ".rtc_fast.persistent")]
                static mut FOO:[u8;10] = [0;10];
                const _: () = crate::__macro_implementation::assert_is_persistable::<[u8; 10]>();
            }
            .to_string()
        );
    }

    #[cfg(feature = "rtc-slow")]
    #[test]
    fn test_rtc_slow_data_zeroed() {
        let result = ram(
            quote::quote! {
                unstable(rtc_slow,zeroed)
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
                #[unsafe (link_section = ".rtc_slow.bss")]
                static mut FOO:[u8;10] = [0;10];
                const _: () = crate::__macro_implementation::assert_is_zeroable::<[u8; 10]>();
            }
            .to_string()
        );
    }

    #[cfg(feature = "rtc-slow")]
    #[test]
    fn test_rtc_slow_data_persistent() {
        let result = ram(
            quote::quote! {
                unstable(rtc_slow,persistent)
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
                #[unsafe (link_section = ".rtc_slow.persistent")]
                static mut FOO:[u8;10] = [0;10];
                const _: () = crate::__macro_implementation::assert_is_persistable::<[u8; 10]>();
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg() {
        let result = ram(
            quote::quote! {
                test()
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
                ::core::compile_error!{ "Unsupported attribute syntax for `ram`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg2() {
        let result = ram(
            quote::quote! {
                unstable(unstable(unstable))
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
                ::core::compile_error!{ "Expected identifiers inside `unstable(...)`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg3() {
        let result = ram(
            quote::quote! {
                unstable(unknown)
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
                ::core::compile_error!{ "Unknown unstable argument `unknown`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg4() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast,rtc_fast)
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
                ::core::compile_error!{ "Argument `rtc_fast` is already set" }
            }
            .to_string()
        );
    }

    #[cfg(feature = "rtc-slow")]
    #[test]
    fn test_illegal_arg5() {
        let result = ram(
            quote::quote! {
                unstable(rtc_slow,rtc_fast)
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
                ::core::compile_error!{ "Invalid combination of ram arguments" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg6() {
        let result = ram(
            quote::quote! {
                rtc_fast
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
                ::core::compile_error!{ "`rtc_fast` must be wrapped in `unstable(...)`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_rtc_fast_data_persistent_on_local_var() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast,persistent)
            }
            .into(),
            quote::quote! {
                mut foo: [u8; 10] = [0; 10];
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "expected one of: `fn`, `extern`, `use`, `static`, `const`, `unsafe`, `mod`, `type`, `struct`, `enum`, `union`, `trait`, `auto`, `impl`, `default`, `macro`, identifier, `self`, `super`, `crate`, `::`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_rtc_fast_data_persistent_on_non_static() {
        let result = ram(
            quote::quote! {
                unstable(rtc_fast,persistent)
            }
            .into(),
            quote::quote! {
                struct Foo {}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[unsafe(link_section = ".rtc_fast.persistent")]
                struct Foo { }
                ::core::compile_error!{ "Expected a `static`" }
            }
            .to_string()
        );
    }
}
