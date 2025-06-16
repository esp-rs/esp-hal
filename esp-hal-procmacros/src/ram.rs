use darling::{Error, FromMeta, ast::NestedMeta};
use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use syn::{Item, parse};

#[derive(Debug, Default, darling::FromMeta)]
#[darling(default)]
struct RamArgs {
    rtc_fast: bool,
    rtc_slow: bool,
    persistent: bool,
    zeroed: bool,
}

pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(Error::from(e).write_errors());
        }
    };

    let RamArgs {
        rtc_fast,
        rtc_slow,
        persistent,
        zeroed,
    } = match FromMeta::from_list(&attr_args) {
        Ok(v) => v,
        Err(e) => {
            return e.write_errors().into();
        }
    };

    let item: Item = parse(input).expect("failed to parse input");

    #[cfg(not(feature = "rtc-slow"))]
    if rtc_slow {
        return syn::Error::new(
            Span::call_site(),
            "rtc_slow is not available for this target",
        )
        .into_compile_error()
        .into();
    }

    let is_fn = matches!(item, Item::Fn(_));
    let section_name = match (is_fn, rtc_fast, rtc_slow, persistent, zeroed) {
        (true, false, false, false, false) => Ok(".rwtext"),
        (true, true, false, false, false) => Ok(".rtc_fast.text"),
        (true, false, true, false, false) => Ok(".rtc_slow.text"),

        (false, false, false, false, false) => Ok(".data"),

        (false, true, false, false, false) => Ok(".rtc_fast.data"),
        (false, true, false, true, false) => Ok(".rtc_fast.persistent"),
        (false, true, false, false, true) => Ok(".rtc_fast.bss"),

        (false, false, true, false, false) => Ok(".rtc_slow.data"),
        (false, false, true, true, false) => Ok(".rtc_slow.persistent"),
        (false, false, true, false, true) => Ok(".rtc_slow.bss"),

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
                .into_compile_error()
                .into();
        }
    };

    let trait_check = if zeroed {
        Some("zeroable")
    } else if persistent {
        Some("persistable")
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

    let output = quote::quote! {
        #section
        #item
        #trait_check
    };

    output.into()
}
