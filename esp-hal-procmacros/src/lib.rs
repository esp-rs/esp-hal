use darling::FromMeta;
use proc_macro::TokenStream;
use proc_macro::{self, Span};
use proc_macro_error::{abort, proc_macro_error};
use quote::quote;
use syn::{parse_macro_input, AttributeArgs};

#[derive(Debug, Default, FromMeta)]
#[darling(default)]
struct RamArgs {
    rtc_fast: bool,
    rtc_slow: bool,
    uninitialized: bool,
    zeroed: bool,
}

/// This attribute allows placing statics and functions into ram.
///
/// Options that can be specified are rtc_slow or rtc_fast to use the
/// RTC slow or RTC fast ram instead of the normal SRAM.
///
/// The uninitialized option will skip initialization of the memory
/// (e.g. to persist it across resets or deep sleep mode for the RTC RAM)
///
/// Not all targets support RTC slow ram.

#[proc_macro_attribute]
#[proc_macro_error]
pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    let attr_args = parse_macro_input!(args as AttributeArgs);

    let RamArgs {
        rtc_fast,
        rtc_slow,
        uninitialized,
        zeroed,
    } = match FromMeta::from_list(&attr_args) {
        Ok(v) => v,
        Err(e) => {
            return e.write_errors().into();
        }
    };

    let item: syn::Item = syn::parse(input).expect("failed to parse input");

    #[cfg(not(feature = "rtc_slow"))]
    if rtc_slow {
        abort!(
            Span::call_site(),
            "rtc_slow is not available for this target"
        );
    }

    let is_fn = matches!(item, syn::Item::Fn(_));
    let section_name = match (is_fn, rtc_fast, rtc_slow, uninitialized, zeroed) {
        (true, false, false, false, false) => Ok(".rwtext"),
        (true, true, false, false, false) => Ok(".rtc_fast.text"),
        (true, false, true, false, false) => Ok(".rtc_slow.text"),

        (false, false, false, false, false) => Ok(".data"),

        (false, true, false, false, false) => Ok(".rtc_fast.data"),
        (false, true, false, true, false) => Ok(".rtc_fast.noinit"),
        (false, true, false, false, true) => Ok(".rtc_fast.bss"),

        (false, false, true, false, false) => Ok(".rtc_slow.data"),
        (false, false, true, true, false) => Ok(".rtc_slow.noinit"),
        (false, false, true, false, true) => Ok(".rtc_slow.bss"),

        _ => Err(()),
    };

    let section = match (is_fn, section_name) {
        (true, Ok(section_name)) => quote! {
            #[link_section = #section_name]
            #[inline(never)] // make certain function is not inlined
        },
        (false, Ok(section_name)) => quote! {
            #[link_section = #section_name]
        },
        (_, Err(_)) => {
            abort!(Span::call_site(), "Invalid combination of ram arguments");
        }
    };

    let output = quote! {
        #section
        #item
    };
    output.into()
}
