use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use proc_macro_crate::{crate_name, FoundCrate};
use proc_macro_error::proc_macro_error;
use quote::{format_ident, quote};
use syn::{parse, parse_macro_input, spanned::Spanned, FnArg, ItemFn, PatType};

#[proc_macro_error]
#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    let found_crate =
        crate_name("esp32c6-lp-hal").expect("esp32c6_lp_hal is present in `Cargo.toml`");

    let hal_crate = match found_crate {
        FoundCrate::Itself => quote!(esp32c6_lp_hal),
        FoundCrate::Name(name) => {
            let ident = Ident::new(&name, Span::call_site());
            quote!( #ident::Something )
        }
    };

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    let f = parse_macro_input!(input as ItemFn);

    let mut argument_types = Vec::new();

    let mut used_pins: Vec<u8> = Vec::new();
    for arg in &f.sig.inputs {
        match arg {
            FnArg::Receiver(_) => {
                return parse::Error::new(arg.span(), "invalid argument")
                    .to_compile_error()
                    .into();
            }
            FnArg::Typed(t) => {
                if get_simplename(&t.ty) != "GpioPin" {
                    return parse::Error::new(arg.span(), "invalid argument to main")
                        .to_compile_error()
                        .into();
                }
                let pin = extract_pin(&t.ty);
                if used_pins.contains(&pin) {
                    return parse::Error::new(arg.span(), "duplicate pin")
                        .to_compile_error()
                        .into();
                }
                used_pins.push(pin);

                argument_types.push(t);
            }
        }
    }

    let magic_symbol_name = make_magic_symbol_name(&argument_types);

    let param_names: Vec<Ident> = argument_types
        .into_iter()
        .enumerate()
        .map(|(num, _)| format_ident!("param{}", num))
        .collect();

    quote!(
        #[allow(non_snake_case)]
        #[export_name = "main"]
        pub fn __risc_v_rt__main() -> ! {
            #[export_name = #magic_symbol_name]
            static ULP_MAGIC: [u32; 0] = [0u32; 0];

            unsafe { ULP_MAGIC.as_ptr().read_volatile(); }

            use #hal_crate as the_hal;
            #(
                let mut #param_names = unsafe { the_hal::gpio::conjour().unwrap() };
            )*

            main(#(#param_names),*);
        }
        #f
    )
    .into()
}

fn make_magic_symbol_name(args: &Vec<&PatType>) -> String {
    let mut res = String::from("__ULP_MAGIC_");

    for &a in args {
        let t = &a.ty;
        let quoted = to_string(&t);
        res.push_str(&quoted);
        res.push_str("$");
    }
    res
}

// this is a specialized implementation - won't fit other use-cases
fn to_string(t: &syn::Type) -> String {
    let mut res = String::new();

    match t {
        syn::Type::Path(p) => {
            let segment = p.path.segments.last().unwrap();
            res.push_str(&segment.ident.to_string());
            match &segment.arguments {
                syn::PathArguments::None => (),
                syn::PathArguments::Parenthesized(_) => (),
                syn::PathArguments::AngleBracketed(g) => {
                    res.push_str("<");
                    for arg in &g.args {
                        match arg {
                            syn::GenericArgument::Type(t) => {
                                res.push_str(&to_string(t));
                            }
                            syn::GenericArgument::Const(c) => {
                                res.push_str(",");
                                res.push_str(&quote! { #c }.to_string());
                            }
                            _ => (),
                        }
                    }
                    res.push_str(">");
                }
            }
        }
        _ => (),
    }
    res
}

fn get_simplename(t: &syn::Type) -> String {
    String::from(match t {
        syn::Type::Path(p) => String::from(&p.path.segments.last().unwrap().ident.to_string()),
        _ => String::new(),
    })
}

fn extract_pin(t: &syn::Type) -> u8 {
    let mut res = 255u8;

    match t {
        syn::Type::Path(p) => {
            let segment = p.path.segments.last().unwrap();
            match &segment.arguments {
                syn::PathArguments::None => (),
                syn::PathArguments::Parenthesized(_) => (),
                syn::PathArguments::AngleBracketed(g) => {
                    for arg in &g.args {
                        match arg {
                            syn::GenericArgument::Type(t) => {
                                res = extract_pin(t);
                            }
                            syn::GenericArgument::Const(c) => {
                                res = (&quote! { #c }.to_string()).parse().unwrap();
                            }
                            _ => (),
                        }
                    }
                }
            }
        }
        _ => (),
    }

    res
}
