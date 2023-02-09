use darling::FromMeta;
use proc_macro::{self, Span, TokenStream};
use proc_macro_error::{abort, proc_macro_error};
use quote::quote;
#[cfg(feature = "interrupt")]
use syn::{
    parse,
    spanned::Spanned,
    AttrStyle,
    Attribute,
    Ident,
    ItemFn,
    Meta::Path,
    ReturnType,
    Type,
    Visibility,
};
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input,
    AttributeArgs,
};

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

/// Marks a function as an interrupt handler
///
/// Used to handle on of the [interrupts](enum.Interrupt.html).
///
/// When specified between braces (`#[interrupt(example)]`) that interrupt will
/// be used and the function can have an arbitrary name. Otherwise the name of
/// the function must be the name of the interrupt.
///
/// Example usage:
///
/// ```rust
/// #[interrupt]
/// fn GPIO() {
///     // code
/// }
/// ```
///
/// The interrupt context can also be supplied by adding a argument to the
/// interrupt function for example, on Xtensa based chips:
///
/// ```rust
/// fn GPIO(context: &mut xtensa_lx_rt::exception::Context) {
///     // code
/// }
/// ```
#[cfg(feature = "interrupt")]
#[proc_macro_attribute]
pub fn interrupt(args: TokenStream, input: TokenStream) -> TokenStream {
    use proc_macro_crate::{crate_name, FoundCrate};

    let mut f: ItemFn = syn::parse(input).expect("`#[interrupt]` must be applied to a function");

    let attr_args = parse_macro_input!(args as AttributeArgs);

    if attr_args.len() > 1 {
        abort!(
            Span::call_site(),
            "This attribute accepts zero or 1 arguments"
        )
    }

    let ident = f.sig.ident.clone();
    let mut ident_s = &ident.clone();

    if attr_args.len() == 1 {
        match &attr_args[0] {
            syn::NestedMeta::Meta(Path(x)) => {
                ident_s = x.get_ident().unwrap();
            }
            _ => {
                abort!(
                    Span::call_site(),
                    format!(
                        "This attribute accepts a string attribute {:?}",
                        attr_args[0]
                    )
                )
            }
        }
    }

    // XXX should we blacklist other attributes?

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::Interrupt) {
        return error;
    }

    let valid_signature = f.sig.constness.is_none()
        && f.vis == Visibility::Inherited
        && f.sig.abi.is_none()
        && f.sig.generics.params.is_empty()
        && f.sig.generics.where_clause.is_none()
        && f.sig.variadic.is_none()
        && match f.sig.output {
            ReturnType::Default => true,
            ReturnType::Type(_, ref ty) => match **ty {
                Type::Tuple(ref tuple) => tuple.elems.is_empty(),
                Type::Never(..) => true,
                _ => false,
            },
        }
        && f.sig.inputs.len() <= 1;

    if !valid_signature {
        return parse::Error::new(
            f.span(),
            "`#[interrupt]` handlers must have signature `[unsafe] fn([&mut Context]) [-> !]`",
        )
        .to_compile_error()
        .into();
    }

    f.sig.ident = Ident::new(
        &format!("__esp_hal_internal_{}", f.sig.ident),
        proc_macro2::Span::call_site(),
    );

    #[cfg(feature = "esp32")]
    let hal_crate = crate_name("esp32-hal");
    #[cfg(feature = "esp32s2")]
    let hal_crate = crate_name("esp32s2-hal");
    #[cfg(feature = "esp32s3")]
    let hal_crate = crate_name("esp32s3-hal");
    #[cfg(feature = "esp32c2")]
    let hal_crate = crate_name("esp32c2-hal");
    #[cfg(feature = "esp32c3")]
    let hal_crate = crate_name("esp32c3-hal");

    #[cfg(feature = "esp32")]
    let hal_crate_name = Ident::new("esp32_hal", Span::call_site().into());
    #[cfg(feature = "esp32s2")]
    let hal_crate_name = Ident::new("esp32s2_hal", Span::call_site().into());
    #[cfg(feature = "esp32s3")]
    let hal_crate_name = Ident::new("esp32s3_hal", Span::call_site().into());
    #[cfg(feature = "esp32c2")]
    let hal_crate_name = Ident::new("esp32c2_hal", Span::call_site().into());
    #[cfg(feature = "esp32c3")]
    let hal_crate_name = Ident::new("esp32c3_hal", Span::call_site().into());

    let interrupt_in_hal_crate = match hal_crate {
        Ok(FoundCrate::Itself) => {
            quote!( #hal_crate_name::peripherals::Interrupt::#ident_s )
        }
        Ok(FoundCrate::Name(ref name)) => {
            let ident = Ident::new(&name, Span::call_site().into());
            quote!( #ident::peripherals::Interrupt::#ident_s )
        }
        Err(_) => {
            quote!( crate::peripherals::Interrupt::#ident_s )
        }
    };

    f.block.stmts.extend(std::iter::once(
        syn::parse2(quote! {{
            // Check that this interrupt actually exists
            #interrupt_in_hal_crate;
        }})
        .unwrap(),
    ));

    let tramp_ident = Ident::new(
        &format!("{}_trampoline", f.sig.ident),
        proc_macro2::Span::call_site(),
    );
    let ident = &f.sig.ident;

    let (ref cfgs, ref attrs) = extract_cfgs(f.attrs.clone());

    let export_name = ident_s.to_string();

    let trap_frame_in_hal_crate = match hal_crate {
        Ok(FoundCrate::Itself) => {
            quote!(#hal_crate_name::trapframe::TrapFrame)
        }
        Ok(FoundCrate::Name(ref name)) => {
            let ident = Ident::new(&name, Span::call_site().into());
            quote!( #ident::trapframe::TrapFrame )
        }
        Err(_) => {
            quote!(crate::trapframe::TrapFrame)
        }
    };

    let context_call =
        (f.sig.inputs.len() == 1).then(|| Ident::new("context", proc_macro2::Span::call_site()));

    quote!(
        macro_rules! foo {
            () => {
            };
        }
        foo!();

        #(#cfgs)*
        #(#attrs)*
        #[doc(hidden)]
        #[export_name = #export_name]
        pub unsafe extern "C" fn #tramp_ident(context: &mut #trap_frame_in_hal_crate) {
            #ident(
                #context_call
            )
        }

        #[inline(always)]
        #f
    )
    .into()
}

#[cfg(feature = "interrupt")]
enum WhiteListCaller {
    Interrupt,
}

#[cfg(feature = "interrupt")]
fn check_attr_whitelist(attrs: &[Attribute], caller: WhiteListCaller) -> Result<(), TokenStream> {
    let whitelist = &[
        "doc",
        "link_section",
        "cfg",
        "allow",
        "warn",
        "deny",
        "forbid",
        "cold",
        "ram",
        "inline",
    ];

    'o: for attr in attrs {
        for val in whitelist {
            if eq(&attr, &val) {
                continue 'o;
            }
        }

        let err_str = match caller {
            WhiteListCaller::Interrupt => {
                "this attribute is not allowed on an interrupt handler controlled by esp-hal"
            }
        };

        return Err(parse::Error::new(attr.span(), &err_str)
            .to_compile_error()
            .into());
    }

    Ok(())
}

/// Returns `true` if `attr.path` matches `name`
#[cfg(feature = "interrupt")]
fn eq(attr: &Attribute, name: &str) -> bool {
    attr.style == AttrStyle::Outer && attr.path.is_ident(name)
}

#[cfg(feature = "interrupt")]
fn extract_cfgs(attrs: Vec<Attribute>) -> (Vec<Attribute>, Vec<Attribute>) {
    let mut cfgs = vec![];
    let mut not_cfgs = vec![];

    for attr in attrs {
        if eq(&attr, "cfg") {
            cfgs.push(attr);
        } else {
            not_cfgs.push(attr);
        }
    }

    (cfgs, not_cfgs)
}

#[derive(Debug)]
struct MakeGpioEnumDispatchMacro {
    name: String,
    filter: Vec<String>,
    elements: Vec<(String, usize)>,
}

impl Parse for MakeGpioEnumDispatchMacro {
    fn parse(input: ParseStream) -> syn::parse::Result<Self> {
        let name = input.parse::<syn::Ident>()?.to_string();
        let filter = input
            .parse::<proc_macro2::Group>()?
            .stream()
            .into_iter()
            .map(|v| match v {
                proc_macro2::TokenTree::Group(_) => String::new(),
                proc_macro2::TokenTree::Ident(ident) => ident.to_string(),
                proc_macro2::TokenTree::Punct(_) => String::new(),
                proc_macro2::TokenTree::Literal(_) => String::new(),
            })
            .filter(|p| !p.is_empty())
            .collect();

        let mut stream = input.parse::<proc_macro2::Group>()?.stream().into_iter();

        let mut elements = vec![];

        let mut element_name = String::new();
        loop {
            match stream.next() {
                Some(v) => match v {
                    proc_macro2::TokenTree::Ident(ident) => {
                        element_name = ident.to_string();
                    }
                    proc_macro2::TokenTree::Literal(lit) => {
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

#[proc_macro]
pub fn make_gpio_enum_dispatch_macro(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as MakeGpioEnumDispatchMacro);

    let mut arms = Vec::new();

    for (gpio_type, num) in input.elements {
        let enum_name = quote::format_ident!("ErasedPin");
        let variant_name = quote::format_ident!("Gpio{}", num);

        if input.filter.contains(&gpio_type) {
            let arm = {
                quote! { #enum_name::#variant_name($target) => $body }
            };
            arms.push(arm);
        } else {
            let arm = {
                quote! {
                    #[allow(unused)]
                    #enum_name::#variant_name($target) => { panic!("Unsupported") }
                }
            };
            arms.push(arm);
        }
    }

    let macro_name = quote::format_ident!("{}", input.name);

    quote! {
        #[doc(hidden)]
        #[macro_export]
        macro_rules! #macro_name {
            ($m:ident, $target:ident, $body:block) => {
                match $m {
                    #(#arms)*
                }
            }
        }

        pub(crate) use #macro_name;
    }
    .into()
}
