//! ## Overview
//!
//! Procedural macros for use with the `esp-hal` family of HAL packages. In
//! general, you should not need to depend on this package directly, as the
//! relevant procmacros are re-exported by the various HAL packages.
//!
//! Provides macros for:
//!
//! - Placing statics and functions into RAM
//! - Marking interrupt handlers
//! - Automatically creating an `embassy` executor instance and spawning the
//!   defined entry point
//!
//! These macros offer developers a convenient way to control memory placement
//! and define interrupt handlers in their embedded applications, allowing for
//! optimized memory usage and precise handling of hardware interrupts.
//!
//! Key Components:
//!  - [interrupt](attr.interrupt.html) - Attribute macro for marking interrupt
//!    handlers. Interrupt handlers are used to handle specific hardware
//!    interrupts generated by peripherals.<br> The macro allows users to
//!    specify the interrupt name explicitly or use the function name to match
//!    the interrupt.
//!  - [main](attr.main.html) - Creates a new `executor`` instance and declares
//!    an application entry point spawning the corresponding function body as an
//!    async task.
//!  - [ram](attr.ram.html) - Attribute macro for placing statics and functions
//!    into specific memory sections, such as SRAM or RTC RAM (slow or fast)
//!    with different initialization options. Supported options are:
//!      - `rtc_fast` - Use RTC fast RAM
//!      - `rtc_slow` - Use RTC slow RAM (not all targets support slow RTC RAM)
//!      - `uninitialized` - Skip initialization of the memory
//!      - `zeroed` - Initialize the memory to zero
//!
//! ## Examples
//!
//! #### `interrupt` macro
//!
//! Requires the `interrupt` feature to be enabled.
//!
//! ```no_run
//! #[interrupt]
//! fn INTR_NAME() {
//!     // Interrupt handling code here
//! }
//! ```
//!
//! #### `main` macro
//!
//! Requires the `embassy` feature to be enabled.
//!
//! ```no_run
//! #[main]
//! async fn main(spawner: Spawner) {
//!     // Your application's entry point
//! }
//! ```
//!
//! #### `ram` macro
//!
//! Requires the `ram` feature to be enabled.
//!
//! ```no_run
//! #[ram(rtc_fast)]
//! static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];
//!
//! #[ram(rtc_fast, uninitialized)]
//! static mut SOME_UNINITED_DATA: [u8; 2] = [0; 2];
//!
//! #[ram(rtc_fast, zeroed)]
//! static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];
//! ```

#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

#[cfg(feature = "ram")]
use darling::{ast::NestedMeta, Error as DarlingError, FromMeta};
use proc_macro::TokenStream;
use proc_macro_error::proc_macro_error;
use quote::quote;
use syn::parse_macro_input;

#[cfg(feature = "embassy")]
mod embassy;
#[cfg(feature = "enum-dispatch")]
mod enum_dispatch;
#[cfg(feature = "interrupt")]
mod interrupt;
#[cfg(any(
    feature = "esp32c6-lp",
    feature = "esp32s2-ulp",
    feature = "esp32s3-ulp"
))]
mod lp_core;

#[cfg(any(
    feature = "esp32c6",
    feature = "esp32s2",
    feature = "esp32s3",
    feature = "interrupt"
))]
fn get_hal_crate() -> (
    Result<proc_macro_crate::FoundCrate, proc_macro_crate::Error>,
    proc_macro2::Ident,
) {
    use proc_macro::Span;
    use proc_macro2::Ident;
    use proc_macro_crate::crate_name;

    // Package name:
    #[cfg(feature = "esp32")]
    let hal_crate = crate_name("esp32-hal");
    #[cfg(feature = "esp32c2")]
    let hal_crate = crate_name("esp32c2-hal");
    #[cfg(feature = "esp32c3")]
    let hal_crate = crate_name("esp32c3-hal");
    #[cfg(feature = "esp32c6")]
    let hal_crate = crate_name("esp32c6-hal");
    #[cfg(feature = "esp32c6-lp")]
    let hal_crate = crate_name("esp32c6-lp-hal");
    #[cfg(feature = "esp32h2")]
    let hal_crate = crate_name("esp32h2-hal");
    #[cfg(feature = "esp32p4")]
    let hal_crate = crate_name("esp32p4-hal");
    #[cfg(feature = "esp32s2")]
    let hal_crate = crate_name("esp32s2-hal");
    #[cfg(feature = "esp32s2-ulp")]
    let hal_crate = crate_name("esp-ulp-riscv-hal");
    #[cfg(feature = "esp32s3")]
    let hal_crate = crate_name("esp32s3-hal");
    #[cfg(feature = "esp32s3-ulp")]
    let hal_crate = crate_name("esp-ulp-riscv-hal");

    // Crate name:
    #[cfg(feature = "esp32")]
    let hal_crate_name = Ident::new("esp32_hal", Span::call_site().into());
    #[cfg(feature = "esp32c2")]
    let hal_crate_name = Ident::new("esp32c2_hal", Span::call_site().into());
    #[cfg(feature = "esp32c3")]
    let hal_crate_name = Ident::new("esp32c3_hal", Span::call_site().into());
    #[cfg(feature = "esp32c6")]
    let hal_crate_name = Ident::new("esp32c6_hal", Span::call_site().into());
    #[cfg(feature = "esp32c6-lp")]
    let hal_crate_name = Ident::new("esp32c6_lp_hal", Span::call_site().into());
    #[cfg(feature = "esp32h2")]
    let hal_crate_name = Ident::new("esp32h2_hal", Span::call_site().into());
    #[cfg(feature = "esp32p4")]
    let hal_crate_name = Ident::new("esp32p4_hal", Span::call_site().into());
    #[cfg(feature = "esp32s2")]
    let hal_crate_name = Ident::new("esp32s2_hal", Span::call_site().into());
    #[cfg(feature = "esp32s2-ulp")]
    let hal_crate_name = Ident::new("esp_ulp_riscv_hal", Span::call_site().into());
    #[cfg(feature = "esp32s3")]
    let hal_crate_name = Ident::new("esp32s3_hal", Span::call_site().into());
    #[cfg(feature = "esp32s3-ulp")]
    let hal_crate_name = Ident::new("esp_ulp_riscv_hal", Span::call_site().into());

    (hal_crate, hal_crate_name)
}

#[cfg(feature = "ram")]
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
#[cfg(feature = "ram")]
#[proc_macro_attribute]
#[proc_macro_error]
pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    use proc_macro::Span;
    use proc_macro_error::abort;

    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(DarlingError::from(e).write_errors());
        }
    };

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
    use std::iter;

    use proc_macro::Span;
    use proc_macro2::Ident;
    use proc_macro_crate::FoundCrate;
    use proc_macro_error::abort;
    use syn::{
        parse::Error as ParseError,
        spanned::Spanned,
        ItemFn,
        Meta,
        ReturnType,
        Type,
        Visibility,
    };

    use self::interrupt::{check_attr_whitelist, extract_cfgs, WhiteListCaller};

    let mut f: ItemFn = syn::parse(input).expect("`#[interrupt]` must be applied to a function");

    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(darling::Error::from(e).write_errors());
        }
    };

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
            NestedMeta::Meta(Meta::Path(x)) => {
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
        return ParseError::new(
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

    let (hal_crate, hal_crate_name) = get_hal_crate();

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

    f.block.stmts.extend(iter::once(
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
#[proc_macro_attribute]
pub fn handler(args: TokenStream, input: TokenStream) -> TokenStream {
    use proc_macro::Span;
    use proc_macro_error::abort;
    use syn::{parse::Error as ParseError, spanned::Spanned, ItemFn, ReturnType, Type};

    use self::interrupt::{check_attr_whitelist, WhiteListCaller};

    let mut f: ItemFn = syn::parse(input).expect("`#[handler]` must be applied to a function");

    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(darling::Error::from(e).write_errors());
        }
    };

    if attr_args.len() > 0 {
        abort!(Span::call_site(), "This attribute accepts no arguments")
    }

    // XXX should we blacklist other attributes?

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::Interrupt) {
        return error;
    }

    let valid_signature = f.sig.constness.is_none()
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
        return ParseError::new(
            f.span(),
            "`#[handler]` handlers must have signature `[unsafe] fn([&mut Context]) [-> !]`",
        )
        .to_compile_error()
        .into();
    }

    f.sig.abi = syn::parse_quote!(extern "C");

    quote!(
        #f
    )
    .into()
}

/// Create an enum for erased GPIO pins, using the enum-dispatch pattern
///
/// Only used internally
#[cfg(feature = "enum-dispatch")]
#[proc_macro]
pub fn make_gpio_enum_dispatch_macro(input: TokenStream) -> TokenStream {
    use quote::format_ident;

    use self::enum_dispatch::MakeGpioEnumDispatchMacro;

    let input = parse_macro_input!(input as MakeGpioEnumDispatchMacro);

    let mut arms = Vec::new();
    for (gpio_type, num) in input.elements {
        let enum_name = format_ident!("ErasedPin");
        let variant_name = format_ident!("Gpio{}", num);

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

    let macro_name = format_ident!("{}", input.name);

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

/// Load code to be run on the LP/ULP core.
///
/// ## Example
/// ```no_run
/// let lp_core_code = load_lp_code!("path.elf");
/// lp_core_code.run(&mut lp_core, lp_core::LpCoreWakeupSource::HpCpu, lp_pin);
/// ````
#[cfg(any(feature = "esp32c6", feature = "esp32s2", feature = "esp32s3"))]
#[proc_macro]
pub fn load_lp_code(input: TokenStream) -> TokenStream {
    use std::{fs, path};

    use object::{Object, ObjectSection, ObjectSymbol};
    use proc_macro::Span;
    use proc_macro_crate::FoundCrate;
    use syn::{parse, Ident};

    let (hal_crate, hal_crate_name) = get_hal_crate();

    let hal_crate = match hal_crate {
        Ok(FoundCrate::Itself) => {
            quote!( #hal_crate_name )
        }
        Ok(FoundCrate::Name(ref name)) => {
            let ident = Ident::new(&name, Span::call_site().into());
            quote!( #ident )
        }
        Err(_) => {
            quote!(crate)
        }
    };

    let first_token = match input.into_iter().next() {
        Some(token) => token,
        None => {
            return parse::Error::new(
                Span::call_site().into(),
                "You need to give the path to an ELF file",
            )
            .to_compile_error()
            .into();
        }
    };
    let arg = match litrs::StringLit::try_from(&first_token) {
        Ok(arg) => arg,
        Err(_) => {
            return parse::Error::new(
                Span::call_site().into(),
                "You need to give the path to an ELF file",
            )
            .to_compile_error()
            .into();
        }
    };
    let elf_file = arg.value();

    if !path::Path::new(elf_file).exists() {
        return parse::Error::new(Span::call_site().into(), "File not found")
            .to_compile_error()
            .into();
    }

    let bin_data = fs::read(elf_file).unwrap();
    let obj_file = object::File::parse(&*bin_data).unwrap();
    let sections = obj_file.sections();

    let mut sections: Vec<object::Section> = sections
        .into_iter()
        .filter(|section| match section.kind() {
            object::SectionKind::Text
            | object::SectionKind::ReadOnlyData
            | object::SectionKind::Data
            | object::SectionKind::UninitializedData => true,
            _ => false,
        })
        .collect();
    sections.sort_by(|a, b| a.address().partial_cmp(&b.address()).unwrap());

    let mut binary: Vec<u8> = Vec::new();
    #[cfg(feature = "esp32c6")]
    let mut last_address = 0x50_000_000;
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    let mut last_address = 0x0;

    for section in sections {
        if section.address() > last_address {
            for _ in 0..(section.address() - last_address) {
                binary.push(0);
            }
        }

        binary.extend_from_slice(section.data().unwrap());
        last_address = section.address() + section.size();
    }

    let magic_symbol = obj_file
        .symbols()
        .find(|s| s.name().unwrap().starts_with("__ULP_MAGIC_"));

    if let None = magic_symbol {
        return parse::Error::new(
            Span::call_site().into(),
            "Given file doesn't seem to be an LP/ULP core application.",
        )
        .to_compile_error()
        .into();
    }

    let magic_symbol = magic_symbol.unwrap().name().unwrap();

    let magic_symbol = magic_symbol.trim_start_matches("__ULP_MAGIC_");
    let args: Vec<proc_macro2::TokenStream> = magic_symbol
        .split("$")
        .into_iter()
        .map(|t| {
            let t = t.replace("GpioPin", "LowPowerPin");
            t.parse().unwrap()
        })
        .filter(|v: &proc_macro2::TokenStream| !v.is_empty())
        .collect();

    #[cfg(feature = "esp32c6")]
    let imports = quote! {
        use #hal_crate::lp_core::LpCore;
        use #hal_crate::lp_core::LpCoreWakeupSource;
        use #hal_crate::gpio::lp_gpio::LowPowerPin;
        use #hal_crate::gpio::*;
        use #hal_crate::uart::lp_uart::LpUart;
    };
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    let imports = quote! {
        use #hal_crate::ulp_core::UlpCore as LpCore;
        use #hal_crate::ulp_core::UlpCoreWakeupSource as LpCoreWakeupSource;
        use #hal_crate::gpio::*;
    };

    #[cfg(feature = "esp32c6")]
    let rtc_code_start = quote! { _rtc_fast_data_start };
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    let rtc_code_start = quote! { _rtc_slow_data_start };

    quote! {
        {
            #imports

            struct LpCoreCode {
            }

            static LP_CODE: &[u8] = &[#(#binary),*];

            extern "C" {
                static #rtc_code_start: u32;
            }

            unsafe {
                core::ptr::copy_nonoverlapping(LP_CODE as *const _ as *const u8, &#rtc_code_start as *const u32 as *mut u8, LP_CODE.len());
            }

            impl LpCoreCode {
                pub fn run(
                        &self,
                        lp_core: &mut LpCore,
                        wakeup_source: LpCoreWakeupSource,
                        #(_: #args),*
                    ) {
                    lp_core.run(wakeup_source);
                }
            }

            LpCoreCode {
            }
        }
    }
    .into()
}

#[cfg(any(
    feature = "esp32c6-lp",
    feature = "esp32s2-ulp",
    feature = "esp32s3-ulp"
))]
#[proc_macro_error]
#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    use proc_macro2::{Ident, Span};
    use proc_macro_crate::{crate_name, FoundCrate};
    use quote::{format_ident, quote};
    use syn::{parse, parse_macro_input, spanned::Spanned, FnArg, ItemFn};

    use self::lp_core::{extract_pin, get_simplename, make_magic_symbol_name};

    #[cfg(feature = "esp32c6-lp")]
    let found_crate =
        crate_name("esp32c6-lp-hal").expect("esp32c6_lp_hal is present in `Cargo.toml`");
    #[cfg(any(feature = "esp32s2-ulp", feature = "esp32s3-ulp"))]
    let found_crate =
        crate_name("esp-ulp-riscv-hal").expect("esp-ulp-riscv-hal is present in `Cargo.toml`");

    let hal_crate = match found_crate {
        #[cfg(feature = "esp32c6-lp")]
        FoundCrate::Itself => quote!(esp32c6_lp_hal),
        #[cfg(any(feature = "esp32s2-ulp", feature = "esp32s3-ulp"))]
        FoundCrate::Itself => quote!(esp_ulp_riscv_hal),
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
    let mut create_peripheral = Vec::new();

    let mut used_pins: Vec<u8> = Vec::new();

    for (num, arg) in f.sig.inputs.iter().enumerate() {
        let param_name = format_ident!("param{}", num);
        match arg {
            FnArg::Receiver(_) => {
                return parse::Error::new(arg.span(), "invalid argument")
                    .to_compile_error()
                    .into();
            }
            FnArg::Typed(t) => {
                match get_simplename(&t.ty).as_str() {
                    "GpioPin" => {
                        let pin = extract_pin(&t.ty);
                        if used_pins.contains(&pin) {
                            return parse::Error::new(arg.span(), "duplicate pin")
                                .to_compile_error()
                                .into();
                        }
                        used_pins.push(pin);
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::gpio::conjour().unwrap() };
                        ));
                    }
                    "LpUart" => {
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::uart::conjour().unwrap() };
                        ));
                    }
                    _ => {
                        return parse::Error::new(arg.span(), "invalid argument to main")
                            .to_compile_error()
                            .into();
                    }
                }
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
                #create_peripheral;
            )*

            main(#(#param_names),*);
        }
        #f
    )
    .into()
}

/// Creates a new `executor` instance and declares an application entry point
/// spawning the corresponding function body as an async task.
///
/// The following restrictions apply:
///
/// * The function must accept exactly 1 parameter, an
///   `embassy_executor::Spawner` handle that it can use to spawn additional
///   tasks.
/// * The function must be declared `async`.
/// * The function must not use generics.
/// * Only a single `main` task may be declared.
///
/// ## Examples
/// Spawning a task:
///
/// ``` rust
/// #[main]
/// async fn main(_s: embassy_executor::Spawner) {
///     // Function body
/// }
/// ```
#[cfg(all(feature = "embassy"))]
#[proc_macro_attribute]
pub fn main(args: TokenStream, item: TokenStream) -> TokenStream {
    use self::embassy::{
        main::{main, run},
        Args,
    };

    let args = parse_macro_input!(args as Args);
    let f = parse_macro_input!(item as syn::ItemFn);

    run(&args.meta, f, main()).unwrap_or_else(|x| x).into()
}
