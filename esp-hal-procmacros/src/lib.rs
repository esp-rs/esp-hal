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
//!  - [`interrupt`](attr.interrupt.html) - Attribute macro for marking
//!    interrupt handlers. Interrupt handlers are used to handle specific
//!    hardware interrupts generated by peripherals.
//!
//!    The macro allows users to specify the interrupt name explicitly or use
//!    the function name to match the interrupt.
//!  - [`main`](attr.main.html) - Creates a new `executor` instance and declares
//!    an application entry point spawning the corresponding function body as an
//!    async task.
//!  - [`ram`](attr.ram.html) - Attribute macro for placing statics and
//!    functions into specific memory sections, such as SRAM or RTC RAM (slow or
//!    fast) with different initialization options. See its documentation for
//!    details.
//!
//! ## Examples
//!
//! #### `main` macro
//!
//! Requires the `embassy` feature to be enabled.
//!
//! ```rust, no_run
//! #[main]
//! async fn main(spawner: Spawner) {
//!     // Your application's entry point
//! }
//! ```
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

use darling::{ast::NestedMeta, Error, FromMeta};
use proc_macro::{Span, TokenStream};
use proc_macro2::Ident;
use proc_macro_crate::{crate_name, FoundCrate};
use proc_macro_error2::abort;
use quote::{format_ident, quote};
use syn::{
    parse,
    parse::Error as ParseError,
    spanned::Spanned,
    Data,
    DataStruct,
    GenericArgument,
    Item,
    ItemFn,
    Path,
    PathArguments,
    PathSegment,
    ReturnType,
    Type,
};

use self::interrupt::{check_attr_whitelist, WhiteListCaller};

#[cfg(feature = "embassy")]
mod embassy;
mod interrupt;
#[cfg(any(
    feature = "is-lp-core",
    feature = "is-ulp-core",
    feature = "has-lp-core",
    feature = "has-ulp-core"
))]
mod lp_core;

#[derive(Debug, Default, darling::FromMeta)]
#[darling(default)]
struct RamArgs {
    rtc_fast: bool,
    rtc_slow: bool,
    persistent: bool,
    zeroed: bool,
}

/// Sets which segment of RAM to use for a function or static and how it should
/// be initialized.
///
/// Requires the `ram` feature.
///
/// # Options
///
/// - `rtc_fast`: Use RTC fast RAM.
/// - `rtc_slow`: Use RTC slow RAM. **Note**: not available on all targets.
/// - `persistent`: Persist the contents of the `static` across resets. See [the
///   section below](#persistent) for details.
/// - `zeroed`: Initialize the memory of the `static` to zero. The initializer
///   expression will be discarded. Types used must implement
///   [`bytemuck::Zeroable`].
///
/// Using both `rtc_fast` and `rtc_slow` or `persistent` and `zeroed` together
/// is an error.
///
/// ## `persistent`
///
/// Initialize the memory to zero after the initial boot. Thereafter,
/// initialization is skipped to allow communication across `software_reset()`,
/// deep sleep, watchdog timeouts, etc.
///
/// Types used must implement [`bytemuck::AnyBitPattern`].
///
/// ### Warnings
///
/// - A system-level or lesser reset occurring before the ram has been zeroed
///   *could* skip initialization and start the application with the static
///   filled with random bytes.
/// - There is no way to keep some kinds of resets from happening while updating
///   a persistent static—not even a critical section.
///
/// If these are issues for your application, consider adding a checksum
/// alongside the data.
///
/// # Examples
///
/// ```rust, no_run
/// #[ram(rtc_fast)]
/// static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];
///
/// #[ram(rtc_fast, persistent)]
/// static mut SOME_PERSISTENT_DATA: [u8; 2] = [0; 2];
///
/// #[ram(rtc_fast, zeroed)]
/// static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];
/// ```
///
/// See the `ram` example in the esp-hal repository for a full usage example.
///
/// [`bytemuck::AnyBitPattern`]: https://docs.rs/bytemuck/1.9.0/bytemuck/trait.AnyBitPattern.html
/// [`bytemuck::Zeroable`]: https://docs.rs/bytemuck/1.9.0/bytemuck/trait.Zeroable.html
#[proc_macro_attribute]
#[proc_macro_error2::proc_macro_error]
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
        abort!(
            Span::call_site(),
            "rtc_slow is not available for this target"
        );
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
            #[link_section = #section_name]
            #[inline(never)] // make certain function is not inlined
        },
        (false, Ok(section_name)) => quote::quote! {
            #[link_section = #section_name]
        },
        (_, Err(_)) => {
            abort!(Span::call_site(), "Invalid combination of ram arguments");
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
        use proc_macro_crate::{crate_name, FoundCrate};

        let hal = Ident::new(
            if let Ok(FoundCrate::Name(ref name)) = crate_name("esp-hal") {
                name
            } else {
                "crate"
            },
            Span::call_site().into(),
        );

        let assertion = quote::format_ident!("assert_is_{name}");
        let Item::Static(ref item) = item else {
            abort!(item, "Expected a `static`");
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

/// Mark a function as an interrupt handler.
///
/// Optionally a priority can be specified, e.g. `#[handler(priority =
/// esp_hal::interrupt::Priority::Priority2)]`.
///
/// If no priority is given, `Priority::min()` is assumed
#[proc_macro_attribute]
#[proc_macro_error2::proc_macro_error]
pub fn handler(args: TokenStream, input: TokenStream) -> TokenStream {
    #[derive(Debug, FromMeta)]
    struct MacroArgs {
        priority: Option<syn::Expr>,
    }

    let mut f: ItemFn = syn::parse(input).expect("`#[handler]` must be applied to a function");
    let original_span = f.span();

    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(darling::Error::from(e).write_errors());
        }
    };

    let args = match MacroArgs::from_list(&attr_args) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(e.write_errors());
        }
    };

    let root = Ident::new(
        if let Ok(FoundCrate::Name(ref name)) = crate_name("esp-hal") {
            name
        } else {
            "crate"
        },
        Span::call_site().into(),
    );

    let priority = if let Some(priority) = args.priority {
        quote::quote!( #priority )
    } else {
        quote::quote! { #root::interrupt::Priority::min() }
    };

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

    f.sig.abi = syn::parse_quote_spanned!(original_span => extern "C");
    let orig = f.sig.ident;
    let vis = f.vis.clone();
    f.sig.ident = Ident::new(
        &format!("__esp_hal_internal_{}", orig),
        proc_macro2::Span::call_site(),
    );
    let new = f.sig.ident.clone();

    quote::quote_spanned!(original_span =>
        #f

        #[allow(non_upper_case_globals)]
        #vis const #orig: #root::interrupt::InterruptHandler = #root::interrupt::InterruptHandler::new(#new, #priority);
    )
    .into()
}

/// Load code to be run on the LP/ULP core.
///
/// ## Example
/// ```rust, no_run
/// let lp_core_code = load_lp_code!("path.elf");
/// lp_core_code.run(&mut lp_core, lp_core::LpCoreWakeupSource::HpCpu, lp_pin);
/// ````
#[cfg(any(feature = "has-lp-core", feature = "has-ulp-core"))]
#[proc_macro]
pub fn load_lp_code(input: TokenStream) -> TokenStream {
    lp_core::load_lp_code(input)
}

/// Marks the entry function of a LP core / ULP program.
#[cfg(any(feature = "is-lp-core", feature = "is-ulp-core"))]
#[proc_macro_attribute]
#[proc_macro_error2::proc_macro_error]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    lp_core::entry(args, input)
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
#[cfg(feature = "embassy")]
#[proc_macro_attribute]
pub fn main(args: TokenStream, item: TokenStream) -> TokenStream {
    use self::embassy::{
        main::{main, run},
        Args,
    };

    let args = syn::parse_macro_input!(args as Args);
    let f = syn::parse_macro_input!(item as syn::ItemFn);

    run(&args.meta, f, main()).unwrap_or_else(|x| x).into()
}

/// Automatically implement the [Builder Lite] pattern for a struct.
///
/// This will create an `impl` which contains methods for each field of a
/// struct, allowing users to easily set the values. The generated methods will
/// be the field name prefixed with `with_`, and calls to these methods can be
/// chained as needed.
///
/// ## Example
///
/// ```rust, no_run
/// #[derive(Default)]
/// enum MyEnum {
///     #[default]
///     A,
///     B,
/// }
///
/// #[derive(Default, BuilderLite)]
/// #[non_exhaustive]
/// struct MyStruct {
///     enum_field: MyEnum,
///     bool_field: bool,
///     option_field: Option<i32>,
/// }
///
/// MyStruct::default()
///     .with_enum_field(MyEnum::B)
///     .with_bool_field(true)
///     .with_option_field(-5);
/// ```
///
/// [Builder Lite]: https://matklad.github.io/2022/05/29/builder-lite.html
#[proc_macro_derive(BuilderLite)]
pub fn builder_lite_derive(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let span = input.span();
    let ident = input.ident;

    let mut fns = Vec::new();
    if let Data::Struct(DataStruct { fields, .. }) = &input.data {
        for field in fields {
            let field_ident = field.ident.as_ref().unwrap();
            let field_type = &field.ty;

            let function_ident = format_ident!("with_{}", field_ident);

            let maybe_path_type = extract_type_path(field_type)
                .and_then(|path| extract_option_segment(path))
                .and_then(|path_seg| match path_seg.arguments {
                    PathArguments::AngleBracketed(ref params) => params.args.first(),
                    _ => None,
                })
                .and_then(|generic_arg| match *generic_arg {
                    GenericArgument::Type(ref ty) => Some(ty),
                    _ => None,
                });

            let (field_type, field_assigns) = if let Some(inner_type) = maybe_path_type {
                (inner_type, quote! { Some(#field_ident) })
            } else {
                (field_type, quote! { #field_ident })
            };

            fns.push(quote! {
                #[doc = concat!(" Assign the given value to the `", stringify!(#field_ident) ,"` field.")]
                pub fn #function_ident(mut self, #field_ident: #field_type) -> Self {
                    self.#field_ident = #field_assigns;
                    self
                }
            });
        }
    } else {
        return ParseError::new(
            span,
            "#[derive(Builder)] is only defined for structs, not for enums or unions!",
        )
        .to_compile_error()
        .into();
    }

    let implementation = quote! {
        #[automatically_derived]
        impl #ident {
            #(#fns)*
        }
    };

    implementation.into()
}

// https://stackoverflow.com/a/56264023
fn extract_type_path(ty: &Type) -> Option<&Path> {
    match *ty {
        Type::Path(ref typepath) if typepath.qself.is_none() => Some(&typepath.path),
        _ => None,
    }
}

// https://stackoverflow.com/a/56264023
fn extract_option_segment(path: &Path) -> Option<&PathSegment> {
    let idents_of_path = path.segments.iter().fold(String::new(), |mut acc, v| {
        acc.push_str(&v.ident.to_string());
        acc.push('|');
        acc
    });

    vec!["Option|", "std|option|Option|", "core|option|Option|"]
        .into_iter()
        .find(|s| idents_of_path == *s)
        .and_then(|_| path.segments.last())
}
