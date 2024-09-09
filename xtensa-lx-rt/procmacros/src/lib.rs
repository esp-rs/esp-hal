//! Internal implementation details of `xtensa-lx-rt`.
//!
//! Do not use this crate directly.

#![deny(warnings)]

extern crate proc_macro;

use std::collections::HashSet;

use darling::ast::NestedMeta;
use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{
    parse,
    parse_macro_input,
    spanned::Spanned,
    AttrStyle,
    Attribute,
    FnArg,
    Ident,
    Item,
    ItemFn,
    ItemStatic,
    ReturnType,
    StaticMutability,
    Stmt,
    Type,
    Visibility,
};

/// Marks a function as the main function to be called on program start
#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f = parse_macro_input!(input as ItemFn);

    // check the function signature
    let valid_signature = f.sig.constness.is_none()
        && f.vis == Visibility::Inherited
        && f.sig.abi.is_none()
        && f.sig.inputs.is_empty()
        && f.sig.generics.params.is_empty()
        && f.sig.generics.where_clause.is_none()
        && f.sig.variadic.is_none()
        && match f.sig.output {
            ReturnType::Default => false,
            ReturnType::Type(_, ref ty) => match **ty {
                Type::Never(_) => true,
                _ => false,
            },
        };

    if !valid_signature {
        return parse::Error::new(
            f.span(),
            "`#[entry]` function must have signature `[unsafe] fn() -> !`",
        )
        .to_compile_error()
        .into();
    }

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    let (statics, stmts) = match extract_static_muts(f.block.stmts) {
        Err(e) => return e.to_compile_error().into(),
        Ok(x) => x,
    };

    f.sig.ident = Ident::new(
        &format!("__xtensa_lx_rt_{}", f.sig.ident),
        Span::call_site(),
    );
    f.sig.inputs.extend(statics.iter().map(|statik| {
        let ident = &statik.ident;
        let ty = &statik.ty;
        let attrs = &statik.attrs;

        // Note that we use an explicit `'static` lifetime for the entry point
        // arguments. This makes it more flexible, and is sound here, since the
        // entry will not be called again, ever.
        syn::parse::<FnArg>(
            quote!(#[allow(non_snake_case)] #(#attrs)* #ident: &'static mut #ty).into(),
        )
        .unwrap()
    }));
    f.block.stmts = stmts;

    let tramp_ident = Ident::new(&format!("{}_trampoline", f.sig.ident), Span::call_site());
    let ident = &f.sig.ident;

    let resource_args = statics
        .iter()
        .map(|statik| {
            let (ref cfgs, ref attrs) = extract_cfgs(statik.attrs.clone());
            let ident = &statik.ident;
            let ty = &statik.ty;
            let expr = &statik.expr;
            quote! {
                #(#cfgs)*
                {
                    #(#attrs)*
                    static mut #ident: #ty = #expr;
                    &mut #ident
                }
            }
        })
        .collect::<Vec<_>>();

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::Entry) {
        return error;
    }

    let (ref cfgs, ref attrs) = extract_cfgs(f.attrs.clone());

    quote!(
        #(#cfgs)*
        #(#attrs)*
        #[doc(hidden)]
        #[export_name = "main"]
        pub unsafe extern "C" fn #tramp_ident() {
            #ident(
                #(#resource_args),*
            )
        }

        #[doc(hidden)]
        #[allow(clippy::inline_always)]
        #[inline(always)]
        #f
    )
    .into()
}

/// Marks a function as the exception handler
#[proc_macro_attribute]
pub fn exception(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f = parse_macro_input!(input as ItemFn);

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::Exception) {
        return error;
    }

    let valid_signature = f.sig.constness.is_none()
        && f.vis == Visibility::Inherited
        && f.sig.abi.is_none()
        && f.sig.inputs.len() <= 2
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
        };

    if !valid_signature {
        return parse::Error::new(
            f.span(),
            "`#[exception]` handlers must have signature `[unsafe] fn([ExceptionCause[, Context]) [-> !]`",
        )
        .to_compile_error()
        .into();
    }

    let (statics, stmts) = match extract_static_muts(f.block.stmts) {
        Err(e) => return e.to_compile_error().into(),
        Ok(x) => x,
    };

    f.sig.ident = Ident::new(&format!("__xtensa_lx_6_{}", f.sig.ident), Span::call_site());
    f.sig.inputs.extend(statics.iter().map(|statik| {
        let ident = &statik.ident;
        let ty = &statik.ty;
        let attrs = &statik.attrs;
        syn::parse::<FnArg>(quote!(#[allow(non_snake_case)] #(#attrs)* #ident: &mut #ty).into())
            .unwrap()
    }));
    f.block.stmts = stmts;

    let (ref cfgs, ref attrs) = extract_cfgs(f.attrs.clone());

    quote!(
        #(#cfgs)*
        #(#attrs)*
        #[doc(hidden)]
        #[export_name = "__user_exception"]
        #f
    )
    .into()
}

/// Marks a function as the interrupt handler, with optional interrupt level
/// indicated
///
/// When the function is also marked `#[naked]`, it is a low-level interrupt
/// handler: no entry and exit code to store processor state will be generated.
/// The user needs to ensure that all registers which are used are saved and
/// restored and that the proper return instruction is used.
#[proc_macro_attribute]
pub fn interrupt(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f: ItemFn = syn::parse(input).expect("`#[interrupt]` must be applied to a function");

    let attr_args = match NestedMeta::parse_meta_list(args.into()) {
        Ok(v) => v,
        Err(e) => {
            return TokenStream::from(darling::Error::from(e).write_errors());
        }
    };

    if attr_args.len() > 1 {
        return parse::Error::new(
            Span::call_site(),
            "This attribute accepts zero or 1 arguments",
        )
        .to_compile_error()
        .into();
    }

    let mut level = 1;

    if attr_args.len() == 1 {
        match &attr_args[0] {
            NestedMeta::Lit(syn::Lit::Int(lit_int)) => match lit_int.base10_parse::<u32>() {
                Ok(x) => level = x,
                Err(_) => {
                    return parse::Error::new(
                        Span::call_site(),
                        "This attribute accepts an integer attribute",
                    )
                    .to_compile_error()
                    .into()
                }
            },
            _ => {
                return parse::Error::new(
                    Span::call_site(),
                    "This attribute accepts an integer attribute",
                )
                .to_compile_error()
                .into()
            }
        }
    }

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::Interrupt) {
        return error;
    }

    let naked = f.attrs.iter().position(|x| eq(x, "naked")).is_some();

    let ident_s = if naked {
        format!("__naked_level_{}_interrupt", level)
    } else {
        format!("__level_{}_interrupt", level)
    };

    if naked && (level < 2 || level > 7) {
        return parse::Error::new(
            f.span(),
            "`#[naked]` `#[interrupt]` handlers must have interrupt level >=2 and <=7",
        )
        .to_compile_error()
        .into();
    } else if !naked && (level < 1 || level > 7) {
        return parse::Error::new(
            f.span(),
            "`#[interrupt]` handlers must have interrupt level >=1 and <=7",
        )
        .to_compile_error()
        .into();
    }

    let valid_signature = f.sig.constness.is_none()
        && f.vis == Visibility::Inherited
        && f.sig.abi.is_none()
        && ((!naked && f.sig.inputs.len() <= 2) || (naked && f.sig.inputs.len() == 0))
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
        };

    if !valid_signature {
        if naked {
            return parse::Error::new(
                f.span(),
                "`#[naked]` `#[interrupt]` handlers must have signature `[unsafe] fn() [-> !]`",
            )
            .to_compile_error()
            .into();
        } else {
            return parse::Error::new(
                f.span(),
                "`#[interrupt]` handlers must have signature `[unsafe] fn([u32[, Context]]) [-> !]`",
            )
            .to_compile_error()
            .into();
        }
    }

    let (statics, stmts) = match extract_static_muts(f.block.stmts.iter().cloned()) {
        Err(e) => return e.to_compile_error().into(),
        Ok(x) => x,
    };

    let inputs = f.sig.inputs.clone();

    let args = inputs.iter().map(|arg| match arg {
        syn::FnArg::Typed(x) => {
            let pat = &*x.pat;
            quote!(#pat)
        }
        _ => quote!(#arg),
    });

    f.sig.ident = Ident::new(&format!("__xtensa_lx_6_{}", f.sig.ident), Span::call_site());
    f.sig.inputs.extend(statics.iter().map(|statik| {
        let ident = &statik.ident;
        let ty = &statik.ty;
        let attrs = &statik.attrs;
        syn::parse::<FnArg>(quote!(#[allow(non_snake_case)] #(#attrs)* #ident: &mut #ty).into())
            .unwrap()
    }));
    f.block.stmts = stmts;

    let tramp_ident = Ident::new(&format!("{}_trampoline", f.sig.ident), Span::call_site());
    let ident = &f.sig.ident;

    let resource_args = statics
        .iter()
        .map(|statik| {
            let (ref cfgs, ref attrs) = extract_cfgs(statik.attrs.clone());
            let ident = &statik.ident;
            let ty = &statik.ty;
            let expr = &statik.expr;
            quote! {
                #(#cfgs)*
                {
                    #(#attrs)*
                    static mut #ident: #ty = #expr;
                    &mut #ident
                }
            }
        })
        .collect::<Vec<_>>();

    let (ref cfgs, ref attrs) = extract_cfgs(f.attrs.clone());

    if naked {
        quote!(
            #(#cfgs)*
            #(#attrs)*
            #[doc(hidden)]
            #[export_name = #ident_s]
            pub unsafe extern "C" fn #tramp_ident() {
                #ident(
                    #(#resource_args),*
                )
            }

            #[doc(hidden)]
            #[allow(clippy::inline_always)]
            #[inline(always)]
            #f
        )
        .into()
    } else {
        quote!(
            #(#cfgs)*
            #(#attrs)*
            #[doc(hidden)]
            #[export_name = #ident_s]
            pub unsafe extern "C" fn #tramp_ident(
                level: u32,
                frame: xtensa_lx_rt::exception::Context
            ) {
                    #ident(#(#args),*
                    #(#resource_args),*
                )
            }

            #[doc(hidden)]
            #[allow(clippy::inline_always)]
            #[inline(always)]
            #f
        )
        .into()
    }
}

/// Marks a function as the pre_init function. This function is called before
/// main and *before the memory is initialized*.
#[proc_macro_attribute]
pub fn pre_init(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);

    // check the function signature
    let valid_signature = f.sig.constness.is_none()
        && f.vis == Visibility::Inherited
        && f.sig.unsafety.is_some()
        && f.sig.abi.is_none()
        && f.sig.inputs.is_empty()
        && f.sig.generics.params.is_empty()
        && f.sig.generics.where_clause.is_none()
        && f.sig.variadic.is_none()
        && match f.sig.output {
            ReturnType::Default => true,
            ReturnType::Type(_, ref ty) => match **ty {
                Type::Tuple(ref tuple) => tuple.elems.is_empty(),
                _ => false,
            },
        };

    if !valid_signature {
        return parse::Error::new(
            f.span(),
            "`#[pre_init]` function must have signature `unsafe fn()`",
        )
        .to_compile_error()
        .into();
    }

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    if let Err(error) = check_attr_whitelist(&f.attrs, WhiteListCaller::PreInit) {
        return error;
    }

    let attrs = f.attrs;
    let ident = f.sig.ident;
    let block = f.block;

    quote!(
        #[export_name = "__pre_init"]
        #[allow(missing_docs)]  // we make a private fn public, which can trigger this lint
        #(#attrs)*
        pub unsafe fn #ident() #block
    )
    .into()
}

/// Extracts `static mut` vars from the beginning of the given statements
fn extract_static_muts(
    stmts: impl IntoIterator<Item = Stmt>,
) -> Result<(Vec<ItemStatic>, Vec<Stmt>), parse::Error> {
    let mut istmts = stmts.into_iter();

    let mut seen = HashSet::new();
    let mut statics = vec![];
    let mut stmts = vec![];
    while let Some(stmt) = istmts.next() {
        match stmt {
            Stmt::Item(Item::Static(var)) => match var.mutability {
                StaticMutability::Mut(_) => {
                    if seen.contains(&var.ident) {
                        return Err(parse::Error::new(
                            var.ident.span(),
                            format!("the name `{}` is defined multiple times", var.ident),
                        ));
                    }

                    seen.insert(var.ident.clone());
                    statics.push(var);
                }
                StaticMutability::None => {
                    stmts.push(Stmt::Item(Item::Static(var)));
                }
                _ => unimplemented!(), // `StaticMutability` is `#[non_exhaustive]`
            },
            _ => {
                stmts.push(stmt);
                break;
            }
        }
    }

    stmts.extend(istmts);

    Ok((statics, stmts))
}

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

enum WhiteListCaller {
    Entry,
    Exception,
    Interrupt,
    PreInit,
}

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
    ];

    'o: for attr in attrs {
        for val in whitelist {
            if eq(&attr, &val) {
                continue 'o;
            }
        }

        let err_str = match caller {
            WhiteListCaller::Entry => "this attribute is not allowed on a xtensa-lx-rt entry point",
            WhiteListCaller::Exception => {
                "this attribute is not allowed on an exception handler controlled by xtensa-lx-rt"
            }
            WhiteListCaller::Interrupt => {
                if eq(&attr, "naked") {
                    continue 'o;
                }

                "this attribute is not allowed on an interrupt handler controlled by xtensa-lx-rt"
            }
            WhiteListCaller::PreInit => {
                "this attribute is not allowed on a pre-init controlled by xtensa-lx-rt"
            }
        };

        return Err(parse::Error::new(attr.span(), &err_str)
            .to_compile_error()
            .into());
    }

    Ok(())
}

/// Returns `true` if `attr.path` matches `name`
fn eq(attr: &Attribute, name: &str) -> bool {
    attr.style == AttrStyle::Outer && attr.path().is_ident(name)
}
