use std::{cell::RefCell, fmt::Display, thread};

use proc_macro2::{TokenStream, TokenStream as TokenStream2};
use quote::{ToTokens, quote};
use syn::{
    Attribute,
    Meta,
    ReturnType,
    Token,
    Type,
    parse::{Parse, ParseBuffer},
    punctuated::Punctuated,
};

/// Parsed arguments for the `main` macro.
pub struct Args {
    pub(crate) meta: Vec<Meta>,
}

impl Parse for Args {
    fn parse(input: &ParseBuffer) -> syn::Result<Self> {
        let meta = Punctuated::<Meta, Token![,]>::parse_terminated(input)?;
        Ok(Args {
            meta: meta.into_iter().collect(),
        })
    }
}

/// Procedural macro entry point for the async `main` function.
pub fn main(args: TokenStream, item: TokenStream) -> TokenStream {
    let args: Args = crate::unwrap_or_compile_error!(syn::parse2(args));
    let f: syn::ItemFn = crate::unwrap_or_compile_error!(syn::parse2(item));

    run(&args.meta, f, main_fn()).unwrap_or_else(|x| x)
}

/// Expands and validates the async `main` function into a task entry point.
pub fn run(
    _args: &[Meta],
    f: syn::ItemFn,
    main: TokenStream2,
) -> Result<TokenStream2, TokenStream2> {
    let fargs = f.sig.inputs.clone();

    let ctxt = Ctxt::new();

    if f.sig.asyncness.is_none() {
        ctxt.error_spanned_by(&f.sig, "main function must be async");
    }
    if !f.sig.generics.params.is_empty() {
        ctxt.error_spanned_by(&f.sig, "main function must not be generic");
    }
    if f.sig.generics.where_clause.is_some() {
        ctxt.error_spanned_by(&f.sig, "main function must not have `where` clauses");
    }
    if f.sig.abi.is_some() {
        ctxt.error_spanned_by(&f.sig, "main function must not have an ABI qualifier");
    }
    if f.sig.variadic.is_some() {
        ctxt.error_spanned_by(&f.sig, "main function must not be variadic");
    }
    match &f.sig.output {
        ReturnType::Default => {}
        ReturnType::Type(_, ty) => match &**ty {
            Type::Tuple(tuple) if tuple.elems.is_empty() => {}
            Type::Never(_) => {}
            _ => ctxt.error_spanned_by(
                &f.sig,
                "main function must either not return a value, return `()` or return `!`",
            ),
        },
    }

    if fargs.len() != 1 {
        ctxt.error_spanned_by(&f.sig, "main function must have 1 argument: the spawner.");
    }

    let fattrs = f.attrs;
    let lint_attrs: Vec<Attribute> = fattrs
        .clone()
        .into_iter()
        .filter(|item| {
            item.path().is_ident("deny")
                || item.path().is_ident("allow")
                || item.path().is_ident("warn")
        })
        .collect();

    ctxt.check()?;

    let f_body = f.block;
    let out = &f.sig.output;

    let result = quote! {
        #(#lint_attrs)*
        pub(crate) mod __main {
            use super::*;

            #[doc(hidden)]
            #(#fattrs)*
            #[::embassy_executor::task()]
            async fn __embassy_main(#fargs) #out {
                #f_body
            }

            #[doc(hidden)]
            unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
                ::core::mem::transmute(t)
            }

            #(#fattrs)*
            #main
        }
    };

    Ok(result)
}

/// A type to collect errors together and format them.
///
/// Dropping this object will cause a panic. It must be consumed using
/// `check`.
///
/// References can be shared since this type uses run-time exclusive mut
/// checking.
#[derive(Default)]
pub struct Ctxt {
    // The contents will be set to `None` during checking. This is so that checking can be
    // enforced.
    errors: RefCell<Option<Vec<syn::Error>>>,
}

impl Ctxt {
    /// Create a new context object.
    ///
    /// This object contains no errors, but will still trigger a panic if it
    /// is not `check`ed.
    pub fn new() -> Self {
        Ctxt {
            errors: RefCell::new(Some(Vec::new())),
        }
    }

    /// Add an error to the context object with a tokenenizable object.
    ///
    /// The object is used for spanning in error messages.
    pub fn error_spanned_by<A: ToTokens, T: Display>(&self, obj: A, msg: T) {
        self.errors
            .borrow_mut()
            .as_mut()
            .unwrap()
            // Curb monomorphization from generating too many identical methods.
            .push(syn::Error::new_spanned(obj.into_token_stream(), msg));
    }

    /// Consume this object, producing a formatted error string if there are
    /// errors.
    pub fn check(self) -> Result<(), TokenStream2> {
        let errors = self.errors.borrow_mut().take().unwrap();
        match errors.len() {
            0 => Ok(()),
            _ => Err(to_compile_errors(errors)),
        }
    }
}

fn to_compile_errors(errors: Vec<syn::Error>) -> TokenStream2 {
    let compile_errors = errors.iter().map(syn::Error::to_compile_error);
    quote!(#(#compile_errors)*)
}

impl Drop for Ctxt {
    fn drop(&mut self) {
        if !thread::panicking() && self.errors.borrow().is_some() {
            panic!("forgot to check for errors");
        }
    }
}

/// Generates the `main` function that initializes and runs the async executor.
pub fn main_fn() -> TokenStream2 {
    let root = match proc_macro_crate::crate_name("esp-hal") {
        Ok(proc_macro_crate::FoundCrate::Name(ref name)) => quote::format_ident!("{name}"),
        _ => quote::format_ident!("esp_hal"),
    };

    quote! {
        #[#root::main]
        fn main() -> ! {
            let mut executor = ::esp_rtos::embassy::Executor::new();
            let executor = unsafe { __make_static(&mut executor) };
            executor.run(|spawner| {
                spawner.must_spawn(__embassy_main(spawner));
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let result = main(
            quote::quote! {}.into(),
            quote::quote! {
                async fn foo(spawner: Spawner){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                pub (crate) mod __main {
                    use super::*;
                    #[doc(hidden)]
                    #[::embassy_executor::task()]
                    async fn __embassy_main (spawner : Spawner) {
                        { }
                    }

                    #[doc(hidden)]
                    unsafe fn __make_static < T > (t : & mut T) -> & 'static mut T {
                        ::core::mem::transmute(t)
                    }

                    # [esp_hal :: main]
                    fn main () -> ! {
                        let mut executor = ::esp_rtos::embassy::Executor::new();
                        let executor = unsafe { __make_static (& mut executor) };
                        executor . run (| spawner | {
                            spawner.must_spawn(__embassy_main (spawner));
                        })
                    }
                }
            }
            .to_string()
        );
    }
}
