use darling::ast::NestedMeta;
use syn::{
    parse::{Parse, ParseBuffer},
    punctuated::Punctuated,
    Token,
};

pub(crate) struct Args {
    pub(crate) meta: Vec<NestedMeta>,
}

impl Parse for Args {
    fn parse(input: &ParseBuffer) -> syn::Result<Self> {
        let meta = Punctuated::<NestedMeta, Token![,]>::parse_terminated(input)?;
        Ok(Args {
            meta: meta.into_iter().collect(),
        })
    }
}

pub(crate) mod main {
    use std::{cell::RefCell, fmt::Display, thread};

    use darling::{export::NestedMeta, FromMeta};
    use proc_macro2::TokenStream;
    use quote::{quote, ToTokens};
    use syn::{ReturnType, Type};

    #[derive(Debug, FromMeta)]
    struct Args {}

    pub fn run(
        args: &[NestedMeta],
        f: syn::ItemFn,
        main: TokenStream,
    ) -> Result<TokenStream, TokenStream> {
        let _args = Args::from_list(args).map_err(|e| e.write_errors())?;

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

        ctxt.check()?;

        let f_body = f.block;
        let out = &f.sig.output;

        let result = quote! {
            #[doc(hidden)]
            #[::embassy_executor::task()]
            async fn __embassy_main(#fargs) #out {
                #f_body
            }

            #[doc(hidden)]
            unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
                ::core::mem::transmute(t)
            }

            #main
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
        pub fn check(self) -> Result<(), TokenStream> {
            let errors = self.errors.borrow_mut().take().unwrap();
            match errors.len() {
                0 => Ok(()),
                _ => Err(to_compile_errors(errors)),
            }
        }
    }

    fn to_compile_errors(errors: Vec<syn::Error>) -> proc_macro2::TokenStream {
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

    pub fn main() -> TokenStream {
        quote! {
            #[esp_hal::entry]
            fn main() -> ! {
                let mut executor = ::esp_hal_embassy::Executor::new();
                let executor = unsafe { __make_static(&mut executor) };
                executor.run(|spawner| {
                    spawner.must_spawn(__embassy_main(spawner));
                })
            }
        }
    }
}
