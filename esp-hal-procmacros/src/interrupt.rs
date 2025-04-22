use darling::{FromMeta, ast::NestedMeta};
use proc_macro::{Span, TokenStream};
use proc_macro_crate::{FoundCrate, crate_name};
use proc_macro2::Ident;
use syn::{
    AttrStyle,
    Attribute,
    ItemFn,
    ReturnType,
    Type,
    parse::Error as SynError,
    spanned::Spanned,
};

pub enum WhiteListCaller {
    Interrupt,
}

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
        match crate_name("esp-hal") {
            Ok(FoundCrate::Name(ref name)) => name,
            _ => "crate",
        },
        Span::call_site().into(),
    );

    let priority = match args.priority {
        Some(priority) => {
            quote::quote!( #priority )
        }
        _ => {
            quote::quote! { #root::interrupt::Priority::min() }
        }
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
        return SynError::new(
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

        const _: () = {
            core::assert!(
            match #priority {
                #root::interrupt::Priority::None => false,
                _ => true,
            },
            "Priority::None is not supported");
        };

        #[allow(non_upper_case_globals)]
        #vis const #orig: #root::interrupt::InterruptHandler = #root::interrupt::InterruptHandler::new(#new, #priority);
    )
    .into()
}

pub fn check_attr_whitelist(
    attrs: &[Attribute],
    caller: WhiteListCaller,
) -> Result<(), TokenStream> {
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
            if eq(attr, val) {
                continue 'o;
            }
        }

        let err_str = match caller {
            WhiteListCaller::Interrupt => {
                "this attribute is not allowed on an interrupt handler controlled by esp-hal"
            }
        };

        return Err(SynError::new(attr.span(), err_str)
            .to_compile_error()
            .into());
    }

    Ok(())
}

/// Returns `true` if `attr.path` matches `name`
fn eq(attr: &Attribute, name: &str) -> bool {
    attr.style == AttrStyle::Outer && attr.path().is_ident(name)
}
