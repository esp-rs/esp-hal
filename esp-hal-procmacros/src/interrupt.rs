use proc_macro::{Span, TokenStream};
use proc_macro_crate::{FoundCrate, crate_name};
use proc_macro2::Ident;
use syn::{
    AttrStyle,
    Attribute,
    ItemFn,
    Meta,
    ReturnType,
    Token,
    Type,
    parse::{Error as SynError, Parser},
    punctuated::Punctuated,
    spanned::Spanned,
};

pub enum WhiteListCaller {
    Interrupt,
}

pub fn handler(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f: ItemFn = syn::parse(input).expect("`#[handler]` must be applied to a function");
    let original_span = f.span();

    let attr_args = match Punctuated::<Meta, Token![,]>::parse_terminated.parse2(args.into()) {
        Ok(v) => v,
        Err(e) => return e.into_compile_error().into(),
    };

    let mut priority = None;

    for arg in attr_args {
        match arg {
            Meta::NameValue(meta_name_value) => {
                if meta_name_value.path.is_ident("priority") {
                    if priority.is_some() {
                        return SynError::new(
                            meta_name_value.span(),
                            "duplicate `priority` attribute",
                        )
                        .into_compile_error()
                        .into();
                    }
                    priority = Some(meta_name_value.value);
                } else {
                    return SynError::new(meta_name_value.span(), "expected `priority = <value>`")
                        .into_compile_error()
                        .into();
                }
            }
            other => {
                return SynError::new(other.span(), "expected `priority = <value>`")
                    .into_compile_error()
                    .into();
            }
        }
    }

    let root = Ident::new(
        match crate_name("esp-hal") {
            Ok(FoundCrate::Name(ref name)) => name,
            _ => "crate",
        },
        Span::call_site().into(),
    );

    let priority = match priority {
        Some(ref priority) => quote::quote!( {
            const {
                core::assert!(
                    !matches!(#priority, #root::interrupt::Priority::None),
                    "Priority::None is not supported",
                );
            };

            #priority
        } ),
        _ => quote::quote! { #root::interrupt::Priority::min() },
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
        &format!("__esp_hal_internal_{orig}"),
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
