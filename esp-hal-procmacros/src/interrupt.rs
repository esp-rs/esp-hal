use proc_macro_crate::{FoundCrate, crate_name};
use proc_macro2::{Ident, Span, TokenStream};
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
    let mut f: ItemFn = crate::unwrap_or_compile_error!(syn::parse2(input));
    let original_span = f.span();

    let attr_args = match Punctuated::<Meta, Token![,]>::parse_terminated.parse2(args) {
        Ok(v) => v,
        Err(e) => return e.into_compile_error(),
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
                        .into_compile_error();
                    }
                    priority = Some(meta_name_value.value);
                } else {
                    return SynError::new(meta_name_value.span(), "expected `priority = <value>`")
                        .into_compile_error();
                }
            }
            other => {
                return SynError::new(other.span(), "expected `priority = <value>`")
                    .into_compile_error();
            }
        }
    }

    let root = Ident::new(
        match crate_name("esp-hal") {
            Ok(FoundCrate::Name(ref name)) => name,
            _ => "crate",
        },
        Span::call_site(),
    );

    let priority = match priority {
        Some(ref priority) => quote::quote! { #priority },
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
        .to_compile_error();
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
        if let Some(attr_name) = get_attr_name(attr)
            && whitelist.contains(&attr_name.as_str())
        {
            continue 'o;
        }

        let err_str = match caller {
            WhiteListCaller::Interrupt => {
                "this attribute is not allowed on an interrupt handler controlled by esp-hal"
            }
        };

        return Err(SynError::new(attr.span(), err_str).to_compile_error());
    }

    Ok(())
}

/// Extracts the base name of an attribute, including unwrapping of `#[unsafe(...)]`.
fn get_attr_name(attr: &Attribute) -> Option<String> {
    if !matches!(attr.style, AttrStyle::Outer) {
        return None;
    }

    let name = attr.path().get_ident().map(|x| x.to_string());

    match &name {
        Some(name) if name == "unsafe" => {
            // Try to parse the inner meta of #[unsafe(...)]
            if let Ok(inner_meta) = attr.parse_args::<syn::Meta>() {
                inner_meta.path().get_ident().map(|x| x.to_string())
            } else {
                None
            }
        }
        _ => name,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let result = handler(
            quote::quote! {}.into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(result.to_string(), quote::quote! {
            extern "C" fn __esp_hal_internal_foo() {}
            #[allow(non_upper_case_globals)]
            const foo: crate::interrupt::InterruptHandler = crate::interrupt::InterruptHandler::new(
                __esp_hal_internal_foo,
                crate::interrupt::Priority::min()
            );
        }.to_string());
    }

    #[test]
    fn test_priority() {
        let result = handler(
            quote::quote! {
                priority = esp_hal::interrupt::Priority::Priority2
            }
            .into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                extern "C" fn __esp_hal_internal_foo() {}
                #[allow(non_upper_case_globals)]
                const foo: crate::interrupt::InterruptHandler =
                    crate::interrupt::InterruptHandler::new(
                        __esp_hal_internal_foo,
                        esp_hal::interrupt::Priority::Priority2
                    );
            }
            .to_string()
        );
    }

    #[test]
    fn test_forbidden_attr() {
        let result = handler(
            quote::quote! {}.into(),
            quote::quote! {
                #[forbidden]
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(result.to_string(), quote::quote! {
            ::core::compile_error!{ "this attribute is not allowed on an interrupt handler controlled by esp-hal" }
        }.to_string());
    }

    #[test]
    fn test_duplicate_priority() {
        let result = handler(
            quote::quote! {
                priority = esp_hal::interrupt::Priority::Priority2,
                priority = esp_hal::interrupt::Priority::Priority1,
            }
            .into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "duplicate `priority` attribute" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_wrong_args() {
        let result = handler(
            quote::quote! {
                true
            }
            .into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "expected identifier, found keyword `true`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg() {
        let result = handler(
            quote::quote! {
                not_allowed = true,
            }
            .into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "expected `priority = <value>`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_arg2() {
        let result = handler(
            quote::quote! {
                A,B
            }
            .into(),
            quote::quote! {
                fn foo(){}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{ "expected `priority = <value>`" }
            }
            .to_string()
        );
    }

    #[test]
    fn test_illegal_sig() {
        let result = handler(
            quote::quote! {}.into(),
            quote::quote! {
                fn foo() -> u32 {}
            }
            .into(),
        );

        assert_eq!(result.to_string(), quote::quote! {
            ::core::compile_error!{ "`#[handler]` handlers must have signature `[unsafe] fn([&mut Context]) [-> !]`" }
        }.to_string());
    }
}
