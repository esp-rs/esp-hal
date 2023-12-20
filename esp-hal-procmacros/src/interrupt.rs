use proc_macro::{self, TokenStream};
use syn::{parse::Error, spanned::Spanned, AttrStyle, Attribute};

use super::*;

pub(crate) enum WhiteListCaller {
    Interrupt,
}

pub(crate) fn check_attr_whitelist(
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
            if eq(&attr, &val) {
                continue 'o;
            }
        }

        let err_str = match caller {
            WhiteListCaller::Interrupt => {
                "this attribute is not allowed on an interrupt handler controlled by esp-hal"
            }
        };

        return Err(Error::new(attr.span(), &err_str).to_compile_error().into());
    }

    Ok(())
}

pub(crate) fn extract_cfgs(attrs: Vec<Attribute>) -> (Vec<Attribute>, Vec<Attribute>) {
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

/// Returns `true` if `attr.path` matches `name`
fn eq(attr: &Attribute, name: &str) -> bool {
    attr.style == AttrStyle::Outer && attr.path().is_ident(name)
}

pub fn interrupt(args: TokenStream, input: TokenStream, internal: bool) -> TokenStream {
    use std::iter;

    use proc_macro::Span;
    use proc_macro2::Ident;
    use proc_macro_error::abort;
    use syn::{parse::Error as ParseError, ItemFn, Meta, ReturnType, Type, Visibility};

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

    let export_name = if !internal {
        ident_s.to_string()
    } else {
        format!("{}_HAL", ident_s.to_string())
    };

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
