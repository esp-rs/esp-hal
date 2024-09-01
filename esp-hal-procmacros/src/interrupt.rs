use proc_macro::TokenStream;
use syn::{parse::Error, spanned::Spanned, AttrStyle, Attribute};

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
            if eq(attr, val) {
                continue 'o;
            }
        }

        let err_str = match caller {
            WhiteListCaller::Interrupt => {
                "this attribute is not allowed on an interrupt handler controlled by esp-hal"
            }
        };

        return Err(Error::new(attr.span(), err_str).to_compile_error().into());
    }

    Ok(())
}

/// Returns `true` if `attr.path` matches `name`
fn eq(attr: &Attribute, name: &str) -> bool {
    attr.style == AttrStyle::Outer && attr.path().is_ident(name)
}
