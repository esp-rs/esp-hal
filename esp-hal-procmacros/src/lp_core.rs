use quote::quote;
use syn::{GenericArgument, PatType, PathArguments, Type};

pub(crate) fn make_magic_symbol_name(args: &Vec<&PatType>) -> String {
    let mut res = String::from("__ULP_MAGIC_");
    for &a in args {
        let t = &a.ty;
        let quoted = to_string(&t);
        res.push_str(&quoted);
        res.push_str("$");
    }

    res
}

pub(crate) fn get_simplename(t: &Type) -> String {
    String::from(match t {
        Type::Path(p) => String::from(&p.path.segments.last().unwrap().ident.to_string()),
        _ => String::new(),
    })
}

pub(crate) fn extract_pin(ty: &Type) -> u8 {
    let mut res = 255u8;
    if let Type::Path(p) = ty {
        let segment = p.path.segments.last().unwrap();
        if let PathArguments::AngleBracketed(g) = &segment.arguments {
            for arg in &g.args {
                match arg {
                    GenericArgument::Type(t) => {
                        res = extract_pin(t);
                    }
                    GenericArgument::Const(c) => {
                        res = (&quote! { #c }.to_string()).parse().unwrap();
                    }
                    _ => (),
                }
            }
        }
    }

    res
}

// This is a specialized implementation - won't fit other use-cases
fn to_string(ty: &Type) -> String {
    let mut res = String::new();
    if let Type::Path(p) = ty {
        let segment = p.path.segments.last().unwrap();
        res.push_str(&segment.ident.to_string());

        if let PathArguments::AngleBracketed(g) = &segment.arguments {
            res.push_str("<");
            for arg in &g.args {
                match arg {
                    GenericArgument::Type(t) => {
                        res.push_str(&to_string(t));
                    }
                    GenericArgument::Const(c) => {
                        res.push_str(",");
                        res.push_str(&quote! { #c }.to_string());
                    }
                    _ => (),
                }
            }
            res.push_str(">");
        }
    }

    res
}
