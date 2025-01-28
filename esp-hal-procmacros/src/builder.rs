use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::Error as ParseError,
    punctuated::Punctuated,
    spanned::Spanned,
    Attribute,
    Data,
    DataStruct,
    GenericArgument,
    Ident,
    Path,
    PathArguments,
    PathSegment,
    Token,
    Type,
};

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

            let (mut field_type, mut field_assigns) = if let Some(inner_type) = maybe_path_type {
                (quote! { #inner_type }, quote! { Some(#field_ident) })
            } else {
                (quote! { #field_type }, quote! { #field_ident })
            };

            let helper_attributes = match collect_helper_attrs(&field.attrs) {
                Ok(attr) => attr,
                Err(err) => return err.to_compile_error().into(),
            };

            // Wrap type and assignment with `Into` if needed.
            if helper_attributes.iter().any(|h| h == "into") {
                field_type = quote! { impl Into<#field_type> };
                field_assigns = quote! { #field_ident .into() };
            }

            fns.push(quote! {
                #[doc = concat!(" Assign the given value to the `", stringify!(#field_ident) ,"` field.")]
                #[must_use]
                pub fn #function_ident(mut self, #field_ident: #field_type) -> Self {
                    self.#field_ident = #field_assigns;
                    self
                }
            });

            if maybe_path_type.is_some() {
                let function_ident = format_ident!("with_{}_none", field_ident);
                fns.push(quote! {
                    #[doc = concat!(" Set the value of `", stringify!(#field_ident), "` to `None`.")]
                    #[must_use]
                    pub fn #function_ident(mut self) -> Self {
                        self.#field_ident = None;
                        self
                    }
                });
            }
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

fn collect_helper_attrs(attrs: &[Attribute]) -> Result<Vec<Ident>, syn::Error> {
    const KNOWN_HELPERS: &[&str] = &["into"];

    let mut helper_attributes = Vec::new();
    for attr in attrs
        .iter()
        .filter(|attr| attr.path().is_ident("builder_lite"))
    {
        for helper in attr.parse_args_with(|input: syn::parse::ParseStream| {
            Punctuated::<Ident, Token![,]>::parse_terminated(input)
        })? {
            if !KNOWN_HELPERS.iter().any(|known| helper == *known) {
                return Err(ParseError::new(
                    helper.span(),
                    format!(
                        "Unknown helper attribute `{}`. Only the following are allowed: {}",
                        helper,
                        KNOWN_HELPERS.join(", ")
                    ),
                ));
            }

            helper_attributes.push(helper);
        }
    }

    Ok(helper_attributes)
}
