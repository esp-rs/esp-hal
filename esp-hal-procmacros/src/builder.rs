use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
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
    parse::Error as ParseError,
    punctuated::Punctuated,
    spanned::Spanned,
};

const KNOWN_HELPERS: &[&str] = &[
    // Generate the setter with `impl Into<FieldType>` as the argument
    "into",
    // Do not generate a getter or setter
    "skip",
    // Do not generate a setter
    "skip_setter",
    // Do not generate a getter
    "skip_getter",
    // Feature gate the generated setters and getters by the "unstable" feature
    "unstable",
    // Generate a by-reference getter instead of a by-value getter for non-`Copy` types
    "reference",
];

/// A lightweight version of the `Builder` derive macro that only generates
/// setters and getters for each field of a struct.
///
/// <https://matklad.github.io/2022/05/29/builder-lite.html>
pub fn builder_lite_derive(item: TokenStream) -> TokenStream {
    let input: syn::DeriveInput = crate::unwrap_or_compile_error!(syn::parse2(item));

    let span = input.span();
    let ident = input.ident;
    let generics = input.generics;

    let mut fns = Vec::new();
    let Data::Struct(DataStruct { fields, .. }) = &input.data else {
        return ParseError::new(
            span,
            "#[derive(Builder)] is only defined for structs, not for enums or unions!",
        )
        .to_compile_error();
    };
    for field in fields {
        let helper_attributes = match collect_helper_attrs(&field.attrs) {
            Ok(attr) => attr,
            Err(err) => return err.to_compile_error(),
        };

        // Ignore field if it has a `skip` helper attribute.
        if helper_attributes.iter().any(|h| h == "skip") {
            continue;
        }

        let unstable = if helper_attributes.iter().any(|h| h == "unstable") {
            quote! {
                #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                #[cfg(feature = "unstable")]
            }
        } else {
            quote! {}
        };

        let field_ident = field.ident.as_ref().unwrap();
        let field_type = &field.ty;

        let function_ident = format_ident!("{}", format!("with_{field_ident}").replace("__", "_"));

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

        let (mut field_setter_type, mut field_assigns) = if let Some(inner_type) = maybe_path_type {
            (quote! { #inner_type }, quote! { Some(#field_ident) })
        } else {
            (quote! { #field_type }, quote! { #field_ident })
        };

        // Wrap type and assignment with `Into` if needed.
        if helper_attributes.iter().any(|h| h == "into") {
            field_setter_type = quote! { impl Into<#field_setter_type> };
            field_assigns = quote! { #field_ident .into() };
        }

        if !helper_attributes.iter().any(|h| h == "skip_setter") {
            fns.push(quote! {
                #[doc = concat!(" Assign the given value to the `", stringify!(#field_ident) ,"` field.")]
                #unstable
                #[must_use]
                pub fn #function_ident(mut self, #field_ident: #field_setter_type) -> Self {
                    self.#field_ident = #field_assigns;
                    self
                }
            });

            if maybe_path_type.is_some() {
                let function_ident = format_ident!("{}_none", function_ident);
                fns.push(quote! {
                    #[doc = concat!(" Set the value of `", stringify!(#field_ident), "` to `None`.")]
                    #unstable
                    #[must_use]
                    pub fn #function_ident(mut self) -> Self {
                        self.#field_ident = None;
                        self
                    }
                });
            }
        }

        if !helper_attributes.iter().any(|h| h == "skip_getter") {
            let docs = field.attrs.iter().filter_map(|attr| {
                let syn::Meta::NameValue(ref attr) = attr.meta else {
                    return None;
                };

                if attr.path.is_ident("doc") {
                    let docstr = &attr.value;
                    Some(quote! { #[doc = #docstr] })
                } else {
                    None
                }
            });

            let is_non_copy = helper_attributes.iter().any(|h| h == "reference");

            if is_non_copy {
                // Generate a by-reference getter for non-`Copy` types
                fns.push(quote! {
                    #(#docs)*
                    #unstable
                    pub fn #field_ident(&self) -> &#field_type {
                        &self.#field_ident
                    }
                });
            } else {
                // Generate a by-value getter for `Copy` types (the default)
                fns.push(quote! {
                    #(#docs)*
                    #unstable
                    pub fn #field_ident(&self) -> #field_type {
                        self.#field_ident
                    }
                });
            }
        }
    }

    let implementation = quote! {
        #[automatically_derived]
        impl #generics #ident #generics {
            #(#fns)*
        }
    };

    implementation
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrong_item() {
        let result = builder_lite_derive(
            quote::quote! {
                fn main() {}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{"expected one of: `struct`, `enum`, `union`"}
            }
            .to_string()
        );
    }

    #[test]
    fn test_wrong_item2() {
        let result = builder_lite_derive(
            quote::quote! {
                enum Foo {}
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{"#[derive(Builder)] is only defined for structs, not for enums or unions!"}
            }
            .to_string()
        );
    }

    #[test]
    fn test_wrong_item_attr() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    #[builder_lite(foo)]
                    foo: u32,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                ::core::compile_error!{"Unknown helper attribute `foo`. Only the following are allowed: into, skip, skip_setter, skip_getter, unstable, reference"}
            }
            .to_string()
        );
    }

    #[test]
    fn test_basic() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    #[doc = "keep docs"]
                    bar: u32,
                    baz: bool,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[automatically_derived]
                impl Foo {
                    #[doc = concat!(" Assign the given value to the `" , stringify ! (bar) , "` field.")]
                    #[must_use]
                    pub fn with_bar(mut self , bar : u32) -> Self {
                        self.bar = bar;
                        self
                    }

                    #[doc = "keep docs"]
                    pub fn bar(&self) -> u32 {
                        self.bar
                    }

                    #[doc = concat!(" Assign the given value to the `" , stringify ! (baz) , "` field.")]
                    #[must_use]
                    pub fn with_baz(mut self, baz: bool) -> Self {
                        self.baz = baz;
                        self
                    }

                    pub fn baz (& self) -> bool {
                        self.baz
                    }
                }
            }
            .to_string()
        );
    }

    #[test]
    fn test_option_field() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    bar: Option<u32>,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[automatically_derived]
                impl Foo {
                    #[doc = concat!(" Assign the given value to the `" , stringify!(bar) , "` field.")]
                    #[must_use]
                    pub fn with_bar(mut self, bar: u32) -> Self {
                        self.bar = Some(bar);
                        self
                    }

                    #[doc = concat!(" Set the value of `" , stringify ! (bar) , "` to `None`.")]
                    #[must_use]
                    pub fn with_bar_none (mut self) -> Self {
                        self.bar = None;
                        self
                    }

                    pub fn bar(&self) -> Option <u32>{
                        self.bar
                    }
                }
            }
            .to_string()
        );
    }

    #[test]
    fn test_field_attrs() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    #[builder_lite(unstable)]
                    bar: u32,

                    #[builder_lite(skip_setter)]
                    baz: bool,

                    #[builder_lite(reference)]
                    boo: String,

                    #[builder_lite(into)]
                    bam: Foo,

                    #[builder_lite(unstable)]
                    #[builder_lite(into)]
                    foo: Foo,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[automatically_derived]
                impl Foo {
                    # [doc = concat ! (" Assign the given value to the `" , stringify ! (bar) , "` field.")]
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    #[cfg(feature = "unstable")]
                    #[must_use]
                    pub fn with_bar(mut self, bar: u32) -> Self {
                        self.bar = bar;
                        self
                    }
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    #[cfg(feature = "unstable")]
                    pub fn bar(&self) -> u32 {
                        self.bar
                    }
                    pub fn baz(&self) -> bool {
                        self.baz
                    }
                    # [doc = concat ! (" Assign the given value to the `" , stringify ! (boo) , "` field.")]
                    #[must_use]
                    pub fn with_boo(mut self, boo: String) -> Self {
                        self.boo = boo;
                        self
                    }
                    pub fn boo(&self) -> &String {
                        &self.boo
                    }
                    # [doc = concat ! (" Assign the given value to the `" , stringify ! (bam) , "` field.")]
                    #[must_use]
                    pub fn with_bam(mut self, bam: impl Into<Foo>) -> Self {
                        self.bam = bam.into();
                        self
                    }
                    pub fn bam(&self) -> Foo {
                        self.bam
                    }
                    # [doc = concat ! (" Assign the given value to the `" , stringify ! (foo) , "` field.")]
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    #[cfg(feature = "unstable")]
                    #[must_use]
                    pub fn with_foo(mut self, foo: impl Into<Foo>) -> Self {
                        self.foo = foo.into();
                        self
                    }
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    #[cfg(feature = "unstable")]
                    pub fn foo(&self) -> Foo {
                        self.foo
                    }
                }
            }
            .to_string()
        );
    }

    #[test]
    fn test_skip_getter() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    #[builder_lite(skip_getter)]
                    bar: Option<u32>,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[automatically_derived]
                impl Foo {
                    #[doc = concat!(" Assign the given value to the `" , stringify!(bar) , "` field.")]
                    #[must_use]
                    pub fn with_bar(mut self, bar: u32) -> Self {
                        self.bar = Some(bar);
                        self
                    }

                    #[doc = concat!(" Set the value of `" , stringify ! (bar) , "` to `None`.")]
                    #[must_use]
                    pub fn with_bar_none (mut self) -> Self {
                        self.bar = None;
                        self
                    }
                }
            }
            .to_string()
        );
    }

    #[test]
    fn test_skip() {
        let result = builder_lite_derive(
            quote::quote! {
                struct Foo {
                    #[builder_lite(skip)]
                    bar: Option<u32>,
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote::quote! {
                #[automatically_derived]
                impl Foo {
                }
            }
            .to_string()
        );
    }
}
