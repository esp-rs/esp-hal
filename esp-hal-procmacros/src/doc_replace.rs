use std::{collections::HashMap, str::FromStr};

use proc_macro2::{TokenStream, TokenStream as TokenStream2};
use quote::quote;
use syn::{
    AttrStyle,
    Attribute,
    Expr,
    ExprLit,
    Item,
    Lit,
    LitStr,
    Meta,
    MetaNameValue,
    Token,
    braced,
    parse::Parse,
    punctuated::Punctuated,
    spanned::Spanned,
    token,
};

struct Replacements {
    // Placeholder => [attribute contents]
    //
    // Replaces `# {tag}` placeholders with the attribute contents. Replaces the entire line.
    line_replacements: HashMap<String, Vec<TokenStream2>>,

    // Placeholder => [(condition, contents)], may be unconditional.
    //
    // Replaces `__tag__` placeholders with the contents. Replaces only the placeholder
    // inside the line, and applies the condition to the entire line if present.
    inline_replacements: HashMap<String, Vec<(Option<TokenStream>, Inline)>>,
}

/// The value of an inline replacement.
#[derive(Clone)]
enum Inline {
    /// A string, spliced into the line as it is.
    Text(String),

    /// Tokens that expand to a string literal, such as a call to a macro that knows the current
    /// chip. The line is assembled with `concat!`, as the value is only known once the doc
    /// attribute is expanded.
    Expanded(TokenStream2),
}

impl Inline {
    fn new(value: &Expr) -> Self {
        if let Expr::Lit(ExprLit {
            lit: Lit::Str(lit_str),
            ..
        }) = value
        {
            Self::Text(lit_str.value())
        } else {
            Self::Expanded(quote! { #value })
        }
    }
}

/// A piece of a documentation line: either a chunk of the line, or a placeholder to replace.
enum Piece<'a> {
    Text(&'a str),
    Replacement(&'a str),
}

impl Replacements {
    fn get(&self, lines: &str, outer: bool, span: proc_macro2::Span) -> Vec<Attribute> {
        let mut attributes = vec![];
        for line in lines.split('\n') {
            let mut attrs = vec![];
            let trimmed = line.trim();
            if let Some(lines) = self.line_replacements.get(trimmed) {
                for line in lines {
                    if outer {
                        attrs.push(syn::parse_quote_spanned! { span => #[ #line ] });
                    } else {
                        attrs.push(syn::parse_quote_spanned! { span => #![ #line ] });
                    }
                }
            } else {
                for attr_inner in self.inline_attributes(line) {
                    if outer {
                        attrs.push(syn::parse_quote_spanned! { span => #[ #attr_inner ] });
                    } else {
                        attrs.push(syn::parse_quote_spanned! { span => #![ #attr_inner ] });
                    }
                }
            }
            attributes.extend(attrs);
        }

        attributes
    }

    /// Splits a line into chunks of text and the placeholders between them.
    fn split_line<'a>(&'a self, mut line: &'a str) -> Vec<Piece<'a>> {
        let mut pieces = vec![];

        loop {
            // Take the placeholder that comes first. Should multiple placeholders start at the same
            // position, the longest one wins, as the shorter ones are then part of its name.
            let next = self
                .inline_replacements
                .keys()
                .filter_map(|placeholder| {
                    line.find(placeholder.as_str())
                        .map(|at| (at, placeholder.len(), placeholder))
                })
                .min_by_key(|(at, length, _)| (*at, std::cmp::Reverse(*length)));

            let Some((at, length, placeholder)) = next else {
                if !line.is_empty() {
                    pieces.push(Piece::Text(line));
                }
                return pieces;
            };

            if at > 0 {
                pieces.push(Piece::Text(&line[..at]));
            }
            pieces.push(Piece::Replacement(placeholder));
            line = &line[at + length..];
        }
    }

    /// Returns the contents of the `doc` attributes that `line` expands to.
    ///
    /// A line without placeholders expands to a single attribute. Otherwise, each placeholder
    /// contributes as many attributes as it has conditional values, so a line is emitted for every
    /// combination of the values of its placeholders.
    fn inline_attributes(&self, line: &str) -> Vec<TokenStream2> {
        let pieces = self.split_line(line);

        // The placeholders of the line, in order of first appearance. A placeholder used multiple
        // times in the same line takes the same value in each of its places.
        let mut placeholders: Vec<&str> = vec![];
        for piece in pieces.iter() {
            if let Piece::Replacement(placeholder) = piece
                && !placeholders.contains(placeholder)
            {
                placeholders.push(placeholder);
            }
        }

        if placeholders.is_empty() {
            let line = create_raw_string(line);
            return vec![quote! { doc = #line }];
        }

        let values = placeholders
            .iter()
            .map(|placeholder| &self.inline_replacements[*placeholder])
            .collect::<Vec<_>>();

        let combinations = values.iter().map(|values| values.len()).product();

        let mut attributes = vec![];
        for combination in 0..combinations {
            // Decode the number of the combination into an index into each placeholder's values.
            // The last placeholder varies the fastest.
            let mut rest = combination;
            let mut choices = vec![0; values.len()];
            for (choice, values) in choices.iter_mut().zip(values.iter()).rev() {
                *choice = rest % values.len();
                rest /= values.len();
            }

            let choice = |placeholder: &str| {
                let index = placeholders.iter().position(|p| *p == placeholder).unwrap();
                &values[index][choices[index]]
            };

            let conditions = placeholders
                .iter()
                .filter_map(|placeholder| choice(placeholder).0.as_ref())
                .collect::<Vec<_>>();

            let doc = quote_doc_line(pieces.iter().map(|piece| match piece {
                Piece::Text(text) => Inline::Text((*text).to_string()),
                Piece::Replacement(placeholder) => choice(placeholder).1.clone(),
            }));

            attributes.push(match conditions.as_slice() {
                [] => doc,
                [condition] => quote! { cfg_attr(#condition, #doc) },
                conditions => quote! { cfg_attr(all( #(#conditions),* ), #doc) },
            });
        }

        attributes
    }
}

/// Assembles the contents of a `doc` attribute out of the pieces of a line.
///
/// Text is concatenated as far as possible, so that a line that is fully known here becomes a
/// single string. The rest is left to `concat!`, which runs once the attribute is expanded and the
/// remaining pieces are known.
fn quote_doc_line(pieces: impl Iterator<Item = Inline>) -> TokenStream2 {
    let mut parts: Vec<Inline> = vec![];
    for piece in pieces {
        match (parts.last_mut(), piece) {
            (Some(Inline::Text(text)), Inline::Text(next)) => text.push_str(&next),
            (_, piece) => parts.push(piece),
        }
    }

    if let [Inline::Text(text)] = parts.as_slice() {
        let line = create_raw_string(text);
        return quote! { doc = #line };
    }

    let parts = parts.iter().map(|part| match part {
        Inline::Text(text) => create_raw_string(text),
        Inline::Expanded(tokens) => tokens.clone(),
    });

    quote! { doc = concat!( #(#parts),* ) }
}

fn create_raw_string(line: &str) -> TokenStream2 {
    let hash = if line.contains("#\"") {
        "##"
    } else if line.contains('"') {
        "#"
    } else {
        ""
    };

    TokenStream2::from_str(&format!("r{hash}\"{line}\"{hash}")).unwrap()
}

impl Parse for Replacements {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut line_replacements = HashMap::new();
        let mut inline_replacements = HashMap::new();

        let mut add_line_replacement = |placeholder: &str, replacement: Vec<TokenStream2>| {
            line_replacements.insert(format!("# {{{placeholder}}}"), replacement);
        };
        let mut add_inline_replacement =
            |placeholder: &str, replacement: Vec<(Option<TokenStream2>, Inline)>| {
                // The placeholder must be a valid Rust identifier to keep rustfmt happy
                inline_replacements.insert(format!("__{placeholder}__"), replacement);
            };

        if !input.is_empty() {
            let args = Punctuated::<Replacement, Token![,]>::parse_terminated(input)?;
            for arg in args {
                match arg.replacement {
                    ReplacementKind::Literal(expr) => {
                        add_inline_replacement(&arg.placeholder, vec![(None, Inline::new(&expr))]);

                        add_line_replacement(
                            &arg.placeholder,
                            vec![quote! {
                                doc = #expr
                            }],
                        );
                    }
                    ReplacementKind::Choice(items) => {
                        let mut conditions = vec![];
                        let mut bodies = vec![];
                        let mut cfgs = vec![];

                        for branch in items {
                            let body = branch.body;

                            match branch.condition {
                                Some(Meta::List(cfg)) if cfg.path.is_ident("cfg") => {
                                    let condition = cfg.tokens;

                                    cfgs.push(condition.clone());
                                    conditions.push(condition);
                                    bodies.push(body);
                                }
                                None => {
                                    conditions.push(quote! { not(any( #(#cfgs),*) ) });
                                    bodies.push(body);
                                }
                                _ => {
                                    return Err(syn::Error::new(
                                        branch.condition.span(),
                                        "Expected a cfg condition or catch-all condition using `_`",
                                    ));
                                }
                            }
                        }

                        let branches = conditions
                            .iter()
                            .zip(bodies.iter())
                            .map(|(condition, body)| {
                                quote! {
                                    cfg_attr(#condition, doc = #body)
                                }
                            })
                            .collect::<Vec<_>>();
                        add_line_replacement(&arg.placeholder, branches);

                        let branches = conditions
                            .into_iter()
                            .map(Some)
                            .zip(bodies.iter().map(Inline::new))
                            .collect::<Vec<_>>();
                        add_inline_replacement(&arg.placeholder, branches);
                    }
                }
            }
        }

        Ok(Self {
            line_replacements,
            inline_replacements,
        })
    }
}

struct Replacement {
    placeholder: String,
    replacement: ReplacementKind,
}

impl Parse for Replacement {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let placeholder: LitStr = input.parse()?;
        let _arrow: Token![=>] = input.parse()?;
        let replacement: ReplacementKind = input.parse()?;

        Ok(Self {
            placeholder: placeholder.value(),
            replacement,
        })
    }
}

enum ReplacementKind {
    Literal(Expr),
    Choice(Punctuated<Branch, Token![,]>),
}

impl Parse for ReplacementKind {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        if !input.peek(token::Brace) {
            return input.parse().map(Self::Literal);
        }

        let choices;
        braced!(choices in input);

        let branches = Punctuated::<Branch, Token![,]>::parse_terminated(&choices)?;

        Ok(Self::Choice(branches))
    }
}

struct Branch {
    condition: Option<Meta>,
    body: Expr,
}

impl Parse for Branch {
    fn parse(input: syn::parse::ParseStream<'_>) -> syn::Result<Self> {
        let condition: Option<Meta> = if input.parse::<Token![_]>().is_ok() {
            None
        } else {
            Some(input.parse()?)
        };

        input.parse::<syn::Token![=>]>()?;

        let body: Expr = input.parse()?;

        Ok(Branch { condition, body })
    }
}

pub(crate) fn replace(attr: TokenStream, input: TokenStream) -> TokenStream {
    let mut replacements: Replacements = match syn::parse2(attr) {
        Ok(replacements) => replacements,
        Err(e) => return e.into_compile_error(),
    };

    replacements.line_replacements.insert(
        "# {before_snippet}".to_string(),
        vec![quote! { doc = crate::before_snippet!() }],
    );
    replacements.line_replacements.insert(
        "# {after_snippet}".to_string(),
        vec![quote! { doc = crate::after_snippet!() }],
    );

    let mut item: Item = crate::unwrap_or_compile_error!(syn::parse2(input));

    let mut replacement_attrs = Vec::new();

    let attrs = item.attrs_mut();
    for attr in attrs {
        if let Meta::NameValue(MetaNameValue { path, value, .. }) = &attr.meta
            && let Some(ident) = path.get_ident()
            && ident == "doc"
            && let Expr::Lit(lit) = value
            && let Lit::Str(doc) = &lit.lit
        {
            let replacement = replacements.get(
                doc.value().as_str(),
                attr.style == AttrStyle::Outer,
                attr.span(),
            );
            replacement_attrs.extend(replacement);
        } else {
            replacement_attrs.push(attr.clone());
        }
    }

    *item.attrs_mut() = replacement_attrs;

    quote! { #item }
}

trait ItemLike {
    fn attrs_mut(&mut self) -> &mut Vec<syn::Attribute>;
}

impl ItemLike for Item {
    fn attrs_mut(&mut self) -> &mut Vec<syn::Attribute> {
        match self {
            Item::Fn(item_fn) => &mut item_fn.attrs,
            Item::Struct(item_struct) => &mut item_struct.attrs,
            Item::Enum(item_enum) => &mut item_enum.attrs,
            Item::Trait(item_trait) => &mut item_trait.attrs,
            Item::Mod(module) => &mut module.attrs,
            Item::Macro(item_macro) => &mut item_macro.attrs,
            _ => panic!("Unsupported item type for switch macro"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let result = replace(
            quote! {}.into(),
            quote! {
                /// # Configuration
                ///
                /// ## Overview
                ///
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## Examples
                /// ### Default initialization
                /// ```rust, no_run
                /// # {before_snippet}
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                /// # {after_snippet}
                /// ```
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// # Configuration
                ///
                /// ## Overview
                ///
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## Examples
                /// ### Default initialization
                /// ```rust, no_run
                #[doc = crate::before_snippet!()]
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                #[doc = crate::after_snippet!()]
                /// ```
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_one_doc_attr() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = r#" # Configuration
 ## Overview
 This module contains the initial configuration for the system.
 ## Configuration
 In the [`esp_hal::init()`][crate::init] method, we can configure different
 parameters for the system:
 - CPU clock configuration.
 - Watchdog configuration.
 ## Examples
 ### Default initialization
 ```rust, no_run
 # {before_snippet}
 let peripherals = esp_hal::init(esp_hal::Config::default());
 # {after_snippet}
 ```"#]
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// # Configuration
                /// ## Overview
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## Examples
                /// ### Default initialization
                /// ```rust, no_run
                #[doc = crate::before_snippet!()]
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                #[doc = crate::after_snippet!()]
                /// ```
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_custom_replacements() {
        let result = replace(
            quote! {
                "freq" => {
                    cfg(esp32h2) => "let freq = Rate::from_mhz(32);",
                    _ => "let freq = Rate::from_mhz(80);"
                },
                "other" => "replacement"
            }.into(),
            quote! {
                #[doc = " # Configuration"]
                #[doc = " ## Overview"]
                #[doc = " This module contains the initial configuration for the system."]
                #[doc = " ## Configuration"]
                #[doc = " In the [`esp_hal::init()`][crate::init] method, we can configure different"]
                #[doc = " parameters for the system:"]
                #[doc = " - CPU clock configuration."]
                #[doc = " - Watchdog configuration."]
                #[doc = " ## Examples"]
                #[doc = " ### Default initialization"]
                #[doc = " ```rust, no_run"]
                #[doc = " # {freq}"]
                #[doc = " # {before_snippet}"]
                #[doc = " let peripherals = esp_hal::init(esp_hal::Config::default());"]
                #[doc = " # {after_snippet}"]
                #[doc = " ```"]
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// # Configuration
                /// ## Overview
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## Examples
                /// ### Default initialization
                /// ```rust, no_run
                #[cfg_attr (esp32h2 , doc = "let freq = Rate::from_mhz(32);")]
                #[cfg_attr (not (any (esp32h2)) , doc = "let freq = Rate::from_mhz(80);")]
                #[doc = crate::before_snippet!()]
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                #[doc = crate::after_snippet!()]
                /// ```
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_custom_inline_replacements() {
        let result = replace(
            quote! {
                "freq" => {
                    cfg(esp32h2) => "32",
                    _ => "80"
                },
                "other" => "Replacement"
            }
            .into(),
            quote! {
                /// # Configuration
                /// ## Overview
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## __other__ Examples
                /// ### Default initialization
                /// ```rust, no_run
                /// let freq = Rate::from_mhz(__freq__);
                /// # {before_snippet}
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                /// # {after_snippet}
                /// ```
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// # Configuration
                /// ## Overview
                /// This module contains the initial configuration for the system.
                /// ## Configuration
                /// In the [`esp_hal::init()`][crate::init] method, we can configure different
                /// parameters for the system:
                /// - CPU clock configuration.
                /// - Watchdog configuration.
                /// ## Replacement Examples
                /// ### Default initialization
                /// ```rust, no_run
                #[cfg_attr (esp32h2 , doc = r" let freq = Rate::from_mhz(32);")]
                #[cfg_attr (not (any (esp32h2)) , doc = r" let freq = Rate::from_mhz(80);")]
                #[doc = crate::before_snippet!()]
                /// let peripherals = esp_hal::init(esp_hal::Config::default());
                #[doc = crate::after_snippet!()]
                /// ```
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_inline_macro_replacements() {
        let result = replace(
            quote! {
                "pin" => gpio_for_signal!(USB_FS_DP)
            }
            .into(),
            quote! {
                /// ```rust, no_run
                /// let pin = peripherals.__pin__;
                /// ```
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// ```rust, no_run
                #[doc = concat!(r" let pin = peripherals.", gpio_for_signal!(USB_FS_DP), r";")]
                /// ```
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_conditional_inline_macro_replacements() {
        let result = replace(
            quote! {
                "pin" => {
                    cfg(esp32s3) => gpio_for_signal!(SAR_I2C_SDA_0),
                    _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
                }
            }
            .into(),
            quote! {
                /// let pin = peripherals.__pin__;
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                #[cfg_attr(esp32s3, doc = concat!(r" let pin = peripherals.", gpio_for_signal!(SAR_I2C_SDA_0), r";"))]
                #[cfg_attr(not(any(esp32s3)), doc = concat!(r" let pin = peripherals.", gpio_for_signal!(LP_I2C_SDA, "GPIO6"), r";"))]
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_multiple_inline_replacements() {
        let result = replace(
            quote! {
                "first" => "Alpha",
                "second" => "Beta"
            }
            .into(),
            quote! {
                /// __first__ and __second__, __first__ again.
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                #[doc = r" Alpha and Beta, Alpha again."]
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_multiple_conditional_inline_replacements() {
        let result = replace(
            quote! {
                "first" => {
                    cfg(esp32) => "one",
                    _ => "two"
                },
                "second" => {
                    cfg(esp32c6) => "three",
                    _ => "four"
                }
            }
            .into(),
            quote! {
                /// __first__ __second__
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                #[cfg_attr(all(esp32, esp32c6), doc = r" one three")]
                #[cfg_attr(all(esp32, not(any(esp32c6))), doc = r" one four")]
                #[cfg_attr(all(not(any(esp32)), esp32c6), doc = r" two three")]
                #[cfg_attr(all(not(any(esp32)), not(any(esp32c6))), doc = r" two four")]
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_multiple_mixed_inline_replacements() {
        let result = replace(
            quote! {
                "pin" => gpio_for_signal!(USB_FS_DP),
                "index" => {
                    cfg(esp32) => "1",
                    _ => "2"
                }
            }
            .into(),
            quote! {
                /// __pin__ is DP of USB __index__.
                struct Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                #[cfg_attr(esp32, doc = concat!(r" ", gpio_for_signal!(USB_FS_DP), r" is DP of USB 1."))]
                #[cfg_attr(not(any(esp32)), doc = concat!(r" ", gpio_for_signal!(USB_FS_DP), r" is DP of USB 2."))]
                struct Foo {}
            }
            .to_string()
        );
    }

    #[test]
    fn test_custom_fail() {
        let result = replace(
            quote! {
                "freq" => {
                    abc(esp32h2) => "let freq = Rate::from_mhz(32);",
                },
            }
            .into(),
            quote! {}.into(),
        );

        assert_eq!(result.to_string(), quote! {
            ::core::compile_error!{ "Expected a cfg condition or catch-all condition using `_`" }
        }.to_string());
    }

    #[test]
    fn test_basic_fn() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                fn foo() {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// docs
                #[doc = crate::before_snippet!()]
                fn foo () { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_basic_enum() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                enum Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// docs
                #[doc = crate::before_snippet!()]
                enum Foo { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_basic_trait() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                trait Foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// docs
                #[doc = crate::before_snippet!()]
                trait Foo { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_basic_mod() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                mod foo {
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// docs
                #[doc = crate::before_snippet!()]
                mod foo { }
            }
            .to_string()
        );
    }

    #[test]
    fn test_basic_macro() {
        let result = replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                macro_rules! foo {
                    () => {
                    };
                }
            }
            .into(),
        );

        assert_eq!(
            result.to_string(),
            quote! {
                /// docs
                #[doc = crate::before_snippet!()]
                macro_rules! foo {
                    () => {
                    };
                }
            }
            .to_string()
        );
    }

    // TODO panicking is not the nicest way to handle this
    #[test]
    #[should_panic]
    fn test_basic_fail_wrong_item() {
        replace(
            quote! {}.into(),
            quote! {
                #[doc = " docs"]
                #[doc = " # {before_snippet}"]
                static FOO: u32 = 0u32;
            }
            .into(),
        );
    }
}
