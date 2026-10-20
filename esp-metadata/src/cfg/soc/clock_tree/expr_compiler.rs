use std::collections::HashMap;

use proc_macro2::TokenStream;
use quote::quote;
use somni_expr::DefaultTypeSet;
use somni_parser::{ast, lexer::Token};

use crate::{
    cfg::{
        ClockTreeNodeInstance,
        ProcessedClockData,
        clock_tree::{Bounds, Expression},
    },
    number,
};

/// A variable an expression can refer to: the code that reads it, and the range of values it can
/// take.
#[derive(Debug, Clone)]
pub struct Operand {
    tokens: TokenStream,
    bounds: Bounds,
}

impl Operand {
    pub fn new(tokens: TokenStream, bounds: Bounds) -> Self {
        Self { tokens, bounds }
    }
}

/// A compiled (sub)expression.
struct Compiled {
    tokens: TokenStream,
    bounds: Bounds,
    /// Whether `tokens` evaluate to a `u64`. Frequencies and parameters are `u32`, and
    /// subexpressions stay `u32` unless their bounds don't fit.
    wide: bool,
    /// The value of the expression, if it is an integer literal. Literals are widened by writing a
    /// suffix, which avoids a pointless cast in the generated code.
    literal: Option<u64>,
}

impl Compiled {
    fn number(value: u64) -> Self {
        Self {
            tokens: number(value),
            bounds: Bounds::exact(value),
            wide: value > u32::MAX as u64,
            literal: Some(value),
        }
    }

    fn boolean(tokens: TokenStream) -> Self {
        Self {
            tokens,
            bounds: Bounds::new(0, 1),
            wide: false,
            literal: None,
        }
    }

    /// Returns the expression as a `u64`.
    fn widened(self) -> TokenStream {
        if self.wide {
            self.tokens
        } else if let Some(value) = self.literal {
            number(format_args!("{value}u64"))
        } else {
            let tokens = self.tokens;
            quote! { (#tokens as u64) }
        }
    }

    /// Returns the expression as the `u32` that frequencies and parameter values are stored in.
    fn narrowed(self) -> TokenStream {
        if self.wide {
            let tokens = self.tokens;
            quote! { (#tokens) as u32 }
        } else {
            self.tokens
        }
    }
}

pub struct ExprCompiler<'ctx> {
    variables: &'ctx HashMap<&'ctx str, Operand>,
}

impl<'ctx> ExprCompiler<'ctx> {
    pub fn new(variables: &'ctx HashMap<&'ctx str, Operand>) -> Self {
        Self { variables }
    }

    pub fn compile_expression(
        &self,
        expression: &Expression,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let source = expression.source.as_str();
        self.compile(source, &expression.expr, instance, tree)
            .narrowed()
    }

    pub fn compile_right_hand_expression(
        &self,
        source: &str,
        expression: &ast::RightHandExpression<DefaultTypeSet>,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        self.compile(source, expression, instance, tree).narrowed()
    }

    fn compile(
        &self,
        source: &str,
        expression: &ast::RightHandExpression<DefaultTypeSet>,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Compiled {
        match expression {
            ast::RightHandExpression::Variable { variable } => {
                self.compile_variable(source, variable)
            }
            ast::RightHandExpression::Literal { value } => self.compile_literal(value),
            ast::RightHandExpression::UnaryOperator { name, operand } => {
                self.compile_unary_operator(source, name, operand, instance, tree)
            }
            ast::RightHandExpression::BinaryOperator { name, operands } => {
                self.compile_binary_operator(source, name, operands, instance, tree)
            }
            ast::RightHandExpression::FunctionCall { name, arguments } => {
                self.compile_function_call(source, name, arguments, instance, tree)
            }
            _ => unimplemented!("Unsupported expression: {:#?}", expression),
        }
    }

    fn compile_variable(&self, source: &str, variable: &Token) -> Compiled {
        let variable_name = variable.source(source);
        let operand = self
            .variables
            .get(variable_name)
            .unwrap_or_else(|| panic!("Variable not found: {variable_name:?}"));

        Compiled {
            tokens: operand.tokens.clone(),
            bounds: operand.bounds,
            wide: false,
            literal: None,
        }
    }

    fn compile_unary_operator(
        &self,
        source: &str,
        name: &Token,
        operand: &ast::RightHandExpression<DefaultTypeSet>,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Compiled {
        let operator = match name.source(source) {
            "!" => quote! { ! },
            "-" => quote! { - },
            other => todo!("Unsupported unary operator: {other}"),
        };

        let operand = self.compile(source, operand, instance, tree);
        let bounds = operand.bounds;
        let wide = operand.wide;
        let tokens = operand.tokens;

        Compiled {
            tokens: quote! { #operator (#tokens) },
            bounds,
            wide,
            literal: None,
        }
    }

    fn compile_binary_operator(
        &self,
        source: &str,
        name: &Token,
        operands: &[ast::RightHandExpression<DefaultTypeSet>; 2],
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Compiled {
        let operator_str = name.source(source);
        let operator = match operator_str {
            "+" => quote! { + },
            "-" => quote! { - },
            "*" => quote! { * },
            "/" => quote! { / },
            "%" => quote! { % },
            "&&" => quote! { && },
            "||" => quote! { || },
            "==" => quote! { == },
            "!=" => quote! { != },
            "<" => quote! { < },
            ">" => quote! { > },
            "<=" => quote! { <= },
            ">=" => quote! { >= },
            other => todo!("Unsupported binary operator: {other}"),
        };

        let lhs = self.compile(source, &operands[0], instance, tree);
        let rhs = self.compile(source, &operands[1], instance, tree);

        let Some(bounds) = lhs.bounds.apply_operator(operator_str, rhs.bounds) else {
            // Comparisons and logical operators produce a boolean. Their operands still need to
            // agree on a width.
            let (lhs, rhs) = if lhs.wide || rhs.wide {
                (lhs.widened(), rhs.widened())
            } else {
                (lhs.tokens, rhs.tokens)
            };

            return Compiled::boolean(quote! { (#lhs #operator #rhs) });
        };

        // Compute in 64 bits if the result can't be represented in 32, or if an operand already
        // had to be widened.
        let wide = lhs.wide || rhs.wide || !bounds.fits_in_u32();
        let (lhs, rhs) = if wide {
            (lhs.widened(), rhs.widened())
        } else {
            (lhs.tokens, rhs.tokens)
        };

        Compiled {
            tokens: quote! { (#lhs #operator #rhs) },
            bounds,
            wide,
            literal: None,
        }
    }

    fn compile_literal(&self, value: &ast::Literal<DefaultTypeSet>) -> Compiled {
        match &value.value {
            ast::LiteralValue::Integer(n) => Compiled::number(*n),
            ast::LiteralValue::Float(n) => Compiled {
                tokens: number(n),
                bounds: Bounds::UNKNOWN,
                wide: false,
                literal: None,
            },
            ast::LiteralValue::String(s) => Compiled::boolean(quote! { #s }),
            ast::LiteralValue::Boolean(b) => Compiled::boolean(quote! { #b }),
        }
    }

    fn compile_function_call(
        &self,
        source: &str,
        name: &Token,
        arguments: &[ast::RightHandExpression<DefaultTypeSet>],
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> Compiled {
        // property(NODE)
        let [ast::RightHandExpression::Variable { variable }] = arguments else {
            panic!("Function calls must be of the form property(NODE)")
        };

        let referred_node = instance.resolve_node(tree, variable.source(source));
        let config_accessor = referred_node.properties.indexed_config_accessor();
        let name = quote::format_ident!("{}", name.source(source));

        Compiled {
            tokens: quote! { unwrap!(#config_accessor).#name() },
            bounds: Bounds::UNKNOWN,
            wide: false,
            literal: None,
        }
    }
}
