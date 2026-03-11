use std::collections::HashMap;

use proc_macro2::TokenStream;
use quote::quote;
use somni_expr::DefaultTypeSet;
use somni_parser::{ast, lexer::Token};

use crate::{cfg::clock_tree::Expression, number};

pub struct ExprCompiler<'ctx> {
    variables: &'ctx HashMap<&'ctx str, TokenStream>,
}

impl<'ctx> ExprCompiler<'ctx> {
    pub fn new(variables: &'ctx HashMap<&'ctx str, TokenStream>) -> Self {
        Self { variables }
    }

    pub fn compile_expression(&self, expression: &Expression) -> TokenStream {
        let source = expression.source.as_str();
        let ast::Expression::Expression { expression } = &expression.expr else {
            panic!("Invalid expression");
        };
        self.compile_right_hand_expression(source, expression)
    }

    fn compile_right_hand_expression(
        &self,
        source: &str,
        expression: &ast::RightHandExpression<DefaultTypeSet>,
    ) -> TokenStream {
        match expression {
            ast::RightHandExpression::Variable { variable } => {
                self.compile_variable(source, variable)
            }
            ast::RightHandExpression::Literal { value } => self.compile_literal(value),
            ast::RightHandExpression::UnaryOperator { name, operand } => {
                self.compile_unary_operator(source, name, operand)
            }
            ast::RightHandExpression::BinaryOperator { name, operands } => {
                self.compile_binary_operator(source, name, operands)
            }
            ast::RightHandExpression::FunctionCall { .. } => {
                panic!("Function calls are not supported")
            }
        }
    }

    fn compile_variable(&self, source: &str, variable: &Token) -> TokenStream {
        let variable_name = variable.source(source);
        self.variables
            .get(variable_name)
            .cloned()
            .unwrap_or_else(|| panic!("Variable not found: {variable_name:?}"))
    }

    fn compile_unary_operator(
        &self,
        source: &str,
        name: &Token,
        operand: &ast::RightHandExpression<DefaultTypeSet>,
    ) -> TokenStream {
        let operator = match name.source(source) {
            "!" => quote! { ! },
            "-" => quote! { - },
            other => todo!("Unsupported unary operator: {other}"),
        };

        let operand = self.compile_right_hand_expression(source, operand);
        quote! { #operator (#operand) }
    }

    fn compile_binary_operator(
        &self,
        source: &str,
        name: &Token,
        operands: &[ast::RightHandExpression<DefaultTypeSet>; 2],
    ) -> TokenStream {
        let operator = match name.source(source) {
            "+" => quote! { + },
            "-" => quote! { - },
            "*" => quote! { * },
            "/" => quote! { / },
            "%" => quote! { % },
            "&&" => quote! { && },
            "==" => quote! { == },
            other => todo!("Unsupported binary operator: {other}"),
        };

        let lhs = self.compile_right_hand_expression(source, &operands[0]);
        let rhs = self.compile_right_hand_expression(source, &operands[1]);
        quote! { (#lhs #operator #rhs) }
    }

    fn compile_literal(&self, value: &ast::Literal<DefaultTypeSet>) -> TokenStream {
        match &value.value {
            ast::LiteralValue::Integer(n) => number(n),
            ast::LiteralValue::Float(n) => number(n),
            ast::LiteralValue::String(s) => quote! { #s },
            ast::LiteralValue::Boolean(b) => quote! { #b },
        }
    }
}
