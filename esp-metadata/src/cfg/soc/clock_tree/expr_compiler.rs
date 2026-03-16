use std::collections::HashMap;

use proc_macro2::TokenStream;
use quote::quote;
use somni_expr::DefaultTypeSet;
use somni_parser::{ast, lexer::Token};

use crate::{
    cfg::{ClockTreeNodeInstance, ProcessedClockData, clock_tree::Expression},
    number,
};

pub struct ExprCompiler<'ctx> {
    variables: &'ctx HashMap<&'ctx str, TokenStream>,
}

impl<'ctx> ExprCompiler<'ctx> {
    pub fn new(variables: &'ctx HashMap<&'ctx str, TokenStream>) -> Self {
        Self { variables }
    }

    pub fn compile_expression(
        &self,
        expression: &Expression,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let source = expression.source.as_str();
        self.compile_right_hand_expression(source, &expression.expr, instance, tree)
    }

    pub fn compile_right_hand_expression(
        &self,
        source: &str,
        expression: &ast::RightHandExpression<DefaultTypeSet>,
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
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
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        let operator = match name.source(source) {
            "!" => quote! { ! },
            "-" => quote! { - },
            other => todo!("Unsupported unary operator: {other}"),
        };

        let operand = self.compile_right_hand_expression(source, operand, instance, tree);
        quote! { #operator (#operand) }
    }

    fn compile_binary_operator(
        &self,
        source: &str,
        name: &Token,
        operands: &[ast::RightHandExpression<DefaultTypeSet>; 2],
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
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

        let lhs = self.compile_right_hand_expression(source, &operands[0], instance, tree);
        let rhs = self.compile_right_hand_expression(source, &operands[1], instance, tree);
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

    fn compile_function_call(
        &self,
        source: &str,
        name: &Token,
        arguments: &[ast::RightHandExpression<DefaultTypeSet>],
        instance: &ClockTreeNodeInstance,
        tree: &ProcessedClockData,
    ) -> TokenStream {
        // property(NODE)
        let [ast::RightHandExpression::Variable { variable }] = arguments else {
            panic!("Function calls must be of the form property(NODE)")
        };

        let referred_node = instance.resolve_node(tree, variable.source(source));
        let node_props = tree.properties(referred_node.name_str());
        let node_field = quote::format_ident!("{}", node_props.field_name());
        let name = quote::format_ident!("{}", name.source(source));
        quote! { unwrap!(clocks.#node_field).#name() }
    }
}
