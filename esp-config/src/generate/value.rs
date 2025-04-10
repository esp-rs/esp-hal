use std::fmt;

use serde::Serialize;

use super::Error;

/// Supported configuration value types.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum Value {
    /// Booleans.
    Bool(bool),
    /// Integers.
    Integer(i128),
    /// Strings.
    String(String),
}

// TODO: Do we want to handle negative values for non-decimal values?
impl Value {
    pub(crate) fn parse_in_place(&mut self, s: &str) -> Result<(), Error> {
        *self = match self {
            Value::Bool(_) => match s {
                "true" => Value::Bool(true),
                "false" => Value::Bool(false),
                _ => {
                    return Err(Error::parse(format!(
                        "Expected 'true' or 'false', found: '{s}'"
                    )))
                }
            },
            Value::Integer(_) => {
                let inner = match s.as_bytes() {
                    [b'0', b'x', ..] => i128::from_str_radix(&s[2..], 16),
                    [b'0', b'o', ..] => i128::from_str_radix(&s[2..], 8),
                    [b'0', b'b', ..] => i128::from_str_radix(&s[2..], 2),
                    _ => i128::from_str_radix(&s, 10),
                }
                .map_err(|_| Error::parse(format!("Expected valid intger value, found: '{s}'")))?;

                Value::Integer(inner)
            }
            Value::String(_) => Value::String(s.into()),
        };

        Ok(())
    }

    /// Convert the value to a [bool].
    pub fn as_bool(&self) -> bool {
        match self {
            Value::Bool(value) => *value,
            _ => panic!("attempted to convert non-bool value to a bool"),
        }
    }

    /// Convert the value to an [i128].
    pub fn as_integer(&self) -> i128 {
        match self {
            Value::Integer(value) => *value,
            _ => panic!("attempted to convert non-integer value to an integer"),
        }
    }

    /// Convert the value to a [String].
    pub fn as_string(&self) -> String {
        match self {
            Value::String(value) => value.to_owned(),
            _ => panic!("attempted to convert non-string value to a string"),
        }
    }

    /// Is the value a bool?
    pub fn is_bool(&self) -> bool {
        matches!(self, Value::Bool(_))
    }

    /// Is the value an integer?
    pub fn is_integer(&self) -> bool {
        matches!(self, Value::Integer(_))
    }

    /// Is the value a string?
    pub fn is_string(&self) -> bool {
        matches!(self, Value::String(_))
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(b) => write!(f, "{b}"),
            Value::Integer(i) => write!(f, "{i}"),
            Value::String(s) => write!(f, "{s}"),
        }
    }
}
