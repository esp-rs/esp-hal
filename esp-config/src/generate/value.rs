use std::fmt;

use serde::{Deserialize, Serialize};

use super::Error;

/// Supported configuration value types.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Value {
    /// Booleans.
    Bool(bool),
    /// Integers.
    Integer(i128),
    /// Strings.
    String(String),
}

impl Serialize for Value {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match self {
            Value::String(s) => serializer.serialize_str(&format!("\"{}\"", s)),
            Value::Integer(n) => serializer.serialize_str(&format!("{}", n)),
            Value::Bool(b) => serializer.serialize_str(&format!("{}", b)),
        }
    }
}

impl<'de> Deserialize<'de> for Value {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct ValueVisitor;

        impl<'de> serde::de::Visitor<'de> for ValueVisitor {
            type Value = String;

            fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                formatter.write_str("a String representing the Value")
            }

            fn visit_string<E>(self, v: String) -> Result<Self::Value, E>
            where
                E: serde::de::Error,
            {
                Ok(v)
            }

            fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
            where
                E: serde::de::Error,
            {
                Ok(v.to_string())
            }
        }

        let str_repr = deserializer.deserialize_string(ValueVisitor)?;
        let str_repr = str_repr.as_str();

        if let Some(remaining) = str_repr.strip_prefix("\"") {
            let s = &remaining[..remaining.len() - 1];
            return Ok(Value::String(s.to_string()));
        }

        if str_repr == "true" {
            return Ok(Value::Bool(true));
        }

        if str_repr == "false" {
            return Ok(Value::Bool(false));
        }

        Ok(Value::Integer(str_repr.parse().map_err(
            |e: core::num::ParseIntError| serde::de::Error::custom(e.to_string()),
        )?))
    }
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
                    )));
                }
            },
            Value::Integer(_) => {
                let inner = match s.as_bytes() {
                    [b'0', b'x', ..] => i128::from_str_radix(&s[2..], 16),
                    [b'0', b'o', ..] => i128::from_str_radix(&s[2..], 8),
                    [b'0', b'b', ..] => i128::from_str_radix(&s[2..], 2),
                    _ => s.parse(),
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

impl From<bool> for Value {
    fn from(value: bool) -> Self {
        Value::Bool(value)
    }
}

impl From<i128> for Value {
    fn from(value: i128) -> Self {
        Value::Integer(value)
    }
}

impl From<&str> for Value {
    fn from(value: &str) -> Self {
        Value::String(value.to_string())
    }
}

impl From<String> for Value {
    fn from(value: String) -> Self {
        Value::String(value)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn deserialization_number() {
        let yml = "128";
        let value: Value = serde_yml::from_str(yml).unwrap();
        assert_eq!(value, Value::Integer(128));
    }

    #[test]
    fn deserialization_string() {
        let yml = "'\"Hello\"'";
        let value: Value = serde_yml::from_str(yml).unwrap();
        assert_eq!(value, Value::String("Hello".to_string()));
    }
}
