use std::{io::Write, ops::Range};

use serde::Serialize;

use super::{snake_case, value::Value, Error};

/// Configuration value validation functions.
#[derive(Serialize)]
pub enum Validator {
    /// Only allow negative integers, i.e. any values less than 0.
    NegativeInteger,
    /// Only allow non-negative integers, i.e. any values greater than or
    /// equal to 0.
    NonNegativeInteger,
    /// Only allow positive integers, i.e. any values greater than to 0.
    PositiveInteger,
    /// Ensure that an integer value falls within the specified range.
    IntegerInRange(Range<i128>),
    /// String-Enumeration. Only allows one of the given Strings.
    Enumeration(Vec<String>),
    /// A custom validation function to run against any supported value
    /// type.
    #[serde(serialize_with = "serialize_custom")]
    #[serde(untagged)]
    Custom(Box<dyn Fn(&Value) -> Result<(), Error>>),
}

pub(crate) fn serialize_custom<S>(
    _: &Box<dyn Fn(&Value) -> Result<(), Error>>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    serializer.serialize_str("Custom")
}

impl Validator {
    pub(crate) fn validate(&self, value: &Value) -> Result<(), Error> {
        match self {
            Validator::NegativeInteger => negative_integer(value)?,
            Validator::NonNegativeInteger => non_negative_integer(value)?,
            Validator::PositiveInteger => positive_integer(value)?,
            Validator::IntegerInRange(range) => integer_in_range(range, value)?,
            Validator::Enumeration(values) => enumeration(values, value)?,
            Validator::Custom(validator_fn) => validator_fn(value)?,
        }

        Ok(())
    }

    pub(crate) fn description(&self) -> Option<String> {
        match self {
            Validator::NegativeInteger => Some(String::from("Negative integer")),
            Validator::NonNegativeInteger => Some(String::from("Positive integer or 0")),
            Validator::PositiveInteger => Some(String::from("Positive integer")),
            Validator::IntegerInRange(range) => {
                Some(format!("Integer in range {}..{}", range.start, range.end))
            }
            Validator::Enumeration(values) => Some(format!(
                "One of: <ul>{}</ul>",
                values
                    .iter()
                    .map(|v| format!("<li>{}</li>", v))
                    .collect::<Vec<_>>()
                    .join("")
            )),
            Validator::Custom(_) => None,
        }
    }

    pub(crate) fn emit_cargo_extras(
        &self,
        mut stdout: impl Write,
        config_key: &str,
        actual_value: &Value,
    ) {
        match self {
            Validator::Enumeration(values) => {
                for possible_value in values {
                    writeln!(
                        stdout,
                        "cargo:rustc-check-cfg=cfg({config_key}_{})",
                        snake_case(possible_value)
                    )
                    .ok();
                }

                writeln!(
                    stdout,
                    "cargo:rustc-cfg={config_key}_{}",
                    snake_case(&actual_value.to_string())
                )
                .ok();
            }
            _ => (),
        }
    }
}

pub(crate) fn enumeration(values: &Vec<String>, value: &Value) -> Result<(), Error> {
    if let Value::String(value) = value {
        if !values.contains(value) {
            return Err(Error::validation(format!(
                "Expected one of {:?}, found '{}'",
                values, value
            )));
        }

        Ok(())
    } else {
        return Err(Error::parse(
            "Validator::Enumeration can only be used with string values",
        ));
    }
}

pub(crate) fn negative_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::NegativeInteger can only be used with integer values",
        ));
    } else if value.as_integer() >= 0 {
        return Err(Error::validation(format!(
            "Expected negative integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

pub(crate) fn non_negative_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::NonNegativeInteger can only be used with integer values",
        ));
    } else if value.as_integer() < 0 {
        return Err(Error::validation(format!(
            "Expected non-negative integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

pub(crate) fn positive_integer(value: &Value) -> Result<(), Error> {
    if !value.is_integer() {
        return Err(Error::validation(
            "Validator::PositiveInteger can only be used with integer values",
        ));
    } else if value.as_integer() <= 0 {
        return Err(Error::validation(format!(
            "Expected positive integer, found '{}'",
            value.as_integer()
        )));
    }

    Ok(())
}

pub(crate) fn integer_in_range(range: &Range<i128>, value: &Value) -> Result<(), Error> {
    if !value.is_integer() || !range.contains(&value.as_integer()) {
        Err(Error::validation(format!(
            "Value '{}' does not fall within range '{:?}'",
            value, range
        )))
    } else {
        Ok(())
    }
}
