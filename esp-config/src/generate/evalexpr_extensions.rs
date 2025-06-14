#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct I128NumericTypes;

#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct I128(pub i128);

impl FromStr for I128 {
    type Err = core::num::ParseIntError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(I128(s.parse()?))
    }
}

impl core::fmt::Display for I128 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

use core::str::FromStr;

use evalexpr::{EvalexprError, EvalexprResult};

impl<NumericTypes: evalexpr::EvalexprNumericTypes<Int = Self>> evalexpr::EvalexprInt<NumericTypes>
    for I128
{
    const MIN: Self = I128(i128::MIN);

    const MAX: Self = I128(i128::MAX);

    fn from_usize(int: usize) -> EvalexprResult<Self, NumericTypes> {
        Ok(I128(int.try_into().map_err(|_| {
            EvalexprError::IntFromUsize { usize_int: int }
        })?))
    }

    fn into_usize(&self) -> EvalexprResult<usize, NumericTypes> {
        if self.0 >= 0 {
            (self.0 as u64)
                .try_into()
                .map_err(|_| EvalexprError::IntIntoUsize { int: *self })
        } else {
            Err(EvalexprError::IntIntoUsize { int: *self })
        }
    }

    fn from_hex_str(literal: &str) -> Result<Self, ()> {
        Ok(I128(i128::from_str_radix(literal, 16).map_err(|_| ())?))
    }

    fn checked_add(&self, rhs: &Self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_add(rhs.0);
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::AdditionError {
                augend: evalexpr::Value::<NumericTypes>::from_int(*self),
                addend: evalexpr::Value::<NumericTypes>::from_int(*rhs),
            })
        }
    }

    fn checked_sub(&self, rhs: &Self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_sub(rhs.0);
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::SubtractionError {
                minuend: evalexpr::Value::<NumericTypes>::from_int(*self),
                subtrahend: evalexpr::Value::<NumericTypes>::from_int(*rhs),
            })
        }
    }

    fn checked_neg(&self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_neg();
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::NegationError {
                argument: evalexpr::Value::<NumericTypes>::from_int(*self),
            })
        }
    }

    fn checked_mul(&self, rhs: &Self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_mul(rhs.0);
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::MultiplicationError {
                multiplicand: evalexpr::Value::<NumericTypes>::from_int(*self),
                multiplier: evalexpr::Value::<NumericTypes>::from_int(*rhs),
            })
        }
    }

    fn checked_div(&self, rhs: &Self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_div(rhs.0);
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::DivisionError {
                dividend: evalexpr::Value::<NumericTypes>::from_int(*self),
                divisor: evalexpr::Value::<NumericTypes>::from_int(*rhs),
            })
        }
    }

    fn checked_rem(&self, rhs: &Self) -> EvalexprResult<Self, NumericTypes> {
        let result = (self.0).checked_rem(rhs.0);
        if let Some(result) = result {
            Ok(I128(result))
        } else {
            Err(EvalexprError::ModulationError {
                dividend: evalexpr::Value::<NumericTypes>::from_int(*self),
                divisor: evalexpr::Value::<NumericTypes>::from_int(*rhs),
            })
        }
    }

    fn abs(&self) -> EvalexprResult<Self, NumericTypes> {
        Ok(I128(self.0.abs()))
    }

    fn bitand(&self, rhs: &Self) -> Self {
        I128(std::ops::BitAnd::bitand(self.0, rhs.0))
    }

    fn bitor(&self, rhs: &Self) -> Self {
        I128(std::ops::BitOr::bitor(self.0, rhs.0))
    }

    fn bitxor(&self, rhs: &Self) -> Self {
        I128(std::ops::BitXor::bitxor(self.0, rhs.0))
    }

    fn bitnot(&self) -> Self {
        I128(std::ops::Not::not(self.0))
    }

    fn bit_shift_left(&self, rhs: &Self) -> Self {
        I128(std::ops::Shl::shl(self.0, rhs.0))
    }

    fn bit_shift_right(&self, rhs: &Self) -> Self {
        I128(std::ops::Shr::shr(self.0, rhs.0))
    }
}

impl evalexpr::EvalexprNumericTypes for I128NumericTypes {
    type Int = I128;
    type Float = f64;

    fn int_as_float(int: &Self::Int) -> Self::Float {
        int.0 as Self::Float
    }

    fn float_as_int(float: &Self::Float) -> Self::Int {
        I128(float.trunc() as i128)
    }
}
