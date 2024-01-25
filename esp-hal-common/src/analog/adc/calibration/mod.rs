#[cfg(any(esp32c3, esp32c6, esp32s3))]
pub use self::curve::{AdcCalCurve, AdcHasCurveCal};
#[cfg(any(esp32c2, esp32c3, esp32c6, esp32s3))]
pub use self::{
    basic::AdcCalBasic,
    line::{AdcCalLine, AdcHasLineCal},
};

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32s3))]
mod basic;
#[cfg(any(esp32c3, esp32c6, esp32s3))]
mod curve;
#[cfg(any(esp32c2, esp32c3, esp32c6, esp32s3))]
mod line;
