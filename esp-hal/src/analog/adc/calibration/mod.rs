#[cfg(adc_basic_calibration)]
pub use self::basic::AdcCalBasic;
#[cfg(adc_curve_calibration)]
pub use self::curve::{AdcCalCurve, AdcHasCurveCal};
#[cfg(adc_line_calibration)]
pub use self::line::{AdcCalLine, AdcHasLineCal};

#[cfg(adc_basic_calibration)]
mod basic;
#[cfg(adc_curve_calibration)]
mod curve;
#[cfg(adc_line_calibration)]
mod line;
