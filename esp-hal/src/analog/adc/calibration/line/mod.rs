#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "s2.rs")]
#[cfg_attr(not(any(esp32, esp32s2)), path = "one_point.rs")]
mod line_impl;
pub use line_impl::AdcCalLine;

/// Marker trait for ADC units which support line fitting.
pub trait AdcHasLineCal: crate::private::Sealed {
    /// ADC unit index used by the eFuse calibration tables (0 = ADC1, 1 = ADC2).
    const UNIT: u8;
}

#[cfg(soc_has_adc1)]
impl AdcHasLineCal for crate::peripherals::ADC1<'_> {
    const UNIT: u8 = 0;
}

#[cfg(soc_has_adc2)]
impl AdcHasLineCal for crate::peripherals::ADC2<'_> {
    const UNIT: u8 = 1;
}
