use core::marker::PhantomData;

use crate::analog::adc::{
    AdcCalEfuse,
    AdcCalScheme,
    AdcCalSource,
    AdcConfig,
    Attenuation,
    CalibrationAccess,
};

/// Basic ADC calibration scheme
///
/// Basic calibration sets the initial ADC bias value so that a zero voltage
/// gives a reading of zero. The correct bias value is usually stored in efuse,
/// but can also be measured at runtime by connecting the ADC input to ground
/// internally.
///
/// Failing to apply basic calibration can substantially reduce the ADC's output
/// range because bias correction is done *before* the ADC's output is truncated
/// to 12 bits.
#[derive(Clone, Copy)]
pub struct AdcCalBasic<ADCI> {
    /// Calibration value to set to ADC unit
    cal_val: u16,

    _phantom: PhantomData<ADCI>,
}

impl<ADCI> crate::private::Sealed for AdcCalBasic<ADCI> {}

impl<ADCI> AdcCalScheme<ADCI> for AdcCalBasic<ADCI>
where
    ADCI: AdcCalEfuse + CalibrationAccess,
{
    fn new_cal(atten: Attenuation) -> Self {
        // Try to get init code (Dout0) from efuse
        // Dout0 means mean raw ADC value when zero voltage applied to input.
        let cal_val = ADCI::init_code(atten).unwrap_or_else(|| {
            // As a fallback try to calibrate via connecting input to ground internally.
            AdcConfig::<ADCI>::adc_calibrate(atten, AdcCalSource::Gnd)
        });

        Self {
            cal_val,
            _phantom: PhantomData,
        }
    }

    fn adc_cal(&self) -> u16 {
        self.cal_val
    }
}
