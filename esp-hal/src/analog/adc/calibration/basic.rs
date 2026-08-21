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
pub struct AdcCalBasic<ADCX> {
    /// Calibration value to set to ADC unit
    cal_val: u16,

    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
    chan_compens: i32,

    _phantom: PhantomData<ADCX>,
}

impl<ADCX> crate::private::Sealed for AdcCalBasic<ADCX> {}

impl<ADCX> AdcCalScheme<ADCX> for AdcCalBasic<ADCX>
where
    ADCX: AdcCalEfuse + CalibrationAccess,
{
    fn new_cal(atten: Attenuation) -> Self {
        Self::new_cal_with_channel(atten, 0)
    }

    fn new_cal_with_channel(atten: Attenuation, _channel: u8) -> Self {
        // Try to get init code (Dout0) from efuse
        // Dout0 means mean raw ADC value when zero voltage applied to input.
        let cal_val = ADCX::init_code(atten).unwrap_or_else(|| {
            // As a fallback try to calibrate via connecting input to ground internally.
            AdcConfig::<ADCX>::adc_calibrate(atten, AdcCalSource::Gnd)
        });

        #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
        let chan_compens = ADCX::cal_chan_compens(atten, _channel).unwrap_or(0);

        Self {
            cal_val,
            #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
            chan_compens,
            _phantom: PhantomData,
        }
    }

    fn adc_cal(&self) -> u16 {
        self.cal_val
    }

    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
    fn adc_val(&self, val: u16) -> u16 {
        (val as i32 - self.chan_compens).clamp(0, ADCX::ADC_VAL_MASK as i32) as u16
    }
}
