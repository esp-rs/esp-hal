use core::marker::PhantomData;

use crate::adc::{AdcCalEfuse, AdcCalScheme, AdcCalSource, AdcConfig, Attenuation, RegisterAccess};

/// Basic ADC calibration scheme
///
/// Basic calibration is related to setting some initial bias value in ADC.
/// Such values usually is stored in efuse bit fields but also can be measured
/// in runtime by connecting ADC input to ground internally a fallback when
/// it is not available.
#[derive(Clone, Copy)]
pub struct AdcCalBasic<ADCI> {
    /// Calibration value to set to ADC unit
    cal_val: u16,

    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcCalScheme<ADCI> for AdcCalBasic<ADCI>
where
    ADCI: AdcCalEfuse + RegisterAccess,
{
    fn new_cal(atten: Attenuation) -> Self {
        // Try to get init code (Dout0) from efuse
        // Dout0 means mean raw ADC value when zero voltage applied to input.
        let cal_val = ADCI::get_init_code(atten).unwrap_or_else(|| {
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
