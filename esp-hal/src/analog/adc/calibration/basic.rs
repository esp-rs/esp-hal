use core::marker::PhantomData;

use crate::analog::adc::{AdcCalEfuse, AdcCalScheme, Attenuation, CalibrationAccess};

/// Basic ADC calibration scheme.
///
/// The zero-voltage bias (`Dout0`) is applied by the driver for every
/// conversion, whether or not a calibration scheme is in use, because the
/// hardware subtracts it before truncating the result.
///
/// What is left for this scheme is the per-channel correction to that bias,
/// which only some chips characterize in eFuse. On the others this scheme
/// returns readings unchanged.
#[derive(Clone, Copy)]
pub struct AdcCalBasic<ADCX> {
    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
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

    fn new_cal_with_channel(_atten: Attenuation, _channel: u8) -> Self {
        Self {
            #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
            chan_compens: ADCX::cal_chan_compens(_atten, _channel).unwrap_or(0),
            _phantom: PhantomData,
        }
    }

    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
    fn adc_val(&self, val: u16) -> u16 {
        (val as i32 - self.chan_compens).clamp(0, ADCX::ADC_VAL_MASK as i32) as u16
    }
}
