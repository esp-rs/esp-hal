use core::marker::PhantomData;

use crate::analog::adc::{AdcCalEfuse, AdcCalScheme, Attenuation, CalibrationAccess, Error};

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
    #[cfg(adc_channel_compensation)]
    chan_compens: i32,

    _phantom: PhantomData<ADCX>,
}

impl<ADCX> crate::private::Sealed for AdcCalBasic<ADCX> {}

impl<ADCX> AdcCalScheme<ADCX> for AdcCalBasic<ADCX>
where
    ADCX: AdcCalEfuse + CalibrationAccess,
{
    fn new_cal(atten: Attenuation) -> Result<Self, Error> {
        Self::new_cal_with_channel(atten, 0)
    }

    fn new_cal_with_channel(_atten: Attenuation, _channel: u8) -> Result<Self, Error> {
        Ok(Self {
            // A missing per-channel offset means the chip did not characterize this
            // channel, so no correction applies. `channel` always comes from a valid
            // ADC pin, and unburnt eFuses likewise mean zero correction.
            #[cfg(adc_channel_compensation)]
            chan_compens: ADCX::cal_chan_compens(_atten, _channel).unwrap_or(0),
            _phantom: PhantomData,
        })
    }

    #[cfg(adc_channel_compensation)]
    fn adc_val(&self, val: u16) -> u16 {
        (val as i32 - self.chan_compens).clamp(0, ADCX::ADC_VAL_MASK as i32) as u16
    }
}
