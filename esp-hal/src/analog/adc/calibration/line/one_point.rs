use core::marker::PhantomData;

use super::AdcHasLineCal;
use crate::analog::adc::{
    AdcCalBasic,
    AdcCalEfuse,
    AdcCalScheme,
    Attenuation,
    CalibrationAccess,
    Error,
};

/// Gain is stored as a `u32`, but is really a fixed-point number.
const GAIN_SCALE: u32 = 1 << 16;

/// Line fitting ADC calibration scheme.
///
/// This scheme implements gain correction based on reference points, and
/// returns readings in mV.
///
/// A reference point is a pair of a reference voltage and the corresponding
/// mean raw digital ADC value. Those values are stored in eFuse bit fields for each supported
/// attenuation, and there is no way to establish them at runtime: the ADC can only be switched to
/// internal ground, which gives the zero-voltage offset rather than a second point to derive a
/// gain from.
///
/// This scheme also includes basic calibration ([`AdcCalBasic`]).
///
/// # Errors
///
/// Returns [`Error::UnsupportedAttenuation`] if the attenuation has no
/// reference point on this chip, or [`Error::MissingCalibrationData`] if the
/// chip carries no line-fitting calibration data or the stored reference
/// point is zero.
#[derive(Clone, Copy)]
pub struct AdcCalLine<ADCX> {
    basic: AdcCalBasic<ADCX>,

    /// ADC gain.
    ///
    /// After being de-biased by the basic calibration, the reading is
    /// multiplied by this value. Despite the type, it is a fixed-point
    /// number with 16 fractional bits.
    gain: u32,

    _phantom: PhantomData<ADCX>,
}

impl<ADCX> crate::private::Sealed for AdcCalLine<ADCX> {}

impl<ADCX> AdcCalScheme<ADCX> for AdcCalLine<ADCX>
where
    ADCX: AdcCalEfuse + AdcHasLineCal + CalibrationAccess,
{
    fn new_cal(atten: Attenuation) -> Result<Self, Error> {
        Self::new_cal_with_channel(atten, 0)
    }

    fn new_cal_with_channel(atten: Attenuation, channel: u8) -> Result<Self, Error> {
        let basic = AdcCalBasic::<ADCX>::new_cal_with_channel(atten, channel)?;

        // Get the reference point (Dout, Vin) from efuse. Dout means mean raw ADC value when
        // specified Vin applied to input.
        // `cal_mv` goes first: it rejects attenuations without a reference point on the ESP32-C2.
        let Some(mv) = ADCX::cal_mv(atten) else {
            return Err(Error::UnsupportedAttenuation(atten));
        };
        let Some(code) = ADCX::cal_code(atten) else {
            return Err(Error::MissingCalibrationData);
        };

        // A zero reference point would divide by zero below; it means the eFuse
        // field was never burnt.
        if code == 0 {
            return Err(Error::MissingCalibrationData);
        }

        // Estimate the (assumed) linear relationship between the measured raw value and
        // the voltage with the previously done measurement when the chip was
        // manufactured.
        //
        // Note that the constant term is zero because the driver programs the
        // zero-voltage bias for every conversion.
        let gain = mv as u32 * GAIN_SCALE / code as u32;

        Ok(Self {
            basic,
            gain,
            _phantom: PhantomData,
        })
    }

    fn adc_val(&self, val: u16) -> u16 {
        let val = self.basic.adc_val(val);

        (val as u32 * self.gain / GAIN_SCALE) as u16
    }
}
