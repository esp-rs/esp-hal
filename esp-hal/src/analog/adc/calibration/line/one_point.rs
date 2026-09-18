use core::marker::PhantomData;

use super::AdcHasLineCal;
use crate::analog::adc::{AdcCalBasic, AdcCalEfuse, AdcCalScheme, Attenuation, CalibrationAccess};

/// Gain is stored as a `u32`, but is really a fixed-point number.
const GAIN_SCALE: u32 = 1 << 16;

/// Line fitting ADC calibration scheme
///
/// This scheme implements gain correction based on reference points, and
/// returns readings in mV.
///
/// A reference point is a pair of a reference voltage and the corresponding
/// mean raw digital ADC value. Those values are stored in eFuse bit fields for each supported
/// attenuation, and there is no way to establish them at runtime: the ADC can only be switched to
/// internal ground, which gives the zero-voltage offset rather than a second point to derive a
/// gain from. ESP-IDF likewise refuses to build this scheme without the eFuse data.
///
/// This scheme also includes basic calibration ([`AdcCalBasic`]).
///
/// # Panics
///
/// Panics if the chip carries no line-fitting calibration data in eFuse, or if that data gives a
/// reference point of zero.
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
    fn new_cal(atten: Attenuation) -> Self {
        Self::new_cal_with_channel(atten, 0)
    }

    fn new_cal_with_channel(atten: Attenuation, channel: u8) -> Self {
        let basic = AdcCalBasic::<ADCX>::new_cal_with_channel(atten, channel);

        // Get the reference point (Dout, Vin) from efuse. Dout means mean raw ADC value when
        // specified Vin applied to input.
        let Some(code) = ADCX::cal_code(atten) else {
            panic!("This chip needs eFuse calibration data for line fitting")
        };
        let mv = ADCX::cal_mv(atten);

        // Guards the division below, which would otherwise divide by zero.
        assert!(code != 0, "ADC calibration reference point is zero");

        // Estimate the (assumed) linear relationship between the measured raw value and
        // the voltage with the previously done measurement when the chip was
        // manufactured.
        //
        // Note that the constant term is zero because the driver programs the
        // zero-voltage bias for every conversion.
        let gain = mv as u32 * GAIN_SCALE / code as u32;

        Self {
            basic,
            gain,
            _phantom: PhantomData,
        }
    }

    fn adc_val(&self, val: u16) -> u16 {
        let val = self.basic.adc_val(val);

        (val as u32 * self.gain / GAIN_SCALE) as u16
    }
}
