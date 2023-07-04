use core::marker::PhantomData;

use crate::adc::{
    AdcCalBasic,
    AdcCalEfuse,
    AdcCalScheme,
    AdcCalSource,
    AdcConfig,
    Attenuation,
    RegisterAccess,
};

/// Marker trait for ADC units which support line fitting
///
/// Usually it means that reference points are stored in efuse.
/// See also [`AdcCalLine`].
pub trait AdcHasLineCal {}

/// Coefficients is actually a fixed-point numbers.
/// It is scaled to put them into integer.
const GAIN_SCALE: u32 = 1 << 16;

/// Line fitting ADC calibration scheme
///
/// This scheme implements gain correction based on reference points.
///
/// A reference point is a pair of a reference voltage and the corresponding
/// mean raw digital ADC value. Such values are usually stored in efuse bit
/// fields for each supported attenuation.
///
/// Also it can be measured in runtime by connecting ADC to reference voltage
/// internally but this method is not so good because actual reference voltage
/// may varies in range 1.0..=1.2 V. Currently this method is used as a fallback
/// (with 1.1 V by default) when calibration data is missing.
///
/// This scheme also includes basic calibration ([`AdcCalBasic`]).
#[derive(Clone, Copy)]
pub struct AdcCalLine<ADCI> {
    basic: AdcCalBasic<ADCI>,

    /// Gain of ADC-value
    gain: u32,

    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcCalScheme<ADCI> for AdcCalLine<ADCI>
where
    ADCI: AdcCalEfuse + AdcHasLineCal + RegisterAccess,
{
    fn new_cal(atten: Attenuation) -> Self {
        let basic = AdcCalBasic::<ADCI>::new_cal(atten);

        // Try get the reference point (Dout, Vin) from efuse
        // Dout means mean raw ADC value when specified Vin applied to input.
        let (code, mv) = ADCI::get_cal_code(atten)
            .map(|code| (code, ADCI::get_cal_mv(atten)))
            .unwrap_or_else(|| {
                // As a fallback try to calibrate using reference voltage source.
                // This methos is no to good because actual reference voltage may varies
                // in range 1000..=1200 mV and this value currently cannot be given from efuse.
                (
                    AdcConfig::<ADCI>::adc_calibrate(atten, AdcCalSource::Ref),
                    1100, // use 1100 mV as a middle of typical reference voltage range
                )
            });

        // Estimate the (assumed) linear relationship between the measured raw value and
        // the voltage with the previously done measurement when the chip was
        // manufactured.
        //
        // Rounding formula: R = (OP(A * 2) + 1) / 2
        // where R - result, A - argument, O - operation
        let gain =
            ((mv as u32 * GAIN_SCALE * 2 / code as u32 + 1) * 4096 / atten.ref_mv() as u32 + 1) / 2;

        Self {
            basic,
            gain,
            _phantom: PhantomData,
        }
    }

    fn adc_cal(&self) -> u16 {
        self.basic.adc_cal()
    }

    fn adc_val(&self, val: u16) -> u16 {
        let val = self.basic.adc_val(val);

        // pointers are checked in the upper layer
        (val as u32 * self.gain / GAIN_SCALE) as u16
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6))]
impl AdcHasLineCal for crate::adc::ADC1 {}

#[cfg(esp32c3)]
impl AdcHasLineCal for crate::adc::ADC2 {}
