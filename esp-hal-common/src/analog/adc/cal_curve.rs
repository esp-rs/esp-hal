use core::marker::PhantomData;

use crate::adc::{
    AdcCalEfuse,
    AdcCalLine,
    AdcCalScheme,
    AdcHasLineCal,
    Attenuation,
    RegisterAccess,
};

const COEFF_MUL: i64 = 1 << 52;

type CurveCoeff = i64;

/// Polynomial coefficients for specified attenuation.
pub struct CurveCoeffs {
    /// Attenuation
    atten: Attenuation,
    /// Polynomial coefficients
    coeff: &'static [CurveCoeff],
}

type CurvesCoeffs = &'static [CurveCoeffs];

/// Marker trait for ADC which support curve fitting
///
/// See also [`AdcCalCurve`].
pub trait AdcHasCurveCal {
    /// Coefficients for calculating the reading voltage error.
    ///
    /// A sets of coefficients for each attenuation.
    const CURVES_COEFFS: CurvesCoeffs;
}

/// Curve fitting ADC calibration scheme
///
/// This scheme implements final polynomial error correction using predefined
/// coefficient sets for each attenuation.
///
/// This scheme also includes basic calibration ([`super::AdcCalBasic`]) and
/// line fitting ([`AdcCalLine`]).
#[derive(Clone, Copy)]
pub struct AdcCalCurve<ADCI> {
    line: AdcCalLine<ADCI>,

    /// Coefficients for each term (3..=5)
    coeff: &'static [CurveCoeff],

    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcCalScheme<ADCI> for AdcCalCurve<ADCI>
where
    ADCI: AdcCalEfuse + AdcHasLineCal + AdcHasCurveCal + RegisterAccess,
{
    fn new_cal(atten: Attenuation) -> Self {
        let line = AdcCalLine::<ADCI>::new_cal(atten);

        let coeff = ADCI::CURVES_COEFFS
            .iter()
            .find(|item| item.atten == atten)
            .expect("No curve coefficients for given attenuation")
            .coeff;

        Self {
            line,
            coeff,
            _phantom: PhantomData,
        }
    }

    fn adc_cal(&self) -> u16 {
        self.line.adc_cal()
    }

    fn adc_val(&self, val: u16) -> u16 {
        let val = self.line.adc_val(val);

        let err = if val == 0 {
            0
        } else {
            // err = coeff[0] + coeff[1] * val + coeff[2] * val^2 + ... + coeff[n] * val^n
            let mut var = 1i64;
            let mut err = (var * self.coeff[0] as i64 / COEFF_MUL) as i32;

            for coeff in &self.coeff[1..] {
                var = var * val as i64;
                err += (var * *coeff as i64 / COEFF_MUL) as i32;
            }

            err
        };

        (val as i32 - err) as u16
    }
}

macro_rules! coeff_tables {
    ($($(#[$($meta:meta)*])* $name:ident [ $($att:ident => [ $($val:literal,)* ],)* ];)*) => {
        $(
            $(#[$($meta)*])*
            const $name: CurvesCoeffs = &[
                $(CurveCoeffs {
                    atten: Attenuation::$att,
                    coeff: &[
                        $(($val as f64 * COEFF_MUL as f64 * 4096f64 / Attenuation::$att.ref_mv() as f64) as CurveCoeff,)*
                    ],
                },)*
            ];
        )*
    };
}

#[cfg(any(esp32c3, esp32c6, esp32s3))]
mod impls {
    use super::*;

    impl AdcHasCurveCal for crate::adc::ADC1 {
        const CURVES_COEFFS: CurvesCoeffs = CURVES_COEFFS1;
    }

    #[cfg(esp32c3)]
    impl AdcHasCurveCal for crate::adc::ADC2 {
        const CURVES_COEFFS: CurvesCoeffs = CURVES_COEFFS1;
    }

    #[cfg(esp32s3)]
    impl AdcHasCurveCal for crate::adc::ADC2 {
        const CURVES_COEFFS: CurvesCoeffs = CURVES_COEFFS2;
    }

    coeff_tables! {
        /// Error curve coefficients derived from <https://github.com/espressif/esp-idf/blob/903af13e8/components/esp_adc/esp32c3/curve_fitting_coefficients.c>
        #[cfg(esp32c3)]
        CURVES_COEFFS1 [
            Attenuation0dB => [
                -0.2259664705000430,
                -0.0007265418501948,
                0.0000109410402681,
            ],
            Attenuation2p5dB => [
                0.4229623392600516,
                -0.0000731527490903,
                0.0000088166562521,
            ],
            Attenuation6dB => [
                -1.0178592392364350,
                -0.0097159265299153,
                0.0000149794028038,
            ],
            Attenuation11dB => [
                -1.4912262772850453,
                -0.0228549975564099,
                0.0000356391935717,
                -0.0000000179964582,
                0.0000000000042046,
            ],
        ];

        /// Error curve coefficients derived from <https://github.com/espressif/esp-idf/blob/903af13e8/components/esp_adc/esp32c6/curve_fitting_coefficients.c>
        #[cfg(esp32c6)]
        CURVES_COEFFS1 [
            Attenuation0dB => [
                -0.0487166399931449,
                0.0006436483033201,
                0.0000030410131806,
            ],
            Attenuation2p5dB => [
                -0.8665498165817785,
                0.0015239070452946,
                0.0000013818878844,
            ],
            Attenuation6dB => [
                -1.2277821756674387,
                0.0022275554717885,
                0.0000005924302667,
            ],
            Attenuation11dB => [
                -0.3801417550380255,
                -0.0006020352420772,
                0.0000012442478488,
            ],
        ];

        /// Error curve coefficients derived from <https://github.com/espressif/esp-idf/blob/903af13e8/components/esp_adc/esp32s3/curve_fitting_coefficients.c>
        #[cfg(esp32s3)]
        CURVES_COEFFS1 [
            Attenuation0dB => [
                -2.7856531419538344,
                -0.0050871540569528,
                0.0000097982495890,
            ],
            Attenuation2p5dB => [
                -2.9831022915028695,
                -0.0049393185868806,
                0.0000101379430548,
            ],
            Attenuation6dB => [
                -2.3285545746296417,
                -0.0147640181047414,
                0.0000208385525314,
            ],
            Attenuation11dB => [
                -0.6444034182694780,
                -0.0644334888647536,
                0.0001297891447611,
                -0.0000000707697180,
                0.0000000000135150,
            ],
        ];

        /// Error curve coefficients derived from <https://github.com/espressif/esp-idf/blob/903af13e8/components/esp_adc/esp32s3/curve_fitting_coefficients.c>
        #[cfg(esp32s3)]
        CURVES_COEFFS2 [
            Attenuation0dB => [
                -2.5668651654328927,
                0.0001353548869615,
                0.0000036615265189,
            ],
            Attenuation2p5dB => [
                -2.3690184690298404,
                -0.0066319894226185,
                0.0000118964995959,
            ],
            Attenuation6dB => [
                -0.9452499397020617,
                -0.0200996773954387,
                0.00000259011467956,
            ],
            Attenuation11dB => [
                1.2247719764336924,
                -0.0755717904943462,
                0.0001478791187119,
                -0.0000000796725280,
                0.0000000000150380,
            ],
        ];
    }
}
