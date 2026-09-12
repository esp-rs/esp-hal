use core::marker::PhantomData;

use super::AdcHasLineCal;
use crate::analog::adc::{AdcCalBasic, AdcCalEfuse, AdcCalScheme, Attenuation, CalibrationAccess};

const COEFF_A_SCALING: i64 = 65536;
const COEFF_B_SCALING: i64 = 1024;
const V_HIGH: [i64; 4] = [600, 800, 1000, 2000];
const V_LOW: i64 = 250;

/// Line fitting ADC calibration scheme
///
/// ESP32-S2 uses two-point characterization (eFuse calibration v1) or
/// one-point characterization (v2). Readings are in mV.
///
/// This scheme also includes basic calibration ([`AdcCalBasic`]).
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/esp_adc/esp32s2/adc_cali_line_fitting.c#L87>
#[derive(Clone, Copy)]
pub struct AdcCalLine<ADCX> {
    basic: AdcCalBasic<ADCX>,
    coeff_a: u32,
    coeff_b: i32,
    _phantom: PhantomData<ADCX>,
}

fn characterize_two_point(atten: Attenuation, high: i64, low: i64) -> (u32, i32) {
    let v_high = V_HIGH[atten as usize];
    let denom = (high - low).max(1);
    let coeff_a = (COEFF_A_SCALING * (v_high - V_LOW) / denom) as u32;
    let coeff_b = (COEFF_B_SCALING * (V_LOW * high - v_high * low) / denom) as i32;
    (coeff_a, coeff_b)
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
        let version = crate::efuse::rtc_calib_version();

        let (coeff_a, coeff_b) = match version {
            1 => {
                let low = crate::efuse::rtc_calib_reading(
                    version,
                    ADCX::UNIT,
                    atten,
                    crate::efuse::RtcCalibParam::Vlow,
                );
                let high = crate::efuse::rtc_calib_reading(
                    version,
                    ADCX::UNIT,
                    atten,
                    crate::efuse::RtcCalibParam::Vhigh,
                );
                characterize_two_point(atten, high as i64, low as i64)
            }
            2 => {
                let high = ADCX::cal_code(atten).unwrap_or(1).max(1) as i64;
                let mv = ADCX::cal_mv(atten) as i64;
                (((COEFF_A_SCALING * mv) / high) as u32, 0)
            }
            _ => {
                let low = crate::efuse::rtc_calib_reading_inner(
                    1,
                    ADCX::UNIT,
                    atten,
                    crate::efuse::RtcCalibParam::Vlow,
                    true,
                );
                let high = crate::efuse::rtc_calib_reading_inner(
                    1,
                    ADCX::UNIT,
                    atten,
                    crate::efuse::RtcCalibParam::Vhigh,
                    true,
                );
                characterize_two_point(atten, high as i64, low as i64)
            }
        };

        Self {
            basic,
            coeff_a,
            coeff_b,
            _phantom: PhantomData,
        }
    }

    fn adc_cal(&self) -> u16 {
        self.basic.adc_cal()
    }

    fn adc_val(&self, val: u16) -> u16 {
        let val = self.basic.adc_val(val) as i64;
        let voltage = (val * self.coeff_a as i64 / (COEFF_A_SCALING / COEFF_B_SCALING)
            + self.coeff_b as i64)
            / COEFF_B_SCALING;
        voltage.clamp(0, u16::MAX as i64) as u16
    }
}
