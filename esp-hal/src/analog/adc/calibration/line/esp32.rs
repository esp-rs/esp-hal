use core::marker::PhantomData;

use super::AdcHasLineCal;
use crate::{
    analog::adc::{AdcCalScheme, Attenuation},
    efuse,
};

const COEFF_A_SCALE: u32 = 65536;
const COEFF_A_ROUND: u32 = COEFF_A_SCALE / 2;

const VREF_MASK: u32 = 0x1F;
const VREF_STEP_SIZE: i32 = 7;
const VREF_OFFSET: i32 = 1100;

const TP_LOW1_OFFSET: i32 = 278;
const TP_LOW2_OFFSET: i32 = 421;
const TP_LOW_MASK: u32 = 0x7F;
const TP_LOW_VOLTAGE: u32 = 150;
const TP_HIGH1_OFFSET: i32 = 3265;
const TP_HIGH2_OFFSET: i32 = 3406;
const TP_HIGH_MASK: u32 = 0x1FF;
const TP_HIGH_VOLTAGE: u32 = 850;
const TP_STEP_SIZE: i32 = 4;

const LUT_VREF_LOW: i32 = 1000;
const LUT_VREF_HIGH: i32 = 1200;
const LUT_ADC_STEP_SIZE: u32 = 64;
const LUT_POINTS: usize = 20;
const LUT_LOW_THRESH: u32 = 2880;
const LUT_HIGH_THRESH: u32 = LUT_LOW_THRESH + LUT_ADC_STEP_SIZE;
const ADC_12_BIT_RES: u32 = 4096;

const ADC1_TP_ATTEN_SCALE: [u32; 4] = [65504, 86975, 120389, 224310];
const ADC2_TP_ATTEN_SCALE: [u32; 4] = [65467, 86861, 120416, 224708];
const ADC1_TP_ATTEN_OFFSET: [u32; 4] = [0, 1, 27, 54];
const ADC2_TP_ATTEN_OFFSET: [u32; 4] = [0, 9, 26, 66];

const ADC1_VREF_ATTEN_SCALE: [u32; 4] = [57431, 76236, 105481, 196602];
const ADC2_VREF_ATTEN_SCALE: [u32; 4] = [57236, 76175, 105678, 197170];
const ADC1_VREF_ATTEN_OFFSET: [u32; 4] = [75, 78, 107, 142];
const ADC2_VREF_ATTEN_OFFSET: [u32; 4] = [63, 66, 89, 128];

const LUT_ADC1_LOW: [u32; LUT_POINTS] = [
    2240, 2297, 2352, 2405, 2457, 2512, 2564, 2616, 2664, 2709, 2754, 2795, 2832, 2868, 2903, 2937,
    2969, 3000, 3030, 3060,
];
const LUT_ADC1_HIGH: [u32; LUT_POINTS] = [
    2667, 2706, 2745, 2780, 2813, 2844, 2873, 2901, 2928, 2956, 2982, 3006, 3032, 3059, 3084, 3110,
    3135, 3160, 3184, 3209,
];
const LUT_ADC2_LOW: [u32; LUT_POINTS] = [
    2238, 2293, 2347, 2399, 2451, 2507, 2561, 2613, 2662, 2710, 2754, 2792, 2831, 2869, 2904, 2937,
    2968, 2999, 3029, 3059,
];
const LUT_ADC2_HIGH: [u32; LUT_POINTS] = [
    2657, 2698, 2738, 2774, 2807, 2838, 2867, 2894, 2921, 2946, 2971, 2996, 3020, 3043, 3067, 3092,
    3116, 3139, 3162, 3185,
];

/// Line fitting ADC calibration scheme
///
/// ESP32 uses two-point or Vref characterization from eFuse, and a lookup table
/// at 11 dB for the non-linear region. Readings are in mV.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/esp_adc/esp32/adc_cali_line_fitting.c#L155>
#[derive(Clone, Copy)]
pub struct AdcCalLine<ADCX> {
    coeff_a: u32,
    coeff_b: u32,
    vref: u32,
    use_lut: bool,
    unit: u8,
    _phantom: PhantomData<ADCX>,
}

fn decode_bits(bits: u32, mask: u32, twos_compl: bool) -> i32 {
    let sign_bit = !(mask >> 1) & mask;
    if bits & sign_bit != 0 {
        if twos_compl {
            -(((!bits).wrapping_add(1) & (mask >> 1)) as i32)
        } else {
            -((bits & (mask >> 1)) as i32)
        }
    } else {
        (bits & (mask >> 1)) as i32
    }
}

fn read_efuse_vref() -> u32 {
    let bits = efuse::read_field_le::<u32>(efuse::ADC_VREF);
    (VREF_OFFSET + decode_bits(bits, VREF_MASK, false) * VREF_STEP_SIZE) as u32
}

fn check_efuse_vref() -> bool {
    efuse::read_field_le::<u32>(efuse::ADC_VREF) != 0
}

fn check_efuse_tp() -> bool {
    if !efuse::read_bit(efuse::BLK3_PART_RESERVE) {
        return false;
    }
    efuse::read_field_le::<u32>(efuse::ADC1_TP_LOW) != 0
        && efuse::read_field_le::<u32>(efuse::ADC2_TP_LOW) != 0
        && efuse::read_field_le::<u32>(efuse::ADC1_TP_HIGH) != 0
        && efuse::read_field_le::<u32>(efuse::ADC2_TP_HIGH) != 0
}

fn read_efuse_tp_low(unit: u8) -> u32 {
    let (high_offset, efuse) = if unit == 0 {
        (TP_LOW1_OFFSET, efuse::ADC1_TP_LOW)
    } else {
        (TP_LOW2_OFFSET, efuse::ADC2_TP_LOW)
    };

    let efuse_value = decode_bits(efuse::read_field_le::<u32>(efuse), TP_LOW_MASK, true);
    (high_offset + efuse_value * TP_STEP_SIZE) as u32
}

fn read_efuse_tp_high(unit: u8) -> u32 {
    let (high_offset, efuse) = if unit == 0 {
        (TP_HIGH1_OFFSET, efuse::ADC1_TP_HIGH)
    } else {
        (TP_HIGH2_OFFSET, efuse::ADC2_TP_HIGH)
    };

    let efuse_value = decode_bits(efuse::read_field_le::<u32>(efuse), TP_HIGH_MASK, true);
    (high_offset + efuse_value * TP_STEP_SIZE) as u32
}

fn characterize_using_two_point(unit: u8, atten: usize, high: u32, low: u32) -> (u32, u32) {
    let (scales, offsets) = if unit == 0 {
        (&ADC1_TP_ATTEN_SCALE, &ADC1_TP_ATTEN_OFFSET)
    } else {
        (&ADC2_TP_ATTEN_SCALE, &ADC2_TP_ATTEN_OFFSET)
    };
    let delta_x = high - low;
    let delta_v = TP_HIGH_VOLTAGE - TP_LOW_VOLTAGE;
    let coeff_a = (delta_v * scales[atten] + (delta_x / 2)) / delta_x;
    let coeff_b = TP_HIGH_VOLTAGE - ((delta_v * high + (delta_x / 2)) / delta_x) + offsets[atten];
    (coeff_a, coeff_b)
}

fn characterize_using_vref(unit: u8, atten: usize, vref: u32) -> (u32, u32) {
    let (scales, offsets) = if unit == 0 {
        (&ADC1_VREF_ATTEN_SCALE, &ADC1_VREF_ATTEN_OFFSET)
    } else {
        (&ADC2_VREF_ATTEN_SCALE, &ADC2_VREF_ATTEN_OFFSET)
    };
    let coeff_a = (vref * scales[atten]) / ADC_12_BIT_RES;
    (coeff_a, offsets[atten])
}

fn calculate_voltage_linear(adc: u32, coeff_a: u32, coeff_b: u32) -> u32 {
    ((coeff_a * adc + COEFF_A_ROUND) / COEFF_A_SCALE) + coeff_b
}

fn interpolate_two_points(y1: u32, y2: u32, x_step: u32, x: u32) -> u32 {
    ((y1 * x_step) + (y2 * x) - (y1 * x) + (x_step / 2)) / x_step
}

fn calculate_voltage_lut(
    adc: u32,
    vref: u32,
    low: &[u32; LUT_POINTS],
    high: &[u32; LUT_POINTS],
) -> u32 {
    let i = ((adc - LUT_LOW_THRESH) / LUT_ADC_STEP_SIZE) as usize;
    let i = i.min(LUT_POINTS - 2);

    let x2dist = LUT_VREF_HIGH - vref as i32;
    let x1dist = vref as i32 - LUT_VREF_LOW;
    let y2dist = ((i as u32 + 1) * LUT_ADC_STEP_SIZE + LUT_LOW_THRESH) as i32 - adc as i32;
    let y1dist = adc as i32 - (i as u32 * LUT_ADC_STEP_SIZE + LUT_LOW_THRESH) as i32;

    let q11 = low[i] as i32;
    let q12 = low[i + 1] as i32;
    let q21 = high[i] as i32;
    let q22 = high[i + 1] as i32;

    let mut voltage = (q11 * x2dist * y2dist)
        + (q21 * x1dist * y2dist)
        + (q12 * x2dist * y1dist)
        + (q22 * x1dist * y1dist);
    voltage += ((LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE as i32) / 2;
    voltage /= (LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE as i32;
    voltage as u32
}

impl<ADCX> crate::private::Sealed for AdcCalLine<ADCX> {}

impl<ADCX> AdcCalScheme<ADCX> for AdcCalLine<ADCX>
where
    ADCX: AdcHasLineCal,
{
    fn new_cal(atten: Attenuation) -> Self {
        let unit = ADCX::UNIT;
        let atten_idx = atten as usize;

        let vref = if check_efuse_vref() {
            read_efuse_vref()
        } else {
            1100
        };

        let (coeff_a, coeff_b) = if check_efuse_tp() {
            let high = read_efuse_tp_high(unit);
            let low = read_efuse_tp_low(unit);
            characterize_using_two_point(unit, atten_idx, high, low)
        } else {
            characterize_using_vref(unit, atten_idx, vref)
        };

        Self {
            coeff_a,
            coeff_b,
            vref,
            use_lut: atten == Attenuation::_11dB,
            unit,
            _phantom: PhantomData,
        }
    }

    fn adc_val(&self, val: u16) -> u16 {
        let raw = val as u32;

        if self.use_lut && raw >= LUT_LOW_THRESH {
            let (low, high) = if self.unit == 0 {
                (&LUT_ADC1_LOW, &LUT_ADC1_HIGH)
            } else {
                (&LUT_ADC2_LOW, &LUT_ADC2_HIGH)
            };
            let lut_voltage = calculate_voltage_lut(raw, self.vref, low, high);
            if raw <= LUT_HIGH_THRESH {
                let linear = calculate_voltage_linear(raw, self.coeff_a, self.coeff_b);
                interpolate_two_points(linear, lut_voltage, LUT_ADC_STEP_SIZE, raw - LUT_LOW_THRESH)
                    as u16
            } else {
                lut_voltage as u16
            }
        } else {
            calculate_voltage_linear(raw, self.coeff_a, self.coeff_b) as u16
        }
    }
}
