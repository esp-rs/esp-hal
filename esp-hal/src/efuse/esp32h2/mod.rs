use crate::{analog::adc::Attenuation, peripherals::EFUSE};

#[cfg_attr(not(feature = "unstable"), allow(dead_code))]
mod fields;
#[instability::unstable]
pub use fields::*;

/// Selects which ADC we are interested in the efuse calibration data for
#[instability::unstable]
pub enum AdcCalibUnit {
    /// Select efuse calibration data for ADC1
    ADC1,
}

/// Get status of SPI boot encryption.
#[instability::unstable]
pub fn flash_encryption() -> bool {
    !super::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT)
        .count_ones()
        .is_multiple_of(2)
}

/// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
#[instability::unstable]
pub fn rwdt_multiplier() -> u8 {
    super::read_field_le::<u8>(WDT_DELAY_SEL)
}

/// Get efuse block version
///
/// See: <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/efuse_hal.c#L27-L30>
#[instability::unstable]
pub fn block_version() -> (u8, u8) {
    (
        super::read_field_le::<u8>(BLK_VERSION_MAJOR),
        super::read_field_le::<u8>(BLK_VERSION_MINOR),
    )
}

/// Get a signed value from the raw data from eFuse.
///
/// `sign_bit` is the index of the sign bit, starting from 0.
fn get_signed_val(data: u32, sign_bit: u32) -> i32 {
    let sign_mask = 1u32 << sign_bit;
    if data & sign_mask != 0 {
        -((data & !sign_mask) as i32)
    } else {
        data as i32
    }
}

/// Get version of RTC calibration block
///
/// See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L20>
#[instability::unstable]
pub fn rtc_calib_version() -> u8 {
    let (_major, minor) = block_version();
    if minor >= 2 { 1 } else { 0 }
}

/// Get ADC initial code for specified attenuation from efuse
///
/// See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L33>
#[instability::unstable]
pub fn rtc_calib_init_code(_unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    let version = rtc_calib_version();

    if version != 1 {
        return None;
    }

    // See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_table.csv#L184-L187>
    let init_code: u16 = super::read_field_le(match atten {
        Attenuation::_0dB => ADC1_AVE_INITCODE_ATTEN0,
        Attenuation::_2p5dB => ADC1_AVE_INITCODE_ATTEN1,
        Attenuation::_6dB => ADC1_AVE_INITCODE_ATTEN2,
        Attenuation::_11dB => ADC1_AVE_INITCODE_ATTEN3,
    });

    Some(init_code + 1600)
}

/// Get the channel specific calibration compensation
///
/// See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L58>
#[instability::unstable]
pub fn rtc_calib_get_chan_compens(
    _unit: AdcCalibUnit,
    channel: u8,
    atten: Attenuation,
) -> Option<i32> {
    let chan_diff: u32 = super::read_field_le(match channel {
        0 => ADC1_CH0_ATTEN0_INITCODE_DIFF,
        1 => ADC1_CH1_ATTEN0_INITCODE_DIFF,
        2 => ADC1_CH2_ATTEN0_INITCODE_DIFF,
        3 => ADC1_CH3_ATTEN0_INITCODE_DIFF,
        _ => ADC1_CH4_ATTEN0_INITCODE_DIFF,
    });

    Some(get_signed_val(chan_diff, 3) * (4 - atten as i32))
}

/// Get ADC reference point voltage for specified attenuation in millivolts
///
/// See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L91>
#[instability::unstable]
pub fn rtc_calib_cal_mv(_unit: AdcCalibUnit, atten: Attenuation) -> u16 {
    const INPUT_VOUT_MV: [[u16; 4]; 1] = [
        [750, 1000, 1500, 2800], // Calibration V1 coefficients
    ];

    let version = rtc_calib_version();

    if version != 1 {
        // The required efuse bits for this chip are not burnt.
        // 1100 is the middle of the reference voltage range.
        return 1100;
    }

    INPUT_VOUT_MV[0][atten as usize]
}

/// Returns the calibration code
///
/// See: <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L91>
#[instability::unstable]
pub fn rtc_calib_cal_code(_unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    let version = rtc_calib_version();

    if version != 1 {
        return None;
    }

    let cal_vol: u16 = super::read_field_le(match atten {
        Attenuation::_0dB => ADC1_HI_DOUT_ATTEN0,
        Attenuation::_2p5dB => ADC1_HI_DOUT_ATTEN1,
        Attenuation::_6dB => ADC1_HI_DOUT_ATTEN2,
        Attenuation::_11dB => ADC1_HI_DOUT_ATTEN3,
    });
    let chk_offset = if atten == Attenuation::_6dB {
        2970
    } else {
        2900
    };
    Some((chk_offset + get_signed_val(cal_vol as u32, 9)) as u16)
}

/// Returns the major hardware revision
#[instability::unstable]
pub fn major_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MAJOR)
}

/// Returns the minor hardware revision
#[instability::unstable]
pub fn minor_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MINOR)
}

#[derive(Debug, Clone, Copy, strum::FromRepr)]
#[repr(u32)]
pub(crate) enum EfuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
    Block4,
    Block5,
    Block6,
    Block7,
    Block8,
    Block9,
    Block10,
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        let efuse = EFUSE::regs();
        match self {
            Self::Block0 => efuse.rd_wr_dis().as_ptr(),
            Self::Block1 => efuse.rd_mac_sys_0().as_ptr(),
            Self::Block2 => efuse.rd_sys_part1_data0().as_ptr(),
            Self::Block3 => efuse.rd_usr_data0().as_ptr(),
            Self::Block4 => efuse.rd_key0_data0().as_ptr(),
            Self::Block5 => efuse.rd_key1_data0().as_ptr(),
            Self::Block6 => efuse.rd_key2_data0().as_ptr(),
            Self::Block7 => efuse.rd_key3_data0().as_ptr(),
            Self::Block8 => efuse.rd_key4_data0().as_ptr(),
            Self::Block9 => efuse.rd_key5_data0().as_ptr(),
            Self::Block10 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}
