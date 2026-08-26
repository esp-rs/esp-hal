use crate::{analog::adc::Attenuation, peripherals::EFUSE};

#[cfg_attr(not(feature = "unstable"), allow(dead_code))]
mod fields;
#[instability::unstable]
pub use fields::*;

/// Selects which ADC the eFuse calibration data applies to.
#[instability::unstable]
pub enum AdcCalibUnit {
    /// Selects efuse calibration data for ADC1.
    ADC1,
    /// Selects efuse calibration data for ADC2.
    ADC2,
}

/// Returns whether SPI boot encryption is enabled.
#[instability::unstable]
pub fn flash_encryption() -> bool {
    !super::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT)
        .count_ones()
        .is_multiple_of(2)
}

/// Returns the multiplier for the timeout value of the RWDT STAGE 0 register.
#[instability::unstable]
pub fn rwdt_multiplier() -> u8 {
    super::read_field_le::<u8>(WDT_DELAY_SEL)
}

/// Returns the eFuse block version.
///
/// see <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/efuse_hal.c#L27-L30>
#[instability::unstable]
pub fn block_version() -> (u8, u8) {
    // see <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/esp32c6/include/hal/efuse_ll.h#L65-L73>
    // <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c6/esp_efuse_table.csv#L156>
    (
        super::read_field_le::<u8>(BLK_VERSION_MAJOR),
        super::read_field_le::<u8>(BLK_VERSION_MINOR),
    )
}

/// Returns a signed value from the raw data from eFuse.
///
/// `sign_bit` is the index of the sign bit, starting from 0
fn get_signed_val(data: u32, sign_bit: u32) -> i32 {
    let sign_mask = 1u32 << sign_bit;
    if data & sign_mask != 0 {
        -((data & !sign_mask) as i32)
    } else {
        data as i32
    }
}

/// Returns the version of RTC calibration block.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_rtc_calib.c#L20>
#[instability::unstable]
pub fn rtc_calib_version() -> u8 {
    let (_major, minor) = block_version();
    if minor == 1 {
        1
    } else if minor >= 2 {
        2
    } else {
        0
    }
}

/// Returns the ADC initial code for specified attenuation from efuse.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_rtc_calib.c#L35>
#[instability::unstable]
pub fn rtc_calib_init_code(_unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    let version = rtc_calib_version();

    if !(1..=2).contains(&version) {
        return None;
    }

    // See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_table.csv#L178-L181>
    let init_code: u16 = super::read_field_le(match atten {
        Attenuation::_0dB => ADC1_INIT_CODE_ATTEN0,
        Attenuation::_2p5dB => ADC1_INIT_CODE_ATTEN1,
        Attenuation::_6dB => ADC1_INIT_CODE_ATTEN2,
        Attenuation::_11dB => ADC1_INIT_CODE_ATTEN3,
    });

    Some(init_code + 1600)
}

/// Returns the channel specific calibration compensation.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_rtc_calib.c#L60>
#[instability::unstable]
pub fn rtc_calib_get_chan_compens(
    _unit: AdcCalibUnit,
    channel: u8,
    atten: Attenuation,
) -> Option<i32> {
    let chan_diff: u32 = super::read_field_le(match channel {
        0 => ADC1_INIT_CODE_ATTEN0_CH0,
        1 => ADC1_INIT_CODE_ATTEN0_CH1,
        2 => ADC1_INIT_CODE_ATTEN0_CH2,
        3 => ADC1_INIT_CODE_ATTEN0_CH3,
        4 => ADC1_INIT_CODE_ATTEN0_CH4,
        5 => ADC1_INIT_CODE_ATTEN0_CH5,
        _ => ADC1_INIT_CODE_ATTEN0_CH6,
    });

    Some(get_signed_val(chan_diff, 3) * (4 - atten as i32))
}

/// Returns the ADC reference point voltage for specified attenuation in millivolts.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_rtc_calib.c#L98>
#[instability::unstable]
pub fn rtc_calib_cal_mv(_unit: AdcCalibUnit, atten: Attenuation) -> u16 {
    let version = rtc_calib_version();
    let input_vout_mv = match version {
        2 => [750, 1000, 1500, 2800],
        _ => [400, 550, 750, 1370],
    };

    input_vout_mv[atten as usize]
}

/// Returns the ADC reference point digital code for specified attenuation.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_rtc_calib.c#L98>
#[instability::unstable]
pub fn rtc_calib_cal_code(_unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    let version = rtc_calib_version();

    if !(1..=2).contains(&version) {
        return None;
    }

    // See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32c6/esp_efuse_table.csv#L182-L185>
    let cal_vol: u16 = super::read_field_le(match atten {
        Attenuation::_0dB => ADC1_CAL_VOL_ATTEN0,
        Attenuation::_2p5dB => ADC1_CAL_VOL_ATTEN1,
        Attenuation::_6dB => ADC1_CAL_VOL_ATTEN2,
        Attenuation::_11dB => ADC1_CAL_VOL_ATTEN3,
    });

    let chk_offset = if version == 1 {
        1500
    } else if atten == Attenuation::_6dB {
        2900
    } else {
        2850
    };

    Some((chk_offset + get_signed_val(cal_vol as u32, 9)) as u16)
}

/// Returns the major hardware revision.
#[instability::unstable]
pub fn major_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MAJOR)
}

/// Returns the minor hardware revision.
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
            Self::Block1 => efuse.rd_mac_spi_sys_0().as_ptr(),
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
