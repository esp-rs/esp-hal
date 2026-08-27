//! # Reading of eFuses (ESP32-P4)

use crate::{analog::adc::Attenuation, peripherals::EFUSE};

#[cfg_attr(not(feature = "unstable"), allow(dead_code))]
mod fields;
#[instability::unstable]
pub use fields::*;

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
    (
        super::read_field_le::<u8>(BLK_VERSION_MAJOR),
        super::read_field_le::<u8>(BLK_VERSION_MINOR),
    )
}

/// Returns the version of RTC calibration block.
///
/// see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L21>
#[instability::unstable]
pub fn rtc_calib_version() -> u8 {
    // ESP-IDF compares `major * 100 + minor` against 1, so any non-zero block version
    // selects calibration version 1.
    let (major, minor) = block_version();
    if major > 0 || minor > 0 { 1 } else { 0 }
}

/// Returns the major hardware revision.
pub(crate) fn major_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MAJOR)
}

/// Returns the minor hardware revision.
pub(crate) fn minor_chip_version() -> u8 {
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
        use EfuseBlock::*;
        let efuse = EFUSE::regs();
        match self {
            Block0 => efuse.rd_wr_dis().as_ptr(),
            Block1 => efuse.rd_mac_sys0().as_ptr(),
            Block2 => efuse.rd_sys_part1_data(0).as_ptr(),
            Block3 => efuse.rd_usr_data(0).as_ptr(),
            Block4 => efuse.rd_key0_data(0).as_ptr(),
            Block5 => efuse.rd_key1_data(0).as_ptr(),
            Block6 => efuse.rd_key2_data(0).as_ptr(),
            Block7 => efuse.rd_key3_data(0).as_ptr(),
            Block8 => efuse.rd_key4_data(0).as_ptr(),
            Block9 => efuse.rd_key5_data(0).as_ptr(),
            Block10 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}

/// Selects which ADC the eFuse calibration data applies to.
#[instability::unstable]
pub enum AdcCalibUnit {
    /// Selects efuse calibration data for ADC1.
    ADC1,
    /// Selects efuse calibration data for ADC2.
    ADC2,
}

/// Returns a signed value from the raw data from eFuse. Sign bit is the index of the sign bit,
/// starting from 0. see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L19>.
fn get_signed_val(data: u32, sign_bit: u32) -> i32 {
    let sign_mask = 1u32 << sign_bit;
    if data & sign_mask != 0 {
        -((data & !sign_mask) as i32)
    } else {
        data as i32
    }
}

/// Returns the ADC initial code for specified attenuation from efuse.
///
/// see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L34>
#[instability::unstable]
pub fn rtc_calib_init_code(unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    if rtc_calib_version() != 1 {
        return None;
    }

    let init_code: u16 = super::read_field_le(match (unit, atten) {
        (AdcCalibUnit::ADC1, Attenuation::_0dB) => ADC1_AVE_INITCODE_ATTEN0,
        (AdcCalibUnit::ADC1, Attenuation::_2p5dB) => ADC1_AVE_INITCODE_ATTEN1,
        (AdcCalibUnit::ADC1, Attenuation::_6dB) => ADC1_AVE_INITCODE_ATTEN2,
        (AdcCalibUnit::ADC1, Attenuation::_11dB) => ADC1_AVE_INITCODE_ATTEN3,
        (AdcCalibUnit::ADC2, Attenuation::_0dB) => ADC2_AVE_INITCODE_ATTEN0,
        (AdcCalibUnit::ADC2, Attenuation::_2p5dB) => ADC2_AVE_INITCODE_ATTEN1,
        (AdcCalibUnit::ADC2, Attenuation::_6dB) => ADC2_AVE_INITCODE_ATTEN2,
        (AdcCalibUnit::ADC2, Attenuation::_11dB) => ADC2_AVE_INITCODE_ATTEN3,
    });

    Some(init_code + 1400) // version 1 logic
}

/// Returns the channel specific calibration compensation.
///
/// see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L131>
#[instability::unstable]
pub fn rtc_calib_get_chan_compens(
    unit: AdcCalibUnit,
    channel: u16,
    atten: Attenuation,
) -> Option<i32> {
    let chan_diff: u32 = super::read_field_le(match (unit, channel) {
        (AdcCalibUnit::ADC1, 0) => ADC1_CH0_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 1) => ADC1_CH1_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 2) => ADC1_CH2_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 3) => ADC1_CH3_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 4) => ADC1_CH4_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 5) => ADC1_CH5_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 6) => ADC1_CH6_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC1, 7) => ADC1_CH7_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 0) => ADC2_CH0_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 1) => ADC2_CH1_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 2) => ADC2_CH2_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 3) => ADC2_CH3_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 4) => ADC2_CH4_ATTEN0_INITCODE_DIFF,
        (AdcCalibUnit::ADC2, 5) => ADC2_CH5_ATTEN0_INITCODE_DIFF,
        _ => return None,
    });

    Some(get_signed_val(chan_diff, 3) * (4 - atten as i32))
}

/// Returns the ADC calibration reference point voltage.
///
/// see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L88>
#[instability::unstable]
pub fn rtc_calib_cal_mv(_unit: AdcCalibUnit, atten: Attenuation) -> u16 {
    match atten {
        Attenuation::_0dB => 600,
        Attenuation::_2p5dB => 800,
        Attenuation::_6dB => 1200,
        Attenuation::_11dB => 2300,
    }
}

/// Returns the ADC calibration code.
///
/// see <https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/efuse/esp32p4/esp_efuse_rtc_calib.c#L71>
#[instability::unstable]
pub fn rtc_calib_cal_code(unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    if rtc_calib_version() != 1 {
        return None;
    }

    let cal_vol: u16 = super::read_field_le(match (unit, atten) {
        (AdcCalibUnit::ADC1, Attenuation::_0dB) => ADC1_HI_DOUT_ATTEN0,
        (AdcCalibUnit::ADC1, Attenuation::_2p5dB) => ADC1_HI_DOUT_ATTEN1,
        (AdcCalibUnit::ADC1, Attenuation::_6dB) => ADC1_HI_DOUT_ATTEN2,
        (AdcCalibUnit::ADC1, Attenuation::_11dB) => ADC1_HI_DOUT_ATTEN3,
        (AdcCalibUnit::ADC2, Attenuation::_0dB) => ADC2_HI_DOUT_ATTEN0,
        (AdcCalibUnit::ADC2, Attenuation::_2p5dB) => ADC2_HI_DOUT_ATTEN1,
        (AdcCalibUnit::ADC2, Attenuation::_6dB) => ADC2_HI_DOUT_ATTEN2,
        (AdcCalibUnit::ADC2, Attenuation::_11dB) => ADC2_HI_DOUT_ATTEN3,
    });

    let chk_offset = match atten {
        Attenuation::_0dB | Attenuation::_2p5dB => 2300,
        Attenuation::_6dB | Attenuation::_11dB => 2350,
    };

    Some((chk_offset + get_signed_val(cal_vol as u32, 9)) as u16)
}
