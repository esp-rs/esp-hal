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
    /// Select efuse calibration data for ADC2
    ADC2,
}

impl AdcCalibUnit {
    fn index(self) -> u8 {
        match self {
            Self::ADC1 => 0,
            Self::ADC2 => 1,
        }
    }
}

/// Parameter selected from the ESP32-S2 RTC calibration table
#[derive(Clone, Copy)]
#[instability::unstable]
pub enum RtcCalibParam {
    /// Low-voltage ADC reading (calibration v1)
    Vlow,
    /// High-voltage ADC reading
    Vhigh,
    /// Initial code (calibration v2)
    Vinit,
}

struct RtcCalibEntry {
    begin_bit: u16,
    length: u8,
    multiplier: i16,
    base: i16,
    depends: u8,
}

// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32s2/esp_efuse_rtc_table.c#L48-L89>
const RTC_CALIB_MAP: [RtcCalibEntry; 33] = [
    RtcCalibEntry {
        begin_bit: 0,
        length: 0,
        multiplier: 0,
        base: 0,
        depends: 0,
    },
    // V1 low (tags 1-8)
    RtcCalibEntry {
        begin_bit: 208,
        length: 6,
        multiplier: 4,
        base: 2231,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 214,
        length: 6,
        multiplier: 4,
        base: 1643,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 220,
        length: 6,
        multiplier: 4,
        base: 1290,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 226,
        length: 6,
        multiplier: 4,
        base: 701,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 232,
        length: 6,
        multiplier: 4,
        base: 2305,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 238,
        length: 6,
        multiplier: 4,
        base: 1693,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 244,
        length: 6,
        multiplier: 4,
        base: 1343,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 250,
        length: 6,
        multiplier: 4,
        base: 723,
        depends: 0,
    },
    // V1 high (tags 9-16)
    RtcCalibEntry {
        begin_bit: 144,
        length: 8,
        multiplier: 4,
        base: 5775,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 152,
        length: 8,
        multiplier: 4,
        base: 5693,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 160,
        length: 8,
        multiplier: 4,
        base: 5723,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 168,
        length: 8,
        multiplier: 4,
        base: 6209,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 176,
        length: 8,
        multiplier: 4,
        base: 5817,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 184,
        length: 8,
        multiplier: 4,
        base: 5703,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 192,
        length: 8,
        multiplier: 4,
        base: 5731,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 200,
        length: 8,
        multiplier: 4,
        base: 6157,
        depends: 0,
    },
    // V2 high (tags 17-24)
    RtcCalibEntry {
        begin_bit: 197,
        length: 6,
        multiplier: 2,
        base: 169,
        depends: 19,
    },
    RtcCalibEntry {
        begin_bit: 203,
        length: 6,
        multiplier: 2,
        base: -26,
        depends: 19,
    },
    RtcCalibEntry {
        begin_bit: 209,
        length: 9,
        multiplier: 2,
        base: 126,
        depends: 22,
    },
    RtcCalibEntry {
        begin_bit: 218,
        length: 7,
        multiplier: 2,
        base: 387,
        depends: 19,
    },
    RtcCalibEntry {
        begin_bit: 225,
        length: 7,
        multiplier: 2,
        base: 177,
        depends: 22,
    },
    RtcCalibEntry {
        begin_bit: 232,
        length: 10,
        multiplier: 2,
        base: 5815,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 242,
        length: 7,
        multiplier: 2,
        base: 27,
        depends: 22,
    },
    RtcCalibEntry {
        begin_bit: 249,
        length: 7,
        multiplier: 2,
        base: 410,
        depends: 22,
    },
    // V2 init (tags 25-32)
    RtcCalibEntry {
        begin_bit: 147,
        length: 8,
        multiplier: 2,
        base: 1519,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 155,
        length: 6,
        multiplier: 2,
        base: 88,
        depends: 25,
    },
    RtcCalibEntry {
        begin_bit: 161,
        length: 5,
        multiplier: 2,
        base: 8,
        depends: 26,
    },
    RtcCalibEntry {
        begin_bit: 166,
        length: 6,
        multiplier: 2,
        base: 70,
        depends: 27,
    },
    RtcCalibEntry {
        begin_bit: 172,
        length: 8,
        multiplier: 2,
        base: 1677,
        depends: 0,
    },
    RtcCalibEntry {
        begin_bit: 180,
        length: 6,
        multiplier: 2,
        base: 23,
        depends: 29,
    },
    RtcCalibEntry {
        begin_bit: 186,
        length: 5,
        multiplier: 2,
        base: 6,
        depends: 30,
    },
    RtcCalibEntry {
        begin_bit: 191,
        length: 6,
        multiplier: 2,
        base: 13,
        depends: 31,
    },
];

fn signed_bit_to_int(number: u32, len: u32) -> i32 {
    if number >> (len - 1) != 0 {
        -((number ^ (1 << (len - 1))) as i32)
    } else {
        number as i32
    }
}

fn rtc_calib_tag(version: u8, unit: u8, atten: Attenuation, param: RtcCalibParam) -> Option<u8> {
    let atten = atten as u8;
    let offset = match (version, param) {
        (1, RtcCalibParam::Vlow) => 1,
        (1, RtcCalibParam::Vhigh) => 9,
        (2, RtcCalibParam::Vhigh) => 17,
        (2, RtcCalibParam::Vinit) => 25,
        _ => return None,
    };
    Some(offset + unit * 4 + atten)
}

fn rtc_calib_parsed(tag: u8, skip_efuse: bool) -> i32 {
    if tag == 0 {
        return 0;
    }
    let entry = &RTC_CALIB_MAP[tag as usize];
    let raw = if skip_efuse {
        0
    } else {
        let field = crate::efuse::EfuseField::new(
            2,
            (entry.begin_bit as u32) / 32,
            entry.begin_bit as u32,
            entry.length as u32,
        );
        let bits = super::read_field_le::<u32>(field);
        signed_bit_to_int(bits, entry.length as u32) * i32::from(entry.multiplier)
    };
    raw + i32::from(entry.base) + rtc_calib_parsed(entry.depends, skip_efuse)
}

/// Reads a parsed RTC calibration table value.
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32s2/esp_efuse_rtc_table.c#L145>
#[instability::unstable]
pub fn rtc_calib_reading(version: u8, unit: u8, atten: Attenuation, param: RtcCalibParam) -> i32 {
    rtc_calib_reading_inner(version, unit, atten, param, false)
}

pub(crate) fn rtc_calib_reading_inner(
    version: u8,
    unit: u8,
    atten: Attenuation,
    param: RtcCalibParam,
    skip_efuse: bool,
) -> i32 {
    rtc_calib_tag(version, unit, atten, param)
        .map(|tag| rtc_calib_parsed(tag, skip_efuse))
        .unwrap_or(0)
}

/// Get version of RTC calibration block
///
/// See <https://github.com/espressif/esp-idf/blob/027613140/components/efuse/esp32s2/esp_efuse_rtc_table.c#L92>
#[instability::unstable]
pub fn rtc_calib_version() -> u8 {
    super::read_field_le::<u8>(BLK_VERSION_MINOR)
}

/// Get ADC initial code for specified attenuation from efuse
#[instability::unstable]
pub fn rtc_calib_init_code(unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    if rtc_calib_version() != 2 {
        return None;
    }
    Some(rtc_calib_reading(2, unit.index(), atten, RtcCalibParam::Vinit) as u16)
}

/// Get ADC reference point voltage for specified attenuation in millivolts
#[instability::unstable]
pub fn rtc_calib_cal_mv(_unit: AdcCalibUnit, atten: Attenuation) -> u16 {
    match atten {
        Attenuation::_0dB => 600,
        Attenuation::_2p5dB => 800,
        Attenuation::_6dB => 1000,
        Attenuation::_11dB => 2000,
    }
}

/// Get ADC reference point digital code for specified attenuation
#[instability::unstable]
pub fn rtc_calib_cal_code(unit: AdcCalibUnit, atten: Attenuation) -> Option<u16> {
    if rtc_calib_version() != 2 {
        return None;
    }
    Some(rtc_calib_reading(2, unit.index(), atten, RtcCalibParam::Vhigh) as u16)
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

/// Returns the major hardware revision
#[instability::unstable]
pub fn major_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MAJOR)
}

/// Returns the minor hardware revision
#[instability::unstable]
pub fn minor_chip_version() -> u8 {
    super::read_field_le::<u8>(WAFER_VERSION_MINOR_HI) << 3
        | super::read_field_le::<u8>(WAFER_VERSION_MINOR_LO)
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
            Self::Block2 => efuse.rd_sys_data_part1_(0).as_ptr(),
            Self::Block3 => efuse.rd_usr_data(0).as_ptr(),
            Self::Block4 => efuse.rd_key0_data(0).as_ptr(),
            Self::Block5 => efuse.rd_key1_data(0).as_ptr(),
            Self::Block6 => efuse.rd_key2_data(0).as_ptr(),
            Self::Block7 => efuse.rd_key3_data(0).as_ptr(),
            Self::Block8 => efuse.rd_key4_data(0).as_ptr(),
            Self::Block9 => efuse.rd_key5_data(0).as_ptr(),
            Self::Block10 => efuse.rd_sys_data_part2_(0).as_ptr(),
        }
    }
}
