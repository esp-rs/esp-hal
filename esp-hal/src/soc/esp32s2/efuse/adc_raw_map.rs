// https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/include/esp_efuse_rtc_table.h#L31C1-L65C34
// These are the tags. Either use them directly or use
// esp_efuse_rtc_table_get_tag to calculate the corresponding tag.

use super::EfuseBlock;

pub const TAG_RTCCALIB_V1IDX_A10L: usize = 1;
pub const TAG_RTCCALIB_V1IDX_A11L: usize = 2;
pub const TAG_RTCCALIB_V1IDX_A12L: usize = 3;
pub const TAG_RTCCALIB_V1IDX_A13L: usize = 4;
pub const TAG_RTCCALIB_V1IDX_A20L: usize = 5;
pub const TAG_RTCCALIB_V1IDX_A21L: usize = 6;
pub const TAG_RTCCALIB_V1IDX_A22L: usize = 7;
pub const TAG_RTCCALIB_V1IDX_A23L: usize = 8;
pub const TAG_RTCCALIB_V1IDX_A10H: usize = 9;
pub const TAG_RTCCALIB_V1IDX_A11H: usize = 10;
pub const TAG_RTCCALIB_V1IDX_A12H: usize = 11;
pub const TAG_RTCCALIB_V1IDX_A13H: usize = 12;
pub const TAG_RTCCALIB_V1IDX_A20H: usize = 13;
pub const TAG_RTCCALIB_V1IDX_A21H: usize = 14;
pub const TAG_RTCCALIB_V1IDX_A22H: usize = 15;
pub const TAG_RTCCALIB_V1IDX_A23H: usize = 16;
pub const TAG_RTCCALIB_V2IDX_A10H: usize = 17;
pub const TAG_RTCCALIB_V2IDX_A11H: usize = 18;
pub const TAG_RTCCALIB_V2IDX_A12H: usize = 19;
pub const TAG_RTCCALIB_V2IDX_A13H: usize = 20;
pub const TAG_RTCCALIB_V2IDX_A20H: usize = 21;
pub const TAG_RTCCALIB_V2IDX_A21H: usize = 22;
pub const TAG_RTCCALIB_V2IDX_A22H: usize = 23;
pub const TAG_RTCCALIB_V2IDX_A23H: usize = 24;
pub const TAG_RTCCALIB_V2IDX_A10I: usize = 25;
pub const TAG_RTCCALIB_V2IDX_A11I: usize = 26;
pub const TAG_RTCCALIB_V2IDX_A12I: usize = 27;
pub const TAG_RTCCALIB_V2IDX_A13I: usize = 28;
pub const TAG_RTCCALIB_V2IDX_A20I: usize = 29;
pub const TAG_RTCCALIB_V2IDX_A21I: usize = 30;
pub const TAG_RTCCALIB_V2IDX_A22I: usize = 31;
pub const TAG_RTCCALIB_V2IDX_A23I: usize = 32;
pub const TAG_RTCCALIB_IDX_TMPSENSOR: usize = 33;

/// See <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/esp_efuse_rtc_table.c#L48>
pub struct MapInfo {
    pub block: EfuseBlock,
    pub begin_bit: u16,
    pub length: u16,
    pub multiplier: i32,
    pub base: i32,
    pub dependency: usize,
}

pub const RAW_MAP: [MapInfo; 34] = [
    MapInfo {
        block: EfuseBlock::Block0,
        begin_bit: 0,
        length: 0,
        multiplier: 0,
        base: 0,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 208,
        length: 6,
        multiplier: 4,
        base: 2231,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 214,
        length: 6,
        multiplier: 4,
        base: 1643,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 220,
        length: 6,
        multiplier: 4,
        base: 1290,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 226,
        length: 6,
        multiplier: 4,
        base: 701,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 232,
        length: 6,
        multiplier: 4,
        base: 2305,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 238,
        length: 6,
        multiplier: 4,
        base: 1693,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 244,
        length: 6,
        multiplier: 4,
        base: 1343,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 250,
        length: 6,
        multiplier: 4,
        base: 723,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 144,
        length: 8,
        multiplier: 4,
        base: 5775,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 152,
        length: 8,
        multiplier: 4,
        base: 5693,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 160,
        length: 8,
        multiplier: 4,
        base: 5723,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 168,
        length: 8,
        multiplier: 4,
        base: 6209,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 176,
        length: 8,
        multiplier: 4,
        base: 5817,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 184,
        length: 8,
        multiplier: 4,
        base: 5703,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 192,
        length: 8,
        multiplier: 4,
        base: 5731,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 200,
        length: 8,
        multiplier: 4,
        base: 6157,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 197,
        length: 6,
        multiplier: 2,
        base: 169,
        dependency: TAG_RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 203,
        length: 6,
        multiplier: 2,
        base: -26,
        dependency: TAG_RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 209,
        length: 9,
        multiplier: 2,
        base: 126,
        dependency: TAG_RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 218,
        length: 7,
        multiplier: 2,
        base: 387,
        dependency: TAG_RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 225,
        length: 7,
        multiplier: 2,
        base: 177,
        dependency: TAG_RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 232,
        length: 10,
        multiplier: 2,
        base: 5815,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 242,
        length: 7,
        multiplier: 2,
        base: 27,
        dependency: TAG_RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 249,
        length: 7,
        multiplier: 2,
        base: 410,
        dependency: TAG_RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 147,
        length: 8,
        multiplier: 2,
        base: 1519,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 155,
        length: 6,
        multiplier: 2,
        base: 88,
        dependency: TAG_RTCCALIB_V2IDX_A10I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 161,
        length: 5,
        multiplier: 2,
        base: 8,
        dependency: TAG_RTCCALIB_V2IDX_A11I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 166,
        length: 6,
        multiplier: 2,
        base: 70,
        dependency: TAG_RTCCALIB_V2IDX_A12I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 172,
        length: 8,
        multiplier: 2,
        base: 1677,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 180,
        length: 6,
        multiplier: 2,
        base: 23,
        dependency: TAG_RTCCALIB_V2IDX_A20I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 186,
        length: 5,
        multiplier: 2,
        base: 6,
        dependency: TAG_RTCCALIB_V2IDX_A21I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 191,
        length: 6,
        multiplier: 2,
        base: 13,
        dependency: TAG_RTCCALIB_V2IDX_A22I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 135,
        length: 9,
        multiplier: 1,
        base: 0,
        dependency: 0,
    },
];
