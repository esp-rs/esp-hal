//! Reading of eFuses

use crate::peripherals::EFUSE;

pub struct Efuse;

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    ///
    /// # Example
    ///
    /// ```
    /// let mac_address = Efuse::get_mac_address();
    /// writeln!(
    ///     serial_tx,
    ///     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
    ///     mac_address[0],
    ///     mac_address[1],
    ///     mac_address[2],
    ///     mac_address[3],
    ///     mac_address[4],
    ///     mac_address[5]
    /// );
    /// ```
    pub fn get_mac_address() -> [u8; 6] {
        let efuse = unsafe { &*EFUSE::ptr() };

        let mac_low: u32 = efuse.rd_mac_spi_sys_0.read().mac_0().bits();
        let mac_high: u32 = efuse.rd_mac_spi_sys_1.read().mac_1().bits() as u32;

        let mac_low_bytes = mac_low.to_be_bytes();
        let mac_high_bytes = mac_high.to_be_bytes();

        [
            mac_high_bytes[2],
            mac_high_bytes[3],
            mac_low_bytes[0],
            mac_low_bytes[1],
            mac_low_bytes[2],
            mac_low_bytes[3],
        ]
    }

    /// Get status of SPI boot encryption.
    pub fn get_flash_encryption() -> bool {
        let efuse = unsafe { &*EFUSE::ptr() };
        (efuse
            .rd_repeat_data1
            .read()
            .spi_boot_crypt_cnt()
            .bits()
            .count_ones()
            % 2)
            != 0
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn get_rwdt_multiplier() -> u8 {
        let efuse = unsafe { &*EFUSE::ptr() };
        efuse.rd_repeat_data1.read().wdt_delay_sel().bits()
    }
}

const MAX_BLK_LEN: u32 = 256;

// #[derive(Debug, PartialEq)]
// pub enum EfuseErr {
//     /// Success
//     Ok = 0,
//     /// Generic indication of failure
//     Fail = -1,
//     /// Out of memory
//     NoMem = 0x101,
//     /// Invalid argument
//     InvalidArg = 0x102,
//     /// Invalid state
//     InvalidState = 0x103,
//     /// Invalid size
//     InvalidSize = 0x104,
//     /// requested resource not found
//     NotFound = 0x105,
//     /// Operation or feature not supported
//     NotSupported = 0x106,
//     /// Operation timed out
//     Timeout = 0x107,
//     /// Received response was invalid
//     InvalidResponse = 0x108,
//     /// CRC or checksum was invalid
//     InvalidCrc = 0x109,
//     /// Version was invalid
//     InvalidVersion = 0x10A,
//     /// MAC address was invalid
//     InvalidMac = 0x10B,
//     /// There are items remained to retrieve
//     NotFinished = 0x10C,
//     /// Starting number of WiFi error codes
//     WifiBase = 0x3000,
//     /// Starting number of MESH error codes
//     MeshBase = 0x4000,
//     /// Starting number of flash error codes
//     FlashBase = 0x6000,
//     /// Starting number of HW cryptography module error codes
//     HwCryptoBase = 0xc000,
//     /// Starting number of Memory Protection API error codes
//     MemProtBase = 0xd000,
//     /// Error while a encoding operation
//     ErrCoding = 0x1600 + 0x04,
// }

#[derive(Debug, PartialEq)]
pub enum EfuseErr {
    Ok = 0,
    BadArgs,
    Efuse = 0x1600,
    OkEfuseCnt,
    EfuseCntIsFull,
    EfuseRepeatedProg,
    Coding,
    NotEnoughUnusedKeyBlocks,
    DamagedReading,
}

pub fn efuse_calib_get_version() {

}


fn efuse_utility_get_number_of_items(bits: u32, size_of_base: u32) -> u32 {
    bits / size_of_base + (bits % size_of_base > 0) as u32
}

pub fn efuse_field_size(field: &[&EFuseAddress]) -> u32  {
    let mut bits_counter: u32 = 0;
    for i in 0..field.len() {
        if let Some(desc) = field.get(i) {
            bits_counter += desc.size as u32;
        }
    }
    bits_counter

}

pub fn check_range_of_bits(blk: EFuseBlock, offset_in_bits: u8, size_bits: u8) -> bool {
    let max_num_bits = offset_in_bits as u32 % MAX_BLK_LEN + size_bits as u32;
    if max_num_bits > MAX_BLK_LEN {
        return false
    }
    true
}

type EfuseFuncProc = fn(
    blk: EFuseBlock,
    num_reg: u32,
    bit_start: u32,
    bit_count: u32,
    arr: *mut core::ffi::c_void,
    bits_counter: &mut u32,
) -> EfuseErr;

pub fn get_reg_num(bit_offset: i32, bit_count: i32, i_reg: i32) -> i32{
    let bit_start = bit_offset as u32 % MAX_BLK_LEN;
    let num_reg = i_reg + bit_start as i32 / 32;

    if num_reg > (bit_start as i32 + bit_count - 1) / 32 {
      return -1;
    }

  num_reg
}

pub fn get_count_bits_in_reg(bit_offset: u32, bit_count: u32, i_reg: u32) -> u32 {
    let mut ret_count = 0;
    let mut num_reg = 0;
    let bit_start = bit_offset as u32 % MAX_BLK_LEN;
    let last_used_bit = bit_start + bit_count - 1;

    for num_bit in bit_start..=last_used_bit {
      ret_count += 1;
      if (((num_bit + 1) % 32) == 0) || (num_bit == last_used_bit) {
          if i_reg == num_reg {
              return ret_count;
            }

          num_reg += 1;
          ret_count = 0;
        }
    }
    0
}

pub fn efuse_get_mask(bit_count: u32, shift: u32) -> u32 {
    let mut mask: u32 = 0;

    if bit_count != 32 {
        mask = (1 << bit_count) - 1;
    } else {
        mask = 0xffff_ffff;
    }
    mask << shift
}

pub fn efuse_fill_buff(blk: EFuseBlock, num_reg: u32, bit_offset: u32, mut bit_count: u32, arr_out: &mut [u8], bits_counter: &mut u32) -> EfuseErr {
    let blob = arr_out;
    let efuse_block = bit_offset as u32/ MAX_BLK_LEN;
    let bit_start = bit_offset as u32 % MAX_BLK_LEN;
    let reg = unsafe {
        ((*blk.address() + num_reg * 4) as *const u32).read_volatile()
    };
    let reg_of_aligned_bits = (reg >> bit_start) & efuse_get_mask(bit_count, 0);
    let mut sum_shift = 0;
    let mut shift_bit = (*bits_counter) % 8;

    if shift_bit != 0 {
        blob[((*bits_counter) % 8) as usize] |= (reg_of_aligned_bits << shift_bit) as u8;
        shift_bit = (8 - shift_bit).min(bit_count);
        (*bits_counter) += shift_bit;
        bit_count -= shift_bit;
    }

    while bit_count > 0 {
        sum_shift += shift_bit;
        blob[((*bits_counter) % 8) as usize] |= (reg_of_aligned_bits >> sum_shift) as u8;
        shift_bit = bit_count.min(8);
        (*bits_counter) += shift_bit;
        bit_count -= shift_bit;
    }

    EfuseErr::Ok
}

pub fn efuse_utility_process(
    field: &[&EFuseAddress],
    ptr: *mut core::ffi::c_void,
    ptr_size_bits: u32,
    func_proc: EfuseFuncProc,
) -> EfuseErr {
    let mut err = EfuseErr::Ok;
    let mut bits_counter = 0;

    // Get and check size.
    let field_len = efuse_field_size(field);
    let req_size = if ptr_size_bits == 0 {
        field_len
    } else {
        core::cmp::min(ptr_size_bits, field_len)
    };

    let mut i = 0;
    let count_before = 0;
    while err == EfuseErr::Ok && req_size > bits_counter && i < field.len() {
        if let Some(desc) = field.get(i) {
            if check_range_of_bits(desc.block, desc.offset, desc.size) == false {
                return EfuseErr::Coding;
            }
            let mut i_reg:u32 = 0;
            let num_reg = get_reg_num(desc.offset as i32, desc.size as i32, i_reg as i32);
            while err == EfuseErr::Ok && req_size > bits_counter
                && num_reg != -1
            {
                let mut num_bits = get_count_bits_in_reg(
                    field[i].offset as u32,
                    field[i].size as u32,
                    i_reg);
                let bit_offset = field[i].offset;

                if (bits_counter + num_bits) > req_size {
                /* Limits the length of the field */
                num_bits = req_size - bits_counter;
                }

                err = func_proc((unsafe { *field.as_ptr() }).block, num_reg as u32, bit_offset as u32, num_bits, ptr, &mut bits_counter);
                i_reg += 1;
            }
        }
        i += 1;
    }
    // let count_after = 0;
    // if err == EfuseErr::Ok
    //     && (func_proc == esp_efuse_utility_fill_buff || func_proc == esp_efuse_utility_count_once) // These functions are used for read APIs: read_field_blob and read_field_cnt.
    //     && (count_before != count_after || (count_after & 1) == 1)
    // {
    //     err = EfuseErr::DamagedReading;
    // }
    assert!(bits_counter <= req_size);
    err
}

pub fn efuse_fill_buff_wrapper(
    blk: EFuseBlock,
    num_reg: u32,
    bit_offset: u32,
    bit_count: u32,
    arr_out: *mut core::ffi::c_void,
    bits_counter: &mut u32,
) -> EfuseErr {
    let arr_out = arr_out as *mut u8;
    let arr_out = unsafe { core::slice::from_raw_parts_mut(arr_out, 8) };
    efuse_fill_buff(blk, num_reg, bit_offset, bit_count, arr_out, bits_counter)
}

pub fn efuse_read_field(field: &EFuseAddress, dst: *mut core::ffi::c_void, dst_size_bits: u32) -> EfuseErr {
    let mut err = EfuseErr::Ok;

    if dst.is_null() || dst_size_bits == 0 {
        return EfuseErr::BadArgs;
    } else {
        let dst_len = efuse_utility_get_number_of_items(dst_size_bits, 8);
        unsafe { core::ptr::write_bytes(dst, 0, dst_len as usize) };
        err = efuse_utility_process(&[&field], dst, dst_size_bits, efuse_fill_buff_wrapper);
    }
    err

    // loop {
    //     unsafe {
    //         let dst_len = efuse_utility_get_number_of_items(dst_size_bits, 8);
    //         core::ptr::write_bytes(dst, 0, dst_len as usize);
    //         let err = efuse_utility_process(&[&field], dst as *mut core::ffi::c_void, dst_size_bits, efuse_fill_buff);
    //         Ok(())
    //     }
    // }
}

pub fn calib_get_version() {
    let blk_ver_major: u32 = 0;

}

pub fn calib_get_init_code() {

}

#[allow(unused)]
#[derive(Copy, Clone, PartialEq)]
pub enum EFuseBlock {
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

impl EFuseBlock {
    fn address(self) -> *const u32 {
        let efuse = unsafe { &*EFUSE::ptr() };
        match self {
            EFuseBlock::Block0 => efuse.rd_wr_dis.as_ptr(),
            EFuseBlock::Block1 => efuse.rd_mac_spi_sys_0.as_ptr(),
            EFuseBlock::Block2 => efuse.rd_sys_part1_data0.as_ptr(),
            EFuseBlock::Block3 => efuse.rd_usr_data0.as_ptr(),
            EFuseBlock::Block4 => efuse.rd_key0_data0.as_ptr(),
            EFuseBlock::Block5 => efuse.rd_key1_data0.as_ptr(),
            EFuseBlock::Block6 => efuse.rd_key2_data0.as_ptr(),
            EFuseBlock::Block7 => efuse.rd_key3_data0.as_ptr(),
            EFuseBlock::Block8 => efuse.rd_key4_data0.as_ptr(),
            EFuseBlock::Block9 => efuse.rd_key5_data0.as_ptr(),
            EFuseBlock::Block10 => efuse.rd_sys_part2_data0.as_ptr(),
        }
    }
}

pub struct EFuseAddress {
    block: EFuseBlock,
    offset: u8,
    size: u8,
}

impl EFuseAddress {
    pub fn read(&self) -> u32 {
        let first_word_address = unsafe { self.block.address().add(self.offset as usize / 32) };
        let first_word: u32 = unsafe { first_word_address.read_volatile() };

        let offset_in_word = self.offset % 32;
        let mask = u32::MAX >> (32 - self.size);

        if offset_in_word + self.size > 32 {
            let second_word: u32 = unsafe { first_word_address.add(1).read_volatile() };
            ((first_word >> offset_in_word) | (second_word << (32 - offset_in_word))) & mask
        } else {
            (first_word >> offset_in_word) & mask
        }
    }

    // taken from https://github.com/espressif/esp-idf/blob/045163a2ec99eb3cb7cc69e2763afd145156c4cf/components/efuse/esp32s3/esp_efuse_table.csv

    /// BLK_VERSION_MAJOR of BLOCK2 change of this bit means users need to
    /// update firmware
    pub const BLK_VERSION_MAJOR: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 128,
        size: 2,
    };

    /// ADC1 init code at atten0
    pub const ADC1_INIT_CODE_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 148,
        size: 10,
    };
    /// ADC1 init code at atten1
    pub const ADC1_INIT_CODE_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 158,
        size: 10,
    };
    /// ADC1 init code at atten2
    pub const ADC1_INIT_CODE_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 168,
        size: 10,
    };
    /// ADC1 init code at atten3
    pub const ADC1_INIT_CODE_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 178,
        size: 10,
    };
    /// ADC2 init code at atten0
    // pub const ADC2_INIT_CODE_ATTEN0: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 175,
    //     size: 8,
    // };
    // /// ADC2 init code at atten1
    // pub const ADC2_INIT_CODE_ATTEN1: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 183,
    //     size: 6,
    // };
    // /// ADC2 init code at atten2
    // pub const ADC2_INIT_CODE_ATTEN2: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 189,
    //     size: 6,
    // };
    // /// ADC2 init code at atten3
    // pub const ADC2_INIT_CODE_ATTEN3: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 195,
    //     size: 6,
    // };
    /// ADC1 calibration voltage at atten0
    pub const ADC1_CAL_VOL_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 188,
        size: 10,
    };
    /// ADC1 calibration voltage at atten1
    pub const ADC1_CAL_VOL_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 198,
        size: 10,
    };
    /// ADC1 calibration voltage at atten2
    pub const ADC1_CAL_VOL_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 208,
        size: 10,
    };
    /// ADC1 calibration voltage at atten3
    pub const ADC1_CAL_VOL_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 218,
        size: 10,
    };
    // /// ADC2 calibration voltage at atten0
    // pub const ADC2_CAL_VOL_ATTEN0: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 233,
    //     size: 8,
    // };
    // /// ADC2 calibration voltage at atten1
    // pub const ADC2_CAL_VOL_ATTEN1: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 241,
    //     size: 7,
    // };
    // /// ADC2 calibration voltage at atten2
    // pub const ADC2_CAL_VOL_ATTEN2: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block2,
    //     offset: 248,
    //     size: 7,
    // };
    // /// ADC2 calibration voltage at atten3
    // pub const ADC2_CAL_VOL_ATTEN3: EFuseAddress = EFuseAddress {
    //     block: EFuseBlock::Block1,
    //     offset: 186,
    //     size: 6,
    // };
}