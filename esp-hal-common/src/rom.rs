pub use paste::paste;

/// Pauses execution for us microseconds
#[inline(always)]
pub unsafe fn esp_rom_delay_us(us: u32) {
    #[cfg(esp32)]
    const ESP_ROM_DELAY_US: u32 = 0x4000_8534;
    #[cfg(esp32s2)]
    const ESP_ROM_DELAY_US: u32 = 0x4000_d888;
    #[cfg(esp32s3)]
    const ESP_ROM_DELAY_US: u32 = 0x4000_0600;
    #[cfg(esp32c3)]
    const ESP_ROM_DELAY_US: u32 = 0x4000_0050;

    // cast to usize is just needed because of the way we run clippy in CI
    let fn_esp_rom_delay_us: fn(us: u32) = core::mem::transmute(ESP_ROM_DELAY_US as usize);

    fn_esp_rom_delay_us(us);
}

#[inline(always)]
/// Set the real CPU ticks per us to the ets, so that ets_delay_us
/// will be accurate. Call this function when CPU frequency is changed.
pub unsafe fn ets_update_cpu_frequency(ticks_per_us: u32) {
    #[cfg(esp32)]
    const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x4000_8550;
    #[cfg(esp32s2)]
    const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x4000_d8a4;
    #[cfg(esp32s3)]
    const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x4004_3164;
    #[cfg(esp32c3)]
    const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x4000_0588;

    // cast to usize is just needed because of the way we run clippy in CI
    let rom_ets_update_cpu_frequency: fn(ticks_per_us: u32) =
        core::mem::transmute(ETS_UPDATE_CPU_FREQUENCY as usize);

    rom_ets_update_cpu_frequency(ticks_per_us);
}

#[inline(always)]
pub unsafe fn regi2c_ctrl_write_reg(block: u32, block_hostid: u32, reg_add: u32, indata: u32) {
    #[cfg(esp32)]
    const ROM_I2C_WRITEREG: u32 = 0x4000_41a4;
    #[cfg(esp32s2)]
    const ROM_I2C_WRITEREG: u32 = 0x4000_a9a8;
    #[cfg(esp32s3)]
    const ROM_I2C_WRITEREG: u32 = 0x4000_5d60;
    #[cfg(esp32c3)]
    const ROM_I2C_WRITEREG: u32 = 0x4000_195c;

    // cast to usize is just needed because of the way we run clippy in CI
    let i2c_write_reg_raw: fn(block: u32, block_hostid: u32, reg_add: u32, indata: u32) -> i32 =
        core::mem::transmute(ROM_I2C_WRITEREG as usize);

    i2c_write_reg_raw(block, block_hostid, reg_add, indata);
}

#[macro_export]
macro_rules! regi2c_write {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste! {
            regi2c_ctrl_write_reg($block,
                [<$block _HOSTID>],
                $reg_add,
                $indata);
        }
    };
}

#[inline(always)]
pub unsafe fn regi2c_ctrl_write_reg_mask(
    block: u32,
    block_hostid: u32,
    reg_add: u32,
    reg_add_msb: u32,
    reg_add_lsb: u32,
    indata: u32,
) {
    #[cfg(esp32)]
    const ROM_I2C_WRITEREG_MASK: u32 = 0x4000_41fc;
    #[cfg(esp32s2)]
    const ROM_I2C_WRITEREG_MASK: u32 = 0x4000_aa00;
    #[cfg(esp32s3)]
    const ROM_I2C_WRITEREG_MASK: u32 = 0x4000_5d6c;
    #[cfg(esp32c3)]
    const ROM_I2C_WRITEREG_MASK: u32 = 0x4000_1960;

    // cast to usize is just needed because of the way we run clippy in CI
    let i2c_write_reg_mask_raw: fn(
        block: u32,
        block_hostid: u32,
        reg_add: u32,
        reg_add_msb: u32,
        reg_add_lsb: u32,
        indata: u32,
    ) -> i32 = core::mem::transmute(ROM_I2C_WRITEREG_MASK as usize);

    i2c_write_reg_mask_raw(
        block,
        block_hostid,
        reg_add,
        reg_add_msb,
        reg_add_lsb,
        indata,
    );
}

#[macro_export]
macro_rules! regi2c_write_mask {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste! {
            regi2c_ctrl_write_reg_mask($block,
                [<$block _HOSTID>],
                $reg_add,
                [<$reg_add _MSB>],
                [<$reg_add _LSB>],
                $indata);
        }
    };
}
