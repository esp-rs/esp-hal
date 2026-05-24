use super::{Config, ConfigError, Driver, RegisterBlock, configure_clock};

/// Sets the frequency of the I2C interface by calculating and applying the
/// associated timings - corresponds to i2c_ll_cal_bus_clk and
/// i2c_ll_set_bus_timing in ESP-IDF
pub(super) fn set_frequency(driver: &Driver<'_>, clock_config: &Config) -> Result<(), ConfigError> {
    let timeout = clock_config.timeout;

    // TODO: could be REF_TICK
    let source_clk = crate::soc::clocks::apb_clk_frequency();

    let bus_freq = clock_config.frequency.as_hz();

    let half_cycle: u32 = source_clk / bus_freq / 2;
    // SCL
    let scl_low = half_cycle;
    // default, scl_wait_high < scl_high
    let scl_high = half_cycle / 2 + 2;
    let scl_wait_high = half_cycle - scl_high;
    let sda_hold = half_cycle / 2;
    // scl_wait_high < sda_sample <= scl_high
    let sda_sample = half_cycle / 2 - 1;
    let setup = half_cycle;
    let hold = half_cycle;

    // scl period
    let scl_low_period = scl_low - 1;
    let scl_high_period = scl_high;
    let scl_wait_high_period = scl_wait_high;
    // sda sample
    let sda_hold_time = sda_hold;
    let sda_sample_time = sda_sample;
    // setup
    let scl_rstart_setup_time = setup;
    let scl_stop_setup_time = setup;
    // hold
    let scl_start_hold_time = hold - 1;
    let scl_stop_hold_time = hold;

    configure_clock(
        driver.info,
        0,
        scl_low_period,
        scl_high_period,
        scl_wait_high_period,
        sda_hold_time,
        sda_sample_time,
        scl_rstart_setup_time,
        scl_stop_setup_time,
        scl_start_hold_time,
        scl_stop_hold_time,
        timeout.apb_cycles(half_cycle)?,
    )?;

    Ok(())
}

/// Resets the transmit and receive FIFO buffers.
pub(super) fn reset_fifo(driver: &Driver<'_>) {
    driver.regs().fifo_conf().write(|w| unsafe {
        w.tx_fifo_rst().set_bit();
        w.rx_fifo_rst().set_bit();
        w.nonfifo_en().clear_bit();
        w.fifo_prt_en().set_bit();
        w.rxfifo_wm_thrhd().bits(1);
        w.txfifo_wm_thrhd().bits(8)
    });

    driver.regs().fifo_conf().modify(|_, w| {
        w.tx_fifo_rst().clear_bit();
        w.rx_fifo_rst().clear_bit()
    });

    driver.regs().int_clr().write(|w| {
        w.rxfifo_wm().clear_bit_by_one();
        w.txfifo_wm().clear_bit_by_one()
    });

    driver.update_registers();
}

pub(super) fn read_fifo(register_block: &RegisterBlock) -> u8 {
    // Apparently the ESP32-S2 can read just fine using DPORT, so use this workaround on S2 only.
    let peri_offset =
        register_block as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
    let fifo_ptr =
        (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
    unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
}

pub(super) fn write_fifo(register_block: &RegisterBlock, data: u8) {
    let peri_offset =
        register_block as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
    let fifo_ptr =
        (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
    unsafe {
        fifo_ptr.write_volatile(data as u32);
    }
}
