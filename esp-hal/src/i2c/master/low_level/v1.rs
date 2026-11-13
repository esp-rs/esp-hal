use super::{Config, ConfigError, Driver, RegisterBlock, configure_clock};

/// Largest value of the 14-bit `SCL_LOW_PERIOD`, `SCL_HIGH_PERIOD` and
/// `SCL_STOP_HOLD` fields.
const PERIOD_MAX: u32 = 0x3FFF;
/// Largest value of the 10-bit `SDA_HOLD`, `SDA_SAMPLE`, `SCL_RSTART_SETUP`,
/// `SCL_STOP_SETUP` and `SCL_START_HOLD` fields.
const TIME_MAX: u32 = 0x3FF;

/// Sets the frequency of the I2C interface by calculating and applying the
/// associated timings - corresponds to i2c_ll_cal_bus_clk and
/// i2c_ll_set_bus_timing in ESP-IDF.
pub(super) fn set_frequency(driver: &Driver<'_>, clock_config: &Config) -> Result<(), ConfigError> {
    let timeout = clock_config.timeout;
    let bus_freq = clock_config.frequency.as_hz();

    // ESP32 I2C is hardwired to APB; configure_function_clock is a no-op.
    let source_clk = driver.info.clock_instance.function_clock_frequency();

    let half_cycle: u32 = source_clk / bus_freq / 2;
    let scl_low = half_cycle;
    let scl_high = half_cycle;
    let sda_hold = half_cycle / 2;
    let sda_sample = scl_high / 2;
    let setup = half_cycle;
    let hold = half_cycle;

    // SCL period. According to the TRM, we should always subtract 1 to SCL low
    // period
    let scl_low = scl_low
        .checked_sub(1)
        .ok_or(ConfigError::FrequencyOutOfRange)?;
    // Still according to the TRM, if filter is not enabled, we have to subtract 7,
    // if SCL filter is enabled, we have to subtract:
    //   8 if SCL filter is between 0 and 2 (included)
    //   6 + SCL threshold if SCL filter is between 3 and 7 (included)
    // to SCL high period
    let mut scl_high = scl_high;
    // In the "worst" case, we will subtract 13, make sure the result will still be
    // correct

    // FIXME since we always set the filter threshold to 7 we don't need conditional
    // code here once that changes we need the conditional code here
    scl_high = scl_high
        .checked_sub(7 + 6)
        .ok_or(ConfigError::FrequencyOutOfRange)?;

    // if (filter_cfg_en) {
    //     if (thres <= 2) {
    //         scl_high -= 8;
    //     } else {
    //         assert(hw->scl_filter_cfg.thres <= 7);
    //         scl_high -= thres + 6;
    //     }
    // } else {
    //    scl_high -= 7;
    //}

    let scl_high_period = scl_high;
    let scl_low_period = scl_low;
    // The SCL periods set the bus frequency, so they cannot be clamped.
    if scl_low_period > PERIOD_MAX {
        return Err(ConfigError::FrequencyOutOfRange);
    }

    // The fields below are narrower than the SCL periods. Clamp them instead of
    // letting the register write truncate them. The setup and hold times stay
    // above the minimums of the I2C specification at the field maximum.
    // sda sample
    let sda_hold_time = sda_hold.min(TIME_MAX);
    let sda_sample_time = sda_sample.min(TIME_MAX);
    // setup
    let scl_rstart_setup_time = setup.min(TIME_MAX);
    let scl_stop_setup_time = setup.min(TIME_MAX);
    // hold
    let scl_start_hold_time = hold.min(TIME_MAX);
    let scl_stop_hold_time = hold.min(PERIOD_MAX);

    configure_clock(
        driver.info,
        scl_low_period,
        scl_high_period,
        0,
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
        w.nonfifo_rx_thres().bits(1);
        w.nonfifo_tx_thres().bits(32)
    });

    driver.regs().fifo_conf().modify(|_, w| {
        w.tx_fifo_rst().clear_bit();
        w.rx_fifo_rst().clear_bit()
    });

    driver
        .regs()
        .int_clr()
        .write(|w| w.rxfifo_full().clear_bit_by_one());
}

pub(super) fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data().read().fifo_rdata().bits()
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
