use super::{Config, ConfigError, Driver, RegisterBlock, configure_clock};

/// Sets the frequency of the I2C interface by calculating and applying the
/// associated timings - corresponds to i2c_ll_cal_bus_clk and
/// i2c_ll_set_bus_timing in ESP-IDF
pub(super) fn set_frequency(driver: &Driver<'_>, clock_config: &Config) -> Result<(), ConfigError> {
    let timeout = clock_config.timeout;

    let source_clk = crate::soc::clocks::xtal_clk_frequency();

    let bus_freq = clock_config.frequency.as_hz();

    let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
    let sclk_freq: u32 = source_clk / clkm_div;
    let half_cycle: u32 = sclk_freq / bus_freq / 2;
    // SCL
    let scl_low = half_cycle;
    // default, scl_wait_high < scl_high
    // Make 80KHz as a boundary here, because when working at lower frequency, too
    // much scl_wait_high will faster the frequency according to some
    // hardware behaviors.
    let scl_wait_high = if bus_freq >= 80 * 1000 {
        half_cycle / 2 - 2
    } else {
        half_cycle / 4
    };
    let scl_high = half_cycle - scl_wait_high;
    let sda_hold = half_cycle / 4;
    let sda_sample = half_cycle / 2;
    let setup = half_cycle;
    let hold = half_cycle;

    // According to the Technical Reference Manual, the following timings must be
    // subtracted by 1. However, according to the practical measurement and
    // some hardware behaviour, if wait_high_period and scl_high minus one.
    // The SCL frequency would be a little higher than expected. Therefore, the
    // solution here is not to minus scl_high as well as scl_wait high, and
    // the frequency will be absolutely accurate to all frequency
    // to some extent.
    let scl_low_period = scl_low - 1;
    let scl_high_period = scl_high;
    let scl_wait_high_period = scl_wait_high;
    // sda sample
    let sda_hold_time = sda_hold - 1;
    let sda_sample_time = sda_sample - 1;
    // setup
    let scl_rstart_setup_time = setup - 1;
    let scl_stop_setup_time = setup - 1;
    // hold
    let scl_start_hold_time = hold - 1;
    let scl_stop_hold_time = hold - 1;

    configure_clock(
        driver.info,
        clkm_div,
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
    register_block.data().read().fifo_rdata().bits()
}

pub(super) fn write_fifo(register_block: &RegisterBlock, data: u8) {
    cfg_if::cfg_if! {
        if #[cfg(esp32p4)] {
            // P4: data register is read-only (RX FIFO only). TX uses txfifo_start_addr.
            // PAC txfifo_start_addr is also read-only in SVD, use direct MMIO.
            // TODO: file an esp-pacs issue/PR so the P4 SVD marks the TX FIFO
            // port writable like other chips' I2C PAC. Once that lands, this
            // branch can collapse into the general `else` arm below.
            let base = register_block as *const _ as usize;
            unsafe {
                ((base + 0x100) as *mut u32).write_volatile(data as u32);
            }
        } else {
            register_block
                .data()
                .write(|w| unsafe { w.fifo_rdata().bits(data) });
        }
    }
}
