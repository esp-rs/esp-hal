//! ESP32-S2-specific implementation
use crate::i2c::*;

pub(crate) const I2C_CHUNK_SIZE: usize = 32;
pub(crate) const I2C_LL_INTR_MASK: u32 = 0x1ffff;

impl Driver<'_> {
    pub(crate) async fn write_remaining_tx_fifo(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        if start_index >= bytes.len() {
            return Ok(());
        }

        for b in bytes {
            self.write_fifo(*b);
            self.check_errors()?;
        }

        Ok(())
    }

    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(&self, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = Clocks::get().apb_clock.raw();
        let bus_freq = bus_freq.raw();

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
        // default we set the timeout value to 10 bus cycles
        let time_out_value = timeout.unwrap_or(half_cycle * 20);

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
        let time_out_en = true;

        configure_clock(
            self.info.register_block(),
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
            time_out_value,
            time_out_en,
        );
    }

    pub(crate) fn read_fifo(&self) -> u8 {
        let base_addr = self.info.register_block().scl_low_period().as_ptr();
        let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
            0x6001301c
        } else {
            0x6002701c
        }) as *mut u32;
        unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
    }
}
