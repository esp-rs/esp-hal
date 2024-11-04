//! ESP32-specific implementation
use crate::i2c::master::*;

pub(crate) const I2C_CHUNK_SIZE: usize = 32;
pub(crate) const I2C_LL_INTR_MASK: u32 = 0x3ffff;

impl Driver<'_> {
    pub(crate) async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        // for ESP32 we need a timeout here but wasting a timer seems unnecessary
        // given the short time we spend here

        let mut tout = MAX_ITERATIONS / 10; // adjust the timeout because we are yielding in the loop
        loop {
            let interrupts = self.info.register_block().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed (either a STOP condition or END was
            // processed)
            if (!end_only && interrupts.trans_complete().bit_is_set())
                || interrupts.end_detect().bit_is_set()
            {
                break;
            }

            tout -= 1;
            if tout == 0 {
                return Err(Error::TimeOut);
            }

            embassy_futures::yield_now().await;
        }
        self.check_all_commands_done()?;
        Ok(())
    }

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

    /// Checks for I2C transmission errors and handles them.
    ///
    /// This function inspects specific I2C-related interrupts to detect errors
    /// during communication, such as timeouts, failed acknowledgments, or
    /// arbitration loss. If an error is detected, the function handles it
    /// by resetting the I2C peripheral to clear the error condition and then
    /// returns an appropriate error.
    pub(crate) fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.info.register_block().int_raw().read();

        let retval = if interrupts.time_out().bit_is_set() {
            Error::TimeOut
        } else if interrupts.ack_err().bit_is_set() {
            Error::AckCheckFailed
        } else if interrupts.arbitration_lost().bit_is_set() {
            Error::ArbitrationLost
        } else {
            return Ok(());
        };

        self.reset();

        Err(retval)
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    pub(crate) fn reset(&self) {
        // Clear all I2C interrupts
        self.info
            .register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Resets the transmit and receive FIFO buffers
    pub(crate) fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.info
            .register_block()
            .fifo_conf()
            .modify(|_, w| unsafe {
                w.tx_fifo_rst().set_bit();
                w.rx_fifo_rst().set_bit();
                w.nonfifo_en().clear_bit();
                w.nonfifo_rx_thres().bits(1);
                w.nonfifo_tx_thres().bits(32)
            });

        self.info.register_block().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.info
            .register_block()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(&self, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = Clocks::get().i2c_clock.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        let timeout = timeout.map_or(Some(0xF_FFFF), |to_bus| {
            Some((to_bus * 2 * half_cycle).min(0xF_FFFF))
        });

        // SCL period. According to the TRM, we should always subtract 1 to SCL low
        // period
        let scl_low = scl_low - 1;
        // Still according to the TRM, if filter is not enbled, we have to subtract 7,
        // if SCL filter is enabled, we have to subtract:
        //   8 if SCL filter is between 0 and 2 (included)
        //   6 + SCL threshold if SCL filter is between 3 and 7 (included)
        // to SCL high period
        let mut scl_high = scl_high;
        // In the "worst" case, we will subtract 13, make sure the result will still be
        // correct

        // FIXME since we always set the filter threshold to 7 we don't need conditional
        // code here once that changes we need the conditional code here
        scl_high -= 7 + 6;

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
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold;
        let scl_stop_hold_time = hold;

        configure_clock(
            self.info.register_block(),
            0,
            scl_low_period,
            scl_high_period,
            0,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        );
    }

    pub(crate) fn read_fifo(&self) -> u8 {
        self.info.register_block().data().read().fifo_rdata().bits()
    }

    pub(crate) fn write_fifo(&self, data: u8) {
        let base_addr = self.info.register_block().scl_low_period().as_ptr();
        let fifo_ptr = (if base_addr as u32 == 0x3FF53000 {
            0x6001301c
        } else {
            0x6002701c
        }) as *mut u32;
        unsafe { fifo_ptr.write_volatile(data as u32) };
    }
}
