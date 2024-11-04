use crate::i2c::master::{version::v2_v3_common::*, *};

pub(crate) const I2C_CHUNK_SIZE: usize = 254;
pub(crate) const I2C_LL_INTR_MASK: u32 = 0x3ffff;

impl Driver<'_> {
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    pub(crate) fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            while !self
                .info
                .register_block()
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {
                self.check_errors()?;
            }

            self.info
                .register_block()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            while !self
                .info
                .register_block()
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {
                self.check_errors()?;
            }

            if index >= bytes.len() {
                break Ok(());
            }

            self.write_fifo(bytes[index]);
            index += 1;
        }
    }

    pub(crate) async fn write_remaining_tx_fifo(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            self.info
                .register_block()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            if index >= bytes.len() {
                break Ok(());
            }

            self.write_fifo(bytes[index]);
            index += 1;
        }
    }

    /// Reads all bytes from the RX FIFO.
    pub(crate) fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            loop {
                self.check_errors()?;

                let reg = self.info.register_block().fifo_st().read();
                if reg.rxfifo_raddr().bits() != reg.rxfifo_waddr().bits() {
                    break;
                }
            }

            *byte = self.read_fifo();
        }

        Ok(())
    }

    pub(crate) async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        self.read_all_from_fifo_blocking(buffer)
    }

    /// Updates the configuration of the I2C peripheral.
    ///
    /// This function ensures that the configuration values, such as clock
    /// settings, SDA/SCL filtering, timeouts, and other operational
    /// parameters, which are configured in other functions, are properly
    /// propagated to the I2C hardware. This step is necessary to synchronize
    /// the software-configured settings with the peripheral's internal
    /// registers, ensuring that the hardware behaves according to the
    /// current configuration.
    pub(crate) fn update_config(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        self.info
            .register_block()
            .ctr()
            .modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a
    /// pulse must be present to pass the filter
    pub(crate) fn set_filter(&self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        let register_block = self.info.register_block();
        register_block.filter_cfg().modify(|_, w| {
            if let Some(threshold) = sda_threshold {
                unsafe { w.sda_filter_thres().bits(threshold) };
            }
            if let Some(threshold) = scl_threshold {
                unsafe { w.scl_filter_thres().bits(threshold) };
            }
            w.sda_filter_en().bit(sda_threshold.is_some());
            w.scl_filter_en().bit(scl_threshold.is_some())
        });
    }

    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(&self, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = Clocks::get().xtal_clock.raw();
        let bus_freq = bus_freq.raw();

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
        let sda_sample = half_cycle / 2 + scl_wait_high;
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
            self.info.register_block(),
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
            timeout.map(|to_bus| {
                let to_peri = (to_bus * 2 * half_cycle).max(1);
                let log2 = to_peri.ilog2();
                // Round up so that we don't shorten timeouts.
                let raw = if to_peri != 1 << log2 { log2 + 1 } else { log2 };
                raw.min(0x1F)
            }),
        );
    }

    pub(crate) fn read_fifo(&self) -> u8 {
        self.info.register_block().data().read().fifo_rdata().bits()
    }

    /// Fills the TX FIFO with data from the provided slice.
    pub(crate) fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        let mut index = 0;
        while index < bytes.len()
            && !self
                .info
                .register_block()
                .int_raw()
                .read()
                .txfifo_ovf()
                .bit_is_set()
        {
            self.write_fifo(bytes[index]);
            index += 1;
        }
        if self
            .info
            .register_block()
            .int_raw()
            .read()
            .txfifo_ovf()
            .bit_is_set()
        {
            index -= 1;
            self.info
                .register_block()
                .int_clr()
                .write(|w| w.txfifo_ovf().clear_bit_by_one());
        }
        index
    }
}
