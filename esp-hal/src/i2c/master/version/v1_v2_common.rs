use crate::i2c::master::*;

fn assert_read_length(len: usize) {
    if len <= 32 {
        return;
    }

    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            panic!("On ESP32 the max I2C read is limited to 32 bytes");
        } else {
            panic!("On ESP32-S2 the max I2C read is limited to 32 bytes");
        }
    }
}

impl Driver<'_> {
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    pub(crate) fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if start_index >= bytes.len() {
            return Ok(());
        }

        // this is only possible when writing the I2C address in release mode
        // from [perform_write_read]
        for b in bytes {
            self.write_fifo(*b);
            self.check_errors()?;
        }

        Ok(())
    }

    pub(crate) fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        assert_read_length(buffer.len());

        // wait for completion - then we can just read the data from FIFO
        // once we change to non-fifo mode to support larger transfers that
        // won't work anymore
        self.wait_for_completion_blocking(false)?;

        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            *byte = self.read_fifo();
        }

        Ok(())
    }

    pub(crate) async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        assert_read_length(buffer.len());

        self.wait_for_completion(false).await?;

        for byte in buffer.iter_mut() {
            *byte = self.read_fifo();
        }

        Ok(())
    }

    /// Updates the configuration of the I2C peripheral.
    pub(crate) fn update_config(&self) {}

    /// Sets the filter with a supplied threshold in clock cycles for which a
    /// pulse must be present to pass the filter
    pub(crate) fn set_filter(&self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        let register_block = self.info.register_block();
        register_block.sda_filter_cfg().modify(|_, w| {
            if let Some(threshold) = sda_threshold {
                unsafe { w.sda_filter_thres().bits(threshold) };
            }
            w.sda_filter_en().bit(sda_threshold.is_some())
        });
        register_block.scl_filter_cfg().modify(|_, w| {
            if let Some(threshold) = scl_threshold {
                unsafe { w.scl_filter_thres().bits(threshold) };
            }
            w.scl_filter_en().bit(scl_threshold.is_some())
        });
    }

    /// Fills the TX FIFO with data from the provided slice.
    pub(crate) fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if bytes.len() > 31 {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    panic!("On ESP32 the max I2C transfer is limited to 31 bytes");
                } else {
                    panic!("On ESP32-S2 the max I2C transfer is limited to 31 bytes");
                }
            }
        }

        for b in bytes {
            self.write_fifo(*b);
        }

        bytes.len()
    }
}
