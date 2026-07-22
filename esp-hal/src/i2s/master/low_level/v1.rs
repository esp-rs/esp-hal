use super::Info;
#[cfg(not(esp32))]
use crate::i2s::master::Endianness;
use crate::{
    RegisterToggle,
    i2s::master::{
        Channels,
        Config,
        ConfigError,
        DataFormat,
        UnitConfig,
        WsWidth,
        private::I2sClockDividers,
    },
};

impl Info {
    pub(crate) fn set_clock(&self, clock_settings: I2sClockDividers) {
        self.regs().clkm_conf().modify(|r, w| unsafe {
            // select PLL_160M
            w.bits(r.bits() | (property!("i2s.default_clock_source") << 21))
        });

        #[cfg(esp32)]
        self.regs()
            .clkm_conf()
            .modify(|_, w| w.clka_ena().clear_bit());

        self.regs().clkm_conf().modify(|_, w| unsafe {
            w.clk_en().set_bit();
            w.clkm_div_num().bits(clock_settings.mclk_divider as u8);
            w.clkm_div_a().bits(clock_settings.denominator as u8);
            w.clkm_div_b().bits(clock_settings.numerator as u8)
        });

        self.regs().sample_rate_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num().bits(clock_settings.bclk_divider as u8);
            w.rx_bck_div_num().bits(clock_settings.bclk_divider as u8)
        });
    }

    pub(crate) fn update_tx(&self) {
        // Nothing to do.
    }

    pub(crate) fn update_rx(&self) {
        // Nothing to do.
    }

    pub(crate) fn configure(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate(self)?;

        match config {
            Config::Tdm(c) => {
                self.configure_tx(&c.tx_config, c.data_format)?;
                self.configure_rx(&c.rx_config, c.data_format)?;

                self.set_clock(config.calculate_clock());

                self.regs().sample_rate_conf().modify(|_, w| unsafe {
                    // Having different data formats for each direction would make clock
                    // calculations more tricky
                    w.tx_bits_mod().bits(c.data_format.data_bits());
                    w.rx_bits_mod().bits(c.data_format.data_bits())
                });

                self.regs().conf().modify(|_, w| {
                    w.tx_slave_mod().clear_bit();
                    w.rx_slave_mod().bit(c.signal_loopback);
                    // Send MSB to the right channel to be consistent with ESP32-S3 et al.
                    w.tx_msb_right().set_bit();
                    w.rx_msb_right().set_bit();
                    // ESP32 generates two clock pulses first. If the WS is low, those first
                    // clock pulses are indistinguishable from real
                    // data, which corrupts the first few samples. So we
                    // send the right channel first (which means WS is high during
                    // the first sample) to prevent this issue.
                    w.tx_right_first().set_bit();
                    w.rx_right_first().set_bit();
                    w.tx_mono().clear_bit();
                    w.rx_mono().clear_bit();
                    w.sig_loopback().bit(c.signal_loopback)
                });

                self.regs().fifo_conf().modify(|_, w| w.dscr_en().set_bit());

                self.regs().conf1().modify(|_, w| {
                    w.tx_pcm_bypass().set_bit();
                    w.rx_pcm_bypass().set_bit()
                });

                self.regs().pd_conf().modify(|_, w| {
                    w.fifo_force_pu().set_bit();
                    w.fifo_force_pd().clear_bit()
                });

                self.regs().conf2().modify(|_, w| {
                    w.camera_en().clear_bit();
                    w.lcd_en().clear_bit()
                });
            }
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            Config::Pdm(c) => {
                crate::i2s::pdm::configure_pdm(self, c)?;
            }
        }

        Ok(())
    }

    pub(crate) fn configure_tx(
        &self,
        config: &UnitConfig,
        data_format: DataFormat,
    ) -> Result<(), ConfigError> {
        config.validate()?;

        let chan_mod = match config.channels {
            Channels::STEREO | Channels::MONO => 0,
            Channels::LEFT => 3,
            Channels::RIGHT => 4,
            _ => unreachable!(),
        };

        let fifo_mod = match (data_format.data_bits(), config.channels == Channels::STEREO) {
            (8 | 16, true) => 0,
            (8 | 16, false) => 1,
            (24 | 32, true) => 2,
            (24 | 32, false) => 3,
            _ => unreachable!(),
        };

        self.regs().conf().modify(|_, w| {
            w.tx_msb_shift().bit(config.msb_shift);
            // Short frame synchronization
            w.tx_short_sync().bit(config.ws_width == WsWidth::Bit)
        });

        self.regs().conf1().modify(|_, w| w.tx_stop_en().set_bit());

        #[cfg(not(esp32))]
        self.regs().conf().modify(|_, w| {
            // Channel configurations other than Stereo should use same data from DMA
            // for both channels
            w.tx_dma_equal().bit(config.channels != Channels::STEREO);
            // Byte endianness
            w.tx_big_endian()
                .bit(config.endianness == Endianness::BigEndian)
        });

        self.regs().fifo_conf().modify(|_, w| unsafe {
            w.tx_fifo_mod().bits(fifo_mod);
            w.tx_fifo_mod_force_en().set_bit()
        });

        self.regs()
            .conf_sigle_data()
            .modify(|_, w| unsafe { w.sigle_data().bits(config.channels.fill.unwrap_or(0)) });

        self.regs()
            .conf_chan()
            .modify(|_, w| unsafe { w.tx_chan_mod().bits(chan_mod) });

        Ok(())
    }

    pub(crate) fn configure_rx(
        &self,
        config: &UnitConfig,
        data_format: DataFormat,
    ) -> Result<(), ConfigError> {
        config.validate()?;

        let chan_mod = match config.channels {
            Channels::STEREO => 0,
            Channels::LEFT | Channels::MONO => 1,
            Channels::RIGHT => 2,
            _ => unreachable!(),
        };

        let fifo_mod = match (data_format.data_bits(), config.channels == Channels::STEREO) {
            (8 | 16, true) => 0,
            (8 | 16, false) => 1,
            (24 | 32, true) => 2,
            (24 | 32, false) => 3,
            _ => unreachable!(),
        };

        self.regs().conf().modify(|_, w| {
            w.rx_msb_shift().bit(config.msb_shift);
            // Short frame synchronization
            w.rx_short_sync().bit(config.ws_width == WsWidth::Bit)
        });

        #[cfg(not(esp32))]
        self.regs().conf().modify(|_, w| {
            // Channel configurations other than Stereo should use same data from DMA
            // for both channels
            w.rx_dma_equal().bit(config.channels != Channels::STEREO);
            // Byte endianness
            w.rx_big_endian()
                .bit(config.endianness == Endianness::BigEndian)
        });

        self.regs().fifo_conf().modify(|_, w| unsafe {
            w.rx_fifo_mod().bits(fifo_mod);
            w.rx_fifo_mod_force_en().set_bit()
        });

        self.regs()
            .conf_chan()
            .modify(|_, w| unsafe { w.rx_chan_mod().bits(chan_mod) });

        Ok(())
    }

    pub(crate) fn set_master(&self) {
        self.regs().conf().modify(|_, w| {
            w.rx_slave_mod().clear_bit();
            w.tx_slave_mod().clear_bit()
        });
    }

    pub(crate) fn reset_tx(&self) {
        self.regs().conf().modify(|_, w| w.tx_reset().bit(true));
        self.regs()
            .conf()
            .modify(|_, w| w.tx_fifo_reset().bit(true));

        self.regs().conf().modify(|_, w| {
            w.tx_reset().bit(false);
            w.tx_fifo_reset().bit(false)
        });

        #[cfg(esp32s2)]
        while self.regs().conf().read().tx_reset_st().bit_is_set() {}

        #[cfg(esp32)]
        while self.regs().state().read().tx_fifo_reset_back().bit_is_set() {}

        self.regs().lc_conf().modify(|_, w| w.out_rst().bit(true));
        self.regs().lc_conf().modify(|_, w| w.out_rst().bit(false));

        self.regs().int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    pub(crate) fn tx_start(&self) {
        self.regs().conf().modify(|_, w| w.tx_start().set_bit());

        while self.regs().state().read().tx_idle().bit_is_set() {
            // wait
        }
    }

    pub(crate) fn tx_stop(&self) {
        self.regs().conf().modify(|_, w| w.tx_start().clear_bit());
    }

    pub(crate) fn is_tx_done(&self) -> bool {
        self.regs().state().read().tx_idle().bit_is_set()
    }

    pub(crate) fn reset_rx(&self) {
        self.regs().conf().toggle(|w, bit| {
            w.rx_reset().bit(bit);
            w.rx_fifo_reset().bit(bit)
        });

        #[cfg(esp32s2)]
        while self.regs().conf().read().rx_reset_st().bit_is_set() {}

        self.regs().lc_conf().toggle(|w, bit| w.in_rst().bit(bit));

        self.regs().int_clr().write(|w| {
            w.in_done().clear_bit_by_one();
            w.in_suc_eof().clear_bit_by_one()
        });
    }

    pub(crate) fn rx_start(&self, len: usize) {
        self.regs()
            .int_clr()
            .write(|w| w.in_suc_eof().clear_bit_by_one());

        let eof_num = cfg_select! {
            // On ESP32, the eof_num count in words.
            esp32 => len / 4,
            _ => len - 1,
        };

        self.regs()
            .rxeof_num()
            .modify(|_, w| unsafe { w.rx_eof_num().bits(eof_num as u32) });

        self.regs().conf().modify(|_, w| w.rx_start().set_bit());
    }

    pub(crate) fn rx_stop(&self) {
        self.regs().conf().modify(|_, w| w.rx_start().clear_bit());
    }

    pub(crate) fn is_rx_done(&self) -> bool {
        self.regs().int_raw().read().in_dscr_empty().bit_is_set()
    }
}
