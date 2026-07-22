use bitfield::Bit;

use super::Info;
use crate::i2s::master::{
    BitOrder,
    Config,
    ConfigError,
    Endianness,
    Polarity,
    UnitConfig,
    private::I2sClockDividers,
};

impl Info {
    #[cfg(not(any(i2s_clock_configured_by_pcr, i2s_clock_configured_by_hp_sys_clkrst)))]
    pub(crate) fn set_tx_clock(&self, clock_settings: I2sClockDividers) {
        let clkm_div = clock_settings.mclk_dividers();

        self.regs().tx_clkm_div_conf().modify(|_, w| unsafe {
            w.tx_clkm_div_x().bits(clkm_div.x as u16);
            w.tx_clkm_div_y().bits(clkm_div.y as u16);
            w.tx_clkm_div_yn1().bit(clkm_div.yn1);
            w.tx_clkm_div_z().bits(clkm_div.z as u16)
        });

        self.regs().tx_clkm_conf().modify(|_, w| unsafe {
            w.clk_en().set_bit();
            w.tx_clk_active().set_bit();
            // for now fixed at 160MHz
            w.tx_clk_sel().bits(property!("i2s.default_clock_source"));
            w.tx_clkm_div_num().bits(clock_settings.mclk_divider as u8)
        });

        self.regs().tx_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    #[cfg(not(any(i2s_clock_configured_by_pcr, i2s_clock_configured_by_hp_sys_clkrst)))]
    pub(crate) fn set_rx_clock(&self, clock_settings: I2sClockDividers) {
        let clkm_div = clock_settings.mclk_dividers();

        self.regs().rx_clkm_div_conf().modify(|_, w| unsafe {
            w.rx_clkm_div_x().bits(clkm_div.x as u16);
            w.rx_clkm_div_y().bits(clkm_div.y as u16);
            w.rx_clkm_div_yn1().bit(clkm_div.yn1);
            w.rx_clkm_div_z().bits(clkm_div.z as u16)
        });

        self.regs().rx_clkm_conf().modify(|_, w| unsafe {
            w.rx_clk_active().set_bit();
            // for now fixed at 160MHz
            w.rx_clk_sel().bits(property!("i2s.default_clock_source"));
            w.rx_clkm_div_num().bits(clock_settings.mclk_divider as u8);
            w.mclk_sel().bit(true)
        });

        self.regs().rx_conf().modify(|_, w| unsafe {
            w.rx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    #[cfg(i2s_clock_configured_by_pcr)]
    pub(crate) fn set_tx_clock(&self, clock_settings: I2sClockDividers) {
        // I2S clocks are configured via PCR
        use crate::peripherals::PCR;

        let clkm_div = clock_settings.mclk_dividers();
        let pcr = PCR::regs();

        // Pulse a temporary divider before applying the target coefficients to avoid
        // a hardware glitch where the clock divider applies twice on PCR chips.
        pcr.i2s_tx_clkm_conf()
            .modify(|_, w| unsafe { w.i2s_tx_clkm_div_num().bits(2) });
        pcr.i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_div_yn1().clear_bit();
            w.i2s_tx_clkm_div_y().bits(1);
            w.i2s_tx_clkm_div_z().bits(0);
            w.i2s_tx_clkm_div_x().bits(0)
        });

        pcr.i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_div_x().bits(clkm_div.x as u16);
            w.i2s_tx_clkm_div_y().bits(clkm_div.y as u16);
            w.i2s_tx_clkm_div_yn1().bit(clkm_div.yn1);
            w.i2s_tx_clkm_div_z().bits(clkm_div.z as u16)
        });

        pcr.i2s_tx_clkm_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_en().set_bit();
            // for now fixed at 160MHz for C6 and 96MHz for H2
            w.i2s_tx_clkm_sel()
                .bits(property!("i2s.default_clock_source"));
            w.i2s_tx_clkm_div_num()
                .bits(clock_settings.mclk_divider as u8)
        });

        self.regs().tx_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    #[cfg(i2s_clock_configured_by_pcr)]
    pub(crate) fn set_rx_clock(&self, clock_settings: I2sClockDividers) {
        // I2S clocks are configured via PCR
        use crate::peripherals::PCR;

        let clkm_div = clock_settings.mclk_dividers();
        let pcr = PCR::regs();

        // Pulse a temporary divider before applying the target coefficients to avoid
        // a hardware glitch where the clock divider applies twice on PCR chips.
        pcr.i2s_rx_clkm_conf()
            .modify(|_, w| unsafe { w.i2s_rx_clkm_div_num().bits(2) });
        pcr.i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_div_yn1().clear_bit();
            w.i2s_rx_clkm_div_y().bits(1);
            w.i2s_rx_clkm_div_z().bits(0);
            w.i2s_rx_clkm_div_x().bits(0)
        });

        pcr.i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_div_x().bits(clkm_div.x as u16);
            w.i2s_rx_clkm_div_y().bits(clkm_div.y as u16);
            w.i2s_rx_clkm_div_yn1().bit(clkm_div.yn1);
            w.i2s_rx_clkm_div_z().bits(clkm_div.z as u16)
        });

        pcr.i2s_rx_clkm_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_en().set_bit();
            // for now fixed at 160MHz for C6 and 96MHz for H2
            w.i2s_rx_clkm_sel()
                .bits(property!("i2s.default_clock_source"));
            w.i2s_rx_clkm_div_num()
                .bits(clock_settings.mclk_divider as u8);
            w.i2s_mclk_sel().bit(true)
        });

        self.regs().rx_conf().modify(|_, w| unsafe {
            w.rx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    #[cfg(i2s_clock_configured_by_hp_sys_clkrst)]
    pub(crate) fn set_tx_clock(&self, clock_settings: I2sClockDividers) {
        crate::i2s::hp_sys_clkrst::set_tx_clock(self.peripheral, &clock_settings);

        self.regs().tx_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    #[cfg(i2s_clock_configured_by_hp_sys_clkrst)]
    pub(crate) fn set_rx_clock(&self, clock_settings: I2sClockDividers) {
        crate::i2s::hp_sys_clkrst::set_rx_clock(self.peripheral, &clock_settings);

        self.regs().rx_conf().modify(|_, w| unsafe {
            w.rx_bck_div_num()
                .bits((clock_settings.bclk_divider - 1) as u8)
        });
    }

    pub(crate) fn update_tx(&self) {
        self.regs().tx_conf().modify(|_, w| w.tx_update().set_bit());
        while self.regs().tx_conf().read().tx_update().bit_is_set() {
            // wait
        }
    }

    pub(crate) fn update_rx(&self) {
        self.regs().rx_conf().modify(|_, w| w.rx_update().set_bit());
        while self.regs().rx_conf().read().rx_update().bit_is_set() {
            // wait
        }
    }

    pub(crate) fn configure(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate(self)?;

        match config {
            Config::Tdm(c) => {
                self.configure_tx(&c.tx_config)?;
                self.configure_rx(&c.rx_config)?;

                self.regs()
                    .tx_conf()
                    .modify(|_, w| w.sig_loopback().bit(c.signal_loopback));
                self.regs()
                    .rx_conf()
                    .modify(|_, w| w.rx_slave_mod().bit(c.signal_loopback));
            }
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            Config::Pdm(c) => {
                crate::i2s::pdm::configure_pdm(self, c)?;
            }
        }

        Ok(())
    }

    pub(crate) fn configure_tx(&self, config: &UnitConfig) -> Result<(), ConfigError> {
        let ws_width = config.calculate_ws_width()?;
        self.set_tx_clock(config.calculate_clock());

        self.regs().tx_conf1().modify(|_, w| unsafe {
            #[allow(clippy::useless_conversion)]
            w.tx_tdm_ws_width().bits((ws_width - 1).try_into().unwrap());
            w.tx_bits_mod().bits(config.data_format.data_bits() - 1);
            w.tx_tdm_chan_bits()
                .bits(config.data_format.channel_bits() - 1);
            w.tx_half_sample_bits()
                .bits((config.data_format.data_bits() * config.channels.count) / 2 - 1)
        });

        self.regs().tx_conf().modify(|_, w| unsafe {
            w.tx_mono().clear_bit();
            w.tx_mono_fst_vld().set_bit();
            w.tx_stop_en().set_bit();
            w.tx_chan_equal().bit(config.channels.fill.is_none());
            w.tx_tdm_en().set_bit();
            w.tx_pdm_en().clear_bit();
            w.tx_pcm_bypass().set_bit();
            w.tx_msb_shift().bit(config.msb_shift);
            w.tx_big_endian()
                .bit(config.endianness == Endianness::BigEndian);
            w.tx_bit_order().bit(config.bit_order == BitOrder::LsbFirst);
            w.tx_ws_idle_pol()
                .bit(config.ws_polarity == Polarity::ActiveHigh);
            w.tx_chan_mod().bits(0)
        });

        self.regs().tx_tdm_ctrl().modify(|_, w| unsafe {
            w.tx_tdm_tot_chan_num().bits(config.channels.count - 1);
            w.tx_tdm_chan0_en().bit(config.channels.mask.bit(0));
            w.tx_tdm_chan1_en().bit(config.channels.mask.bit(1));
            w.tx_tdm_chan2_en().bit(config.channels.mask.bit(2));
            w.tx_tdm_chan3_en().bit(config.channels.mask.bit(3));
            w.tx_tdm_chan4_en().bit(config.channels.mask.bit(4));
            w.tx_tdm_chan5_en().bit(config.channels.mask.bit(5));
            w.tx_tdm_chan6_en().bit(config.channels.mask.bit(6));
            w.tx_tdm_chan7_en().bit(config.channels.mask.bit(7));
            w.tx_tdm_chan8_en().bit(config.channels.mask.bit(8));
            w.tx_tdm_chan9_en().bit(config.channels.mask.bit(9));
            w.tx_tdm_chan10_en().bit(config.channels.mask.bit(10));
            w.tx_tdm_chan11_en().bit(config.channels.mask.bit(11));
            w.tx_tdm_chan12_en().bit(config.channels.mask.bit(12));
            w.tx_tdm_chan13_en().bit(config.channels.mask.bit(13));
            w.tx_tdm_chan14_en().bit(config.channels.mask.bit(14));
            w.tx_tdm_chan15_en().bit(config.channels.mask.bit(15));
            w.tx_tdm_skip_msk_en().clear_bit()
        });

        self.regs()
            .conf_sigle_data()
            .modify(|_, w| unsafe { w.single_data().bits(config.channels.fill.unwrap_or(0)) });

        Ok(())
    }

    pub(crate) fn configure_rx(&self, config: &UnitConfig) -> Result<(), ConfigError> {
        let ws_width = config.calculate_ws_width()?;
        self.set_rx_clock(config.calculate_clock());

        self.regs().rx_conf1().modify(|_, w| unsafe {
            #[allow(clippy::useless_conversion)]
            w.rx_tdm_ws_width().bits((ws_width - 1).try_into().unwrap());
            w.rx_bits_mod().bits(config.data_format.data_bits() - 1);
            w.rx_tdm_chan_bits()
                .bits(config.data_format.channel_bits() - 1);
            w.rx_half_sample_bits()
                .bits((config.data_format.data_bits() * config.channels.count) / 2 - 1)
        });

        self.regs().rx_conf().modify(|_, w| unsafe {
            w.rx_mono().clear_bit();
            w.rx_mono_fst_vld().set_bit();
            w.rx_stop_mode().bits(2);
            w.rx_tdm_en().set_bit();
            w.rx_pdm_en().clear_bit();
            w.rx_pcm_bypass().set_bit();
            w.rx_msb_shift().bit(config.msb_shift);
            w.rx_big_endian()
                .bit(config.endianness == Endianness::BigEndian);
            w.rx_bit_order().bit(config.bit_order == BitOrder::LsbFirst);
            w.rx_ws_idle_pol()
                .bit(config.ws_polarity == Polarity::ActiveHigh)
        });

        self.regs().rx_tdm_ctrl().modify(|_, w| unsafe {
            w.rx_tdm_tot_chan_num().bits(config.channels.count - 1);
            w.rx_tdm_pdm_chan0_en().bit(config.channels.mask.bit(0));
            w.rx_tdm_pdm_chan1_en().bit(config.channels.mask.bit(1));
            w.rx_tdm_pdm_chan2_en().bit(config.channels.mask.bit(2));
            w.rx_tdm_pdm_chan3_en().bit(config.channels.mask.bit(3));
            w.rx_tdm_pdm_chan4_en().bit(config.channels.mask.bit(4));
            w.rx_tdm_pdm_chan5_en().bit(config.channels.mask.bit(5));
            w.rx_tdm_pdm_chan6_en().bit(config.channels.mask.bit(6));
            w.rx_tdm_pdm_chan7_en().bit(config.channels.mask.bit(7));
            w.rx_tdm_chan8_en().bit(config.channels.mask.bit(8));
            w.rx_tdm_chan9_en().bit(config.channels.mask.bit(9));
            w.rx_tdm_chan10_en().bit(config.channels.mask.bit(10));
            w.rx_tdm_chan11_en().bit(config.channels.mask.bit(11));
            w.rx_tdm_chan12_en().bit(config.channels.mask.bit(12));
            w.rx_tdm_chan13_en().bit(config.channels.mask.bit(13));
            w.rx_tdm_chan14_en().bit(config.channels.mask.bit(14));
            w.rx_tdm_chan15_en().bit(config.channels.mask.bit(15))
        });

        Ok(())
    }

    pub(crate) fn set_master(&self) {
        self.regs()
            .tx_conf()
            .modify(|_, w| w.tx_slave_mod().clear_bit());
        self.regs()
            .rx_conf()
            .modify(|_, w| w.rx_slave_mod().clear_bit());
    }

    pub(crate) fn reset_tx(&self) {
        // I2S v2/v3: reset fields are write-to-trigger (WT); writing 0 has no effect.
        self.regs().tx_conf().modify(|_, w| {
            w.tx_reset().set_bit();
            w.tx_fifo_reset().set_bit()
        });

        self.regs().int_clr().write(|w| {
            w.tx_done().clear_bit_by_one();
            w.tx_hung().clear_bit_by_one()
        });
    }

    pub(crate) fn tx_start(&self) {
        self.regs().tx_conf().modify(|_, w| w.tx_start().set_bit());
    }

    pub(crate) fn tx_stop(&self) {
        self.regs()
            .tx_conf()
            .modify(|_, w| w.tx_start().clear_bit());
    }

    pub(crate) fn is_tx_done(&self) -> bool {
        self.regs().state().read().tx_idle().bit_is_set()
    }

    pub(crate) fn reset_rx(&self) {
        self.regs()
            .rx_conf()
            .modify(|_, w| w.rx_start().clear_bit());

        // I2S v2/v3: reset fields are write-to-trigger (WT); writing 0 has no effect.
        self.regs().rx_conf().modify(|_, w| {
            w.rx_reset().set_bit();
            w.rx_fifo_reset().set_bit()
        });

        self.regs().int_clr().write(|w| {
            w.rx_done().clear_bit_by_one();
            w.rx_hung().clear_bit_by_one()
        });
    }

    pub(crate) fn rx_start(&self, len: usize) {
        let len = len - 1;

        self.regs()
            .rxeof_num()
            .write(|w| unsafe { w.rx_eof_num().bits(len as u16) });
        // Sync configuration into the I2S clock domain before starting RX.
        self.update_rx();
        self.regs().rx_conf().modify(|_, w| w.rx_start().set_bit());
    }

    pub(crate) fn rx_stop(&self) {
        self.regs()
            .rx_conf()
            .modify(|_, w| w.rx_start().clear_bit());
    }

    pub(crate) fn is_rx_done(&self) -> bool {
        self.regs().int_raw().read().rx_done().bit_is_set()
    }
}
