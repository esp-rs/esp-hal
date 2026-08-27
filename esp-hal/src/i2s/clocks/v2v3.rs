use super::MclkFraction;
use crate::clock::ll::{ClockTree, I2sClkConfig, I2sClkSclk, I2sInstance, I2sMclkOutConfig};

fn clk_sel(sclk: I2sClkSclk) -> u8 {
    cfg_select! {
        esp32c3 => match sclk {
            I2sClkSclk::Xtal => 0,
            I2sClkSclk::Pll160m => 2,
        },
        esp32s3 => match sclk {
            I2sClkSclk::Xtal => 0,
            I2sClkSclk::PllD2 => 1,
            I2sClkSclk::Pll160m => 2,
        },
    }
}

impl I2sInstance {
    fn regs(self) -> &'static crate::pac::i2s0::RegisterBlock {
        match self {
            Self::I2s0 => crate::peripherals::I2S0::regs(),
            #[cfg(soc_has_i2s1)]
            Self::I2s1 => unsafe {
                &*crate::peripherals::I2S1::PTR.cast::<crate::pac::i2s0::RegisterBlock>()
            },
        }
    }

    // I2S_TX_CLK

    pub(crate) fn enable_tx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        self.regs().tx_clkm_conf().modify(|_, w| {
            w.clk_en().bit(en);
            w.tx_clk_active().bit(en)
        });
    }

    pub(crate) fn configure_tx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());

        // The divider applies twice, unless the coefficients are set in this sequence, with a
        // small division before the target one.
        self.regs()
            .tx_clkm_conf()
            .modify(|_, w| unsafe { w.tx_clkm_div_num().bits(2) });
        self.regs().tx_clkm_div_conf().modify(|_, w| unsafe {
            w.tx_clkm_div_yn1().clear_bit();
            w.tx_clkm_div_y().bits(1);
            w.tx_clkm_div_z().bits(0);
            w.tx_clkm_div_x().bits(0)
        });

        self.regs().tx_clkm_div_conf().modify(|_, w| unsafe {
            w.tx_clkm_div_yn1().bit(fraction.yn1);
            w.tx_clkm_div_z().bits(fraction.z);
            w.tx_clkm_div_y().bits(fraction.y);
            w.tx_clkm_div_x().bits(fraction.x)
        });
        self.regs().tx_clkm_conf().modify(|_, w| unsafe {
            w.tx_clk_sel().bits(clk_sel(new_config.sclk()));
            w.tx_clkm_div_num().bits(new_config.div_num() as u8)
        });
    }

    // I2S_RX_CLK

    pub(crate) fn enable_rx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        self.regs()
            .rx_clkm_conf()
            .modify(|_, w| w.rx_clk_active().bit(en));
    }

    pub(crate) fn configure_rx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());

        self.regs()
            .rx_clkm_conf()
            .modify(|_, w| unsafe { w.rx_clkm_div_num().bits(2) });
        self.regs().rx_clkm_div_conf().modify(|_, w| unsafe {
            w.rx_clkm_div_yn1().clear_bit();
            w.rx_clkm_div_y().bits(1);
            w.rx_clkm_div_z().bits(0);
            w.rx_clkm_div_x().bits(0)
        });

        self.regs().rx_clkm_div_conf().modify(|_, w| unsafe {
            w.rx_clkm_div_yn1().bit(fraction.yn1);
            w.rx_clkm_div_z().bits(fraction.z);
            w.rx_clkm_div_y().bits(fraction.y);
            w.rx_clkm_div_x().bits(fraction.x)
        });
        self.regs().rx_clkm_conf().modify(|_, w| unsafe {
            w.rx_clk_sel().bits(clk_sel(new_config.sclk()));
            w.rx_clkm_div_num().bits(new_config.div_num() as u8)
        });
    }

    // I2S_MCLK_OUT

    pub(crate) fn enable_mclk_out_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // The pad is driven as long as the selected module clock runs.
    }

    pub(crate) fn configure_mclk_out_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sMclkOutConfig>,
        new_config: I2sMclkOutConfig,
    ) {
        self.regs()
            .rx_clkm_conf()
            .modify(|_, w| w.mclk_sel().bit(matches!(new_config, I2sMclkOutConfig::Rx)));
    }
}
