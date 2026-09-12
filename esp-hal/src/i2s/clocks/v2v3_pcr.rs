use super::MclkFraction;
use crate::{
    clock::ll::{ClockTree, I2sClkConfig, I2sClkSclk, I2sInstance, I2sMclkOutConfig},
    peripherals::PCR,
};

fn clk_sel(sclk: I2sClkSclk) -> u8 {
    cfg_select! {
        any(esp32c5, esp32c6) => match sclk {
            I2sClkSclk::Xtal => 0,
            I2sClkSclk::PllF240m => 1,
            I2sClkSclk::PllF160m => 2,
        },
        esp32c61 => match sclk {
            I2sClkSclk::Xtal => 0,
            I2sClkSclk::PllF120m => 1,
            I2sClkSclk::PllF160m => 2,
        },
        esp32h2 => match sclk {
            I2sClkSclk::Xtal => 0,
            I2sClkSclk::PllF96m => 1,
            I2sClkSclk::PllF64m => 2,
        },
    }
}

impl I2sInstance {
    // I2S_TX_CLK

    pub(crate) fn enable_tx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        PCR::regs()
            .i2s_tx_clkm_conf()
            .modify(|_, w| w.i2s_tx_clkm_en().bit(en));
    }

    pub(crate) fn configure_tx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());
        let pcr = PCR::regs();

        // The divider applies twice, unless the coefficients are set in this sequence, with a
        // small division before the target one.
        pcr.i2s_tx_clkm_conf()
            .modify(|_, w| unsafe { w.i2s_tx_clkm_div_num().bits(2) });
        pcr.i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_div_yn1().clear_bit();
            w.i2s_tx_clkm_div_y().bits(1);
            w.i2s_tx_clkm_div_z().bits(0);
            w.i2s_tx_clkm_div_x().bits(0)
        });

        pcr.i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_div_yn1().bit(fraction.yn1);
            w.i2s_tx_clkm_div_z().bits(fraction.z);
            w.i2s_tx_clkm_div_y().bits(fraction.y);
            w.i2s_tx_clkm_div_x().bits(fraction.x)
        });
        pcr.i2s_tx_clkm_conf().modify(|_, w| unsafe {
            w.i2s_tx_clkm_sel().bits(clk_sel(new_config.sclk()));
            w.i2s_tx_clkm_div_num().bits(new_config.div_num() as u8)
        });
    }

    // I2S_RX_CLK

    pub(crate) fn enable_rx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        PCR::regs()
            .i2s_rx_clkm_conf()
            .modify(|_, w| w.i2s_rx_clkm_en().bit(en));
    }

    pub(crate) fn configure_rx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());
        let pcr = PCR::regs();

        pcr.i2s_rx_clkm_conf()
            .modify(|_, w| unsafe { w.i2s_rx_clkm_div_num().bits(2) });
        pcr.i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_div_yn1().clear_bit();
            w.i2s_rx_clkm_div_y().bits(1);
            w.i2s_rx_clkm_div_z().bits(0);
            w.i2s_rx_clkm_div_x().bits(0)
        });

        pcr.i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_div_yn1().bit(fraction.yn1);
            w.i2s_rx_clkm_div_z().bits(fraction.z);
            w.i2s_rx_clkm_div_y().bits(fraction.y);
            w.i2s_rx_clkm_div_x().bits(fraction.x)
        });
        pcr.i2s_rx_clkm_conf().modify(|_, w| unsafe {
            w.i2s_rx_clkm_sel().bits(clk_sel(new_config.sclk()));
            w.i2s_rx_clkm_div_num().bits(new_config.div_num() as u8)
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
        PCR::regs().i2s_rx_clkm_conf().modify(|_, w| {
            w.i2s_mclk_sel()
                .bit(matches!(new_config, I2sMclkOutConfig::Rx))
        });
    }
}
