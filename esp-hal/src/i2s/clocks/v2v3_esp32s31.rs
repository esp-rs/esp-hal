use super::MclkFraction;
use crate::{
    clock::ll::{ClockTree, I2sClkConfig, I2sClkSclk, I2sInstance, I2sMclkOutConfig},
    peripherals::HP_SYS_CLKRST,
};

fn clk_sel(sclk: I2sClkSclk) -> u8 {
    match sclk {
        I2sClkSclk::Xtal => 0,
    }
}

/// Applies a module clock configuration to one instance and direction.
///
/// Each instance and direction has a `ctrl0` register with the source selector and the integer
/// part of the divider, and a `div_ctrl0` register with the fraction.
///
/// The divider applies twice, unless the coefficients are set in this sequence, with a small
/// division before the target one.
macro_rules! configure_clk {
    (
        $config:ident, $fraction:ident,
        ctrl = $ctrl_reg:ident,
        div_ctrl = $div_ctrl_reg:ident,
        div_n = $n:ident,
        div_x = $x:ident,
        div_y = $y:ident,
        div_z = $z:ident,
        div_yn1 = $yn1:ident,
        clk_src_sel = $sel:ident,
    ) => {{
        let clkrst = HP_SYS_CLKRST::regs();

        clkrst.$ctrl_reg().modify(|_, w| unsafe { w.$n().bits(2) });
        clkrst.$div_ctrl_reg().modify(|_, w| {
            w.$yn1().clear_bit();
            unsafe {
                w.$y().bits(1);
                w.$z().bits(0);
                w.$x().bits(0)
            }
        });

        clkrst.$div_ctrl_reg().modify(|_, w| {
            w.$yn1().bit($fraction.yn1);
            unsafe {
                w.$z().bits($fraction.z);
                w.$y().bits($fraction.y);
                w.$x().bits($fraction.x)
            }
        });

        clkrst.$ctrl_reg().modify(|_, w| unsafe {
            w.$n().bits($config.div_num() as u8);
            w.$sel().bits(clk_sel($config.sclk()))
        });
    }};
}

impl I2sInstance {
    // I2S_TX_CLK

    pub(crate) fn enable_tx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst.i2s0_tx_ctrl0().modify(|_, w| w.tx_clk_en().bit(en));
            }
            Self::I2s1 => {
                clkrst.i2s1_tx_ctrl0().modify(|_, w| w.tx_clk_en().bit(en));
            }
        }
    }

    pub(crate) fn configure_tx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());

        match self {
            Self::I2s0 => configure_clk!(
                new_config,
                fraction,
                ctrl = i2s0_tx_ctrl0,
                div_ctrl = i2s0_tx_div_ctrl0,
                div_n = tx_div_n,
                div_x = tx_div_x,
                div_y = tx_div_y,
                div_z = tx_div_z,
                div_yn1 = tx_div_yn1,
                clk_src_sel = tx_clk_src_sel,
            ),
            Self::I2s1 => configure_clk!(
                new_config,
                fraction,
                ctrl = i2s1_tx_ctrl0,
                div_ctrl = i2s1_tx_div_ctrl0,
                div_n = tx_div_n,
                div_x = tx_div_x,
                div_y = tx_div_y,
                div_z = tx_div_z,
                div_yn1 = tx_div_yn1,
                clk_src_sel = tx_clk_src_sel,
            ),
        }
    }

    // I2S_RX_CLK

    pub(crate) fn enable_rx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst.i2s0_rx_ctrl0().modify(|_, w| w.rx_clk_en().bit(en));
            }
            Self::I2s1 => {
                clkrst.i2s1_rx_ctrl0().modify(|_, w| w.rx_clk_en().bit(en));
            }
        }
    }

    pub(crate) fn configure_rx_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sClkConfig>,
        new_config: I2sClkConfig,
    ) {
        let fraction = MclkFraction::new(new_config.div_a(), new_config.div_b());

        match self {
            Self::I2s0 => configure_clk!(
                new_config,
                fraction,
                ctrl = i2s0_rx_ctrl0,
                div_ctrl = i2s0_rx_div_ctrl0,
                div_n = rx_div_n,
                div_x = rx_div_x,
                div_y = rx_div_y,
                div_z = rx_div_z,
                div_yn1 = rx_div_yn1,
                clk_src_sel = rx_clk_src_sel,
            ),
            Self::I2s1 => configure_clk!(
                new_config,
                fraction,
                ctrl = i2s1_rx_ctrl0,
                div_ctrl = i2s1_rx_div_ctrl0,
                div_n = rx_div_n,
                div_x = rx_div_x,
                div_y = rx_div_y,
                div_z = rx_div_z,
                div_yn1 = rx_div_yn1,
                clk_src_sel = rx_clk_src_sel,
            ),
        }
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
        // `mst_clk_sel` lives on the RX control register of the instance.
        let tx = matches!(new_config, I2sMclkOutConfig::Tx);
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst
                    .i2s0_rx_ctrl0()
                    .modify(|_, w| w.mst_clk_sel().bit(tx));
            }
            Self::I2s1 => {
                clkrst
                    .i2s1_rx_ctrl0()
                    .modify(|_, w| w.mst_clk_sel().bit(tx));
            }
        }
    }
}
