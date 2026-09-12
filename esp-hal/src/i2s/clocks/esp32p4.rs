use super::MclkFraction;
use crate::{
    clock::ll::{ClockTree, I2sClkConfig, I2sClkSclk, I2sInstance, I2sMclkOutConfig},
    peripherals::HP_SYS_CLKRST,
};

fn clk_sel(sclk: I2sClkSclk) -> u8 {
    match sclk {
        I2sClkSclk::Xtal => 0,
        I2sClkSclk::PllF160m => 3,
    }
}

/// Applies a module clock configuration to one instance and direction.
///
/// The fields of one divider are spread over two `PERI_CLK_CTRLn` registers, and the split
/// differs per instance, so every field is named with the register that holds it.
///
/// The divider applies twice, unless the coefficients are set in this sequence, with a small
/// division before the target one.
macro_rules! configure_clk {
    (
        $config:ident, $fraction:ident,
        div_n = $n_reg:ident :: $n:ident,
        div_x = $x_reg:ident :: $x:ident,
        div_y = $y_reg:ident :: $y:ident,
        div_z = $z_reg:ident :: $z:ident,
        div_yn1 = $yn1_reg:ident :: $yn1:ident,
        clk_src_sel = $sel_reg:ident :: $sel:ident,
    ) => {{
        let clkrst = HP_SYS_CLKRST::regs();

        clkrst
            .$sel_reg()
            .modify(|_, w| unsafe { w.$sel().bits(clk_sel($config.sclk())) });

        clkrst.$n_reg().modify(|_, w| unsafe { w.$n().bits(2) });
        clkrst.$yn1_reg().modify(|_, w| w.$yn1().clear_bit());
        clkrst.$y_reg().modify(|_, w| unsafe { w.$y().bits(1) });
        clkrst.$z_reg().modify(|_, w| unsafe { w.$z().bits(0) });
        clkrst.$x_reg().modify(|_, w| unsafe { w.$x().bits(0) });

        clkrst.$yn1_reg().modify(|_, w| w.$yn1().bit($fraction.yn1));
        clkrst
            .$z_reg()
            .modify(|_, w| unsafe { w.$z().bits($fraction.z) });
        clkrst
            .$y_reg()
            .modify(|_, w| unsafe { w.$y().bits($fraction.y) });
        clkrst
            .$x_reg()
            .modify(|_, w| unsafe { w.$x().bits($fraction.x) });

        clkrst
            .$n_reg()
            .modify(|_, w| unsafe { w.$n().bits($config.div_num() as u8) });
    }};
}

impl I2sInstance {
    // I2S_TX_CLK

    pub(crate) fn enable_tx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst
                    .peri_clk_ctrl13()
                    .modify(|_, w| w.i2s0_tx_clk_en().bit(en));
            }
            Self::I2s1 => {
                clkrst
                    .peri_clk_ctrl15()
                    .modify(|_, w| w.i2s1_tx_clk_en().bit(en));
            }
            Self::I2s2 => {
                clkrst
                    .peri_clk_ctrl18()
                    .modify(|_, w| w.i2s2_tx_clk_en().bit(en));
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
                div_n = peri_clk_ctrl13::i2s0_tx_div_n,
                div_x = peri_clk_ctrl13::i2s0_tx_div_x,
                div_y = peri_clk_ctrl14::i2s0_tx_div_y,
                div_z = peri_clk_ctrl14::i2s0_tx_div_z,
                div_yn1 = peri_clk_ctrl14::i2s0_tx_div_yn1,
                clk_src_sel = peri_clk_ctrl13::i2s0_tx_clk_src_sel,
            ),
            Self::I2s1 => configure_clk!(
                new_config,
                fraction,
                div_n = peri_clk_ctrl16::i2s1_tx_div_n,
                div_x = peri_clk_ctrl16::i2s1_tx_div_x,
                div_y = peri_clk_ctrl16::i2s1_tx_div_y,
                div_z = peri_clk_ctrl17::i2s1_tx_div_z,
                div_yn1 = peri_clk_ctrl17::i2s1_tx_div_yn1,
                clk_src_sel = peri_clk_ctrl15::i2s1_tx_clk_src_sel,
            ),
            Self::I2s2 => configure_clk!(
                new_config,
                fraction,
                div_n = peri_clk_ctrl18::i2s2_tx_div_n,
                div_x = peri_clk_ctrl19::i2s2_tx_div_x,
                div_y = peri_clk_ctrl19::i2s2_tx_div_y,
                div_z = peri_clk_ctrl19::i2s2_tx_div_z,
                div_yn1 = peri_clk_ctrl19::i2s2_tx_div_yn1,
                clk_src_sel = peri_clk_ctrl18::i2s2_tx_clk_src_sel,
            ),
        }
    }

    // I2S_RX_CLK

    pub(crate) fn enable_rx_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst
                    .peri_clk_ctrl11()
                    .modify(|_, w| w.i2s0_rx_clk_en().bit(en));
            }
            Self::I2s1 => {
                clkrst
                    .peri_clk_ctrl14()
                    .modify(|_, w| w.i2s1_rx_clk_en().bit(en));
            }
            Self::I2s2 => {
                clkrst
                    .peri_clk_ctrl17()
                    .modify(|_, w| w.i2s2_rx_clk_en().bit(en));
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
                div_n = peri_clk_ctrl12::i2s0_rx_div_n,
                div_x = peri_clk_ctrl12::i2s0_rx_div_x,
                div_y = peri_clk_ctrl12::i2s0_rx_div_y,
                div_z = peri_clk_ctrl13::i2s0_rx_div_z,
                div_yn1 = peri_clk_ctrl13::i2s0_rx_div_yn1,
                clk_src_sel = peri_clk_ctrl11::i2s0_rx_clk_src_sel,
            ),
            Self::I2s1 => configure_clk!(
                new_config,
                fraction,
                div_n = peri_clk_ctrl14::i2s1_rx_div_n,
                div_x = peri_clk_ctrl15::i2s1_rx_div_x,
                div_y = peri_clk_ctrl15::i2s1_rx_div_y,
                div_z = peri_clk_ctrl15::i2s1_rx_div_z,
                div_yn1 = peri_clk_ctrl15::i2s1_rx_div_yn1,
                clk_src_sel = peri_clk_ctrl14::i2s1_rx_clk_src_sel,
            ),
            Self::I2s2 => configure_clk!(
                new_config,
                fraction,
                div_n = peri_clk_ctrl17::i2s2_rx_div_n,
                div_x = peri_clk_ctrl17::i2s2_rx_div_x,
                div_y = peri_clk_ctrl18::i2s2_rx_div_y,
                div_z = peri_clk_ctrl18::i2s2_rx_div_z,
                div_yn1 = peri_clk_ctrl18::i2s2_rx_div_yn1,
                clk_src_sel = peri_clk_ctrl17::i2s2_rx_clk_src_sel,
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
        let tx = matches!(new_config, I2sMclkOutConfig::Tx);
        let clkrst = HP_SYS_CLKRST::regs();
        match self {
            Self::I2s0 => {
                clkrst
                    .peri_clk_ctrl14()
                    .modify(|_, w| w.i2s0_mst_clk_sel().bit(tx));
            }
            Self::I2s1 => {
                clkrst
                    .peri_clk_ctrl17()
                    .modify(|_, w| w.i2s1_mst_clk_sel().bit(tx));
            }
            Self::I2s2 => {
                clkrst
                    .peri_clk_ctrl19()
                    .modify(|_, w| w.i2s2_mst_clk_sel().bit(tx));
            }
        }
    }
}
