use crate::clock::ll::{ClockTree, I2sInstance, I2sMclkConfig, I2sMclkSclk};

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

    // I2S_MCLK

    pub(crate) fn enable_mclk_impl(self, _clocks: &mut ClockTree, en: bool) {
        self.regs().clkm_conf().modify(|_, w| w.clk_en().bit(en));
    }

    pub(crate) fn configure_mclk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2sMclkConfig>,
        new_config: I2sMclkConfig,
    ) {
        self.regs().clkm_conf().modify(|_, w| {
            cfg_select! {
                esp32 => {
                    // The ESP32 has no source selector. The peripheral uses PLL_F160M_CLK,
                    // unless the APLL is routed to it.
                    w.clka_ena()
                        .bit(matches!(new_config.sclk(), I2sMclkSclk::Apll))
                }
                esp32s2 => unsafe {
                    w.clk_sel().bits(match new_config.sclk() {
                        I2sMclkSclk::Apll => 1,
                        I2sMclkSclk::PllF160m => 2,
                    })
                },
            }
        });

        self.regs().clkm_conf().modify(|_, w| unsafe {
            w.clkm_div_num().bits(new_config.div_num() as u8);
            w.clkm_div_a().bits(new_config.div_a() as u8);
            w.clkm_div_b().bits(new_config.div_b() as u8)
        });
    }
}
