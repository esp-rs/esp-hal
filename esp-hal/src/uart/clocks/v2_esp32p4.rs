use crate::{
    clock::ll::{
        ClockTree,
        UartBaudRateGeneratorConfig,
        UartFunctionClockConfig,
        UartFunctionClockSclk,
        UartInstance,
    },
    peripherals::HP_SYS_CLKRST,
};

// Per-instance clock impl for UART (called on UartInstance enum)
impl UartInstance {
    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let regs = HP_SYS_CLKRST::regs();
        match self {
            // ROM boot/download path seems to expect UART0 clock alive. If Drop turns uart0_clk_en
            // off, next probe soft-reset hits ROM with UART0 gated - chip freezes.
            // Other UARTs have no such ROM role.
            UartInstance::Uart0 => {
                regs.peri_clk_ctrl110()
                    .modify(|_, w| w.uart0_clk_en().bit(en));
            }
            UartInstance::Uart1 => {
                regs.peri_clk_ctrl111()
                    .modify(|_, w| w.uart1_clk_en().bit(en));
            }
            UartInstance::Uart2 => {
                regs.peri_clk_ctrl112()
                    .modify(|_, w| w.uart2_clk_en().bit(en));
            }
            UartInstance::Uart3 => {
                regs.peri_clk_ctrl113()
                    .modify(|_, w| w.uart3_clk_en().bit(en));
            }
            UartInstance::Uart4 => {
                regs.peri_clk_ctrl114()
                    .modify(|_, w| w.uart4_clk_en().bit(en));
            }
        }
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        let sel = match new_config.sclk() {
            UartFunctionClockSclk::Xtal => 0,
            UartFunctionClockSclk::PllF80m => 2,
            UartFunctionClockSclk::RcFast => 1,
        };
        let div = new_config.div_num() as u8;
        let regs = HP_SYS_CLKRST::regs();
        match self {
            UartInstance::Uart0 => {
                regs.peri_clk_ctrl110()
                    .modify(|_, w| unsafe { w.uart0_clk_src_sel().bits(sel) });
                regs.peri_clk_ctrl111()
                    .modify(|_, w| unsafe { w.uart0_sclk_div_num().bits(div) });
            }
            UartInstance::Uart1 => {
                regs.peri_clk_ctrl111()
                    .modify(|_, w| unsafe { w.uart1_clk_src_sel().bits(sel) });
                regs.peri_clk_ctrl112()
                    .modify(|_, w| unsafe { w.uart1_sclk_div_num().bits(div) });
            }
            UartInstance::Uart2 => {
                regs.peri_clk_ctrl112()
                    .modify(|_, w| unsafe { w.uart2_clk_src_sel().bits(sel) });
                regs.peri_clk_ctrl113()
                    .modify(|_, w| unsafe { w.uart2_sclk_div_num().bits(div) });
            }
            UartInstance::Uart3 => {
                regs.peri_clk_ctrl113()
                    .modify(|_, w| unsafe { w.uart3_clk_src_sel().bits(sel) });
                regs.peri_clk_ctrl114()
                    .modify(|_, w| unsafe { w.uart3_sclk_div_num().bits(div) });
            }
            UartInstance::Uart4 => {
                regs.peri_clk_ctrl114()
                    .modify(|_, w| unsafe { w.uart4_clk_src_sel().bits(sel) });
                regs.peri_clk_ctrl115()
                    .modify(|_, w| unsafe { w.uart4_sclk_div_num().bits(div) });
            }
        }
    }

    pub(crate) fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Baud rate generator is always on when UART is enabled
    }

    pub(crate) fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        new_config: UartBaudRateGeneratorConfig,
    ) {
        let regs = match self {
            UartInstance::Uart0 => crate::peripherals::UART0::regs(),
            UartInstance::Uart1 => crate::peripherals::UART1::regs(),
            UartInstance::Uart2 => crate::peripherals::UART2::regs(),
            UartInstance::Uart3 => crate::peripherals::UART3::regs(),
            UartInstance::Uart4 => crate::peripherals::UART4::regs(),
        };
        regs.clkdiv().write(|w| unsafe {
            w.clkdiv().bits(new_config.integral() as _);
            w.clkdiv_frag().bits(new_config.fractional() as _)
        });
    }
}
