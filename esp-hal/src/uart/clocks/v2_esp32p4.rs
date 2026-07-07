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
        let regs = HP_SYS_CLKRST::regs();
        match self {
            UartInstance::Uart0 => {
                regs.peri_clk_ctrl110()
                    .modify(|_, w| unsafe { w.uart0_clk_src_sel().bits(sel) });
            }
            UartInstance::Uart1 => {
                regs.peri_clk_ctrl111()
                    .modify(|_, w| unsafe { w.uart1_clk_src_sel().bits(sel) });
            }
            UartInstance::Uart2 => {
                regs.peri_clk_ctrl112()
                    .modify(|_, w| unsafe { w.uart2_clk_src_sel().bits(sel) });
            }
            UartInstance::Uart3 => {
                regs.peri_clk_ctrl113()
                    .modify(|_, w| unsafe { w.uart3_clk_src_sel().bits(sel) });
            }
            UartInstance::Uart4 => {
                regs.peri_clk_ctrl114()
                    .modify(|_, w| unsafe { w.uart4_clk_src_sel().bits(sel) });
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
