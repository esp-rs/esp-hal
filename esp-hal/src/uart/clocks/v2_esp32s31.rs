use crate::{
    clock::ll::{
        ClockTree,
        UartBaudRateGeneratorConfig,
        UartFunctionClockConfig,
        UartFunctionClockSclk,
        UartInstance,
        UartMemClockConfig,
    },
    peripherals::{HP_SYS, HP_SYS_CLKRST},
};

impl UartInstance {
    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        HP_SYS_CLKRST::regs()
            .uart_ctrl0(self as usize)
            .modify(|_, w| w.clk_en().bit(en));
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        let source = match new_config.sclk() {
            UartFunctionClockSclk::Xtal => 0,
            UartFunctionClockSclk::RcFast => 1,
            UartFunctionClockSclk::PllF80m => 2,
        };
        let divider = new_config.div_num() as u8;
        HP_SYS_CLKRST::regs()
            .uart_ctrl0(self as usize)
            .modify(|_, w| unsafe {
                w.clk_src_sel().bits(source);
                w.sclk_div_num().bits(divider);
                w.sclk_div_numerator().bits(0);
                w.sclk_div_denominator().bits(0)
            });
    }

    pub(crate) fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // The baud-rate generator is enabled by the UART function clock.
    }

    pub(crate) fn enable_mem_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        if en {
            // UART1-3 memories are powered down after reset. Hand control back to
            // the PMU and request the memory to remain powered while active.
            macro_rules! power_up_memory {
                ($register:expr) => {
                    $register.modify(|_, w| unsafe {
                        w.mem_lp_mode()
                            .bits(2)
                            .mem_lp_en()
                            .clear_bit()
                            .mem_force_ctrl()
                            .clear_bit()
                    })
                };
            }

            let regs = HP_SYS::regs();
            match self {
                UartInstance::Uart0 => power_up_memory!(regs.uart0_mem_lp_ctrl()),
                UartInstance::Uart1 => power_up_memory!(regs.uart1_mem_lp_ctrl()),
                UartInstance::Uart2 => power_up_memory!(regs.uart2_mem_lp_ctrl()),
                UartInstance::Uart3 => power_up_memory!(regs.uart3_mem_lp_ctrl()),
            };
        }

        let regs = match self {
            UartInstance::Uart0 => crate::peripherals::UART0::regs(),
            UartInstance::Uart1 => crate::peripherals::UART1::regs(),
            UartInstance::Uart2 => crate::peripherals::UART2::regs(),
            UartInstance::Uart3 => crate::peripherals::UART3::regs(),
        };
        regs.conf0().modify(|_, w| w.mem_clk_en().bit(en));
    }

    pub(crate) fn configure_mem_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartMemClockConfig>,
        _new_config: UartMemClockConfig,
    ) {
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
        };
        regs.clkdiv().write(|w| unsafe {
            w.clkdiv().bits(new_config.integral() as u16);
            w.frag().bits(new_config.fractional() as u8)
        });
    }
}
