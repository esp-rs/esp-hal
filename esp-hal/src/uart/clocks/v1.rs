use crate::clock::ll::{
    ClockTree,
    UartBaudRateGeneratorConfig,
    UartFunctionClockConfig,
    UartFunctionClockSclk,
    UartInstance,
};

impl UartInstance {
    /// Returns whether the instance is clocked and out of reset, so that its transmitter can
    /// drain.
    #[cfg(sleep_driver_supported)]
    pub(crate) fn is_enabled(self) -> bool {
        let system = crate::peripherals::SYSTEM::regs();
        cfg_select! {
            esp32 => {
                let clk = system.perip_clk_en().read();
                let rst = system.perip_rst_en().read();
            }
            _ => {
                let clk = system.perip_clk_en0().read();
                let rst = system.perip_rst_en0().read();
            }
        }
        let bus = match self {
            #[cfg(soc_has_uart0)]
            Self::Uart0 => clk.uart_clk_en().bit() && !rst.uart_rst().bit(),
            #[cfg(soc_has_uart1)]
            Self::Uart1 => clk.uart1_clk_en().bit() && !rst.uart1_rst().bit(),
            #[cfg(soc_has_uart2)]
            Self::Uart2 => cfg_select! {
                esp32s3 => {
                    system.perip_clk_en1().read().uart2_clk_en().bit()
                        && !system.perip_rst_en1().read().uart2_rst().bit()
                }
                _ => clk.uart2_clk_en().bit() && !rst.uart2_rst().bit(),
            },
        };

        #[cfg(uart_has_sclk_divider)]
        let bus = bus && {
            let regs = match self {
                #[cfg(soc_has_uart0)]
                Self::Uart0 => crate::peripherals::UART0::regs(),
                #[cfg(soc_has_uart1)]
                Self::Uart1 => crate::peripherals::UART1::regs(),
                #[cfg(soc_has_uart2)]
                Self::Uart2 => crate::peripherals::UART2::regs(),
            };
            regs.clk_conf().read().sclk_en().bit()
        };

        bus
    }

    // UART_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        #[cfg(uart_has_sclk_divider)]
        {
            let regs = match self {
                #[cfg(soc_has_uart0)]
                Self::Uart0 => crate::peripherals::UART0::regs(),
                #[cfg(soc_has_uart1)]
                Self::Uart1 => crate::peripherals::UART1::regs(),
                #[cfg(soc_has_uart2)]
                Self::Uart2 => crate::peripherals::UART2::regs(),
            };

            regs.clk_conf().modify(|_, w| w.sclk_en().bit(_en));
        }
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        let regs = match self {
            #[cfg(soc_has_uart0)]
            Self::Uart0 => crate::peripherals::UART0::regs(),
            #[cfg(soc_has_uart1)]
            Self::Uart1 => crate::peripherals::UART1::regs(),
            #[cfg(soc_has_uart2)]
            Self::Uart2 => crate::peripherals::UART2::regs(),
        };

        cfg_select! {
            uart_has_sclk_divider => {
                let sel = match new_config.sclk() {
                    #[cfg(esp32c2)]
                    UartFunctionClockSclk::PllF40m => 1,
                    #[cfg(any(esp32c3, esp32s3))]
                    UartFunctionClockSclk::Apb => 1,
                    UartFunctionClockSclk::RcFast => 2,
                    UartFunctionClockSclk::Xtal => 3,
                };

                regs.clk_conf().modify(|_, w| unsafe {
                    w.sclk_sel().bits(sel);
                    w.sclk_div_a().bits(0);
                    w.sclk_div_b().bits(0);
                    w.sclk_div_num().bits(new_config.div_num() as _)
                });
            }

            _ => {
                regs.conf0().modify(|_, w| {
                    w.tick_ref_always_on()
                        .bit(new_config.sclk() == UartFunctionClockSclk::Apb)
                });
            }
        }
    }

    // UART_MEM_CLOCK

    pub(crate) fn enable_mem_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    // UART_BAUD_RATE_GENERATOR

    pub(crate) fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    pub(crate) fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        new_config: UartBaudRateGeneratorConfig,
    ) {
        let regs = match self {
            #[cfg(soc_has_uart0)]
            Self::Uart0 => crate::peripherals::UART0::regs(),
            #[cfg(soc_has_uart1)]
            Self::Uart1 => crate::peripherals::UART1::regs(),
            #[cfg(soc_has_uart2)]
            Self::Uart2 => crate::peripherals::UART2::regs(),
        };
        regs.clkdiv().write(|w| unsafe {
            w.clkdiv().bits(new_config.integral() as _);
            w.frag().bits(new_config.fractional() as _)
        });
    }
}
