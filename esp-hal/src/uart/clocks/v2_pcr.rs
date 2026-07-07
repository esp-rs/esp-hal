use crate::{
    clock::ll::{
        ClockTree,
        UartBaudRateGeneratorConfig,
        UartFunctionClockConfig,
        UartFunctionClockSclk,
        UartInstance,
    },
    peripherals::{PCR, UART0, UART1},
};

impl UartInstance {
    // UART_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let uart = match self {
            UartInstance::Uart0 => 0,
            UartInstance::Uart1 => 1,
        };
        PCR::regs()
            .uart(uart)
            .clk_conf()
            .modify(|_, w| w.sclk_en().bit(en));
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        let sel = cfg_select! {
            any(esp32c5, esp32c61) => match new_config.sclk() {
                UartFunctionClockSclk::Xtal => 0,
                UartFunctionClockSclk::RcFast => 1,
                UartFunctionClockSclk::PllF80m => 2,
            },
            any(esp32c6, esp32h2) => match new_config.sclk() {
                #[cfg(esp32c6)]
                UartFunctionClockSclk::PllF80m => 1,
                #[cfg(esp32h2)]
                UartFunctionClockSclk::PllF48m => 1,
                UartFunctionClockSclk::RcFast => 2,
                UartFunctionClockSclk::Xtal => 3,
            }
        };

        PCR::regs()
            .uart(match self {
                UartInstance::Uart0 => 0,
                UartInstance::Uart1 => 1,
            })
            .clk_conf()
            .modify(|_, w| unsafe {
                w.sclk_sel().bits(sel);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.sclk_div_num().bits(new_config.div_num() as _);
                w
            });
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
            UartInstance::Uart0 => UART0::regs(),
            UartInstance::Uart1 => UART1::regs(),
        };
        regs.clkdiv().write(|w| unsafe {
            w.clkdiv().bits(new_config.integral() as _);
            w.frag().bits(new_config.fractional() as _)
        });
    }
}
