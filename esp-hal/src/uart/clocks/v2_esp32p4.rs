use crate::clock::ll::{
    ClockTree,
    UartBaudRateGeneratorConfig,
    UartFunctionClockConfig,
    UartInstance,
};

// Per-instance clock impl for UART (called on UartInstance enum)

impl UartInstance {
    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // UART function clock enable is handled by peripheral clock gates in system.rs
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        _new_config: UartFunctionClockConfig,
    ) {
        // TODO: Configure UART clock source selection
        // HP_SYS_CLKRST PERI_CLK_CTRL110-114 for UART0-4
    }

    pub(crate) fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Baud rate generator is always on when UART is enabled
    }

    pub(crate) fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        _new_config: UartBaudRateGeneratorConfig,
    ) {
        // Baud rate is configured directly in UART registers, not here
    }
}
