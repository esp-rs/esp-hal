//! Build-time configuration of the external 32 kHz crystal (XTAL32K).
//!
//! The crystal is kept powered down unless `ESP_HAL_CONFIG_USE_XTAL32K` is set. Powering it up
//! drives the dedicated crystal pads, which stops those pins from reaching a true high-impedance
//! state when an application uses them as GPIO.
//!
//! When the option is enabled, those pads are removed from [`Peripherals`] so safe code cannot
//! take them. Pin types, `for_each_gpio`, and analog mappings stay defined for HAL internals;
//! taking the pads via `GPIOn::steal` / [`AnyPin`] while the crystal is on is unsound and can
//! electrically conflict with the oscillator (GPIO, ADC, LP, and similar).
//!
//! This module only exists for chips with dedicated crystal pads (`soc_has_xtal32k_pads`), the
//! same set the `use-xtal32k` option is active for
//!
//! [`Peripherals`]: crate::peripherals::Peripherals
//! [`AnyPin`]: crate::gpio::AnyPin

/// Whether esp-hal should power and use the external 32 kHz crystal.
#[inline(always)]
pub(crate) const fn use_xtal32k() -> bool {
    cfg!(use_xtal32k)
}

/// The LP slow clock source to select by default.
#[cfg(soc_has_clock_node_lp_slow_clk)]
pub(crate) const fn default_lp_slow_clk() -> crate::soc::clocks::LpSlowClkConfig {
    if use_xtal32k() {
        crate::soc::clocks::LpSlowClkConfig::Xtal32k
    } else {
        crate::soc::clocks::LpSlowClkConfig::RcSlow
    }
}

/// The RTC slow clock source to select by default.
#[cfg(soc_has_clock_node_rtc_slow_clk)]
pub(crate) const fn default_rtc_slow_clk() -> crate::soc::clocks::RtcSlowClkConfig {
    if use_xtal32k() {
        crate::soc::clocks::RtcSlowClkConfig::Xtal32k
    } else {
        crate::soc::clocks::RtcSlowClkConfig::RcSlow
    }
}
