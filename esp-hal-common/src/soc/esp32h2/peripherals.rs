//! # Peripheral instance singletons (ESP32-H2)
//!
//! ## Overview
//!
//! The `Peripherals` module provides singleton instances of various peripherals
//! and allows users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32-H2` chip, such
//! as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32-H2` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `radio` is marked with `false` in the `peripherals!`
//! macro. Basically, that means that there's no real peripheral (no `RADIO`
//! peripheral in the PACs) but we're creating "virtual peripherals" for it in
//! order to ensure the uniqueness of the instance (Singleton).

use esp32h2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES <= AES,
    APB_SARADC <= APB_SARADC,
    ASSIST_DEBUG <= ASSIST_DEBUG,
    DMA <= DMA,
    DS <= DS,
    ECC <= ECC,
    EFUSE <= EFUSE,
    GPIO <= GPIO,
    GPIO_SD <= GPIO_SD,
    HMAC <= HMAC,
    HP_APM <= HP_APM,
    HP_SYS <= HP_SYS,
    I2C0 <= I2C0,
    I2C1 <= I2C1,
    I2S0 <= I2S0,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    INTPRI <= INTPRI,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    LP_ANA <= LP_ANA,
    LP_AON <= LP_AON,
    LP_APM <= LP_APM,
    // RTC_CNTL is derived from LP_CLKRST
    RTC_CNTL <= LP_CLKRST,
    LP_PERI <= LP_PERI,
    LP_TIMER <= LP_TIMER,
    LP_WDT <= LP_WDT,
    MCPWM0 <= MCPWM0,
    MEM_MONITOR <= MEM_MONITOR,
    MODEM_LPCON <= MODEM_LPCON,
    MODEM_SYSCON <= MODEM_SYSCON,
    OTP_DEBUG <= OTP_DEBUG,
    PARL_IO <= PARL_IO,
    PAU <= PAU,
    PCNT <= PCNT,
    // SYSTEM is derived from PCR
    SYSTEM <= PCR,
    PMU <= PMU,
    RMT <= RMT,
    RNG <= RNG,
    RSA <= RSA,
    SHA <= SHA,
    SOC_ETM <= SOC_ETM,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    SPI2 <= SPI2,
    SYSTIMER <= SYSTIMER,
    TEE <= TEE,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    TRACE <= TRACE,
    TWAI0 <= TWAI0,
    UART0 <= UART0,
    UART1 <= UART1,
    UHCI0 <= UHCI0,
    USB_DEVICE <= USB_DEVICE,
    RADIO <= virtual,
}
