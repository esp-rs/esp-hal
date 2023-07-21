//! Peripheral instance singletons(ESP32-H2)
//! 
//! ## Overview
//! 
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32-H2` chip. It provides singleton instances
//! of various peripherals and allows users to access and use them in their applications.
//! 
//! These peripherals provide various functionalities and interfaces for interacting with different hardware
//! components on the `ESP32-H2` chip, such as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more.
//! Users can access and utilize these peripherals by importing the respective singleton instances from this module.
//! 
//! It's important to note that the module also exports the `Interrupt` enum from the `ESP32-H2` `PAC (Peripheral Access Crate)` for
//! users to handle interrupts associated with these peripherals.
//! 
//! ⚠️ NOTE: notice that `radio` is marked with `false` in the `peripherals!` macro.
//! Basically, that means that there's no real peripheral (no `RADIO` peripheral in the PACs) but we're
//! creating "virtual peripherals" for it in order to ensure the uniqueness of the instance (Singleton).

use esp32h2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    DMA => true,
    DS => true,
    ECC => true,
    EFUSE => true,
    GPIO => true,
    HMAC => true,
    HP_APM => true,
    HP_SYS => true,
    I2C0 => true,
    I2C1 => true,
    I2S0 => true,
    INTERRUPT_CORE0 => true,
    INTPRI => true,
    IO_MUX => true,
    LEDC => true,
    LP_ANA => true,
    LP_AON => true,
    LP_APM => true,
    LP_CLKRST => true,
    LP_PERI => true,
    LP_TIMER => true,
    LP_WDT => true,
    MCPWM0 => true,
    MEM_MONITOR => true,
    MODEM_LPCON => true,
    MODEM_SYSCON => true,
    OTP_DEBUG => true,
    PARL_IO => true,
    PAU => true,
    PCNT => true,
    PCR => true,
    PMU => true,
    RMT => true,
    RNG => true,
    RSA => true,
    SHA => true,
    SOC_ETM => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SYSTIMER => true,
    TEE => true,
    TIMG0 => true,
    TIMG1 => true,
    TRACE => true,
    TWAI0 => true,
    UART0 => true,
    UART1 => true,
    UHCI0 => true,
    USB_DEVICE => true,
    RADIO => false,
}
