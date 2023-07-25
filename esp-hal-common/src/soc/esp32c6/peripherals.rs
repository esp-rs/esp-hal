//! Peripheral instance singletons (ESP32-C6)
//!
//! ## Overview
//!
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32-C6`
//! chip. It provides singleton instances of various peripherals and allows
//! users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32-C6` chip, such
//! as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32-C6` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `radio` and `lp_core` are marked with `false` in the
//! `peripherals!` macro. Basically, that means that there's no real peripheral
//! (no `RADIO` nor `LP_CORE` peripheral in the PACs) but we're
//! creating "virtual peripherals" for them in order to ensure the uniqueness of
//! the instances (Singletons).

use esp32c6 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    ATOMIC => true,
    DMA => true,
    DS => true,
    ECC => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    GPIO_SD => true,
    HINF => true,
    HMAC => true,
    HP_APM => true,
    HP_SYS => true,
    I2C0 => true,
    I2S0 => true,
    INTERRUPT_CORE0 => true,
    INTPRI => true,
    IO_MUX => true,
    LEDC => true,
    LP_PERI => true,
    LP_ANA => true,
    LP_AON => true,
    LP_APM => true,
    LP_APM0 => true,
    LP_CLKRST => true,
    LP_I2C0 => true,
    LP_I2C_ANA_MST => true,
    LP_IO => true,
    LP_TEE => true,
    LP_TIMER => true,
    LP_UART => true,
    LP_WDT => true,
    MCPWM0 => true,
    MEM_MONITOR => true,
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
    SLCHOST => true,
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
    TWAI1 => true,
    UART0 => true,
    UART1 => true,
    UHCI0 => true,
    USB_DEVICE => true,
    RADIO => false,
    LP_CORE => false,
}
