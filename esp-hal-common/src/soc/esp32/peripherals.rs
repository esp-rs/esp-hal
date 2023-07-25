//! Peripheral instance singletons (ESP32)
//!
//! ## Overview
//!
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32`
//! chip. It provides singleton instances of various peripherals and allows
//! users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32` chip, such as
//! timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `psram` and `radio` are marked with `false` in the
//! `peripherals!` macro. Basically, that means that there's no real peripheral
//! (no `PSRAM` nor `RADIO` peripheral in the PACs) but we're creating "virtual
//! peripherals" for them in order to ensure the uniqueness of the instances
//! (Singletons).

use esp32 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES => true,
    APB_CTRL => true,
    BB => true,
    DPORT => true,
    EFUSE => true,
    FLASH_ENCRYPTION => true,
    FRC_TIMER => true,
    GPIO => true,
    GPIO_SD => true,
    HINF => true,
    I2C0 => true,
    I2C1 => true,
    I2S0 => true,
    I2S1 => true,
    IO_MUX => true,
    LEDC => true,
    MCPWM0 => true,
    MCPWM1 => true,
    NRX => true,
    PCNT => true,
    RMT => true,
    RNG => true,
    RSA => true,
    RTC_CNTL => true,
    RTC_IO => true,
    RTC_I2C => true,
    SDHOST => true,
    SENS => true,
    SHA => true,
    SLC => true,
    SLCHOST => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SPI3 => true,
    TIMG0 => true,
    TIMG1 => true,
    TWAI0 => true,
    UART0 => true,
    UART1 => true,
    UART2 => true,
    UHCI0 => true,
    UHCI1 => true,
    RADIO => false,
    PSRAM => false,
}
