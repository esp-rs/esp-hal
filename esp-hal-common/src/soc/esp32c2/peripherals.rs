//! Peripheral instance singletons(ESP32-C2)
//! 
//! ## Overview
//! 
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32-C2` chip. It provides singleton instances
//! of various peripherals and allows users to access and use them in their applications.
//! 
//! These peripherals provide various functionalities and interfaces for interacting with different hardware
//! components on the `ESP32-C2` chip, such as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more.
//! Users can access and utilize these peripherals by importing the respective singleton instances from this module.
//! 
//! It's important to note that the module also exports the `Interrupt` enum from the `ESP32-C2` `PAC (Peripheral Access Crate)` for
//! users to handle interrupts associated with these peripherals.
//! 
//! ⚠️ NOTE: notice that `radio` is marked with `false` in the `peripherals!` macro.
//! Basically, that means that there's no real peripheral (no `RADIO` peripheral in the PACs) but we're
//! creating "virtual peripherals" for it in order to ensure the uniqueness of the instance (Singleton).


use esp32c2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    APB_CTRL => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    DMA => true,
    ECC => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    I2C0 => true,
    INTERRUPT_CORE0 => true,
    IO_MUX => true,
    LEDC => true,
    RNG => true,
    RTC_CNTL => true,
    SENSITIVE => true,
    SHA => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SYSTEM => true,
    SYSTIMER => true,
    TIMG0 => true,
    UART0 => true,
    UART1 => true,
    XTS_AES => true,
    RADIO => false
}
