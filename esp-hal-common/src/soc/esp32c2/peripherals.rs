//! # Peripheral instance singletons (ESP32-C2)
//!
//! ## Overview
//!
//! The `Peripherals` module provides singleton instances of various peripherals
//! and allows users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32-C2` chip, such
//! as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32-C2` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `radio` is marked with `false` in the `peripherals!`
//! macro. Basically, that means that there's no real peripheral (no `RADIO`
//! peripheral in the PACs) but we're creating "virtual peripherals" for it in
//! order to ensure the uniqueness of the instance (Singleton).

use esp32c2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    APB_CTRL <= APB_CTRL,
    APB_SARADC <= APB_SARADC,
    ASSIST_DEBUG <= ASSIST_DEBUG,
    DMA <= DMA,
    ECC <= ECC,
    EFUSE <= EFUSE,
    EXTMEM <= EXTMEM,
    GPIO <= GPIO,
    I2C0 <= I2C0,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    LPWR <= RTC_CNTL,
    RNG <= RNG,
    SENSITIVE <= SENSITIVE,
    SHA <= SHA,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    SPI2 <= SPI2,
    SYSTEM <= SYSTEM,
    SYSTIMER <= SYSTIMER,
    TIMG0 <= TIMG0,
    UART0 <= UART0,
    UART1 <= UART1,
    XTS_AES <= XTS_AES,

    // Virtual peripherals:
    BT <= virtual,
    WIFI <= virtual,
}
