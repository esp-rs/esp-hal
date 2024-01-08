//! # Peripheral instance singletons (ESP32)
//!
//! ## Overview
//!
//! The `Peripherals` module provides singleton instances of various peripherals
//! and allows users to access and use them in their applications.
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
    AES <= AES,
    APB_CTRL <= APB_CTRL,
    BB <= BB,
    EFUSE <= EFUSE,
    FLASH_ENCRYPTION <= FLASH_ENCRYPTION,
    FRC_TIMER <= FRC_TIMER,
    GPIO <= GPIO,
    GPIO_SD <= GPIO_SD,
    HINF <= HINF,
    I2C0 <= I2C0,
    I2C1 <= I2C1,
    I2S0 <= I2S0,
    I2S1 <= I2S1,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    MCPWM0 <= MCPWM0,
    MCPWM1 <= MCPWM1,
    NRX <= NRX,
    PCNT <= PCNT,
    RMT <= RMT,
    RNG <= RNG,
    RSA <= RSA,
    LPWR <= RTC_CNTL,
    RTC_IO <= RTC_IO,
    RTC_I2C <= RTC_I2C,
    SDHOST <= SDHOST,
    SENS <= SENS,
    SHA <= SHA,
    SLC <= SLC,
    SLCHOST <= SLCHOST,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    SPI2 <= SPI2,
    SPI3 <= SPI3,
    // SYSTEM is derived from DPORT:
    SYSTEM <= DPORT,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    TWAI0 <= TWAI0,
    UART0 <= UART0,
    UART1 <= UART1,
    UART2 <= UART2,
    UHCI0 <= UHCI0,
    UHCI1 <= UHCI1,

    // Virtual peripherals:
    BT <= virtual,
    PSRAM <= virtual,
    WIFI <= virtual,
}
