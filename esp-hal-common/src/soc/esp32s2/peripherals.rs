//! # Peripheral instance singletons (ESP32-S2)
//!
//! ## Overview
//!
//! The `Peripherals` module singleton instances of various peripherals and
//! allows users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32-S2` chip, such
//! as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32-S2` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `psram`, `radio` and `ulp_riscv_core` are marked with
//! `false` in the `peripherals!` macro. Basically, that means that there's no
//! real peripheral (no `PSRAM`, `RADIO` nor `ULP_RISCV_CORE` peripheral in the
//! `PAC`s) but we're creating "virtual peripherals" for them in order to ensure
//! the uniqueness of the instances (Singletons).

use esp32s2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES <= AES,
    APB_SARADC <= APB_SARADC,
    DEDICATED_GPIO <= DEDICATED_GPIO,
    DS <= DS,
    EFUSE <= EFUSE,
    EXTMEM <= EXTMEM,
    GPIO <= GPIO,
    GPIO_SD <= GPIO_SD,
    HMAC <= HMAC,
    I2C0 <= I2C0,
    I2C1 <= I2C1,
    I2S0 <= I2S0,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    PCNT <= PCNT,
    PMS <= PMS,
    RMT <= RMT,
    RNG <= RNG,
    RSA <= RSA,
    RTC_IO <= RTC_IO,
    RTC_CNTL <= RTC_CNTL,
    RTC_I2C <= RTC_I2C,
    SENS <= SENS,
    SHA <= SHA,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    SPI2 <= SPI2,
    SPI3 <= SPI3,
    SPI4 <= SPI4,
    SYSCON <= SYSCON,
    SYSTEM <= SYSTEM,
    SYSTIMER <= SYSTIMER,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    TWAI0 <= TWAI0,
    UART0 <= UART0,
    UART1 <= UART1,
    UHCI0 <= UHCI0,
    USB0 <= USB0,
    USB_WRAP <= USB_WRAP,
    XTS_AES <= XTS_AES,
    RADIO <= virtual,
    PSRAM <= virtual,
    ULP_RISCV_CORE <= virtual,
}
