//! # Peripheral instance singletons (ESP32-S3)
//!
//! ## Overview
//!
//! The `Peripherals` module provides singleton instances of various peripherals
//! and allows users to access and use them in their applications.
//!
//! These peripherals provide various functionalities and interfaces for
//! interacting with different hardware components on the `ESP32-S3` chip, such
//! as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more. Users can access and
//! utilize these peripherals by importing the respective singleton instances
//! from this module.
//!
//! It's important to note that the module also exports the `Interrupt` enum
//! from the `ESP32-S3` `PAC (Peripheral Access Crate)` for users to handle
//! interrupts associated with these peripherals.
//!
//! ⚠️ NOTE: notice that `psram`, `radio` and `ulp_riscv_core` are marked with
//! `false` in the `peripherals!` macro. Basically, that means that there's no
//! real peripheral (no `PSRAM` nor `RADIO` peripheral in the `PAC`s) but we're
//! creating "virtual peripherals" for them in order to ensure the uniqueness of
//! the instances (Singletons).

use esp32s3 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES <= AES,
    APB_CTRL <= APB_CTRL,
    APB_SARADC <= APB_SARADC,
    ASSIST_DEBUG <= ASSIST_DEBUG,
    DMA <= DMA,
    DS <= DS,
    EFUSE <= EFUSE,
    EXTMEM <= EXTMEM,
    GPIO <= GPIO,
    GPIO_SD <= GPIO_SD,
    HMAC <= HMAC,
    I2C0 <= I2C0,
    I2C1 <= I2C1,
    I2S0 <= I2S0,
    I2S1 <= I2S1,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    INTERRUPT_CORE1 <= INTERRUPT_CORE1,
    IO_MUX <= IO_MUX,
    LCD_CAM <= LCD_CAM,
    LEDC <= LEDC,
    PCNT <= PCNT,
    PERI_BACKUP <= PERI_BACKUP,
    MCPWM0 <= MCPWM0,
    MCPWM1 <= MCPWM1,
    RMT <= RMT,
    RNG <= RNG,
    RSA <= RSA,
    RTC_CNTL <= RTC_CNTL,
    RTC_I2C <= RTC_I2C,
    RTC_IO <= RTC_IO,
    SENS <= SENS,
    SENSITIVE <= SENSITIVE,
    SHA <= SHA,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    SPI2 <= SPI2,
    SPI3 <= SPI3,
    SYSTEM <= SYSTEM,
    SYSTIMER <= SYSTIMER,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    TWAI0 <= TWAI0,
    UART0 <= UART0,
    UART1 <= UART1,
    UART2 <= UART2,
    UHCI0 <= UHCI0,
    UHCI1 <= UHCI1,
    USB0 <= USB0,
    USB_DEVICE <= USB_DEVICE,
    USB_WRAP <= USB_WRAP,
    WCL <= WCL,
    XTS_AES <= XTS_AES,

    // Virtual peripherals:
    BT <= virtual,
    ULP_RISCV_CORE <= virtual,
    WIFI <= virtual,
}
