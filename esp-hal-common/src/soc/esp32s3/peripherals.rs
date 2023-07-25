//! Peripheral instance singletons (ESP32-S3)
//!
//! ## Overview
//!
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32-S3`
//! chip. It provides singleton instances of various peripherals and allows
//! users to access and use them in their applications.
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
    AES => true,
    APB_CTRL => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    DMA => true,
    DS => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    GPIO_SD => true,
    HMAC => true,
    I2C0 => true,
    I2C1 => true,
    I2S0 => true,
    I2S1 => true,
    INTERRUPT_CORE0 => true,
    INTERRUPT_CORE1 => true,
    IO_MUX => true,
    LCD_CAM => true,
    LEDC => true,
    PCNT => true,
    PERI_BACKUP => true,
    MCPWM0 => true,
    MCPWM1 => true,
    RMT => true,
    RNG => true,
    RSA => true,
    RTC_CNTL => true,
    RTC_I2C => true,
    RTC_IO => true,
    SENS => true,
    SENSITIVE => true,
    SHA => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SPI3 => true,
    SYSTEM => true,
    SYSTIMER => true,
    TIMG0 => true,
    TIMG1 => true,
    TWAI0 => true,
    UART0 => true,
    UART1 => true,
    UART2 => true,
    UHCI0 => true,
    UHCI1 => true,
    USB0 => true,
    USB_DEVICE => true,
    USB_WRAP => true,
    WCL => true,
    XTS_AES => true,
    RADIO => false,
    PSRAM => false,
    ULP_RISCV_CORE => false,
}
