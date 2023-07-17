//! Peripheral instance singletons(ESP32-S2)
//! 
//! ## Overview
//! 
//! The `Peripherals` module is a part of the `SOC` functionality of `ESP32-S2` chip. It provides singleton instances
//! of various peripherals and allows users to access and use them in their applications.
//! 
//! These peripherals provide various functionalities and interfaces for interacting with different hardware
//! components on the `ESP32-S2` chip, such as timers, `GPIO` pins, `I2C`, `SPI`, `UART`, and more.
//! Users can access and utilize these peripherals by importing the respective singleton instances from this module.
//! 
//! It's important to note that the module also exports the `Interrupt` enum from the `ESP32-S2` `PAC (Peripheral Access Crate)` for
//! users to handle interrupts associated with these peripherals.
//! 
//! NOTE: notice that `psram`, `radio` and `ulp_riscv_core` are marked with `false` in the `peripherals!` macro.
//! Basically, that means that there's no real peripheral (no `PSRAM` nor `RADIO` peripheral in the `PAC`s) but we're
//! creating "virtual peripherals" for them in order to ensure the uniqueness of the instances (Singletons).

use esp32s2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES => true,
    APB_SARADC => true,
    DEDICATED_GPIO => true,
    DS => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    GPIO_SD => true,
    HMAC => true,
    I2C0 => true,
    I2C1 => true,
    I2S0 => true,
    INTERRUPT_CORE0 => true,
    IO_MUX => true,
    LEDC => true,
    PCNT => true,
    PMS => true,
    RMT => true,
    RNG => true,
    RSA => true,
    RTC_IO => true,
    RTC_CNTL => true,
    RTC_I2C => true,
    SENS => true,
    SHA => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SPI3 => true,
    SPI4 => true,
    SYSCON => true,
    SYSTEM => true,
    SYSTIMER => true,
    TIMG0 => true,
    TIMG1 => true,
    TWAI0 => true,
    UART0 => true,
    UART1 => true,
    UHCI0 => true,
    USB0 => true,
    USB_WRAP => true,
    XTS_AES => true,
    RADIO => false,
    PSRAM => false,
    ULP_RISCV_CORE => false,
}
