//! Peripheral instance singletons

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
    SDMMC => true,
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
