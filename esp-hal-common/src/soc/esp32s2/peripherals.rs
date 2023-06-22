//! Peripheral instance singletons

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
}
